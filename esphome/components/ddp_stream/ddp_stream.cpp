// © Copyright 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

#include "ddp_stream.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "esp_timer.h"

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

// mDNS (ESP-IDF)
extern "C" {
  #include "mdns.h"
}

namespace esphome {
namespace ddp_stream {

static const char *TAG = "ddp_stream";

#ifndef DDP_STREAM_METRICS
#define DDP_STREAM_METRICS 1
#endif

// Require a proper DDP PUSH to present (eliminates partial-tail tearing)
#ifndef DDP_REQUIRE_PUSH
#define DDP_REQUIRE_PUSH 1
#endif

// Optional fallback knobs (used only if DDP_REQUIRE_PUSH == 0)
#ifndef DDP_FORCE_MIN_COVERAGE
#define DDP_FORCE_MIN_COVERAGE 0.985   // 98.5% of RGB bytes
#endif
#ifndef DDP_FORCE_TAIL_GRACE_US
#define DDP_FORCE_TAIL_GRACE_US 2500   // wait up to 2.5 ms for tail packets
#endif

// ---------- file-scope state for deferred mDNS ----------
static bool g_mdns_inited    = false;
static bool g_mdns_published = false;

// Publish _ddp._udp once, with known w/h and stream count.
static void mdns_publish_ddp_once(uint16_t port, int w, int h, int streams) {
  if (g_mdns_published) return;

  if (!g_mdns_inited) {
    if (mdns_init() == ESP_OK) {
      g_mdns_inited = true;
    } else {
      ESP_LOGW(TAG, "mDNS init failed; skipping _ddp._udp advertisement");
      return;
    }
  }

  // Build hostname from first IP if available, else fall back to port.
  auto ips = network::get_ip_addresses();
  std::string host = "ddp-";
  if (!ips.empty()) {
    std::string ip = ips.front().str();       // "192.168.1.42"
    for (char &ch : ip) host.push_back(ch == '.' ? '-' : ch);
  } else {
    char tmp[16]; snprintf(tmp, sizeof(tmp), "%u", (unsigned) port);
    host += tmp;
  }

  mdns_hostname_set(host.c_str());
  mdns_instance_name_set("ESPHome DDP");

  char port_txt[8]; snprintf(port_txt, sizeof(port_txt), "%u", (unsigned) port);
  char w_txt[8];    snprintf(w_txt,    sizeof(w_txt),    "%d", w);
  char h_txt[8];    snprintf(h_txt,    sizeof(h_txt),    "%d", h);
  char n_txt[8];    snprintf(n_txt,    sizeof(n_txt),    "%d", streams);

  mdns_txt_item_t txt[] = {
      {"proto", "ddp"},
      {"brand", "lvgl-ddp-stream"},
      {"port",  port_txt},
      {"w",     w_txt},
      {"h",     h_txt},
      {"streams", n_txt},
  };

  mdns_service_add("ESPHome DDP", "_ddp", "_udp", port,
                   txt, sizeof(txt) / sizeof(txt[0]));
  g_mdns_published = true;
  ESP_LOGI(TAG, "mDNS: advertised _ddp._udp on %u host=%s (w=%d h=%d streams=%d)",
           (unsigned) port, host.c_str(), w, h, streams);
}

// ---------- helpers ----------

static uint16_t LUT_R5[256];
static uint16_t LUT_G6[256];
static uint16_t LUT_B5[256];
static bool     LUT_INITD = false;

static inline void init_rgb_luts_once() {
  if (LUT_INITD) return;
  for (int i = 0; i < 256; ++i) {
    LUT_R5[i] = (uint16_t)((i & 0xF8) << 8);
    LUT_G6[i] = (uint16_t)((i & 0xFC) << 3);
    LUT_B5[i] = (uint16_t)( i >> 3);
  }
  LUT_INITD = true;
}

static inline void canvas_set_buf_rgb565(lv_obj_t *canvas, void *data, int w, int h) {
  lv_canvas_set_buffer(canvas, data, (lv_coord_t) w, (lv_coord_t) h, LV_IMG_CF_TRUE_COLOR);
}

#if DDP_STREAM_METRICS
static inline double ewma(double prev, double sample, double alpha = 0.2) {
  return (prev == 0.0) ? sample : (alpha * sample + (1.0 - alpha) * prev);
}
#endif

// -------- public API --------

void DdpStream::add_stream_binding(uint8_t id,
                                   std::function<lv_obj_t *()> getter,
                                   int w, int h) {
  Binding &dst = bindings_[id];
  dst.getter = std::move(getter);
  dst.w = w;
  dst.h = h;
  this->bind_if_possible_(dst);
}

void DdpStream::set_stream_canvas(uint8_t id, lv_obj_t *canvas, int w, int h) {
  Binding &dst = bindings_[id];
  dst.getter = [canvas]() -> lv_obj_t * { return canvas; };
  dst.canvas = canvas;
  dst.w = w;
  dst.h = h;
  this->bind_if_possible_(dst);
}

bool DdpStream::get_stream_size(uint8_t id, int *w, int *h) const {
  auto it = bindings_.find(id);
  if (it == bindings_.end()) return false;
  const Binding &b = it->second;
  if (b.w <= 0 || b.h <= 0) return false;
  if (w) *w = b.w;
  if (h) *h = b.h;
  return true;
}

// -------- lifecycle --------

void DdpStream::setup() {
  ESP_LOGI(TAG, "setup this=%p port=%u", this, (unsigned) port_);
  init_rgb_luts_once();

  // NOTE: we intentionally DO NOT advertise mDNS in setup().
  // We'll publish _ddp._udp the moment the first binding resolves so w/h are known.

  this->set_interval("ddp_net", 500, [this]() { this->ensure_socket_(); });

  this->set_interval("ddp_bind", 250, [this]() {
    bool all_ready = true;
    bool any_newly_bound = false;
    for (auto &kv : bindings_) {
      Binding &b = kv.second;
      bool before = b.bound;
      this->bind_if_possible_(b);
      bool after = b.bound;
      if (!before && after) {
        any_newly_bound = true;
        ESP_LOGI(TAG, "stream %u bound to canvas=%p (%dx%d)",
                 (unsigned) kv.first, (void *) b.canvas, b.w, b.h);
        lv_canvas_fill_bg(b.canvas, lv_color_black(), LV_OPA_COVER);
#if DDP_STREAM_METRICS
        b.log_t0_us = esp_timer_get_time();
#endif
      }
      all_ready = all_ready && after;
    }

    // If we just bound at least one stream and haven't advertised yet, do it now.
    if (any_newly_bound && !g_mdns_published) {
      int adv_w = 0, adv_h = 0, streams = 0;
      for (const auto &kv : bindings_) {
        const Binding &b = kv.second;
        if (b.w > 0 && b.h > 0) {
          if (streams == 0) { adv_w = b.w; adv_h = b.h; }
          streams++;
        }
      }
      mdns_publish_ddp_once(port_, adv_w, adv_h, streams);
    }

    if (all_ready) {
      ESP_LOGI(TAG, "all DDP bindings resolved; stopping binding resolver");
      this->cancel_interval("ddp_bind");
    }
  });

  ESP_LOGI(TAG, "initialized; waiting for network to open UDP %u", this->port_);
}

void DdpStream::dump_config() {
  ESP_LOGCONFIG(TAG, "DDP stream:");
  ESP_LOGCONFIG(TAG, "  UDP port: %u", port_);
  for (auto &kv : bindings_) {
    const Binding &b = kv.second;
    ESP_LOGCONFIG(TAG, "  stream id %u -> %dx%d canvas=%p (buf=%zu)",
                  kv.first, b.w, b.h, (void *) b.canvas, b.front_buf.size());
  }
}

void DdpStream::loop() {
  for (auto &kv : bindings_) {
    Binding &b = kv.second;

    if (!b.have_ready.exchange(false))
      continue;

    if (!b.canvas || b.w <= 0 || b.h <= 0)
      continue;

    const int w = b.w;
    const int h = b.h;

    // Present by swapping buffers so we never write into what LVGL/DMA is scanning.
    b.front_buf.swap(b.ready_buf);
    lv_canvas_set_buffer(b.canvas, b.front_buf.data(),
                         (lv_coord_t) w, (lv_coord_t) h, LV_IMG_CF_TRUE_COLOR);

    // Invalidate full canvas so LVGL blits the new front buffer.
    lv_area_t a;
    a.x1 = 0; a.y1 = 0;
    a.x2 = static_cast<lv_coord_t>(w - 1);
    a.y2 = static_cast<lv_coord_t>(h - 1);
    lv_obj_invalidate_area(b.canvas, &a);

#if DDP_STREAM_METRICS
    // queue wait time (ready -> present)
    if (b.ready_set_us) {
      int64_t now_us = esp_timer_get_time();
      double qwait_ms = (double) (now_us - b.ready_set_us) / 1000.0;
      b.qwait_ms_ewma = ewma(b.qwait_ms_ewma, qwait_ms, 0.2);
      b.ready_set_us = 0;
    }

    // present latency (first packet -> present)
    if (b.frame_seen_any && b.frame_first_pkt_us != 0) {
      int64_t now = esp_timer_get_time();
      double lat_us = (double) (now - b.frame_first_pkt_us);
      b.present_lat_us_ewma = ewma(b.present_lat_us_ewma, lat_us);
      b.frame_seen_any = false;
      b.frame_first_pkt_us = 0;
    }

    b.frames_presented += 1;
    b.win_frames_presented += 1;
#endif
  }

#if DDP_STREAM_METRICS
  // Periodic windowed log (every ~2s) — unchanged
  int64_t now = esp_timer_get_time();
  for (auto &kv : bindings_) {
    uint8_t id = kv.first;
    Binding &b = kv.second;
    if (!b.bound) continue;
    if (b.log_t0_us == 0) b.log_t0_us = now;
    int64_t dt_us = now - b.log_t0_us;
    if (dt_us >= 2'000'000) {
      double dt_s = (double) dt_us / 1e6;

      double push_fps = b.win_frames_push / dt_s;
      double pres_fps = b.win_frames_presented / dt_s;
      double rx_mbps  = (b.win_rx_bytes * 8.0) / (dt_s * 1e6);
      double cov_pct  = b.coverage_ewma * 100.0;
      double lat_ms   = b.present_lat_us_ewma / 1000.0;
      double qwait_ms = b.qwait_ms_ewma;
      double pkts_per_wakeup = (b.rx_wakeups > 0)
                               ? ((double) b.rx_pkts_in_slices / (double) b.rx_wakeups)
                               : 0.0;

      ESP_LOGI(TAG,
               "id=%u rx_pkts=%llu rx_B=%llu win{push=%.1ffps pres=%.1ffps rx=%.2fMb/s pkt_gap=%u overrun=%u} "
               "tot{push=%u pres=%u overrun=%u} cov(avg)=%.1f%% build(avg)=%.1f ms lat(avg)=%.1f ms qwait(avg)=%.1f ms rx_slc{wakeups=%u pkts/burst=%.2f}",
               (unsigned) id,
               (unsigned long long) b.rx_pkts,
               (unsigned long long) b.rx_bytes,
               push_fps, pres_fps, rx_mbps, b.win_pkt_gap, b.win_overrun,
               b.frames_push, b.frames_presented, b.frames_overrun,
               cov_pct, b.build_ms_ewma, lat_ms, qwait_ms,
               b.rx_wakeups, pkts_per_wakeup
      );

      // reset window; keep totals & EWMA
      b.rx_wakeups = 0;
      b.rx_pkts_in_slices = 0;
      b.win_frames_push = 0;
      b.win_frames_presented = 0;
      b.win_rx_bytes = 0;
      b.win_pkt_gap = 0;
      b.win_overrun = 0;
      b.log_t0_us = now;
    }
  }
#endif
}

// -------- socket/RX --------

void DdpStream::ensure_socket_() {
  bool net_up = network::is_connected();
  if (net_up) {
    if (sock_ < 0) open_socket_();
  } else {
    if (sock_ >= 0) close_socket_();
  }
}

void DdpStream::open_socket_() {
  if (sock_ >= 0) return;

  int s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) { ESP_LOGW(TAG, "socket() failed errno=%d", errno); return; }

  int yes = 1;
  (void) setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  (void) setsockopt(s, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes)); // allow replying to bcast probes

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port_);
  if (bind(s, (sockaddr *) &addr, sizeof(addr)) < 0) {
    ESP_LOGW(TAG, "bind() failed errno=%d", errno);
    ::close(s);
    return;
  }

  sock_ = s;

  if (!udp_opened_) {
    ESP_LOGI(TAG, "DDP listening on UDP %u", port_);
    udp_opened_ = true;
  }

  if (task_ == nullptr) {
    // Higher prio + pin to opposite core from typical LVGL usage
    xTaskCreatePinnedToCore(&DdpStream::recv_task_trampoline, "ddp_rx",
                            6144, this, 6, &task_, 1);
  }
}

void DdpStream::close_socket_() {
  if (task_ != nullptr) {
    TaskHandle_t t = task_;
    task_ = nullptr;
    if (sock_ >= 0) {
      ::shutdown(sock_, SHUT_RDWR);
      ::close(sock_); sock_ = -1;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    vTaskDelete(t);
  } else if (sock_ >= 0) {
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
    sock_ = -1;
  }
  ESP_LOGI(TAG, "DDP socket closed");
}

void DdpStream::recv_task_trampoline(void *arg) {
  reinterpret_cast<DdpStream *>(arg)->recv_task();
}

void DdpStream::recv_task() {
  std::vector<uint8_t> buf(2048);

  while (true) {
    if (task_ == nullptr) break;
    int s = sock_;
    if (s < 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s, &rfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500;  // wait ~0.5ms to reduce burst splitting

    int r = lwip_select(s + 1, &rfds, nullptr, nullptr, &tv);
    if (r > 0 && FD_ISSET(s, &rfds)) {
      const int MAX_PKTS_PER_SLICE = 192;
      const int MAX_SLICE_US       = 6000;  // give tail packets a bit more room
      int handled = 0;
      int64_t slice_start = esp_timer_get_time();

      while (handled < MAX_PKTS_PER_SLICE) {
        sockaddr_in src{};
        socklen_t   slen = sizeof(src);
        ssize_t n = ::recvfrom(s, buf.data(), buf.size(), MSG_DONTWAIT,
                               (sockaddr *) &src, &slen);
        if (n > 0) {
          // --- Lightweight UDP JSON probe handling (discovery) ---
          if (n < 512) {
            // Trim leading whitespace
            size_t i = 0;
            while (i < (size_t) n && (buf[i] == ' ' || buf[i] == '\t' || buf[i] == '\r' || buf[i] == '\n')) i++;
            bool looks_json = (i < (size_t) n && buf[i] == '{');
            if (looks_json) {
              const char *p = (const char *) buf.data();
              if (std::strstr(p, "ddp") || std::strstr(p, "probe") || std::strstr(p, "status")) {
                // Build compact status JSON inline
                auto ips = network::get_ip_addresses();
                std::string ipstr = ips.empty() ? "0.0.0.0" : ips.front().str();

                std::string js;
                js.reserve(128);
                js += "{\"proto\":\"ddp\",\"name\":\"ESPHome DDP\",\"ip\":\"";
                js += ipstr;
                js += "\",\"port\":";
                char num[16]; snprintf(num, sizeof(num), "%u", (unsigned) port_);
                js += num;
                js += ",\"streams\":[";

                bool first = true;
                for (const auto &kv2 : bindings_) {
                  const Binding &b2 = kv2.second;
                  if (b2.w <= 0 || b2.h <= 0) continue;
                  if (!first) js += ",";
                  first = false;
                  char sn[64];
                  snprintf(sn, sizeof(sn), "{\"id\":%u,\"w\":%d,\"h\":%d}",
                           (unsigned) kv2.first, b2.w, b2.h);
                  js += sn;
                }
                js += "]}";

                (void) ::sendto(s, js.c_str(), js.size(), 0, (sockaddr *) &src, slen);
                handled++;
                if ((esp_timer_get_time() - slice_start) >= MAX_SLICE_US)
                  break;
                continue; // do not fall into normal decode path
              }
            }
          }

          // --- Normal DDP fast path ---
          this->handle_packet_(buf.data(), (size_t) n);

          // Only break if THIS packet was a PUSH (end of frame).
          if ((size_t) n >= sizeof(DdpHeader)) {
            const DdpHeader *h = reinterpret_cast<const DdpHeader *>(buf.data());
            if ((h->flags & 0x41) == 0x41) { // ver|PUSH
              handled++;
              break;
            }
          }

          handled++;
          if ((esp_timer_get_time() - slice_start) >= MAX_SLICE_US)
            break;
          continue;
        }
        break; // no more queued data immediately
      }

#if DDP_STREAM_METRICS
      for (auto &kv2 : bindings_) {
        Binding &b2 = kv2.second;
        if (!b2.bound) continue;
        b2.rx_wakeups += 1;
        b2.rx_pkts_in_slices += (uint64_t) handled;
      }
#endif
      taskYIELD();
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// -------- binding helpers --------

void DdpStream::bind_if_possible_(Binding &b) {
  if (b.bound) return;
  if (!b.canvas && b.getter) b.canvas = b.getter();
  if (!b.canvas) return;
  if (b.w <= 0 || b.h <= 0) {
    int W = (int) lv_obj_get_width(b.canvas);
    int H = (int) lv_obj_get_height(b.canvas);
    if (b.w <= 0) b.w = W;
    if (b.h <= 0) b.h = H;
  }
  if (b.w <= 0 || b.h <= 0) return;
  ensure_binding_buffers_(b);
  if (b.front_buf.empty() || b.ready_buf.empty() || b.accum_buf.empty()) return;

  canvas_set_buf_rgb565(b.canvas, b.front_buf.data(), b.w, b.h);

  lv_obj_add_event_cb(b.canvas, &DdpStream::on_canvas_size_changed_, LV_EVENT_SIZE_CHANGED, this);
  b.bound = true;
}

void DdpStream::ensure_binding_buffers_(Binding &b) {
  if (b.w <= 0 || b.h <= 0) return;
  const size_t px = (size_t) b.w * (size_t) b.h;
  if (b.front_buf.size() != px) b.front_buf.assign(px, 0);
  if (b.ready_buf.size() != px) b.ready_buf.assign(px, 0);
  if (b.accum_buf.size() != px) b.accum_buf.assign(px, 0);
}

DdpStream::Binding *DdpStream::find_binding_(uint8_t id) {
  auto it = bindings_.find(id);
  if (it == bindings_.end()) return nullptr;
  return &it->second;
}

void DdpStream::on_canvas_size_changed_(lv_event_t *e) {
  auto *canvas = lv_event_get_target(e);
  auto *self   = static_cast<DdpStream *>(lv_event_get_user_data(e));
  if (!self || !canvas) return;
  for (auto &kv : self->bindings_) {
    Binding &b = kv.second;
    if (b.canvas != canvas) continue;
    bool auto_any = (b.w <= 0 || b.h <= 0);
    if (!auto_any) return;
    int W = (int) lv_obj_get_width(canvas);
    int H = (int) lv_obj_get_height(canvas);
    if (b.w <= 0) b.w = W;
    if (b.h <= 0) b.h = H;
    self->ensure_binding_buffers_(b);
    if (!b.front_buf.empty()) {
      canvas_set_buf_rgb565(b.canvas, b.front_buf.data(), b.w, b.h);
    }
    break;
  }
}

// -------- DDP decoder --------

void DdpStream::handle_packet_(const uint8_t *raw, size_t n) {
  if (n < sizeof(DdpHeader)) return;
  auto *h = reinterpret_cast<const DdpHeader *>(raw);

  bool ver  = (h->flags & 0x40) != 0;
  bool push = (h->flags & 0x01) != 0;
  if (!ver) return;

  uint8_t id = h->id;
  Binding *b = this->find_binding_(id);
  if (!b || !b->canvas || b->w <= 0 || b->h <= 0) return;

  uint32_t offset = ntohl(h->offset_be);
  uint16_t len    = ntohs(h->length_be);

  // Handle PUSH-only packet: present whatever we've accumulated
  if (len == 0) {
    if (push) {
#if DDP_STREAM_METRICS
      b->frames_push += 1;
      b->win_frames_push += 1;
      if (b->frame_first_pkt_us) {
        int64_t now_us = esp_timer_get_time();
        double build_ms = (double) (now_us - b->frame_first_pkt_us) / 1000.0;
        b->build_ms_ewma = ewma(b->build_ms_ewma, build_ms, 0.2);
      }
#endif
      if (b->have_ready.load()) {
#if DDP_STREAM_METRICS
        b->frames_overrun += 1;
        b->win_overrun += 1;
#endif
      }
      this->ensure_binding_buffers_(*b);
      if (!b->accum_buf.empty()) {
        b->ready_buf.swap(b->accum_buf);
        b->have_ready.store(true);
#if DDP_STREAM_METRICS
        b->ready_set_us = esp_timer_get_time();
#endif
      }
    }
    return;
  }

  if (sizeof(DdpHeader) + len > n) return;

  const uint8_t *p = raw + sizeof(DdpHeader);

  if ((offset % 3) != 0 || (len % 3) != 0) return;

  size_t pixel_off = offset / 3;
  size_t px        = len / 3;
  size_t max_px    = (size_t) b->w * (size_t) b->h;
  if (pixel_off >= max_px) return;
  size_t write_px  = std::min(px, max_px - pixel_off);

  this->ensure_binding_buffers_(*b);
  if (b->accum_buf.empty()) return;

#if DDP_STREAM_METRICS
  if (!b->frame_seen_any) {
    b->frames_started += 1;
    b->frame_first_pkt_us = esp_timer_get_time();
    b->frame_bytes_accum = 0;
    b->frame_px_accum = 0;
    b->frame_seen_any = true;
    b->intra_ms_max = 0.0;       // reset per-frame metric
    b->last_pkt_us = 0;
  }

  int64_t now_us_gap = esp_timer_get_time();
  if (b->last_pkt_us) {
    double gap_ms = (double) (now_us_gap - b->last_pkt_us) / 1000.0;
    if (gap_ms > b->intra_ms_max) b->intra_ms_max = gap_ms;
  }
  b->last_pkt_us = now_us_gap;

  b->rx_pkts += 1;
  b->rx_bytes += len;
  b->win_rx_bytes += len;
#endif

  uint16_t *dst = b->accum_buf.data() + pixel_off;
  const uint8_t *sp = p;

#if defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
  for (size_t i = 0; i < write_px; ++i) {
    uint16_t c = LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]];
    dst[i] = (uint16_t) ((c >> 8) | (c << 8));
    sp += 3;
  }
#else
  for (size_t i = 0; i < write_px; ++i) {
    dst[i] = (uint16_t) (LUT_R5[sp[0]] | LUT_G6[sp[1]] | LUT_B5[sp[2]]);
    sp += 3;
  }
#endif

#if DDP_STREAM_METRICS
  b->frame_bytes_accum += len;
  b->frame_px_accum    += write_px;

  uint8_t cur_pkt = (uint8_t) (h->seq & 0x0F);
  if (b->have_last_seq_pkt) {
    uint8_t prev = b->last_seq_pkt & 0x0F;
    uint8_t delta = (uint8_t) ((cur_pkt - prev) & 0x0F);
    if (delta > 1) {
      uint32_t add = (uint32_t) (delta - 1);
      b->pkt_seq_gaps += add;
      b->win_pkt_gap  += add;
    }
  }
  b->last_seq_pkt = cur_pkt;
  b->have_last_seq_pkt = true;
#endif

  if (push) {
#if DDP_STREAM_METRICS
    b->frames_push += 1;
    b->win_frames_push += 1;

    const size_t expected_bytes = (size_t) b->w * (size_t) b->h * 3;
    double cov = (expected_bytes > 0)
                 ? std::min(1.0, (double) b->frame_bytes_accum / (double) expected_bytes)
                 : 0.0;
    b->coverage_ewma = ewma(b->coverage_ewma, cov);

    const double COMPLETE_THRESH = 0.95;
    if (cov >= COMPLETE_THRESH) b->frames_ok += 1;
    else                        b->frames_incomplete += 1;

    uint8_t cur_push = (uint8_t) (h->seq & 0x0F);
    if (b->have_last_seq) {
      uint8_t prev = b->last_seq_push & 0x0F;
      uint8_t exp  = (uint8_t) ((prev + 1) & 0x0F);
      if (cur_push != exp) b->push_seq_misses += 1;
    }
    b->last_seq_push = cur_push;
    b->have_last_seq = true;

    if (b->frame_first_pkt_us) {
      int64_t now_us = esp_timer_get_time();
      double build_ms = (double) (now_us - b->frame_first_pkt_us) / 1000.0;
      b->build_ms_ewma = ewma(b->build_ms_ewma, build_ms, 0.2);
    }
#endif

    if (b->have_ready.load()) {
#if DDP_STREAM_METRICS
      b->frames_overrun += 1;
      b->win_overrun += 1;
#endif
      // overwrite policy: keep newest
    }

    b->ready_buf.swap(b->accum_buf);
    b->have_ready.store(true);
#if DDP_STREAM_METRICS
    b->ready_set_us = esp_timer_get_time();
#endif
  }
}

// -------- optional: force-present fallback (disabled by default) --------

void DdpStream::force_present_if_due_(Binding &b) {
#if DDP_REQUIRE_PUSH
  (void) b;  // disabled: present only on PUSH to avoid tearing
  return;
#else
  if (b.w <= 0 || b.h <= 0) return;
  if (b.accum_buf.empty()) return;

  const size_t expected_bytes = (size_t) b.w * (size_t) b.h * 3;
  const double cur_cov = expected_bytes ? (double) b.frame_bytes_accum / (double) expected_bytes : 1.0;

  const int64_t now_us = esp_timer_get_time();
  const int64_t since_last_pkt_us = b.last_pkt_us ? (now_us - b.last_pkt_us) : (int64_t) 1e9;

  if (cur_cov < DDP_FORCE_MIN_COVERAGE && since_last_pkt_us < DDP_FORCE_TAIL_GRACE_US) {
    // Defer force-present: we expect trailing packets imminently.
    return;
  }

#if DDP_STREAM_METRICS
  if (b.have_ready.load()) {
    b.frames_overrun += 1;
    b.win_overrun += 1;
  }
#endif

  b.ready_buf.swap(b.accum_buf);
  b.have_ready.store(true);

#if DDP_STREAM_METRICS
  b.ready_set_us = now_us;

  if (b.frame_first_pkt_us) {
    double build_ms = (double) (now_us - b.frame_first_pkt_us) / 1000.0;
    b.build_ms_ewma = ewma(b.build_ms_ewma, build_ms, 0.2);
  }
  b.coverage_ewma = ewma(b.coverage_ewma, cur_cov);
#endif

  // Reset per-frame trackers
  b.frame_seen_any = false;
  b.frame_first_pkt_us = 0;
  b.frame_bytes_accum = 0;
  b.frame_px_accum = 0;
  b.intra_ms_max = 0.0;
  b.last_pkt_us = 0;
#endif
}

}  // namespace ddp_stream
}  // namespace esphome
