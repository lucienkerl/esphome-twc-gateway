/*
TWC Gateway for ESP32
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 3
of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "loxone_bridge.h"

#ifdef USE_TWC_LOXONE

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <lwip/inet.h>
#include <lwip/sockets.h>

#include "esphome/components/network/util.h"
#include "esphome/core/log.h"

namespace esphome {
namespace twc_gateway {

static const char *const TAG = "twc-gateway.loxone";

static constexpr uint32_t HTTP_TIMEOUT_MS = 3000;
static constexpr size_t MAX_QUEUE = 4;
// The UDP datagram goes out on change (at most every udp_interval) and at
// least this often, so a Miniserver restart picks the values up again.
static constexpr uint32_t UDP_KEEPALIVE_MS = 10000;
// With UDP only, nothing comes back from the Miniserver; any HTTP answer to
// this request (sent without credentials) proves it alive for the watchdog.
static constexpr const char *ALIVE_PATH = "/jdev/cfg/version";

namespace {

std::string base64(const std::string &in) {
  static const char *const CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  size_t i = 0;
  while (i + 2 < in.size()) {
    uint32_t n = (uint8_t(in[i]) << 16) | (uint8_t(in[i + 1]) << 8) | uint8_t(in[i + 2]);
    out += CHARS[(n >> 18) & 63];
    out += CHARS[(n >> 12) & 63];
    out += CHARS[(n >> 6) & 63];
    out += CHARS[n & 63];
    i += 3;
  }
  if (i + 1 == in.size()) {
    uint32_t n = uint8_t(in[i]) << 16;
    out += CHARS[(n >> 18) & 63];
    out += CHARS[(n >> 12) & 63];
    out += "==";
  } else if (i + 2 == in.size()) {
    uint32_t n = (uint8_t(in[i]) << 16) | (uint8_t(in[i + 1]) << 8);
    out += CHARS[(n >> 18) & 63];
    out += CHARS[(n >> 12) & 63];
    out += CHARS[(n >> 6) & 63];
    out += '=';
  }
  return out;
}

// Percent-encode a path segment. The characters of the SET(...) syntax are
// kept literal, exactly as Loxone's own API Connector examples send them.
std::string encode_path_segment(const std::string &in) {
  std::string out;
  for (char c : in) {
    if (isalnum(static_cast<unsigned char>(c)) || strchr("-._~()&;:,=", c) != nullptr) {
      out += c;
    } else {
      char buf[4];
      snprintf(buf, sizeof(buf), "%%%02X", static_cast<uint8_t>(c));
      out += buf;
    }
  }
  return out;
}

}  // namespace

void LoxoneBridge::set_credentials(const std::string &user, const std::string &password) {
  if (user.empty()) {
    this->auth_header_.clear();
    return;
  }
  this->auth_header_ = "Authorization: Basic " + base64(user + ":" + password) + "\r\n";
}

void LoxoneBridge::reset_push() {
  this->queue_.clear();
  this->last_text_.clear();
  this->last_vc_ = -1;
  this->pushed_once_ = false;
  this->consecutive_failures_ = 0;
  this->miniserver_ok_ = false;
  this->udp_last_text_.clear();
  this->udp_sent_once_ = false;
  this->udp_failures_ = 0;
  struct in_addr addr {};
  if (!this->host_.empty() && inet_aton(this->host_.c_str(), &addr) == 0) {
    ESP_LOGW(TAG, "Miniserver '%s' is not an IPv4 address, push disabled", this->host_.c_str());
    this->host_.clear();
  }
  if (this->push_enabled_()) {
    ESP_LOGI(TAG, "Push to http://%s:%u/dev/sps/io/%s/ (block '%s')", this->host_.c_str(), this->port_,
             this->api_input_.c_str(), this->block_.c_str());
  } else {
    ESP_LOGI(TAG, "API connector push disabled (no miniserver/api_input set)");
  }
  if (this->udp_enabled_())
    ESP_LOGI(TAG, "UDP to %s:%u", this->host_.c_str(), this->udp_port_);
}

void LoxoneBridge::setup() {
  this->base_->init();
  this->base_->add_handler(this);
  LockGuard lock(this->status_lock_);
  this->status_line_ = this->build_status_line_();
}

void LoxoneBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "Loxone Wallbox bridge:");
  ESP_LOGCONFIG(TAG, "  Endpoints: GET /loxone, GET /loxone/set?ca=&tp=&a=");
  if (this->push_enabled_()) {
    ESP_LOGCONFIG(TAG, "  Push: http://%s:%u/dev/sps/io/%s/ -> block '%s'", this->host_.c_str(), this->port_,
                  this->api_input_.c_str(), this->block_.c_str());
    if (!this->vc_input_.empty())
      ESP_LOGCONFIG(TAG, "  Vc via separate virtual input '%s'", this->vc_input_.c_str());
    ESP_LOGCONFIG(TAG, "  Min interval %u s, keepalive %u s, feeds watchdog: %s",
                  (unsigned) (this->min_interval_ms_ / 1000), (unsigned) (this->keepalive_ms_ / 1000),
                  YESNO(this->feed_watchdog_));
  } else {
    ESP_LOGCONFIG(TAG, "  Push: disabled (no miniserver/api_input configured)");
  }
  if (this->udp_enabled_()) {
    ESP_LOGCONFIG(TAG, "  UDP: %s:%u, every change (min %u ms) and every %u s", this->host_.c_str(),
                  this->udp_port_, (unsigned) this->udp_interval_ms_, (unsigned) (UDP_KEEPALIVE_MS / 1000));
  } else {
    ESP_LOGCONFIG(TAG, "  UDP: disabled");
  }
}

std::string LoxoneBridge::build_set_text_() const {
  char buf[200];
  const char *b = this->block_.c_str();
  int vc = this->parent_->is_vehicle_connected() ? 1 : 0;
  int cac = this->parent_->is_charging() ? 1 : 0;
  float cp = this->parent_->power_kw();
  std::string text;
  if (this->vc_input_.empty()) {
    snprintf(buf, sizeof(buf), "SET(%s;Vc;%d)&", b, vc);
    text += buf;
  }
  snprintf(buf, sizeof(buf), "SET(%s;Cac;%d)&SET(%s;Cp;%.2f)", b, cac, b, cp);
  text += buf;
  if (this->parent_->energy_valid()) {
    snprintf(buf, sizeof(buf), "&SET(%s;Mr;%.3f)", b, this->parent_->energy_kwh());
    text += buf;
  }
  return text;
}

std::string LoxoneBridge::build_status_line_() const {
  const auto &m = this->parent_->master();
  const auto &s = m.secondary();
  char buf[240];
  snprintf(buf, sizeof(buf),
           "vc=%d;cac=%d;cp=%.2f;mr=%.3f;a=%.2f;offer=%u;twc=%.2f;set=%u;ca=%d;st=%u;ps=%d;link=%d;failsafe=%d;ms=%d",
           this->parent_->is_vehicle_connected() ? 1 : 0, this->parent_->is_charging() ? 1 : 0,
           this->parent_->power_kw(), this->parent_->energy_valid() ? this->parent_->energy_kwh() : 0.0,
           s.actual_centiamps / 100.0, m.offered_current(), s.reported_max_centiamps / 100.0, m.policy().requested_current(),
           m.policy().enabled() ? 1 : 0, s.vehicle_state, s.plug_state, s.linked ? 1 : 0,
           this->parent_->failsafe_active() ? 1 : 0, this->miniserver_ok_ ? 1 : 0);
  return buf;
}

void LoxoneBridge::loop() {
  uint32_t now = millis();

  if (now - this->last_status_ms_ >= 500) {
    this->last_status_ms_ = now;
    std::string line = this->build_status_line_();
    LockGuard lock(this->status_lock_);
    this->status_line_ = std::move(line);
  }

  if (this->udp_enabled_() && network::is_connected())
    this->send_udp_(now);

  if (this->http_state_ != HttpState::IDLE) {
    this->poll_request_();
    return;
  }
  if (!this->queue_.empty()) {
    this->start_request_();
    return;
  }
  if (!network::is_connected())
    return;
  if (!this->push_enabled_()) {
    // UDP only: prove the Miniserver alive over HTTP now and then, so the
    // setpoint watchdog doesn't trip just because Ca/Tp haven't changed.
    if (this->udp_enabled_() && this->feed_watchdog_ &&
        (!this->pushed_once_ || now - this->last_push_ms_ >= this->keepalive_ms_)) {
      this->queue_get_(ALIVE_PATH, true);
      this->last_push_ms_ = now;
      this->pushed_once_ = true;
    }
    return;
  }

  std::string text = this->build_set_text_();
  int vc = this->parent_->is_vehicle_connected() ? 1 : 0;
  bool changed = text != this->last_text_ || vc != this->last_vc_;
  bool keepalive_due = !this->pushed_once_ || now - this->last_push_ms_ >= this->keepalive_ms_;
  bool vc_changed = vc != this->last_vc_;
  // Plug in/out goes out at once; everything else at most every min_interval.
  bool interval_ok = vc_changed || now - this->last_push_ms_ >= this->min_interval_ms_;
  if (!((changed && interval_ok) || keepalive_due))
    return;

  if (!this->vc_input_.empty())
    this->queue_push_(this->vc_input_, vc ? "1" : "0");
  if (!changed) {
    // A Virtual Text Input only reacts when its text changes; blank it first
    // so the keepalive is evaluated (and the watchdog round trip proven).
    this->queue_push_(this->api_input_, "");
  }
  this->queue_push_(this->api_input_, text);
  this->last_text_ = text;
  this->last_vc_ = vc;
  this->last_push_ms_ = now;
  this->pushed_once_ = true;
}

void LoxoneBridge::queue_push_(const std::string &input, const std::string &text) {
  this->queue_get_("/dev/sps/io/" + encode_path_segment(input) + "/" + encode_path_segment(text));
}

void LoxoneBridge::queue_get_(const std::string &path, bool probe) {
  if (this->queue_.size() >= MAX_QUEUE)
    this->queue_.erase(this->queue_.begin());
  this->queue_.push_back({path, probe});
}

void LoxoneBridge::send_udp_(uint32_t now) {
  // Same line as GET /loxone, so the Virtual UDP Input's commands
  // ("vc=\v", "cp=\v", ...) work for both.
  std::string text = this->build_status_line_();
  bool changed = text != this->udp_last_text_;
  bool due = !this->udp_sent_once_ || now - this->udp_last_ms_ >= UDP_KEEPALIVE_MS ||
             (changed && now - this->udp_last_ms_ >= this->udp_interval_ms_);
  if (!due)
    return;

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(this->udp_port_);
  if (inet_aton(this->host_.c_str(), &addr.sin_addr) == 0)
    return;
  if (this->udp_fd_ < 0) {
    this->udp_fd_ = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (this->udp_fd_ < 0) {
      ESP_LOGW(TAG, "UDP socket() failed");
      return;
    }
  }
  // Marks the attempt either way, so a failing send is retried at the
  // normal cadence instead of in every loop.
  this->udp_last_text_ = text;
  this->udp_last_ms_ = now;
  this->udp_sent_once_ = true;
  ssize_t n = lwip_sendto(this->udp_fd_, text.data(), text.size(), MSG_DONTWAIT,
                          reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
  if (n == static_cast<ssize_t>(text.size())) {
    if (this->udp_failures_ > 0)
      ESP_LOGI(TAG, "UDP to %s:%u works again", this->host_.c_str(), this->udp_port_);
    this->udp_failures_ = 0;
    ESP_LOGV(TAG, "UDP %s", text.c_str());
    return;
  }
  this->udp_failures_++;
  if (this->udp_failures_ == 1 || this->udp_failures_ % 10 == 0)
    ESP_LOGW(TAG, "UDP to %s:%u failed (%u in a row), errno %d", this->host_.c_str(), this->udp_port_,
             (unsigned) this->udp_failures_, errno);
}

void LoxoneBridge::start_request_() {
  Pending p = std::move(this->queue_.front());
  this->queue_.erase(this->queue_.begin());
  this->request_is_probe_ = p.probe;

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(this->port_);
  if (inet_aton(this->host_.c_str(), &addr.sin_addr) == 0) {
    this->finish_request_(false, "miniserver must be an IPv4 address");
    return;
  }

  this->fd_ = lwip_socket(AF_INET, SOCK_STREAM, 0);
  if (this->fd_ < 0) {
    this->finish_request_(false, "socket() failed");
    return;
  }
  int flags = lwip_fcntl(this->fd_, F_GETFL, 0);
  lwip_fcntl(this->fd_, F_SETFL, flags | O_NONBLOCK);

  this->request_ = "GET " + p.path + " HTTP/1.0\r\nHost: " + this->host_ + "\r\n" +
                   (p.probe ? std::string() : this->auth_header_) + "Connection: close\r\n\r\n";
  this->request_sent_ = 0;
  this->response_.clear();
  this->request_started_ms_ = millis();

  int res = lwip_connect(this->fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
  if (res < 0 && errno != EINPROGRESS) {
    this->finish_request_(false, "connect() failed");
    return;
  }
  this->http_state_ = HttpState::CONNECTING;
}

void LoxoneBridge::poll_request_() {
  if (millis() - this->request_started_ms_ > HTTP_TIMEOUT_MS) {
    this->finish_request_(false, "timeout");
    return;
  }

  if (this->http_state_ == HttpState::CONNECTING) {
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(this->fd_, &wfds);
    struct timeval tv {0, 0};
    if (lwip_select(this->fd_ + 1, nullptr, &wfds, nullptr, &tv) <= 0)
      return;  // not connected yet
    int err = 0;
    socklen_t len = sizeof(err);
    lwip_getsockopt(this->fd_, SOL_SOCKET, SO_ERROR, &err, &len);
    if (err != 0) {
      this->finish_request_(false, "connection refused");
      return;
    }
    this->http_state_ = HttpState::SENDING;
  }

  if (this->http_state_ == HttpState::SENDING) {
    ssize_t n = lwip_send(this->fd_, this->request_.data() + this->request_sent_,
                          this->request_.size() - this->request_sent_, 0);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        return;
      this->finish_request_(false, "send() failed");
      return;
    }
    this->request_sent_ += n;
    if (this->request_sent_ < this->request_.size())
      return;
    this->http_state_ = HttpState::READING;
  }

  if (this->http_state_ == HttpState::READING) {
    char buf[64];
    ssize_t n = lwip_recv(this->fd_, buf, sizeof(buf), 0);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        return;
      this->finish_request_(false, "recv() failed");
      return;
    }
    if (n > 0 && this->response_.size() < 64)
      this->response_.append(buf, n);
    // The status line is all we need: "HTTP/1.x 200 ...". Loxone answers
    // 401 for bad credentials and 200 with an error code in the body for
    // an unknown input name - only the status line is checked here.
    size_t eol = this->response_.find("\r\n");
    if (eol == std::string::npos && n > 0)
      return;
    if (eol == std::string::npos) {
      this->finish_request_(false, "connection closed without a response");
      return;
    }
    int code = 0;
    if (sscanf(this->response_.c_str(), "HTTP/%*s %d", &code) != 1)
      code = 0;
    if (code == 200 || (this->request_is_probe_ && code > 0)) {
      this->finish_request_(true, nullptr);
    } else if (code == 401) {
      this->finish_request_(false, "401 - check username/password and the user's rights on the input");
    } else {
      char why[40];
      snprintf(why, sizeof(why), "HTTP status %d", code);
      this->finish_request_(false, why);
    }
  }
}

void LoxoneBridge::finish_request_(bool ok, const char *why) {
  if (this->fd_ >= 0) {
    lwip_close(this->fd_);
    this->fd_ = -1;
  }
  this->http_state_ = HttpState::IDLE;
  if (ok) {
    if (!this->miniserver_ok_ || this->consecutive_failures_ > 0)
      ESP_LOGI(TAG, "Miniserver reachable");
    this->miniserver_ok_ = true;
    this->consecutive_failures_ = 0;
    // A successful round trip proves the Miniserver is alive; with the
    // setpoint watchdog enabled, losing it ends in the failsafe current.
    if (this->feed_watchdog_)
      this->parent_->feed_watchdog();
    return;
  }
  this->consecutive_failures_++;
  this->miniserver_ok_ = false;
  // Log the first failure and then every 10th, not one line per push.
  if (this->consecutive_failures_ == 1 || this->consecutive_failures_ % 10 == 0)
    ESP_LOGW(TAG, "Push to Miniserver failed (%u in a row): %s", (unsigned) this->consecutive_failures_, why);
  // Drop what's queued; the next loop rebuilds an up-to-date text anyway.
  this->queue_.clear();
  this->last_text_.clear();
}

bool LoxoneBridge::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET)
    return false;
#ifdef USE_ESP32
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  StringRef url = request->url_to(url_buf);
  return url == "/loxone" || url == "/loxone/set";
#else
  return request->url() == "/loxone" || request->url() == "/loxone/set";
#endif
}

void LoxoneBridge::handleRequest(AsyncWebServerRequest *request) {
  // Runs in the web server's task: parse here, apply in the main loop.
  bool has_ca = request->hasParam("ca");
  bool has_tp = request->hasParam("tp");
  bool has_a = request->hasParam("a");
  if (has_ca || has_tp || has_a) {
    int ca = has_ca ? atoi(request->getParam("ca")->value().c_str()) : -1;
    float tp = -1.0f;
    if (has_tp) {
      // Accept a decimal comma too ("7,36"), in case <v> is formatted that way.
      std::string v = request->getParam("tp")->value();
      std::replace(v.begin(), v.end(), ',', '.');
      tp = strtof(v.c_str(), nullptr);
    }
    int a = has_a ? atoi(request->getParam("a")->value().c_str()) : -1;
    TWCGateway *parent = this->parent_;
    this->defer([parent, ca, tp, a]() {
      ESP_LOGI(TAG, "/loxone/set: ca=%d tp=%.2f a=%d (-1 = not given)", ca, tp, a);
      if (ca >= 0)
        parent->set_charging_enabled(ca != 0);
      if (tp >= 0)
        parent->set_target_power_kw(tp);
      if (a >= 0)
        parent->set_current_amps(a);
      parent->feed_watchdog();
    });
  }
  std::string line;
  {
    LockGuard lock(this->status_lock_);
    line = this->status_line_;
  }
  request->send(200, "text/plain", line.c_str());
}

}  // namespace twc_gateway
}  // namespace esphome

#endif  // USE_TWC_LOXONE
