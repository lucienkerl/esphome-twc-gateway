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

// Speaks the Loxone Wallbox function block's language directly:
//
//   ESP -> Miniserver  pushes Vc / Cac / Cp / Mr as ONE text,
//                      "SET(Wb;Vc;1)&SET(Wb;Cac;1)&SET(Wb;Cp;7.36)&SET(Wb;Mr;1234.567)",
//                      to a Virtual Text Input wired to the block's API connector,
//                      and/or sends the status line below as one UDP datagram
//                      to a Virtual UDP Input whose commands feed the block's
//                      inputs one by one.
//   Miniserver -> ESP  GET /loxone/set?ca=<0|1>&tp=<kW>  (block outputs Ca/Tp)
//   anyone -> ESP      GET /loxone -> "vc=1;cac=1;cp=7.36;mr=1234.567;..."
//                      for a single Virtual HTTP Input as a pull fallback.
//   anyone -> ESP      GET /loxone/udp.xml -> Loxone Config template for the
//                      Virtual UDP Input, with this device's IP and port.
//
// The push uses a small non-blocking HTTP/1.0 client so the RS-485 timing in
// the main loop never waits on the network.

#pragma once

#include "esphome/core/defines.h"

#ifdef USE_TWC_LOXONE

#include <string>

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/web_server_base/web_server_base.h"

#include "twc_gateway.h"

namespace esphome {
namespace twc_gateway {

/// Loxone Config import file for a Virtual UDP Input matching the status
/// line (GET /loxone/udp.xml serves it with this device's IP and port).
std::string build_udp_template(const std::string &sender_ip, uint16_t port);

class LoxoneBridge : public Component, public AsyncWebHandler {
 public:
  LoxoneBridge(TWCGateway *parent, web_server_base::WebServerBase *base) : parent_(parent), base_(base) {}

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::WIFI - 1.0f; }

  void set_miniserver(const std::string &host, uint16_t port) {
    this->host_ = host;
    this->port_ = port;
  }
  void set_credentials(const std::string &user, const std::string &password);
  void set_api_input(const std::string &name) { this->api_input_ = name; }
  void set_block(const std::string &name) { this->block_ = name; }
  void set_vc_input(const std::string &name) { this->vc_input_ = name; }
  void set_min_interval(uint32_t ms) { this->min_interval_ms_ = ms; }
  void set_keepalive(uint32_t ms) { this->keepalive_ms_ = ms; }
  void set_feed_watchdog(bool v) { this->feed_watchdog_ = v; }
  /// UDP port of a Virtual UDP Input on the Miniserver; 0 = no UDP.
  void set_udp_port(uint16_t port) { this->udp_port_ = port; }
  void set_udp_interval(uint32_t ms) { this->udp_interval_ms_ = ms; }
  // Call after changing the settings above at runtime (e.g. from text
  // entities in the web UI): drops queued pushes and pushes again at once.
  void reset_push();

  // AsyncWebHandler
  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;

 protected:
  enum class HttpState : uint8_t { IDLE, CONNECTING, SENDING, READING };

  bool push_enabled_() const { return !this->host_.empty() && !this->api_input_.empty(); }
  bool udp_enabled_() const { return !this->host_.empty() && this->udp_port_ != 0; }
  void send_udp_(uint32_t now);
  void queue_get_(const std::string &path, bool probe = false);
  void send_udp_template_(AsyncWebServerRequest *request);
  std::string build_set_text_() const;
  std::string build_status_line_() const;
  void queue_push_(const std::string &input, const std::string &text);
  void start_request_();
  void poll_request_();
  void finish_request_(bool ok, const char *why);

  TWCGateway *parent_;
  web_server_base::WebServerBase *base_;

  std::string host_;
  uint16_t port_{80};
  std::string auth_header_;
  std::string api_input_;
  std::string block_{"Wb"};
  std::string vc_input_;
  uint32_t min_interval_ms_{5000};
  uint32_t keepalive_ms_{30000};
  bool feed_watchdog_{true};

  // Push bookkeeping
  std::string last_text_;
  int last_vc_{-1};
  uint32_t last_push_ms_{0};
  bool pushed_once_{false};
  struct Pending {
    std::string path;
    bool probe;  // liveness check: no credentials, any HTTP status counts
  };
  std::vector<Pending> queue_;

  // Non-blocking HTTP client
  HttpState http_state_{HttpState::IDLE};
  int fd_{-1};
  uint32_t request_started_ms_{0};
  std::string request_;
  size_t request_sent_{0};
  std::string response_;
  bool request_is_probe_{false};
  uint32_t consecutive_failures_{0};
  bool miniserver_ok_{false};

  // UDP push
  uint16_t udp_port_{0};
  uint32_t udp_interval_ms_{1000};
  int udp_fd_{-1};
  std::string udp_last_text_;
  uint32_t udp_last_ms_{0};
  bool udp_sent_once_{false};
  uint32_t udp_failures_{0};

  // Status line for GET /loxone, built in the main loop and handed to the
  // web server task under a lock.
  Mutex status_lock_;
  std::string status_line_;
  uint32_t last_status_ms_{0};
};

}  // namespace twc_gateway
}  // namespace esphome

#endif  // USE_TWC_LOXONE
