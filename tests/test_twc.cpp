// Host-side unit tests for the protocol, policy and fake-primary state
// machine. These files have no ESPHome dependency on purpose.
//
//   make -C tests
//
// Byte sequences for secondary frames follow TWCManager's documented
// examples (lib/TWCManager/TWCSlave.py).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "../components/twc-gateway/twc_frames.h"
#include "../components/twc-gateway/twc_master.h"
#include "../components/twc-gateway/twc_policy.h"

using namespace esphome::twc_gateway;
using Bytes = std::vector<uint8_t>;

static int failures = 0;
static int checks = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    checks++;                                                          \
    if (!(cond)) {                                                     \
      failures++;                                                      \
      printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
    }                                                                  \
  } while (0)
#define CHECK_EQ(a, b)                                                                              \
  do {                                                                                              \
    checks++;                                                                                       \
    auto _a = (a);                                                                                  \
    auto _b = (b);                                                                                  \
    if (!(_a == _b)) {                                                                              \
      failures++;                                                                                   \
      printf("  FAIL %s:%d: %s == %s (%lld vs %lld)\n", __FILE__, __LINE__, #a, #b, (long long) _a, \
             (long long) _b);                                                                       \
    }                                                                                               \
  } while (0)
#define CHECK_NEAR(a, b, eps) CHECK(std::fabs((double) (a) - (double) (b)) <= (eps))

// Payload + checksum, as the decoder hands frames to decode().
static Bytes with_checksum(Bytes payload) {
  payload.push_back(proto::checksum(payload.data(), payload.size()));
  return payload;
}

// Run a full wire frame through a decoder; returns the decoded frame (with
// checksum) or empty.
static Bytes slip_roundtrip(const Bytes &wire) {
  proto::SlipDecoder dec;
  for (uint8_t b : wire) {
    if (dec.feed(b))
      return Bytes(dec.frame(), dec.frame() + dec.frame_length());
  }
  return {};
}

static Bytes secondary_linkready(uint16_t id, uint16_t max_centiamps) {
  return with_checksum({0xFD, 0xE2, uint8_t(id >> 8), uint8_t(id), 0x77, uint8_t(max_centiamps >> 8),
                        uint8_t(max_centiamps), 0, 0, 0, 0, 0, 0, 0, 0});
}

static Bytes secondary_heartbeat(uint16_t id, uint16_t dst, uint8_t state, uint16_t max_c, uint16_t actual_c,
                                 bool v2 = true) {
  Bytes p{0xFD, 0xE0, uint8_t(id >> 8), uint8_t(id), uint8_t(dst >> 8), uint8_t(dst), state,
          uint8_t(max_c >> 8), uint8_t(max_c), uint8_t(actual_c >> 8), uint8_t(actual_c), 0, 0};
  if (v2) {
    p.push_back(0);
    p.push_back(0);
  }
  return with_checksum(p);
}

static Bytes power_status(uint16_t id, uint32_t kwh, uint16_t v, uint8_t half_amps_l1, uint8_t half_amps_l2,
                          uint8_t half_amps_l3) {
  return with_checksum({0xFD, 0xEB, uint8_t(id >> 8), uint8_t(id), uint8_t(kwh >> 24), uint8_t(kwh >> 16),
                        uint8_t(kwh >> 8), uint8_t(kwh), uint8_t(v >> 8), uint8_t(v), uint8_t(v >> 8), uint8_t(v),
                        uint8_t(v >> 8), uint8_t(v), half_amps_l1, half_amps_l2, half_amps_l3, 0, 0});
}

// --- proto -------------------------------------------------------------------

static void test_checksum_and_framing() {
  printf("framing\n");
  Bytes payload{0xFB, 0xE0, 0xAB, 0x32, 0x12, 0x34};
  // Checksum skips the first byte.
  CHECK_EQ(proto::checksum(payload.data(), payload.size()), uint8_t(0xE0 + 0xAB + 0x32 + 0x12 + 0x34));

  auto wire = proto::encode_frame(payload);
  CHECK_EQ(wire.front(), proto::SLIP_END);
  CHECK_EQ(wire[wire.size() - 2], proto::SLIP_END);
  CHECK_EQ(wire.back(), uint8_t(0xFF));

  // Bytes that collide with SLIP control characters get escaped and come
  // back unchanged.
  Bytes nasty{0xFB, 0xE0, 0xC0, 0xDB, 0x00, 0xC0};
  auto nasty_wire = proto::encode_frame(nasty);
  CHECK_EQ(nasty_wire.size(), nasty.size() + 1 /*chk*/ + 3 /*escapes*/ + 3 /*C0 C0 FF*/);
  auto back = slip_roundtrip(nasty_wire);
  CHECK_EQ(back.size(), nasty.size() + 1);
  CHECK(Bytes(back.begin(), back.end() - 1) == nasty);

  // The identity-rewriting commands never leave the encoder.
  CHECK(proto::encode_frame({0xFC, 0x19, 0, 0}).empty());
  CHECK(proto::encode_frame({0xFC, 0x1A, 0, 0}).empty());
}

static void test_slip_decoder_robustness() {
  printf("slip decoder\n");
  auto frame = secondary_linkready(0x1234, 3200);
  Bytes wire = proto::encode_frame(Bytes(frame.begin(), frame.end() - 1));

  // Noise and a stray END before the real frame (started listening mid-frame).
  Bytes noisy{0x55, 0x66, proto::SLIP_END, 0x01, proto::SLIP_END};
  noisy.insert(noisy.end(), wire.begin(), wire.end());
  CHECK(slip_roundtrip(noisy) == frame);

  // Bad checksum is dropped and counted.
  Bytes bad = wire;
  bad[3] ^= 0x01;
  proto::SlipDecoder dec;
  bool got = false;
  for (uint8_t b : bad)
    got |= dec.feed(b);
  CHECK(!got);
  CHECK_EQ(dec.checksum_errors(), 1u);

  // Invalid escape drops the frame instead of corrupting the next one.
  Bytes esc{proto::SLIP_END, 0xFD, proto::SLIP_ESC, 0x42, 0x01, proto::SLIP_END};
  esc.insert(esc.end(), wire.begin(), wire.end());
  CHECK(slip_roundtrip(esc) == frame);

  // Overlong frames are discarded.
  Bytes longf{proto::SLIP_END};
  longf.insert(longf.end(), 40, 0x11);
  longf.push_back(proto::SLIP_END);
  proto::SlipDecoder dec2;
  got = false;
  for (uint8_t b : longf)
    got |= dec2.feed(b);
  CHECK(!got);
  CHECK_EQ(dec2.overruns(), 1u);
}

static void test_encoder_lengths() {
  printf("encoder lengths\n");
  // Lengths without checksum, matching TWCManager.
  CHECK_EQ(proto::encode_linkready1(0xAB32, 0x77).size(), size_t(13));
  CHECK_EQ(proto::encode_linkready2(0xAB32, 0x77).size(), size_t(13));
  CHECK_EQ(proto::encode_heartbeat(0xAB32, 0x1234, 0x09, 1600, 2).size(), size_t(15));
  CHECK_EQ(proto::encode_heartbeat(0xAB32, 0x1234, 0x05, 1600, 1).size(), size_t(13));
  CHECK_EQ(proto::encode_query(proto::GET_PWR_STATE, 0xAB32, 0x1234).size(), size_t(14));
  CHECK_EQ(proto::encode_start_stop(true, 0xAB32, 0x1234).size(), size_t(15));

  auto hb = proto::encode_heartbeat(0xAB32, 0x1234, 0x09, 1600, 2);
  CHECK_EQ(hb[0], 0xFB);
  CHECK_EQ(hb[1], 0xE0);
  CHECK_EQ(hb[2], 0xAB);  // IDs go out big-endian
  CHECK_EQ(hb[3], 0x32);
  CHECK_EQ(hb[4], 0x12);
  CHECK_EQ(hb[5], 0x34);
  CHECK_EQ(hb[6], 0x09);
  CHECK_EQ(hb[7], 0x06);  // 1600 = 0x0640
  CHECK_EQ(hb[8], 0x40);

  auto stop = proto::encode_start_stop(false, 0xAB32, 0x1234);
  CHECK_EQ(stop[0], 0xFC);
  CHECK_EQ(stop[1], 0xB2);
}

static void test_decode() {
  printf("decode\n");
  auto lr = secondary_linkready(0x1234, 3200);
  auto m = proto::decode(lr.data(), lr.size());
  CHECK(m.type == proto::MessageType::SECONDARY_LINKREADY);
  CHECK_EQ(m.src, 0x1234);
  CHECK_EQ(m.sign, 0x77);
  CHECK_EQ(m.max_allowable_centiamps, 3200);

  auto hb2 = secondary_heartbeat(0x1234, 0xAB32, 0x01, 1600, 1532, true);
  m = proto::decode(hb2.data(), hb2.size());
  CHECK(m.type == proto::MessageType::SECONDARY_HEARTBEAT);
  CHECK_EQ(m.protocol, 2);
  CHECK_EQ(m.dst, 0xAB32);
  CHECK_EQ(m.state, 1);
  CHECK_EQ(m.reported_max_centiamps, 1600);
  CHECK_EQ(m.actual_centiamps, 1532);

  auto hb1 = secondary_heartbeat(0x1234, 0xAB32, 0x04, 0, 25, false);
  m = proto::decode(hb1.data(), hb1.size());
  CHECK_EQ(m.protocol, 1);

  // Truncated heartbeat: known command, but not enough bytes.
  Bytes shortf = with_checksum({0xFD, 0xE0, 0x12, 0x34, 0xAB});
  m = proto::decode(shortf.data(), shortf.size());
  CHECK(m.type == proto::MessageType::INVALID);

  auto ps = power_status(0x1234, 4321, 231, 32, 31, 0);
  m = proto::decode(ps.data(), ps.size());
  CHECK(m.type == proto::MessageType::POWER_STATUS);
  CHECK_EQ(m.total_kwh, 4321u);
  CHECK_EQ(m.voltage[0], 231);
  CHECK_EQ(m.half_amps[0], 32);
  CHECK_EQ(m.half_amps[2], 0);

  auto plug = with_checksum({0xFD, 0xB4, 0x12, 0x34, 0x03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  m = proto::decode(plug.data(), plug.size());
  CHECK(m.type == proto::MessageType::PLUG_STATE);
  CHECK_EQ(m.plug_state, 3);

  auto fw = with_checksum({0xFD, 0xEC, 0x12, 0x34, 4, 5, 3, 0x7A, 0, 0, 0, 0, 0, 0, 0});
  m = proto::decode(fw.data(), fw.size());
  CHECK(m.type == proto::MessageType::FIRMWARE);
  CHECK_EQ(m.firmware[3], 0x7A);

  std::string out;
  CHECK(proto::sanitize_ascii(std::string("AB12\0\0", 6), out) && out == "AB12");
  CHECK(!proto::sanitize_ascii(std::string("AB\x01", 3), out));
}

// --- policy ------------------------------------------------------------------

static void test_policy_commands() {
  printf("policy commands\n");
  ChargePolicy p;
  PolicyConfig c;
  c.max_current = 16;
  p.configure(c);

  SecondaryReport r;
  r.protocol = 2;

  // 0 A always as 0x05 - never 0x09 with 0 A (relay chatter).
  auto hb = p.heartbeat_command(0, r);
  CHECK_EQ(hb.command, proto::HB_SET_LIMIT_V1);
  CHECK_EQ(hb.centiamps, 0);

  // Not yet confirmed by the secondary -> 0x09 with the limit.
  r.reported_max_centiamps = 0;
  hb = p.heartbeat_command(10, r);
  CHECK_EQ(hb.command, proto::HB_SET_LIMIT_V2);
  CHECK_EQ(hb.centiamps, 1000);

  // Confirmed -> no change.
  r.reported_max_centiamps = 1000;
  r.state = proto::ST_CHARGING;
  hb = p.heartbeat_command(10, r);
  CHECK_EQ(hb.command, proto::HB_NO_CHANGE);

  // State 08 gets the limit resent even when confirmed.
  r.state = proto::ST_STARTING;
  CHECK_EQ(p.heartbeat_command(10, r).command, proto::HB_SET_LIMIT_V2);

  // Protocol 1 uses 0x05.
  r.state = proto::ST_PLUGGED_READY;
  r.protocol = 1;
  r.reported_max_centiamps = 0;
  CHECK_EQ(p.heartbeat_command(10, r).command, proto::HB_SET_LIMIT_V1);

  // Legacy behaviour on request.
  c.reassert_every_heartbeat = true;
  p.configure(c);
  r.protocol = 2;
  r.reported_max_centiamps = 1000;
  CHECK_EQ(p.heartbeat_command(10, r).command, proto::HB_SET_LIMIT_V2);
}

static void test_policy_clamp_and_enable() {
  printf("policy clamp/enable\n");
  ChargePolicy p;
  PolicyConfig c;
  c.min_current = 6;
  c.max_current = 16;
  p.configure(c);
  CHECK_EQ(p.clamp(0), 0);
  CHECK_EQ(p.clamp(-3), 0);
  CHECK_EQ(p.clamp(3), 6);
  CHECK_EQ(p.clamp(12), 12);
  CHECK_EQ(p.clamp(40), 16);

  SecondaryReport r;
  p.set_requested_current(10, 0);
  CHECK_EQ(p.effective_current(100, r), 10);
  p.set_enabled(false, 200);
  CHECK_EQ(p.effective_current(300, r), 0);
  p.set_enabled(true, 400);
  CHECK_EQ(p.effective_current(500, r), 10);

  // Lowering max_current at runtime caps an already requested current.
  c.max_current = 8;
  p.configure(c);
  CHECK_EQ(p.effective_current(600, r), 8);
  c.max_current = 16;
  p.configure(c);
  CHECK_EQ(p.effective_current(700, r), 10);
}

static void test_policy_watchdog() {
  printf("policy watchdog\n");
  ChargePolicy p;
  PolicyConfig c;
  c.max_current = 16;
  c.setpoint_timeout_ms = 120000;
  c.failsafe_current = 6;
  p.configure(c);
  SecondaryReport r;

  p.set_requested_current(16, 1000);
  CHECK_EQ(p.effective_current(60000, r), 16);
  CHECK(!p.watchdog_expired(121000));
  CHECK(p.watchdog_expired(121001));
  CHECK_EQ(p.effective_current(121001, r), 6);
  p.feed_watchdog(130000);
  CHECK_EQ(p.effective_current(130001, r), 16);

  // Never fed at all: counts from boot.
  ChargePolicy q;
  q.configure(c);
  CHECK(!q.watchdog_expired(100000));
  CHECK(q.watchdog_expired(120001));

  // Disabled watchdog never expires.
  c.setpoint_timeout_ms = 0;
  q.configure(c);
  CHECK(!q.watchdog_expired(0xFFFFFFF0u));
}

static void test_policy_min_on_off() {
  printf("policy min on/off\n");
  ChargePolicy p;
  PolicyConfig c;
  c.max_current = 16;
  c.min_on_time_ms = 60000;
  c.min_off_time_ms = 30000;
  p.configure(c);
  SecondaryReport r;

  p.set_requested_current(10, 0);
  CHECK_EQ(p.effective_current(1000, r), 10);  // offer starts at t=1 s
  r.actual_centiamps = 950;                     // car draws
  p.set_requested_current(0, 20000);
  CHECK_EQ(p.effective_current(20000, r), 6);  // held at minimum
  CHECK_EQ(p.effective_current(61001, r), 0);  // min on time over
  r.actual_centiamps = 0;
  p.set_requested_current(10, 70000);
  CHECK_EQ(p.effective_current(70000, r), 0);   // min off time
  CHECK_EQ(p.effective_current(91001, r), 10);  // over

  // Car not drawing: stopping is immediate.
  ChargePolicy q;
  q.configure(c);
  SecondaryReport idle;
  q.set_requested_current(10, 0);
  CHECK_EQ(q.effective_current(0, idle), 10);
  q.set_requested_current(0, 1000);
  CHECK_EQ(q.effective_current(1000, idle), 0);
}

// --- master ------------------------------------------------------------------

struct Harness {
  TwcMaster master;
  std::vector<Bytes> sent;  // decoded payloads (no SLIP, with checksum)
  std::vector<MasterEvent> events;
  uint32_t now{0};

  explicit Harness(MasterConfig mc = MasterConfig{}, PolicyConfig pc = PolicyConfig{}) {
    mc.twc_id = 0xAB32;
    master.configure(mc);
    pc.max_current = pc.max_current == 32 ? 16 : pc.max_current;
    master.policy().configure(pc);
    master.set_send_callback([this](const Bytes &wire) { sent.push_back(slip_roundtrip(wire)); });
    master.set_event_callback([this](MasterEvent e, const SecondaryInfo &, uint16_t) { events.push_back(e); });
  }
  void run_until(uint32_t t, uint32_t step = 50) {
    while (now < t) {
      now += step;
      master.tick(now);
    }
  }
  void rx(const Bytes &frame) { master.on_frame(frame.data(), frame.size(), now); }
  size_t count(uint16_t command) const {
    size_t n = 0;
    for (auto &f : sent)
      if (f.size() >= 2 && ((f[0] << 8) | f[1]) == command)
        n++;
    return n;
  }
  bool has_event(MasterEvent e) const {
    for (auto x : events)
      if (x == e)
        return true;
    return false;
  }
};

static void test_master_startup_and_link() {
  printf("master startup/link\n");
  Harness h;
  h.run_until(10500);
  CHECK_EQ(h.count(proto::PRIMARY_LINKREADY1), size_t(5));
  CHECK_EQ(h.count(proto::PRIMARY_LINKREADY2), size_t(5));
  CHECK_EQ(h.count(proto::PRIMARY_HEARTBEAT), size_t(0));  // nobody to talk to

  h.rx(secondary_linkready(0x1234, 3200));
  CHECK(h.has_event(MasterEvent::LINKED));
  CHECK(h.master.secondary().linked);
  h.sent.clear();
  h.run_until(10500 + 10000);
  // ~1 heartbeat per second, plus one aux frame between heartbeats.
  size_t hbs = h.count(proto::PRIMARY_HEARTBEAT);
  CHECK(hbs >= 10 && hbs <= 11);
  CHECK(h.count(proto::GET_PLUG_STATE) >= 3);
  CHECK(h.count(proto::GET_PWR_STATE) >= 2);
  CHECK(h.count(proto::GET_SERIAL_NUMBER) >= 1);
  CHECK(h.sent.size() <= 2 * hbs + 1);
  // Heartbeat goes to the secondary's ID with protocol 2 length.
  for (auto &f : h.sent) {
    if (((f[0] << 8) | f[1]) == proto::PRIMARY_HEARTBEAT) {
      CHECK_EQ(f.size(), size_t(16));
      CHECK_EQ(f[4], 0x12);
      CHECK_EQ(f[5], 0x34);
      break;
    }
  }
}

static void test_master_timeout_and_values() {
  printf("master timeout/values\n");
  PolicyConfig pc;
  pc.max_current = 16;
  Harness h(MasterConfig{}, pc);
  h.run_until(11000);
  h.rx(secondary_linkready(0x1234, 3200));
  h.master.policy().set_requested_current(10, h.now);
  h.run_until(12000);

  h.rx(secondary_heartbeat(0x1234, 0xAB32, 0x01, 1000, 1532));
  h.rx(power_status(0x1234, 4321, 230, 31, 31, 31));
  CHECK(h.master.charging());
  CHECK_EQ(h.master.active_phases(), 3);
  CHECK_NEAR(h.master.power_w(), 15.32 * 690, 0.5);
  CHECK(h.master.energy().valid());
  CHECK_NEAR(h.master.energy().kwh(), 4321.0, 1e-9);

  auto plug = with_checksum({0xFD, 0xB4, 0x12, 0x34, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  h.rx(plug);
  CHECK(h.master.vehicle_connected());

  // State 9 echo doesn't overwrite the real vehicle state.
  h.rx(secondary_heartbeat(0x1234, 0xAB32, 0x09, 1000, 1532));
  CHECK_EQ(h.master.secondary().vehicle_state, 1);

  // Silence for > 26 s drops the link and all values.
  h.run_until(h.now + 27000);
  CHECK(h.has_event(MasterEvent::LOST));
  CHECK(!h.master.secondary().linked);
  CHECK(!h.master.charging());
  CHECK(!h.master.vehicle_connected());
  CHECK_EQ(h.master.power_w(), 0.0);
}

static void test_master_diagnostics() {
  printf("master diagnostics\n");
  PolicyConfig pc;
  pc.max_current = 16;
  Harness h(MasterConfig{}, pc);
  h.run_until(11000);
  h.rx(secondary_linkready(0x1234, 3200));
  h.master.policy().set_requested_current(10, h.now);
  h.run_until(12000);
  // The last heartbeat sent is visible, so its limit can be logged next to
  // the one the TWC reports back.
  CHECK_EQ(h.master.last_heartbeat().command, proto::HB_SET_LIMIT_V2);
  CHECK_EQ(h.master.last_heartbeat().centiamps, 1000);

  // A frame we don't decode, from our secondary, is reported - not dropped.
  CHECK(!h.has_event(MasterEvent::UNKNOWN_FRAME));
  h.rx(with_checksum({0xFD, 0xA0, 0x12, 0x34, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
  CHECK(h.has_event(MasterEvent::UNKNOWN_FRAME));
}

static void test_master_heartbeat_values() {
  printf("master heartbeat values\n");
  PolicyConfig pc;
  pc.max_current = 16;
  Harness h(MasterConfig{}, pc);
  h.run_until(11000);
  h.rx(secondary_linkready(0x1234, 3200));
  h.master.policy().set_requested_current(12, h.now);
  h.sent.clear();
  h.run_until(h.now + 1000);
  Bytes *hb = nullptr;
  for (auto &f : h.sent)
    if (((f[0] << 8) | f[1]) == proto::PRIMARY_HEARTBEAT)
      hb = &f;
  CHECK(hb != nullptr);
  if (hb) {
    CHECK_EQ((*hb)[6], proto::HB_SET_LIMIT_V2);
    CHECK_EQ(((*hb)[7] << 8) | (*hb)[8], 1200);
  }

  // Secondary confirms -> 0x00 from now on.
  h.rx(secondary_heartbeat(0x1234, 0xAB32, 0x01, 1200, 1150));
  h.sent.clear();
  h.run_until(h.now + 1000);
  for (auto &f : h.sent)
    if (((f[0] << 8) | f[1]) == proto::PRIMARY_HEARTBEAT)
      CHECK_EQ(f[6], proto::HB_NO_CHANGE);

  // A 16 A request on a TWC that reports a 10 A hardware max is capped.
  Harness h2(MasterConfig{}, pc);
  h2.run_until(11000);
  h2.rx(secondary_linkready(0x1234, 1000));
  h2.master.policy().set_requested_current(16, h2.now);
  h2.run_until(h2.now + 1000);
  CHECK_EQ(h2.master.offered_current(), 10);
}

static void test_master_passive() {
  printf("master passive\n");
  MasterConfig mc;
  mc.passive = true;
  Harness h(mc);
  h.run_until(20000);
  CHECK(h.sent.empty());
  h.rx(secondary_heartbeat(0x1234, 0x7777, 0x01, 1600, 1500));
  CHECK(h.master.secondary().linked);
  h.run_until(25000);
  CHECK(h.sent.empty());
}

static void test_master_start_stop() {
  printf("master start/stop\n");
  MasterConfig mc;
  mc.use_start_stop_commands = true;
  mc.query_vin = false;
  PolicyConfig pc;
  pc.max_current = 16;
  Harness h(mc, pc);
  h.run_until(11000);
  h.rx(secondary_linkready(0x1234, 3200));
  h.master.policy().set_requested_current(10, h.now);
  h.run_until(h.now + 2000);
  CHECK_EQ(h.count(proto::START_CHARGING), size_t(1));
  CHECK_EQ(h.count(proto::STOP_CHARGING), size_t(0));
  for (auto &f : h.sent)
    if (((f[0] << 8) | f[1]) == proto::START_CHARGING)
      CHECK_EQ(f.size(), size_t(16));  // 15 + checksum

  h.master.policy().set_enabled(false, h.now);
  h.run_until(h.now + 2000);
  CHECK_EQ(h.count(proto::STOP_CHARGING), size_t(1));
  h.run_until(h.now + 5000);
  CHECK_EQ(h.count(proto::STOP_CHARGING), size_t(1));  // only on change

  h.master.rearm_start_stop();
  h.run_until(h.now + 2000);
  CHECK_EQ(h.count(proto::STOP_CHARGING), size_t(2));
}

static void test_amps_for_power() {
  printf("kW -> A\n");
  PolicyConfig pc;
  pc.max_current = 16;
  Harness h(MasterConfig{}, pc);
  h.run_until(11000);
  h.rx(secondary_linkready(0x1234, 3200));
  // Loxone defaults: min 4.16 kW, max 11 kW on 3 phases.
  CHECK_EQ(h.master.amps_for_power(4.16), 6);
  CHECK_EQ(h.master.amps_for_power(11.0), 16);
  CHECK_EQ(h.master.amps_for_power(22.0), 16);  // capped at max_current
  CHECK_EQ(h.master.amps_for_power(3.0), 0);    // below 6 A
  CHECK_EQ(h.master.amps_for_power(0.0), 0);
  CHECK_EQ(h.master.amps_for_power(6.9), 10);
  // Car turns out to charge on one phase only.
  h.rx(power_status(0x1234, 100, 230, 20, 0, 0));
  CHECK_EQ(h.master.active_phases(), 1);
  CHECK_EQ(h.master.amps_for_power(2.3), 10);
  CHECK_EQ(h.master.amps_for_power(1.38), 6);
}

static void test_energy_meter() {
  printf("energy meter\n");
  EnergyMeter m;
  m.on_power(11000, 0);  // ignored until the counter is known
  m.on_counter(100);
  CHECK_NEAR(m.kwh(), 100.0, 1e-9);
  m.on_power(11000, 0);
  m.on_power(11000, 60000);  // 11 kW for 1 min = 0.1833 kWh
  CHECK_NEAR(m.kwh(), 100.1833, 1e-3);
  m.on_counter(101);  // TWC counter is ahead -> follow it
  CHECK_NEAR(m.kwh(), 101.0, 1e-9);
  for (uint32_t t = 120000; t <= 120000 + 3600000; t += 1000)
    m.on_power(11000, t);  // way more than 1 kWh while the counter is stuck
  CHECK_NEAR(m.kwh(), 101.999, 1e-9);  // bounded
  m.on_counter(100);  // corrupt lower counter is ignored
  CHECK_NEAR(m.kwh(), 101.999, 1e-9);

  EnergyMeter r;
  r.restore(55.42);
  r.on_counter(55);
  CHECK_NEAR(r.kwh(), 55.42, 1e-9);
  EnergyMeter stale;
  stale.restore(12.5);
  stale.on_counter(55);  // saved value doesn't belong to this counter
  CHECK_NEAR(stale.kwh(), 55.0, 1e-9);
}

int main() {
  test_checksum_and_framing();
  test_slip_decoder_robustness();
  test_encoder_lengths();
  test_decode();
  test_policy_commands();
  test_policy_clamp_and_enable();
  test_policy_watchdog();
  test_policy_min_on_off();
  test_master_startup_and_link();
  test_master_timeout_and_values();
  test_master_diagnostics();
  test_master_heartbeat_values();
  test_master_passive();
  test_master_start_stop();
  test_amps_for_power();
  test_energy_meter();
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
