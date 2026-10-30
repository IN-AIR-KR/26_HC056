// 상태머신 안전 전이 검증: 표적 소실 시 안전 상태 진입, 수동 인계 요청, 발사/명중 게이트.
// assert()는 Release(NDEBUG)에서 사라지므로 항상 평가되는 CHECK를 쓴다.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "arms_control/state_machine.hpp"

using arms_control::SMParams;
using arms_control::State;
using arms_control::StateMachine;
using arms_control::to_string;

namespace {

int g_failures = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
      ++g_failures;                                                          \
    }                                                                        \
  } while (0)

std::vector<arms_msgs::msg::BoundingBox> target(float conf = 0.9F) {
  arms_msgs::msg::BoundingBox b;
  b.x_center = 0.6F;
  b.y_center = 0.4F;
  b.width = b.height = 0.05F;
  b.confidence = conf;
  return {b};
}

void sleep_sec(double s) {
  std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

SMParams fast_params() {
  SMParams p;
  p.confidence_threshold = 0.5;
  p.lock_duration_sec = 0.05;
  p.detection_timeout_sec = 0.08;
  p.track_target_lost_timeout_sec = 0.15;
  return p;
}

// SEARCH에서 lock_duration 동안 검출을 넣어 LOCK까지 간다.
void drive_to_lock(StateMachine &sm) {
  sm.arm();
  const auto t0 = std::chrono::steady_clock::now();
  while (sm.state() != State::LOCK &&
         std::chrono::steady_clock::now() - t0 < std::chrono::seconds(1)) {
    sm.on_detection(target());
    sm.tick_detection_timeout();
    sleep_sec(0.01);
  }
}

void test_gating_in_idle() {
  StateMachine sm(fast_params());
  sm.on_launch_button();
  sm.on_hit();
  CHECK(sm.state() == State::IDLE);
  sm.on_detection(target());
  sm.tick_detection_timeout();
  CHECK(sm.state() == State::IDLE);
  CHECK(!sm.consume_manual_handover_request());
}

void test_low_confidence_never_locks() {
  StateMachine sm(fast_params());
  sm.arm();
  CHECK(sm.state() == State::SEARCH);
  for (int i = 0; i < 20; ++i) {
    sm.on_detection(target(0.2F));
    sm.tick_detection_timeout();
    sleep_sec(0.01);
  }
  CHECK(sm.state() == State::SEARCH);
  CHECK(!sm.target_locked());
}

void test_lock_loss_returns_to_search() {
  StateMachine sm(fast_params());
  drive_to_lock(sm);
  CHECK(sm.state() == State::LOCK);
  sleep_sec(0.12);  // > detection_timeout
  sm.tick_detection_timeout();
  CHECK(sm.state() == State::SEARCH);   // LOCK 소실은 재수색이지 IDLE/수동 인계가 아니다
  CHECK(!sm.consume_manual_handover_request());
  CHECK(sm.current_error_x() == 0.0 && sm.current_error_y() == 0.0);
}

void test_track_loss_hands_over_once() {
  std::vector<std::string> logs;
  StateMachine sm(fast_params(), [&](const std::string &m) { logs.push_back(m); });
  drive_to_lock(sm);
  sm.on_launch_button();
  CHECK(sm.state() == State::TRACK);

  // 빈 DetectionArray(검출기는 살아 있으나 표적 없음)는 hold 로 처리되고 timer를 갱신하지 않는다.
  sm.on_detection({});
  CHECK(sm.is_detection_held());
  sleep_sec(0.05);
  sm.tick_detection_timeout();
  CHECK(sm.state() == State::TRACK);   // timeout 전에는 유지
  CHECK(!sm.consume_manual_handover_request());

  sleep_sec(0.15);  // 누적 > track_target_lost_timeout
  sm.on_detection({});
  sm.tick_detection_timeout();
  CHECK(sm.state() == State::IDLE);
  CHECK(!sm.target_locked());
  CHECK(sm.consume_manual_handover_request());
  CHECK(!sm.consume_manual_handover_request());   // 1회성
  // 이후 tick은 IDLE 에서 아무 것도 하지 않는다(재요청 없음).
  sm.tick_detection_timeout();
  CHECK(!sm.consume_manual_handover_request());

  bool reason_logged = false;
  for (const auto &l : logs) {
    if (l.find("TRACK -> IDLE") != std::string::npos &&
        l.find("track_target_lost_timeout_sec") != std::string::npos) {
      reason_logged = true;
    }
  }
  CHECK(reason_logged);
}

void test_detection_refresh_keeps_track() {
  StateMachine sm(fast_params());
  drive_to_lock(sm);
  sm.on_launch_button();
  for (int i = 0; i < 30; ++i) {   // 0.3 s 동안 0.01 s 간격 검출 → timeout 없음
    sm.on_detection(target());
    sm.tick_detection_timeout();
    sleep_sec(0.01);
  }
  CHECK(sm.state() == State::TRACK);
}

void test_rtl_paths() {
  StateMachine sm(fast_params());
  drive_to_lock(sm);
  sm.on_launch_button();
  sm.on_hit();
  CHECK(sm.state() == State::RTL);
  // RTL 에서는 검출 timeout 이 상태를 바꾸지 않는다.
  sleep_sec(0.2);
  sm.tick_detection_timeout();
  CHECK(sm.state() == State::RTL);
  CHECK(!sm.consume_manual_handover_request());
  sm.disarm();
  CHECK(sm.state() == State::IDLE);

  sm.arm();
  sm.request_rtl("test");
  CHECK(sm.state() == State::RTL);
  sm.on_launch_button();
  CHECK(sm.state() == State::RTL);
}

// 주입한 시계(SITL 의 /clock 시뮬 시간에 해당)로만 timeout 이 흐르는지 본다. 벽시계로 아무리
// 기다려도 시뮬 시간이 안 가면 소실 판정이 나면 안 된다(RTF 0.2 에서 벽시계 2 s = 시뮬 0.4 s).
void test_injected_clock_drives_timeouts() {
  double sim_t = 100.0;
  StateMachine sm(fast_params(), nullptr, [&sim_t]() { return sim_t; });
  sm.arm();
  for (int i = 0; i < 6; ++i) {        // 시뮬 0.06 s 동안 검출 → LOCK (lock_duration 0.05)
    sm.on_detection(target());
    sm.tick_detection_timeout();
    sim_t += 0.01;
  }
  CHECK(sm.state() == State::LOCK);
  sm.on_launch_button();
  CHECK(sm.state() == State::TRACK);
  sm.on_detection(target());
  sleep_sec(0.3);                      // 벽시계 0.3 s (> timeout 0.15) 이지만 시뮬 시간 정지
  sm.tick_detection_timeout();
  CHECK(sm.state() == State::TRACK);
  sim_t += 0.14;                       // 시뮬 0.14 s < 0.15
  sm.tick_detection_timeout();
  CHECK(sm.state() == State::TRACK);
  sim_t += 0.02;                       // 시뮬 0.16 s > 0.15 → 소실
  sm.tick_detection_timeout();
  CHECK(sm.state() == State::IDLE);
  CHECK(sm.consume_manual_handover_request());
}

}  // namespace

int main() {
  test_injected_clock_drives_timeouts();
  test_gating_in_idle();
  test_low_confidence_never_locks();
  test_lock_loss_returns_to_search();
  test_track_loss_hands_over_once();
  test_detection_refresh_keeps_track();
  test_rtl_paths();
  if (g_failures) {
    std::fprintf(stderr, "state_machine_safety_test: %d failure(s)\n", g_failures);
    return EXIT_FAILURE;
  }
  std::printf("state_machine_safety_test: OK\n");
  return EXIT_SUCCESS;
}
