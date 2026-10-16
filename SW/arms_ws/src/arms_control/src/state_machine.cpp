#include "arms_control/state_machine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace arms_control {

std::string to_string(State s)
{
  switch (s) {
    case State::IDLE:   return "IDLE";
    case State::SEARCH: return "SEARCH";
    case State::LOCK:   return "LOCK";
    case State::TRACK:  return "TRACK";
    case State::RTL:    return "RTL";
  }
  return "UNKNOWN";
}

StateMachine::StateMachine(const SMParams & params, LogFn log_fn)
: params_(params), log_fn_(std::move(log_fn))
{}

// ---------------------------------------------------------------------------
// External events
// ---------------------------------------------------------------------------

void StateMachine::on_detection(
  const std::vector<arms_msgs::msg::BoundingBox> & detections)
{
  const auto * best = best_detection(detections);

  if (!best || best->confidence < static_cast<float>(params_.confidence_threshold)) {
    // 타임아웃 전이는 control loop의 tick_detection_timeout()만 담당한다.
    // 따라서 detection 노드가 죽어 callback 자체가 멈춰도 같은 정책이 적용된다.
    detection_held_ = last_detection_valid_;
    return;
  }

  // Valid detection
  //   위치 급변(튐) 차단은 detection 노드의 상시 연관 게이트(track.jump_gate)가
  //   담당한다. control 은 detection 결과를 신뢰한다(단일 표적만 발행됨).
  detection_held_ = false;
  auto now = Clock::now();
  last_detection_time_  = now;
  last_detection_valid_ = true;

  // Keep the raw normalized image error for mission/UI publication. The control
  // node converts it to camera tan coordinates before any guidance calculation.
  error_x_ = static_cast<double>(best->x_center) - 0.5;
  error_y_ = static_cast<double>(best->y_center) - 0.5;

  if (state_ == State::SEARCH) {
    if (!lock_timer_running_) {
      lock_start_time_    = now;
      lock_timer_running_ = true;
    }
    lock_elapsed_sec_ =
      std::chrono::duration<double>(now - lock_start_time_).count();

    if (lock_elapsed_sec_ >= params_.lock_duration_sec) {
      transition(State::LOCK);
      target_locked_ = true;
    }
  } else if (state_ == State::LOCK || state_ == State::TRACK) {
    if (lock_timer_running_) {
      lock_elapsed_sec_ =
        std::chrono::duration<double>(now - lock_start_time_).count();
    }
    target_locked_ = true;
  }
}

void StateMachine::tick_detection_timeout()
{
  if (state_ != State::SEARCH && state_ != State::LOCK && state_ != State::TRACK) {
    detection_held_ = false;
    return;
  }

  const auto now = Clock::now();
  const double elapsed = last_detection_valid_
    ? std::chrono::duration<double>(now - last_detection_time_).count()
    : std::numeric_limits<double>::infinity();
  const double timeout = state_ == State::TRACK
    ? params_.track_target_lost_timeout_sec
    : params_.detection_timeout_sec;

  if (elapsed < timeout) {
    // 빈 DetectionArray가 들어왔을 때의 hold 상태는 on_detection()이 유지한다.
    // 정상 검출 프레임 사이에서 I 항을 불필요하게 freeze하지 않는다.
    return;
  }

  detection_held_ = false;
  target_locked_ = false;
  error_x_ = 0.0;
  error_y_ = 0.0;
  on_detection_timeout();
}

void StateMachine::on_launch_button()
{
  if (state_ == State::LOCK) {
    transition(State::TRACK);
  } else if (log_fn_) {
    log_fn_("[StateMachine] Launch button pressed in state " +
            to_string(state_) + " — ignored.");
  }
}

void StateMachine::on_intercept(double tau_sec, double bbox_size, double alignment_error)
{
  if (state_ != State::TRACK) return;
  // ① 최소 크기 게이트: 표적이 화면에서 충분히 커야(=가까워야) 종말 처리. 원거리/노이즈 배제.
  if (bbox_size < params_.loom_s_min) return;
  // ② τ 유효성: 표적이 커지는(닫히는) 중일 때만 τ>0.
  if (!(tau_sec > 0.0)) return;
  // ③ 곧 충돌(τ 작음) + 정면 정렬이면 추가 payload 단계 없이 바로 RTL.
  if (tau_sec < params_.tau_rtl_sec && alignment_error < params_.intercept_align_tol) {
    transition(State::RTL);
  }
}

void StateMachine::on_hit()
{
  // 심판이 실제 직격(거리)을 보고하면 TRACK에서 즉시 RTL.
  if (state_ == State::TRACK) {
    transition(State::RTL);
  }
}

void StateMachine::request_rtl()
{
  // RTL은 ARM 된 기체에 대해서만 control 노드가 호출한다. 현재 미션 단계와
  // 무관하게 유도를 멈추고 FC의 귀환 모드로 넘긴다.
  if (state_ != State::RTL) {
    transition(State::RTL);
  }
}

void StateMachine::arm()
{
  if (state_ == State::IDLE) {
    lock_timer_running_   = false;
    detection_held_       = false;
    last_detection_valid_ = false;
    transition(State::SEARCH);
  }
}

void StateMachine::disarm()
{
  transition(State::IDLE);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void StateMachine::on_detection_timeout()
{
  if (state_ == State::TRACK) {
    // 추적 중 표적을 timeout까지 놓치면 재수색하거나 GPS Rescue로 넘기지 않는다.
    // 유도를 정지하고 control node가 조종사 스틱을 FC에 직접 전달하도록 요청한다.
    manual_handover_requested_ = true;
    transition(State::IDLE);
    return;
  }
  if (state_ == State::SEARCH || state_ == State::LOCK) {
    if (state_ != State::SEARCH) transition(State::SEARCH);
    lock_timer_running_ = false;
    lock_elapsed_sec_   = 0.0;
  }
}

const arms_msgs::msg::BoundingBox * StateMachine::best_detection(
  const std::vector<arms_msgs::msg::BoundingBox> & dets) const
{
  if (dets.empty()) return nullptr;

  const arms_msgs::msg::BoundingBox * best = nullptr;
  for (const auto & d : dets) {
    if (!best || d.confidence > best->confidence) {
      best = &d;
    }
  }
  return best;
}

void StateMachine::transition(State next)
{
  if (log_fn_) {
    log_fn_("[StateMachine] " + to_string(state_) + " -> " + to_string(next));
  }
  state_ = next;
}

}  // namespace arms_control
