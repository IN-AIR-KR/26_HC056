#include "arms_control/state_machine.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

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
    // 홀드: 마지막 유효 오차를 유지하고 PID I만 동결한다. timeout 전이 자체는
    // control loop의 tick()이 수행한다. 따라서 detection 토픽이 통째로 멈춰도
    // 동일하게 처리된다.
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

  // Compute normalized pixel error from frame center
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

void StateMachine::tick()
{
  if (!last_detection_valid_) return;
  if (state_ != State::SEARCH && state_ != State::LOCK && state_ != State::TRACK) return;

  const double timeout_sec =
    state_ == State::TRACK ? params_.track_detection_timeout_sec : params_.detection_timeout_sec;
  const double elapsed =
    std::chrono::duration<double>(Clock::now() - last_detection_time_).count();
  if (elapsed < timeout_sec) return;

  detection_held_ = false;
  target_locked_ = false;
  error_x_ = 0.0;
  error_y_ = 0.0;
  last_detection_valid_ = false;
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

void StateMachine::on_intercept(double tau_sec, double bbox_size, double align_error)
{
  if (state_ != State::TRACK) return;
  // ① 최소 크기 게이트: 표적이 화면에서 충분히 커야(=가까워야) 종말 처리. 원거리/노이즈 배제.
  if (bbox_size < params_.loom_s_min) return;
  // ② τ 유효성: 표적이 커지는(닫히는) 중일 때만 τ>0.
  if (!(tau_sec > 0.0)) return;
  // ③ 곧 충돌(τ 작음) + 정면 정렬이면 추가 payload 단계 없이 바로 RTL.
  if (tau_sec < params_.tau_rtl_sec && align_error < params_.intercept_align_tol) {
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
    // 추적 중 표적을 timeout까지 놓치면 재수색하지 않고 안전하게 귀환한다.
    transition(State::RTL);
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
