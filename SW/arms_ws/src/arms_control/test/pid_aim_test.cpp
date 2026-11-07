#include "arms_control/pid_aim.hpp"

#include <cassert>
#include <cmath>

using arms_control::pid_aim::Input;
using arms_control::pid_aim::Output;
using arms_control::pid_aim::Parameters;
using arms_control::pid_aim::compute;
using arms_control::pn::Mat3;
using arms_control::pn::Quaternion;
using arms_control::pn::Vec3;
using arms_control::pn::kDegToRad;
using arms_control::pn::kGravity;
using arms_control::pn::norm;

namespace {

// 상방 카메라: optical (right, down, forward) → body FRD. optical forward = body up.
Mat3 upward_camera() { return Mat3{{0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0}}; }

// NED 방향 → 수평 기체(자세 0)의 optical 좌표 (r_bc 는 대칭행렬이라 전치 = 자신)
Vec3 ned_to_cam_level(const Vec3 &v) { return upward_camera() * v; }

Input level_input(const Vec3 &los_ned) {
  Input in;
  in.los_c = ned_to_cam_level(los_ned);
  in.r_nb = Quaternion{}.rotation_matrix();
  in.r_bc = upward_camera();
  return in;
}

bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

}  // namespace

int main() {
  Parameters params;
  params.navigation_ratio = 3.0;
  params.approach_accel_mps2 = 6.0;
  params.min_closing_speed_mps = 3.0;
  params.hover_throttle = 0.25;

  // 1) 표적이 바로 위, LOS 회전 없음 → 광축을 그대로 위로. 추력 = hover·(A+g)/g.
  {
    const Output out = compute(params, level_input({0.0, 0.0, -1.0}));
    assert(out.valid);
    assert(near(out.err_x, 0.0, 1e-9) && near(out.err_y, 0.0, 1e-9));
    assert(near(out.ff_x, 0.0, 1e-12) && near(out.ff_y, 0.0, 1e-12));
    assert(near(out.throttle, 0.25 * (6.0 + kGravity) / kGravity, 1e-9));
  }

  // 2) 앙각 60° 북쪽 표적(수평 기체에서 optical +y). 중력 상쇄 때문에 추력축은 LOS 보다
  //    더 서 있어야 한다 → 0 < err_y < 표적 y, 등가 조준점은 표적이 화면 아래(+y)에 놓이는 쪽.
  const Vec3 los60{std::cos(60.0 * kDegToRad), 0.0, -std::sin(60.0 * kDegToRad)};
  {
    const Input in = level_input(los60);
    const double target_y = in.los_c.y / in.los_c.z;
    const Output out = compute(params, in);
    assert(out.valid);
    assert(near(out.err_x, 0.0, 1e-9));
    assert(out.err_y > 0.0 && out.err_y < target_y);
    assert(out.aim_y > 0.0);
    assert(!out.look_limited);
  }

  // 3) 시야각 제약: 5° 로 묶으면 추력축-LOS 각이 5° 를 넘지 않고, 수직 성분을 지키려 추력↑.
  {
    Parameters limited = params;
    limited.max_look_angle_rad = 5.0 * kDegToRad;
    const Output free_out = compute(params, level_input(los60));
    const Output out = compute(limited, level_input(los60));
    assert(out.valid && out.look_limited);
    assert(out.look_angle_deg <= 5.0 + 1e-6);
    assert(free_out.look_angle_deg > 5.0);
    assert(out.throttle > free_out.throttle);
  }

  // 4) 표적 바로 위인데 LOS 가 북쪽으로 돈다(λ̇ = 0.1 rad/s, 북). PN 은 북쪽으로 횡가속을
  //    내므로 광축 목표가 optical +y 로 앞서 나가고, 피드포워드도 +y 로 같은 회전을 낸다.
  {
    Input in = level_input({0.0, 0.0, -1.0});
    in.los_rate_valid = true;
    in.los_rate_ned = {0.1, 0.0, 0.0};
    in.closing_speed_mps = 5.0;
    const Output out = compute(params, in);
    assert(out.valid);
    assert(near(norm(out.accel_pn_ned), 3.0 * 5.0 * 0.1, 1e-9));
    assert(out.err_y > 0.0 && near(out.err_x, 0.0, 1e-9));
    assert(near(out.ff_y, 0.1, 1e-9) && near(out.ff_x, 0.0, 1e-12));

    // 접근속도 추정이 0 이어도 Vc 하한(3 m/s)으로 LOS 회전을 죽인다.
    in.closing_speed_mps = 0.0;
    const Output low = compute(params, in);
    assert(near(low.closing_speed_used_mps, 3.0, 1e-12));
    assert(near(norm(low.accel_pn_ned), 3.0 * 3.0 * 0.1, 1e-9));
  }

  // 5) 접근 감속(approach_scale=0)은 접근 가속만 없앤다: 바로 위 표적이면 호버 추력.
  {
    Input in = level_input({0.0, 0.0, -1.0});
    in.approach_scale = 0.0;
    const Output out = compute(params, in);
    assert(near(out.throttle, 0.25, 1e-9));
  }

  // 6) 표적이 광축 뒤쪽이면 계산하지 않는다.
  {
    Input in = level_input({0.0, 0.0, 1.0});
    assert(!compute(params, in).valid);
  }
  return 0;
}
