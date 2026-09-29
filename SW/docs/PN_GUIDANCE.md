# A.R.M.S. PN 유도 구현 설계

_카메라 bearing으로부터 관성 LOS를 추정하고, 충돌 가속도를 기체 자세와 추력으로 변환하는 방법_

## 문서 목적

이 문서는 A.R.M.S.의 영상 유도를 다음 구조로 구현하기 위한 수학적 기준과 검증 절차를 정의한다.

```text
카메라 bbox
→ bearing 및 inertial LOS-rate 추정
→ PN으로 desired acceleration 계산
→ gravity compensation
→ desired attitude + thrust 계산
→ PX4 attitude/rate controller
```

`feat/pn-test`의 현재 구현은 이 문서의 기본 구조를 따른다. 선택한 범위는 다음과 같다.

- 인접한 두 관성 LOS 벡터의 변화를 constant angular-rate KF로 추정
- bbox angular scale + 알려진 표적 지름의 looming으로 $V_c$ 추정
- center-holding 없이 PN + LOS 방향 접근 가속도만 사용
- 상방 카메라, 초기 yaw 고정
- gravity compensation 후 desired attitude/thrust 생성
- desired roll/pitch를 CRSF로 보내 PX4 Stabilized attitude controller에 연결

terminal 전용 로직과 FOV/center-holding은 이번 구현 범위에서 제외했다.

## 목차

1. [좌표계와 기호](#1-좌표계와-기호)
2. [카메라 bbox → bearing 및 inertial LOS-rate 추정](#2-카메라-bbox--bearing-및-inertial-los-rate-추정)
3. [PN → desired acceleration](#3-pn--desired-acceleration)
4. [Gravity compensation](#4-gravity-compensation)
5. [Desired attitude + thrust 계산](#5-desired-attitude--thrust-계산)
6. [PX4 attitude/rate controller 연결](#6-px4-attituderate-controller-연결)
7. [필터, 시간 동기화, 지연 보상](#7-필터-시간-동기화-지연-보상)
8. [제한 조건과 상태 전이](#8-제한-조건과-상태-전이)
9. [현행 코드와 목표 구조의 차이](#9-현행-코드와-목표-구조의-차이)
10. [단계별 구현 계획](#10-단계별-구현-계획)
11. [SITL 검증 절차](#11-sitl-검증-절차)
12. [필수 로그와 성공 기준](#12-필수-로그와-성공-기준)
13. [참고 자료](#13-참고-자료)

---

## 1. 좌표계와 기호

수식은 먼저 오른손 좌표계인 world ENU를 기준으로 정의한다.

- $W$: world ENU, $x$=동쪽, $y$=북쪽, $z$=위
- $B$: 기체 body 좌표계
- $C$: 카메라 optical 좌표계, $x$=오른쪽, $y$=아래, $z$=전방
- $R_{AB}$: B 좌표계 벡터를 A 좌표계로 회전시키는 행렬
- $\mathbf p_D,\mathbf v_D$: 드론의 위치와 속도
- $\mathbf p_T,\mathbf v_T$: 표적의 위치와 속도
- $\mathbf r=\mathbf p_T-\mathbf p_D$: 드론에서 표적으로 향하는 상대 위치
- $R=\|\mathbf r\|$: 표적 거리
- $\mathbf l=\mathbf r/R$: LOS 단위벡터
- $\mathbf v_r=\mathbf v_T-\mathbf v_D$: 상대속도
- $V_c=-\mathbf l^T\mathbf v_r$: 접근속도. 접근 중이면 양수
- $\boldsymbol\omega_{LOS}$: 관성공간에서 LOS가 회전하는 각속도
- $t_{go}$: 예상 충돌까지 남은 시간

PX4는 내부적으로 NED/FRD를 주로 사용한다. ENU에서 계산한 벡터를 PX4에 전달할 때는
반드시 명시적인 ENU↔NED, FLU↔FRD 변환을 거친다. 축 변환을 부호 몇 개로 흩어 놓지 않고
회전행렬 또는 검증된 변환 함수 한 곳에 모은다.

### 1.1 LOS, look angle, heading error의 구분

- **LOS**: 드론 위치에서 표적 위치로 향하는 선 또는 단위벡터 $\mathbf l$
- **Look angle**: 카메라 광축과 LOS 사이의 각도. 영상 중심 오차에 해당
- **Velocity-to-LOS angle**: 드론 속도 방향과 LOS 사이의 각도
- **LOS-rate**: 관성공간에서 LOS 자체가 회전하는 속도

고정 카메라에서 표적이 화면 중앙에 있다는 것은 look angle이 0이라는 뜻이다. 현재 속도
방향이 LOS와 같거나 미래 miss distance가 0이라는 뜻은 아니다.

---

## 2. 카메라 bbox → bearing 및 inertial LOS-rate 추정

### 2.1 bbox 중심을 픽셀 좌표로 복원

`BoundingBox`의 중심은 영상 크기로 정규화되어 있다. 영상 폭과 높이를 $W,H$라 하면

```math
u=x_{center}W,\qquad v=y_{center}H
```

이다. bbox 중심은 표적의 기하학적 중심과 일치한다는 가정이 들어간다. 풍선이 화면 밖으로
잘리거나 검출 박스가 한쪽으로 치우치면 bearing에도 같은 bias가 생긴다.

### 2.2 렌즈 왜곡 보정

원시 픽셀 $(u,v)$를 바로 pinhole 식에 넣기 전에 카메라 캘리브레이션의 왜곡계수로
undistort한다. OpenCV를 사용한다면 `cv::undistortPoints()`의 출력을 사용한다.

현재 `arms_video/config/camera_info.yaml`은

```math
f_x=f_y=600,\qquad c_x=360,\qquad c_y=240
```

인 플레이스홀더이며 왜곡계수가 모두 0이다. 실제 기체에서 관성 LOS-rate를 사용하려면
카메라 캘리브레이션을 먼저 수행해야 한다.

### 2.3 픽셀을 카메라 bearing으로 변환

카메라 내부행렬을

```math
K=
\begin{bmatrix}
f_x&0&c_x\\
0&f_y&c_y\\
0&0&1
\end{bmatrix}
```

라고 한다. 동차 픽셀벡터 $\tilde{\mathbf z}=[u,v,1]^T$로부터 카메라 좌표계의
LOS ray를 구한다.

```math
\tilde{\mathbf l}_C=K^{-1}\tilde{\mathbf z}
=
\begin{bmatrix}
(u-c_x)/f_x\\
(v-c_y)/f_y\\
1
\end{bmatrix}
```

```math
\boxed{
\mathbf l_C=\frac{\tilde{\mathbf l}_C}{\|\tilde{\mathbf l}_C\|}
}
```

필요하면 방위각과 고각으로도 표현할 수 있다.

```math
\beta_{az}=\operatorname{atan2}(l_{C,x},l_{C,z})
```

```math
\beta_{el}=-\operatorname{atan2}
\left(l_{C,y},\sqrt{l_{C,x}^2+l_{C,z}^2}\right)
```

영상 $y$축은 아래가 양수이므로 elevation에는 음수가 붙는다. 제어와 필터 내부에서는
두 각도보다 단위벡터 $\mathbf l_C$를 유지하는 편이 특이점과 부호 오류가 적다.

### 2.4 카메라 장착각 보정

카메라에서 body로 가는 고정 extrinsic을 $R_{BC}$라 하면

```math
\mathbf l_B=R_{BC}\mathbf l_C
```

이다. 카메라가 기체 정면을 본다는 가정만으로 항등행렬을 쓰면 안 된다. optical frame과
PX4 body frame은 축 순서부터 다르며, 실제 장착 pitch/yaw/roll 오차도 존재한다.

extrinsic은 다음 두 부분으로 관리한다.

```math
R_{BC}=R_{axes}R_{mount}
```

- $R_{axes}$: 카메라 optical 축과 body 축 정의의 고정 변환
- $R_{mount}$: 실제 장착 오차 캘리브레이션

### 2.5 관성 LOS 계산

영상 촬영 시각 $t_k$의 PX4 attitude로부터 body-to-world 회전행렬 $R_{WB}(t_k)$를 얻는다.

```math
\boxed{
\mathbf l_W(t_k)=R_{WB}(t_k)R_{BC}\mathbf l_C(t_k)
}
```

드론과 표적이 정지하고 드론만 회전할 때 $\mathbf l_C$는 크게 변하지만 $\mathbf l_W$는
거의 일정해야 한다. 이것이 body rotation이 제거되었는지 확인하는 가장 중요한 단위 시험이다.

### 2.6 두 LOS 벡터로 inertial LOS-rate 계산

연속된 관성 LOS를 $\mathbf l_{k-1},\mathbf l_k$, 촬영시각 차이를 $\Delta t$라 한다.

작은 각도에서는

```math
\boldsymbol\omega_{LOS}
\approx
\frac{\mathbf l_{k-1}\times\mathbf l_k}{\Delta t}
```

를 사용할 수 있다. 큰 회전에도 유효한 axis-angle 식은 다음과 같다.

```math
\mathbf c=\mathbf l_{k-1}\times\mathbf l_k
```

```math
\theta=\operatorname{atan2}
\left(\|\mathbf c\|,\mathbf l_{k-1}^T\mathbf l_k\right)
```

```math
\boxed{
\boldsymbol\omega_{LOS}
=
\frac{\theta}{\Delta t}
\frac{\mathbf c}{\|\mathbf c\|}
}
```

$\|\mathbf c\|$가 매우 작으면 0으로 처리한다. LOS 단위벡터의 변화율은

```math
\boxed{
\dot{\mathbf l}
=\boldsymbol\omega_{LOS}\times\mathbf l
}
```

이다.

### 2.7 gyro body-rate로 직접 보상하는 대안

카메라 좌표계에서 측정한 ray 변화율을 $\dot{\mathbf l}_C$, 카메라의 관성 각속도를
카메라 좌표계로 표현한 값을 $\boldsymbol\omega_C$라 하면

```math
\left.\dot{\mathbf l}\right|_I^C
=
\dot{\mathbf l}_C
+\boldsymbol\omega_C\times\mathbf l_C
```

이다. rigid mount라면

```math
\boldsymbol\omega_C=R_{CB}\boldsymbol\omega_B
```

이다. 따라서

```math
\boldsymbol\omega_{LOS}^C
=
\mathbf l_C\times
\left(
\dot{\mathbf l}_C+
\boldsymbol\omega_C\times\mathbf l_C
\right)
```

로 구할 수 있다. 고주파 보상에는 gyro 방식이 유리하지만 좌표계와 timestamp 실수가 바로
큰 오차로 이어진다. 첫 구현은 attitude로 각 프레임의 LOS를 world에 회전시키는 방식을
사용하고, ground truth 검증 후 gyro 방식과 비교한다.

---

## 3. PN → desired acceleration

### 3.1 상대운동

상대 위치와 속도는

```math
\mathbf r=\mathbf p_T-\mathbf p_D,
\qquad
\mathbf v_r=\mathbf v_T-\mathbf v_D
```

이다. LOS와 접근속도는

```math
\mathbf l=\frac{\mathbf r}{R},
\qquad
V_c=-\mathbf l^T\mathbf v_r
```

이다. $V_c>0$이면 접근 중이고, $V_c\le0$이면 거리가 줄어들지 않는 상태다.

상대 위치·속도를 알고 있을 때 LOS-rate의 ground truth는

```math
\boxed{
\boldsymbol\omega_{LOS,GT}
=
\frac{\mathbf r\times\mathbf v_r}{R^2}
}
```

이다. SITL에서는 영상 추정값의 정답으로만 사용하며, 실기체용 제어 입력에 Gazebo 값을
섞지 않는다.

### 3.2 3차원 True Proportional Navigation

PN의 목적은 접근 중 LOS 회전을 줄여 constant-bearing collision course를 만드는 것이다.

```math
\boxed{
\mathbf a_{PN}
=N V_c
\left(
\boldsymbol\omega_{LOS}\times\mathbf l
\right)
}
```

$\boldsymbol\omega_{LOS}\times\mathbf l=\dot{\mathbf l}$이므로

```math
\mathbf a_{PN}=N V_c\dot{\mathbf l}
```

로도 쓸 수 있다.

- $N$: 무차원 navigation ratio. 이상적인 단순 모델에서는 보통 3~5에서 시작
- $V_c$: m/s
- $\boldsymbol\omega_{LOS}$: rad/s
- $\mathbf a_{PN}$: m/s²

구 코드의 `pn_nav_gain=175`와 정식 PN의 $N=3\sim5$는 단위와 의미가 다르므로 숫자를
그대로 옮기지 않았다. 현재 기본값은 $N=3$이다.

거리센서가 없으면 $V_c$를 다음 중 하나로 얻는다.

1. 레이더/라이다 거리 $R$를 미분해 $V_c=-\dot R$ 계산
2. 알려진 표적 크기와 bbox scale로 거리 추정
3. looming으로 time-to-contact $\tau$ 추정 후 $V_c/R\approx1/\tau$ 사용
4. 초기 검증 동안 일정한 유효 $V_c$를 사용하고 성능 한계를 명시

$V_c$ 없이 $K\boldsymbol\omega_{LOS}$만 쓰는 방식도 작동할 수 있지만, 이는 단위가
흡수된 PN 근사이며 접근속도가 바뀌면 동일한 gain의 의미가 바뀐다.

### 3.3 Time-to-go 추정

가장 단순한 근사는

```math
\boxed{
t_{go}\approx\frac{R}{\max(V_c,V_{c,min})}
}
```

이다. bbox scale $s$가 표적의 선형 크기에 비례하고 $\dot s>0$이면 looming TTC는

```math
\boxed{
\tau=\frac{s}{\dot s}
}
```

이다. 이상적인 pinhole과 일정 접근속도에서는 $\tau\approx t_{go}$다. bbox가 화면 밖으로
잘리거나 detector가 크기를 흔들면 $\tau$가 크게 오염되므로 유효성 게이트가 필요하다.

### 3.4 PN 횡가속도 명령

LOS-rate 추정과 접근속도 추정이 유효할 때 횡가속도 명령은 다음과 같다.

```math
\boxed{
\mathbf a_\perp=\mathbf a_{PN}
=N V_c(\boldsymbol\omega_{LOS}\times\mathbf l)
}
```

terminal 구간에서도 같은 식을 사용하되, 센서 지연과 기체 응답시간 때문에 남은 시간 안에
실현할 수 없는 명령이 생기지 않도록 acceleration, tilt, thrust, jerk를 제한하고 포화 상태를
로그로 확인한다.

### 3.5 Center-holding과 충돌 유도의 분리

표적을 화면 중앙에 두는 것은 가시성 유지 목적이며, 충돌 조건 자체는 아니다. 고정 카메라를
단 기체는 중력 보상을 위해 표적보다 위로 추력벡터를 향해야 할 수 있으므로 정상적인 충돌
경로에서도 look angle이 0이 아닐 수 있다.

따라서 기존의

```math
K_Ce
```

를 PN 명령에 큰 값으로 바로 더하는 대신 다음 중 하나를 사용한다.

1. 화면 경계에 가까워질 때만 작동하는 FOV barrier/soft constraint
2. 물리적으로 필요한 기준 look angle $\beta_{ref}$를 계산한 뒤
   $e_\beta=\beta-\beta_{ref}$를 작게 유지
3. PN 가속도에 작은 visibility acceleration을 제한적으로 합성

단순한 soft constraint 예시는

```math
a_{FOV}=
\begin{cases}
0,&|\beta|\le\beta_{soft}\\
-K_F\left(|\beta|-\beta_{soft}\right)\operatorname{sgn}(\beta),
&|\beta|>\beta_{soft}
\end{cases}
```

이다. 이 항은 PN보다 낮은 우선순위와 별도 제한을 가져야 한다.

### 3.6 접근 방향 가속도

PN은 LOS에 수직인 가속도를 만든다. 표적을 향한 접근속도는 별도로 설계한다.

```math
\mathbf a_{guidance}
=a_\parallel\mathbf l+\mathbf a_\perp
```

여기서 $a_\parallel$는 목표 접근속도 추종으로 만들 수 있다.

```math
a_\parallel
=K_v\left(V_{c,ref}-V_c\right)
```

가속도 크기는 기체 추력과 tilt 한계를 넘지 않도록 제한한다.

---

## 4. Gravity compensation

world ENU에서 중력가속도는

```math
\mathbf g=
\begin{bmatrix}0&0&-g\end{bmatrix}^T,
\qquad g\approx9.81\ \mathrm{m/s^2}
```

이다. 드론의 실제 관성가속도는

```math
\ddot{\mathbf p}_D=\mathbf f+\mathbf g
```

로 쓸 수 있다. $\mathbf f$는 프로펠러가 만드는 specific thrust acceleration이다.
유도기가 원하는 관성가속도가 $\mathbf a_{guidance}$라면 필요한 specific thrust는

```math
\boxed{
\mathbf f_{des}=\mathbf a_{guidance}-\mathbf g
}
```

이다.

이 식이 정지 표적을 정조준한 드론이 아래로 통과하는 문제의 핵심이다. 추력 방향을 LOS에
그대로 맞추면

```math
\ddot{\mathbf p}_D=a_T\mathbf l+\mathbf g
```

가 되어 합가속도가 LOS 아래로 기운다. 유한한 추력에서는 단순히 throttle을 높여도 이 각도는
작아질 뿐 정확히 0이 되지 않는다. 원하는 합가속도를 LOS 방향으로 만들려면 추력벡터는

```math
\mathbf f_{des}=a_\parallel\mathbf l-\mathbf g
```

처럼 LOS보다 위를 향해야 한다.

### 4.1 tilt 보상과 벡터 중력 보상의 관계

고도 유지에 필요한 단순 tilt compensation은

```math
T=\frac{mg}{\cos\phi\cos\theta}
```

이다. 이는 수직가속도 0이라는 특수한 경우다. PN의 3차원 가속도 명령까지 포함하려면
스칼라 throttle 보정보다 $\mathbf f_{des}=\mathbf a_{guidance}-\mathbf g$를 계산하고 그 벡터에서
자세와 추력을 동시에 만드는 방식이 일반적이다.

### 4.2 공력과 모델 오차

고속 비행에서는 drag를 포함해

```math
\mathbf f_{des}
=\mathbf a_{guidance}-\mathbf g-\hat{\mathbf a}_{drag}
```

로 확장할 수 있다. 첫 구현에서는 drag feedforward를 생략하고 PX4 및 바깥쪽 피드백이
오차를 수정하도록 한다. 중력 보상이 검증된 뒤에 drag 모델을 추가한다.

---

## 5. Desired attitude + thrust 계산

### 5.1 목표 thrust 방향

프로펠러가 만드는 양의 추력축 단위벡터를 $\mathbf b_3$라 정의한다. 목표축은

```math
\boxed{
\mathbf b_{3,d}=\frac{\mathbf f_{des}}{\|\mathbf f_{des}\|}
}
```

이고 총 추력은

```math
\boxed{
T_d=m\|\mathbf f_{des}\|
}
```

이다. 실제 PX4 FRD body에서는 프로펠러 추력이 body $-z$ 방향이므로 메시지와 controller의
축 정의에 맞춰 부호를 변환한다.

### 5.2 목표 yaw를 포함한 자세행렬

목표 yaw를 $\psi_d$라 하면 수평면의 기준 heading 벡터를

```math
\mathbf b_{1,c}=
\begin{bmatrix}
\cos\psi_d\\
\sin\psi_d\\
0
\end{bmatrix}
```

로 둔다. 목표 body 축은

```math
\mathbf b_{2,d}
=
\frac{\mathbf b_{3,d}\times\mathbf b_{1,c}}
{\|\mathbf b_{3,d}\times\mathbf b_{1,c}\|}
```

```math
\mathbf b_{1,d}=\mathbf b_{2,d}\times\mathbf b_{3,d}
```

```math
\boxed{
R_{WB,d}=
\begin{bmatrix}
\mathbf b_{1,d}&\mathbf b_{2,d}&\mathbf b_{3,d}
\end{bmatrix}
}
```

로 만든다. 이를 quaternion $q_d$로 변환해 PX4 attitude setpoint로 전달할 수 있다.

### 5.3 yaw 선택

현 기체는 상방 카메라이고 yaw를 별도 유도할 필요가 없으므로, PN 필터가 초기화될 때의
yaw를 $\psi_d$로 래치한다. CH4는 yaw-rate 0을 지시하며 아래의 LOS 방위각 추종은
현 구현에서 사용하지 않는 확장안이다.

고정 전방 카메라라면 yaw는 다음 목표를 함께 고려해야 한다.

- 표적을 수평 FOV 안에 유지
- thrust 방향으로 필요한 roll/pitch 자유도 확보
- yaw rate 급변 방지

가장 단순한 방식은 LOS의 수평 방위각을 yaw reference로 두고 LPF/rate limit를 적용하는 것이다.

```math
\psi_d=\operatorname{atan2}(l_y,l_x)
```

다만 표적을 항상 광축 중앙에 두도록 roll/pitch까지 강제하면 중력 보상 thrust 방향과 충돌할
수 있다. yaw는 표적을 수평 중앙에 가깝게 유지하되 pitch 방향에는 필요한 look-angle 여유를
허용하는 설계가 적합하다.

### 5.4 추력 한계와 tilt 한계

```math
T_{min}\le T_d\le T_{max}
```

```math
\cos^{-1}(\mathbf b_{3,d}^T\mathbf e_z)\le\theta_{max}
```

를 적용한다. 포화가 발생하면 desired acceleration을 그대로 유지할 수 없으므로 우선순위를
정해야 한다. 권장 우선순위는 다음과 같다.

1. 자세 안정성과 최소 수직 추력
2. terminal 횡방향 miss 감소
3. 접근 가속도
4. 화면 중앙 유지

포화 여부와 포화 전·후 가속도 벡터를 반드시 로그로 남긴다.

---

## 6. PX4 attitude/rate controller 연결

### 6.1 권장 구조: attitude 또는 acceleration setpoint

가능하면 PX4 Offboard의 attitude/thrust 또는 acceleration setpoint 인터페이스를 사용한다.

```text
PN acceleration
→ gravity compensation
→ desired attitude + collective thrust
→ PX4 attitude controller
→ PX4 rate controller
→ motors
```

이 구조에서는 A.R.M.S.가 충돌 기하와 외부 가속도 루프를 담당하고, PX4가 빠른 자세/각속도
루프와 mixer를 담당한다.

### 6.2 현재 CRSF Stabilized 연결

현재 구현은 $R_d$에서 desired roll/pitch를 복원하고 `max_angle_deg`로
정규화해 CRSF CH1/CH2로 보낸다. PX4 Stabilized가 이 입력을 attitude setpoint로
해석하므로 Jetson에서 별도 attitude outer loop를 돌리지 않는다.

```math
u_r=\operatorname{sat}\left(\frac{\phi_d}{\phi_{max}},-1,1\right),\qquad
u_p=\operatorname{sat}\left(\frac{-\theta_d}{\theta_{max}},-1,1\right)
```

pitch에만 음수가 붙는 이유는 좌표 convention 차이다. NED/FRD Euler pitch는
$\theta_d<0$이 기수 하강이지만, PX4 manual pitch 입력은 양의 스틱이 기수 하강이다.
이 변환을 생략하면 세로 유도가 정확히 반대 방향으로 동작한다.

SITL의 `MPC_MAN_TILT_MAX`와 `control.max_angle_deg`는 모두 $55^\circ$로 맞춘다.
실기체 PX4의 Stabilized 풀스틱 기울기도 같은 값으로 설정해야 한다.

### 6.3 직접 MAVLink attitude setpoint 대안

정확한 quaternion/thrust setpoint를 MAVLink `SET_ATTITUDE_TARGET`으로 보내는 방식도
가능하지만, 현재의 SITL/실기체 공통 CRSF 경로를 유지하기 위해 사용하지 않는다.

#### 참고: 명시적 attitude outer loop

만약 ACRO body-rate 경로를 사용한다면 목표 자세와 현재 자세의 오차로
목표 rate를 만들 수 있다.

목표 및 현재 회전행렬을 $R_d,R$라 하면 geometric attitude error는

```math
\mathbf e_R
=
\frac12
\left(R_d^TR-R^TR_d\right)^\vee
```

이다. 목표 body-rate는

```math
\boxed{
\boldsymbol\omega_d
=-K_R\mathbf e_R+\boldsymbol\omega_{ff}
}
```

로 만들 수 있다. 첫 구현에서는 $\boldsymbol\omega_{ff}=0$으로 시작한다. 각속도를 제한한 뒤
CRSF 풀스틱 rate로 정규화한다.

```math
u_{CRSF,i}
=
\operatorname{clip}
\left(
\frac{\omega_{d,i}}{\omega_{max,i}},-1,1
\right)
```

이 자세 바깥루프가 있어야 목표 tilt에 도달한 뒤 rate가 0으로 내려가고 필요한 기울기를
유지한다. 영상오차에서 rate를 바로 만들면 목표 가속도와 실제 가속도의 대응이 명확하지 않다.

### 6.4 thrust 명령 변환

물리 추력 $T_d$와 throttle 사이에는 모터·프로펠러·배터리·ESC에 따른 비선형성이 있다.

```math
T\ne k\,u_{throttle}
```

실기체에서는 thrust stand 또는 비행 로그로 다음 형태의 모델을 식별한다.

```math
T(u)=a_2u^2+a_1u+a_0
```

필요한 throttle은 역함수로 구하고 배터리 전압 보상을 추가한다. SITL에서도 PX4의 hover
thrust 및 motor model과 일치하는지 확인한다.

---

## 7. 필터, 시간 동기화, 지연 보상

### 7.1 촬영시각 기준 동기화

LOS 변환에는 detection callback 시각이 아니라 원본 image header의 촬영시각을 사용한다.
`DetectionArray.header`는 현재 원본 영상 header를 전달하므로 이를 유지한다.

attitude를 시간순 ring buffer에 저장하고, 검출 시각 $t_k$ 양쪽의 quaternion
$q(t_a),q(t_b)$를 SLERP한다.

```math
q(t_k)=\operatorname{SLERP}
\left(q(t_a),q(t_b),\frac{t_k-t_a}{t_b-t_a}\right)
```

현재 attitude를 과거 영상에 적용하면

```math
\Delta\lambda_{delay}\approx\omega_{body}t_{delay}
```

의 가짜 LOS 오차가 생긴다. 빠른 기동에서는 수십 ms도 무시할 수 없다.

### 7.2 LOS-rate 필터

bbox 중심을 미분하면 검출 노이즈가 증폭된다. 권장 상태는 LOS의 두 tangent 성분과 그
각속도다.

```math
\mathbf x=
\begin{bmatrix}
\lambda_1&\lambda_2&\dot\lambda_1&\dot\lambda_2
\end{bmatrix}^T
```

constant angular-rate 모델은

```math
\mathbf x_{k+1}=
\begin{bmatrix}
1&0&\Delta t&0\\
0&1&0&\Delta t\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}
\mathbf x_k+\mathbf w_k
```

로 둘 수 있다. 초기 구현은 alpha-beta 필터를 사용할 수 있다.

```math
\lambda_k^-=\hat\lambda_{k-1}+\dot{\hat\lambda}_{k-1}\Delta t
```

```math
r_k=\lambda_k-\lambda_k^-
```

```math
\hat\lambda_k=\lambda_k^-+\alpha r_k
```

```math
\dot{\hat\lambda}_k
=\dot{\hat\lambda}_{k-1}+\frac{\beta}{\Delta t}r_k
```

### 7.3 중복 필터 지연 방지

현재 경로에는 detection KF, control LPF, PN alpha-beta가 직렬로 존재한다. 여러 필터를
겹치면 terminal 구간의 위상지연이 커진다. 목표 구조에서는 다음 중 하나를 선택한다.

- **현재 선택:** raw detection → guidance용 timestamp-aware constant angular-rate LOS KF 하나
- detector KF 상태와 covariance를 guidance filter에 전달해 재사용

필터 게인은 1차적으로 노이즈 크기가 아니라 허용 지연으로 결정한다. 예를 들어 총 지연이
$t_d$이고 횡속도가 $v_\perp$면 추가 miss는 대략

```math
d_{delay}\approx v_\perp t_d
```

다.

### 7.4 검출 유실

`LOST` 상태에서 detector가 외삽한 bbox를 실제 측정과 동일하게 PN에 넣으면 LOS-rate가
필터 예측을 다시 미분한 값이 된다. 다음 정책을 사용한다.

- `TRACKED`: 정상 갱신
- 짧은 `LOST`: navigation filter prediction만 수행하고 measurement update 중지
- timeout 초과: PN 출력 감쇠, 재획득 모드 진입
- 재획득: 큰 innovation을 rate spike로 해석하지 않도록 filter reset 또는 gate 적용

---

## 8. 제한 조건과 상태 전이

### 8.1 PN 활성 조건

다음을 모두 만족할 때만 정상 PN 명령을 사용한다.

- detection confidence 및 tracker 상태 유효
- camera calibration과 extrinsic 로드 성공
- 촬영시각에 대응하는 attitude 존재
- $\Delta t$가 허용 범위 안에 있음
- 접근속도 또는 TTC 추정 유효
- LOS-rate innovation과 covariance가 허용 범위 안에 있음

### 8.2 비정상 상태 fallback

- 거리/TTC만 무효: 제한된 유효 $V_c$를 사용한 bearing-rate 제어
- attitude timestamp 무효: PN 중지 후 낮은 gain의 image centering
- 표적 유실: 마지막 명령 유지 금지, 설정된 시간상수로 감쇠
- thrust/tilt 포화: 접근 가속도를 먼저 줄이고 자세 안정성과 횡유도를 보존

### 8.3 terminal 구간

terminal 전용 로직은 후속 단계로 미뤄다. 현재는 전 구간에 같은 PN, 가속도 상한,
각속도 상한을 사용한다.

근거리에서는 작은 각도 노이즈도 큰 rate로 보일 수 있고 남은 제어시간은 짧다. 다음을 함께
적용한다.

- $t_{go}$ 최소값 clamp
- acceleration 및 jerk limit
- terminal 구간의 acceleration 및 rate 명령 연속화
- detector bbox clipping 또는 partial visibility 검출 시 신뢰도 하향
- 명령 포화 상태에서 navigation gain을 더 올리지 않음

---

## 9. 구 PN과 현재 구현의 차이

| 항목 | 구 PN 모드 | `feat/pn-test` 현재 구현 |
|---|---|---|
| 영상 입력 | 정규화 bbox 중심 | undistorted pixel + camera intrinsics |
| LOS | LPF 픽셀 오차를 `los`라 명명 | 카메라 ray를 world에 회전한 단위벡터 |
| LOS-rate | 픽셀오차 alpha-beta 미분 | timestamp가 맞는 inertial LOS 벡터의 회전율 |
| 기체 회전 보상 | 없음 | PX4 attitude 또는 gyro body-rate 사용 |
| PN 식 | $K_N\dot e+K_Ce$ | $NV_c(\omega_{LOS}\times l)$ |
| 출력 단위 | deg/s | m/s² |
| center-holding | 큰 gain으로 항상 중앙 복귀 | 사용하지 않음(후속 FOV constraint 후보) |
| 중력 | 고정 throttle에 암묵적으로 맡김 | $f_{des}=a_{guidance}-g$ |
| 자세 변환 | 픽셀오차 → body-rate 직결 | desired acceleration → attitude + thrust |
| PX4 연결 | CRSF ACRO rate 직결 | desired roll/pitch → CRSF → PX4 Stabilized attitude controller |
| terminal miss | 화면오차 중심 | 전용 로직은 후속 구현 |

---

## 10. 단계별 구현 계획

### Phase 0 — 계측 추가

제어를 바꾸기 전에 다음 로그를 추가한다.

- raw/filtered bbox center
- image timestamp, control timestamp, attitude interpolation age
- $\mathbf l_C,\mathbf l_B,\mathbf l_W$
- estimated/ground-truth $\boldsymbol\omega_{LOS}$
- $V_c,t_{go},\tau$
- PN, FOV 각 가속도 항
- gravity compensation 전·후 벡터
- desired/actual attitude와 body-rate
- desired/actual thrust 또는 vertical acceleration
- clamp/포화 플래그

### Phase 1 — inertial LOS estimator, shadow mode

새 estimator를 제어 출력에 연결하지 않고 계산과 로그만 수행한다.

1. `/arms/camera_info` 구독
2. bbox → undistorted $\mathbf l_C$
3. camera extrinsic 적용
4. timestamp에 맞는 PX4 attitude로 $\mathbf l_W$ 계산
5. $\boldsymbol\omega_{LOS}$ 추정과 필터링
6. Gazebo ground truth와 비교

### Phase 2 — acceleration guidance, shadow mode

추정한 LOS-rate와 SITL ground-truth 거리/접근속도로 $\mathbf a_{PN}$을 계산하되 기존 제어를
유지한다. 명령 크기, 방향, 포화율을 검증한다.

### Phase 3 — gravity compensation과 attitude/thrust 변환

정지 풍선부터 다음 순서로 연결한다.

1. $a_\parallel\mathbf l$만 사용
2. gravity compensation 적용
3. desired attitude/thrust 생성
4. 낮은 tilt와 acceleration limit에서 비행
5. PN 횡가속도 추가

### Phase 4 — PN 폐루프

- 정지 표적에서 $N=3$부터 시작
- 작은 초기 offset과 횡속도를 주어 LOS-rate 수렴 확인
- $N$을 올릴 때 miss distance, acceleration, 포화율을 같이 비교
- center gain으로 결과를 덮지 않고 FOV constraint를 별도 튜닝

### Phase 5 — moving target 및 실기체 전환

- 일정 속도 표적
- 가속하는 표적
- 검출 지연과 frame drop
- 카메라 진동 및 장착오차
- 바람과 thrust model 오차
- SITL 입력을 하나씩 실기체 센서 추정값으로 교체

---

## 11. SITL 검증 절차

### 11.1 Bearing 검증

드론 자세를 고정하고 풍선을 알려진 영상 위치에 둔다. 추정 $\mathbf l_C$와 Gazebo 상대
위치를 카메라 frame으로 변환한 값의 각도 오차를 계산한다.

```math
e_l=\cos^{-1}
\left(
\operatorname{clip}(\mathbf l_{est}^T\mathbf l_{GT},-1,1)
\right)
```

### 11.2 기체 회전 제거 시험

드론 위치와 풍선을 고정하고 roll/pitch/yaw만 움직인다.

- image bearing은 움직여야 함
- inertial LOS는 일정해야 함
- inertial LOS-rate 평균은 0에 가까워야 함

이 시험이 실패하면 PN 비행시험으로 넘어가지 않는다.

### 11.3 횡이동 시험

자세를 고정하고 드론을 좌우 또는 상하로 이동한다. 추정값을

```math
\boldsymbol\omega_{LOS,GT}
=\frac{\mathbf r\times\mathbf v_r}{R^2}
```

와 비교한다. 부호, 축, 진폭, 지연을 각각 확인한다.

### 11.4 중력 보상 시험

정지 표적을 향한 동일한 접근을 다음 조건으로 반복한다.

1. 중력 보상 없음
2. scalar tilt compensation
3. vector gravity compensation

고도, 수직속도, 풍선의 elevation bearing, LOS-rate, 실제 miss를 비교한다.

### 11.5 PN 시험

각 시험에서 다음 초기조건을 고정한다.

- 시작 위치와 자세
- 표적 위치
- 초기 속도
- thrust 및 tilt 한계
- 검출/제어 주파수

$N$만 3, 4, 5로 바꾸어 다음을 비교한다.

- 최종 miss distance
- LOS-rate 수렴시간
- 최대/누적 가속도
- tilt 및 thrust 포화시간
- 표적 FOV 유지율

### 11.6 지연 민감도 시험

영상 지연을 0, 20, 50, 100 ms로 인위적으로 추가하고 miss 변화를 측정한다. frame drop과
검출 유실도 별도로 주입한다. PN gain은 지연을 감추는 방향으로 다시 튜닝하지 않고 동일하게
유지한다.

---

## 12. 필수 로그와 성공 기준

### 12.1 권장 토픽 또는 rosbag 필드

```text
header.stamp
bbox_raw, bbox_filtered, tracker_state
los_camera[3], los_body[3], los_world[3]
los_rate_est[3], los_rate_gt[3]
range, closing_velocity, t_go, tau
a_pn[3], a_fov[3], a_guidance[3]
gravity[3], specific_thrust_des[3]
attitude_des, attitude_actual
body_rate_des, body_rate_actual
thrust_des, throttle_cmd
accel_saturated, tilt_saturated, rate_saturated, thrust_saturated
miss_distance, miss_vertical, miss_horizontal
```

### 12.2 SITL ground-truth 비교와 제어 입력 전환

`balloon_referee`는 Gazebo의 드론·표적 pose로 `/arms/guidance_ground_truth`를 발행한다.
이 토픽에는 NED 기준 LOS, LOS-rate, 거리와 signed closing speed가 들어 있다.

다음 두 파라미터는 서로 독립적이다.

```yaml
control:
  pn:
    use_ground_truth_los_rate: false
    use_ground_truth_closing_speed: false
```

- `use_ground_truth_los_rate=true`: PN의 $\dot{\mathbf l}$만 GT로 대체
- `use_ground_truth_closing_speed=true`: PN의 $V_c$만 GT로 대체
- 둘 다 `true`: bbox bearing은 유지하고 PN 운동학 입력 두 개를 모두 GT로 대체
- 둘 다 `false`: 실제 구현 경로인 KF LOS-rate와 bbox looming $V_c$ 사용

GT가 유효하지 않거나 `ground_truth_timeout_sec`보다 오래됐을 때는 추정값으로 몰래
fallback하지 않고 PN measurement 갱신을 보류한다. 사용된 값은 기존 `los_rate_*`,
`closing_speed_mps`에 기록되고, 비교용 참값은 `ground_truth_*` 필드에 항상 함께 기록된다.

### 12.3 estimator 성공 기준

- 제자리 body rotation에서 inertial LOS-rate bias가 충분히 작음
- 횡이동 시 ground-truth LOS-rate와 부호 일치
- 유효 비행 구간 RMS 오차와 시간지연이 요구 miss distance를 만족
- 재획득 순간 rate spike가 acceleration clamp를 지속적으로 때리지 않음

### 12.4 guidance 성공 기준

- 정지 표적 반복시험에서 miss의 평균과 분산이 감소
- 화면 중앙 오차뿐 아니라 LOS-rate와 실제 miss가 함께 감소
- 명중이 gain 포화나 과도한 hit radius에 의존하지 않음
- 중력 보상 후 반복되는 수직 miss bias가 사라짐
- moving target에서도 동일한 $N$의 물리적 의미가 유지됨

---

## 13. 참고 자료

- Johns Hopkins APL, [STANDARD Missile: Guidance System Development](https://secwww.jhuapl.edu/techdigest/content/techdigest/pdf/V02-N04/02-04-Witte.pdf) — pursuit, constant-bearing course, PN, stabilized seeker LOS-rate
- Johns Hopkins APL, [Guidance and Control for Homing Missiles](https://www.jhuapl.edu/spsa/PDF-SPSA/Palumbo_TechDig04.pdf) — guidance filter, guidance law, autopilot의 계층 구조
- Johns Hopkins APL, [Overview of Missile Flight Control Systems](https://www.jhuapl.edu/Content/techdigest/pdf/V29-N01/29-01-Jackson.pdf) — autopilot과 airframe/actuator 제어
- NASA NTRS, [Spacecraft Intercept Guidance Controllers](https://ntrs.nasa.gov/api/citations/20170001430/downloads/20170001430.pdf) — 3차원 PN, closing velocity, LOS-rate
- NASA NTRS, [Six-Degree-of-Freedom Missile Simulation](https://ntrs.nasa.gov/api/citations/19900013752/downloads/19900013752.pdf) — seeker, LOS, guidance, autopilot, actuator, 중력을 포함한 6-DOF 구성
