# PN 유도 별첨: bbox 기반 거리·접근속도 추정

이 문서는 `PN_GUIDANCE.md`의 접근속도 추정부를 보충한다. 현재 구현은 카메라 bbox로부터 순간 거리를 만든 뒤 두 프레임을 직접 차분하지 않고, 거리와 거리 변화율을 함께 추정하는 timestamp-aware Kalman filter를 사용한다.

## 1. 목적

PN의 횡가속도 명령은 다음과 같이 접근속도에 비례한다.

```math
\mathbf a_{PN}=N V_c\dot{\boldsymbol\lambda}
```

따라서 $V_c$가 순간적으로 0이 되거나 포화값까지 튀면 LOS-rate가 정상이어도 PN 이득이 급변한다. GT $V_c$에서 유도가 성공하고 영상 추정 $V_c$에서 실패한다면, LOS-rate나 자세 제어보다 접근속도 추정기를 먼저 검증해야 한다.

## 2. bbox에서 거리 측정값 만들기

정규화 bbox의 폭과 높이를 카메라 초점거리 기준 각크기로 변환한다.

```math
s_x=\frac{w_{norm}W}{f_x},\qquad
s_y=\frac{h_{norm}H}{f_y}
```

구형 표적에서는 두 축의 물리적 지름이 같으므로 기하평균을 사용한다.

```math
s=\sqrt{s_xs_y}
```

기존의 $\max(s_x,s_y)$는 width와 height 중 지배적인 축이 바뀔 때 불연속이 생길 수 있다. 기하평균은 한 축의 작은 bbox 오차가 전체 scale을 독점하지 않게 한다.

알려진 표적 지름 $D$로 순간 거리 측정값을 계산한다.

```math
z_k=R_{raw,k}\approx\frac{D}{s_k}
```

현재 기본값은 $D=0.5\,\mathrm{m}$이다. 이 값이 실제 표적 크기와 다르면 절대 거리와 $V_c$에 같은 비율의 scale bias가 생긴다.

## 3. 기존 직접 차분의 문제

기존 추정기는 연속된 거리 두 개만 사용했다.

```math
V_{c,raw,k}=\frac{R_{raw,k-1}-R_{raw,k}}{\Delta t}
```

bbox 오차를 각각 $n_{k-1}$과 $n_k$라고 하면 속도 오차는 다음과 같다.

```math
\tilde V_{c,k}=\frac{n_{k-1}-n_k}{\Delta t}
```

$\Delta t$가 작을수록 픽셀 단위 bbox 흔들림이 큰 속도 오차로 증폭된다. 차분 뒤 LPF를 적용하면 출력은 부드러워지지만, 이미 증폭된 이상값이 여러 프레임 동안 필터 상태에 남는다.

## 4. 거리·거리변화율 Kalman filter

### 4.1 상태

상태는 거리와 거리 변화율이다.

```math
\mathbf x_k=
\begin{bmatrix}
R_k\\
\dot R_k
\end{bmatrix}
```

접근속도는 거리 변화율의 부호를 반대로 한 값이다.

```math
V_c=-\dot R
```

### 4.2 예측

한 영상 프레임 동안 거리 변화율이 일정하다고 가정한다.

```math
\mathbf x_k^-=\mathbf F_k\mathbf x_{k-1}^+
```

```math
\mathbf F_k=
\begin{bmatrix}
1&\Delta t_k\\
0&1
\end{bmatrix}
```

공분산 예측은 다음과 같다.

```math
\mathbf P_k^-=\mathbf F_k\mathbf P_{k-1}^+\mathbf F_k^T+\mathbf Q_k
```

가속도 잡음을 $\sigma_a$라고 하면 현재 구현의 process noise는 다음과 같다.

```math
\mathbf Q_k=\sigma_a^2
\begin{bmatrix}
\frac{\Delta t_k^4}{4}&\frac{\Delta t_k^3}{2}\\
\frac{\Delta t_k^3}{2}&\Delta t_k^2
\end{bmatrix}
```

### 4.3 측정

필터가 직접 관측하는 값은 bbox 기반 거리 하나뿐이다.

```math
z_k=\mathbf H\mathbf x_k+v_k,qquad
\mathbf H=\begin{bmatrix}1&0\end{bmatrix}
```

같은 픽셀 오차도 원거리에서 더 큰 거리 오차를 만들기 때문에 측정 표준편차를 거리에 비례시킨다.

```math
\sigma_R=\max\left(\sigma_{R,min},\;k_R \hat R_k^-\right)
```

```math
\mathbf R_k=\sigma_R^2
```

기본값은 $k_R=0.04$, $\sigma_{R,min}=0.15\,\mathrm{m}$이다. 측정값 $z_k$가 아니라 갱신 전 예측 거리 $\hat R_k^-$를 사용하므로, 큰 이상 측정이 자신의 측정 잡음과 gate를 동시에 넓힐 수 없다.

### 4.4 Innovation gate

innovation과 그 분산은 다음과 같다.

```math
\nu_k=z_k-\mathbf H\mathbf x_k^-
```

```math
S_k=\mathbf H\mathbf P_k^-\mathbf H^T+\mathbf R_k
```

다음 조건을 벗어난 측정은 bbox 이상값으로 판단해 갱신하지 않는다.

```math
|\nu_k|\le g\sqrt{S_k}
```

기본 gate는 $g=4$이다. 거부된 프레임에서도 시간 예측은 수행하므로 거리와 $V_c$가 직전 값에 고정되지 않고 현재 추세를 따라간다.

### 4.5 갱신

승인된 측정에는 표준 Kalman update를 적용한다.

```math
\mathbf K_k=\mathbf P_k^-\mathbf H^TS_k^{-1}
```

```math
\mathbf x_k^+=\mathbf x_k^-+\mathbf K_k\nu_k
```

```math
\mathbf P_k^+=(\mathbf I-\mathbf K_k\mathbf H)\mathbf P_k^-
```

최종 PN 입력은 물리 상한을 적용한다.

```math
V_c=\operatorname{clamp}\left(-\hat{\dot R},0,V_{c,max}\right)
```

GT override가 활성화된 경우에는 이 추정값 대신 GT $V_c$가 PN에 입력된다. 거리 KF는 계속 실행되므로 같은 로그에서 추정값의 상태를 진단할 수 있다.

## 5. 파라미터

```yaml
control:
  pn:
    target_diameter_m: 0.5
    max_closing_speed_mps: 20.0
    range_kf_accel_noise_mps2: 8.0
    range_kf_measurement_noise_ratio: 0.04
    range_kf_min_measurement_noise_m: 0.15
    range_kf_innovation_gate_sigma: 4.0
```

- `range_kf_accel_noise_mps2`: 크게 하면 실제 가감속을 빨리 추종하지만 $V_c$가 거칠어진다.
- `range_kf_measurement_noise_ratio`: 크게 하면 bbox 거리 측정을 덜 신뢰해 부드러워지지만 지연이 증가한다.
- `range_kf_min_measurement_noise_m`: 근거리에서 필터가 bbox 측정을 과도하게 신뢰하지 않도록 한다.
- `range_kf_innovation_gate_sigma`: 작게 하면 이상값을 더 많이 거부한다. 너무 작으면 실제 급가속도 거부할 수 있다.
- `max_closing_speed_mps`: 필터 튜닝값이 아니라 PN에 들어가는 최종 안전 상한이다.

## 6. `pn_debug` 필드

- `bbox_scale`: 기하평균으로 계산한 관측 각크기 $s$.
- `range_raw_m`: 현재 bbox 하나로 계산한 순간 거리 $R_{raw}$.
- `range_m`: 거리 KF가 추정한 거리 $\hat R$.
- `closing_speed_raw_mps`: 진단용 1-step 거리 차분값.
- `closing_speed_mps`: 실제 PN에 사용된 $V_c$. GT override가 켜져 있으면 GT 값이다.
- `range_innovation_m`: 현재 거리 측정과 KF 예측의 차이 $\nu$.
- `range_measurement_accepted`: innovation gate 통과 여부.
- `ground_truth_range_m`: Gazebo 상대 위치로 계산한 거리.
- `ground_truth_closing_speed_mps`: Gazebo 상대 거리 차분으로 계산한 비교용 $V_c$.

## 7. 로그 판독 순서

먼저 `range_raw_m`, `range_m`, `ground_truth_range_m`을 함께 본다.

1. `range_raw_m`만 흔들리고 `range_m`이 추세를 유지하면 KF가 정상적으로 작동한 것이다.
2. `range_raw_m`이 지속적으로 GT와 다른 비율이면 표적 지름 또는 카메라 intrinsics를 확인한다.
3. `range_measurement_accepted=false`가 연속되면 gate가 너무 작거나 모델의 가속도 잡음이 너무 작다.
4. `range_m`이 GT보다 늦게 반응하면 `range_kf_accel_noise_mps2`를 높이거나 측정 잡음을 낮춘다.

그다음 `closing_speed_raw_mps`, `closing_speed_mps`, `ground_truth_closing_speed_mps`를 비교한다.

1. raw만 튀고 filtered가 GT 추세를 따라가면 정상이다.
2. filtered가 지나치게 늦으면 process noise를 높인다.
3. filtered도 raw를 그대로 따라가면 measurement noise를 높인다.
4. filtered가 자주 0에 붙으면 range-rate 부호와 거리 추세를 확인한다.
5. filtered가 자주 `max_closing_speed_mps`에 붙으면 상한을 먼저 높이지 말고 거리 scale bias와 필터 추종성을 확인한다.

## 8. 검증 기준

추정기 검증과 유도 성능 검증을 분리한다.

```math
e_{V_c}=V_{c,estimate}-V_{c,GT}
```

다음 지표를 한 비행 구간에서 비교한다.

- $V_c$ RMSE와 bias
- gate 거부율
- GT 대비 위상 지연
- PN 가속도 포화 시간 비율
- 최종 최소 접근거리와 명중 여부

GT $V_c$로 성공한 동일 조건에서 추정 $V_c$만 교체해 반복하면, 이후 성능 차이를 접근속도 추정 오차로 한정해서 해석할 수 있다.
