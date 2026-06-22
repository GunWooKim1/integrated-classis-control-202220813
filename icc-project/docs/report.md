

**과목**: 자동제어 — 2026 봄
**제출일**: 2026-05-29
**팀**: 개인

---

## 1. 설계 개요 (1 페이지)


본 프로젝트의 목표는 차량의 횡방향(lateral), 종방향(longitudinal), 수직방향(vertical) 거동을 통합적으로 제어하는 Integrated Chassis Control (ICC) 시스템을 설계하여 차량의 안정성 및 주행 성능을 향상시키는 것이다. 제공된 14DOF 차량 모델과 ISO 기반 시험 시나리오를 이용하여 Active Front Steering (AFS), Electronic Stability Control (ESC), Anti-lock Braking System (ABS), Continuous Damping Control (CDC)을 구현하고, 기준 제어기 대비 성능 향상을 정량적으로 검증하였다.

본 설계에서는 강의에서 다룬 PID 제어기와 Skyhook 기반 감쇠 제어를 중심으로 비교적 단순하면서도 튜닝이 용이한 구조를 채택하였다. 횡방향 제어기에는 PID 기반 yaw rate 추종 제어와 slip angle 제한을 위한 ESC 보조 제어를 적용하였고, 종방향 제어기에는 PI 속도 제어기와 간단한 ABS 기능을 추가하였다. 수직방향 제어기에는 Skyhook 개념을 이용한 반능동 감쇠 제어를 적용하였으며, 최종적으로 Coordinator를 통해 조향각, 브레이크 토크, 감쇠 계수를 각 액추에이터에 분배하였다.

각 제어기의 역할은 다음과 같다.

- ctrl_lateral : PID 기반 yaw rate 추종 제어 + slip angle 기반 ESC yaw moment 생성
    
- ctrl_longitudinal : PI 기반 속도 추종 제어 + brake ratio 기반 pseudo ABS
    
- ctrl_vertical : Skyhook 기반 Continuous Damping Control
    
- ctrl_coordinator : yaw moment를 차동 제동 토크로 변환하고 조향 및 감쇠 명령을 통합 분배
    

제어기 설계 과정에서는 차량의 안정성 확보를 최우선 목표로 설정하였다. 특히 ISO 7401 Step Steer, ISO 3888 Double Lane Change, Brake-in-Turn 시나리오에서 side slip angle과 Load Transfer Ratio(LTR)를 감소시키는 방향으로 제어기를 튜닝하였다.

---

## 2. 수학적 모델링

### 2.1 사용한 Plant 단순화

본 프로젝트에서는 제어기 설계를 위해 Bicycle Model을 사용하였다. Bicycle Model은 차량의 좌우 바퀴를 각각 하나의 등가 바퀴로 근사하여 횡방향 거동을 표현하는 대표적인 차량 모델이다. 실제 차량은 차체 운동, 서스펜션 운동, 타이어 동역학 등을 포함하는 고차 시스템이지만, 제어기 설계 단계에서는 yaw rate와 side slip angle의 거동을 효과적으로 분석하기 위해 단순화된 Bicycle Model을 사용하였다.

제어기 튜닝 및 설계는 Bicycle Model 기반으로 수행하였으며, 최종 성능 평가는 과제에서 제공된 14DOF 차량 모델을 이용하여 검증하였다. 이를 통해 단순 모델 기반 설계의 효율성과 고차 모델 기반 검증의 현실성을 동시에 확보하였다.

### 2.2 State-Space 표현

Bicycle Model의 상태 변수는 차량의 횡속도(vy)와 yaw rate(r)로 정의하였다.

x = [vy, r]^T

입력은 전륜 조향각(delta)로 정의하였다.

u = delta

선형 타이어 가정을 적용하면 차량의 횡방향 운동은 다음과 같이 표현할 수 있다.

vy_dot =  
-(Cf + Cr)/(mVx) * vy

- ((lrCr - lfCf)/(mVx) - Vx) * r
    
- (Cf/m) * delta
    

r_dot =  
((lrCr - lfCf)/(IzVx)) * vy

- ((lf²Cf + lr²Cr)/(IzVx)) * r
    

- (lfCf/Iz) * delta
    

여기서

- Cf : 전륜 코너링 강성
    
- Cr : 후륜 코너링 강성
    
- lf : 무게중심에서 전륜까지의 거리
    
- lr : 무게중심에서 후륜까지의 거리
    
- m : 차량 질량
    
- Iz : 차량의 yaw 관성모멘트
    
- Vx : 종방향 속도
    

를 의미한다.

본 프로젝트에서는 위 모델을 이용하여 yaw rate 추종 성능과 차량 안정성을 분석하고 PID 기반 횡방향 제어기를 설계하였다.

### 2.3 가정 및 한계

본 설계에서는 다음과 같은 가정을 적용하였다.

- 제어기 설계 시 종방향 속도는 일정하다고 가정하였다.
    
- 타이어는 선형 코너링 강성을 갖는다고 가정하였다.
    
- 작은 slip angle 영역에서 차량이 동작한다고 가정하였다.
    
- 횡방향, 종방향, 수직방향 제어기를 개별적으로 설계한 후 통합하였다.
    

이러한 가정은 제어기 설계를 단순화하고 튜닝을 용이하게 만드는 장점이 있다. 그러나 큰 slip angle이 발생하거나 타이어가 비선형 영역에 진입하는 극한 주행 상황에서는 모델 오차가 발생할 수 있다. 따라서 최종 성능은 제공된 14DOF 차량 모델에서 검증하여 실제 차량 거동과의 차이를 최소화하였다.

---

## 3. 제어기 설계 (3-4 페이지)

### 3.1 ctrl_lateral — AFS + ESC

#### 설계 목표

횡방향 제어기의 목표는 차량의 yaw rate를 기준 yaw rate에 빠르게 추종시키면서 과도한 side slip angle 발생을 방지하는 것이다. 이를 위해 Active Front Steering(AFS)을 이용한 yaw rate 추종 제어와 Electronic Stability Control(ESC)을 이용한 안정화 제어를 결합하였다.

설계 목표는 다음과 같이 설정하였다.

- 빠른 yaw rate 응답
    
- 작은 overshoot 유지
    
- 차량의 side slip angle 제한
    
- Brake-in-Turn 상황에서 스핀 방지
    

#### 선택 기법

Yaw rate 추종에는 PID 제어기를 사용하였다. PID 제어기는 구현이 간단하고 강의에서 학습한 대표적인 피드백 제어기이며, 파라미터 튜닝을 통해 응답 속도와 안정성을 동시에 확보할 수 있다.

ESC는 slip angle 기반 안정화 제어 방식으로 구현하였다. 차량의 slip angle이 임계값을 초과하면 추가 yaw moment를 발생시켜 차량 자세를 안정화하였다.

#### 제어 구조

Yaw rate 오차는 다음과 같이 정의하였다.

e_r = r_ref - r

여기서 r_ref는 기준 yaw rate이고 r은 측정 yaw rate이다.

PID 제어 입력은 다음과 같이 계산하였다.

delta_cmd =  
Kp * e_r

- Ki * integral(e_r)
    
- Kd * derivative(e_r)
    

적분항에는 anti-windup 제한을 적용하여 적분기 포화를 방지하였다.

Slip angle이 설정된 임계값을 초과할 경우 ESC가 동작하도록 하였다.

if |beta| > beta_threshold

yawMoment = -K_beta * sign(beta) * beta_error

end

본 설계에서는 beta_threshold를 최대 허용 slip angle의 1.2배로 설정하였다.

#### 최종 게인

최종 튜닝 결과 사용된 파라미터는 다음과 같다.

Kp = (최종값 입력)

Ki = (최종값 입력)

Kd = (최종값 입력)

ESC Gain = 40

Yaw Moment Saturation = ±2000 N·m

#### 설계 결과

제안한 PID 기반 AFS와 ESC를 적용한 결과 Step Steer(A3) 및 Brake-in-Turn(A7) 시나리오에서 우수한 성능을 확인하였다. 특히 A7 시나리오에서 side slip angle이 크게 감소하여 차량의 스핀 경향을 효과적으로 억제할 수 있었다.


### 3.2 ctrl_longitudinal — 속도 + ABS

#### 설계 목표

종방향 제어기의 목표는 기준 속도를 추종하면서 과도한 제동 시 바퀴 잠김(wheel lock)을 방지하는 것이다. 특히 급제동 상황에서 제동력을 적절히 분배하여 차량 안정성을 유지하고자 하였다.

#### 선택 기법

속도 추종에는 PI(Proportional-Integral) 제어기를 사용하였다. 차량의 속도 오차를 이용하여 종방향 힘(Fx)을 계산하고, 이를 기반으로 차량의 가감속을 제어하였다.

또한 실제 ABS와 유사한 효과를 얻기 위하여 제동 요구량에 비례하여 브레이크 토크를 감소시키는 pseudo ABS 알고리즘을 추가하였다.

#### 제어 구조

속도 오차는 다음과 같이 정의하였다.

e_v = V_ref - V

여기서 V_ref는 기준 속도이고 V는 차량의 실제 속도이다.

PI 제어기는 다음과 같이 구성하였다.

Fx_cmd =  
Kp * e_v  
+  
Ki * integral(e_v)

적분항에는 anti-windup 제한을 적용하여 적분기 포화를 방지하였다.

계산된 종방향 힘은 차량의 최대 가감속 한계를 이용하여 포화시켰다.

이후 제동 명령이 발생할 경우 제동 요구량에 비례하여 brakeRatio를 생성하였다.

brakeRatio =  
min(0.55 × brakeDemand, 0.55)

생성된 brakeRatio는 Coordinator 단계에서 전체 브레이크 토크를 감소시키는 방식으로 사용하였다.

#### 최종 게인

최종적으로 사용한 PI 제어기 파라미터는 다음과 같다.

Kp = 3200

Ki = 650

Integral Limit = ±8

Maximum Brake Ratio = 0.55

#### 설계 결과

제안한 PI 기반 종방향 제어기는 일반적인 가감속 상황에서 안정적인 속도 추종 성능을 보였다. 또한 pseudo ABS를 통해 급제동 시 과도한 제동 토크를 일부 완화하여 차량 안정성을 개선하고자 하였다.

그러나 제공된 KPI 기준이 매우 엄격하였기 때문에 B1 시나리오에서는 충분한 성능 향상을 달성하지 못하였다. 이는 실제 ABS와 같은 휠 슬립 피드백 제어가 구현되지 않았기 때문으로 판단된다.

### 3.3 ctrl_vertical — CDC (Continuous Damping Control)

#### 설계 목표

수직방향 제어기의 목표는 차량의 승차감(ride comfort)과 주행 안정성을 동시에 향상시키는 것이다. 특히 차체의 상하 운동(body bounce)과 롤 운동(roll motion)을 억제하여 승차감을 개선하고, 급조향 시 차량의 자세 변화를 감소시키고자 하였다.

#### 선택 기법

수직방향 제어기에는 Skyhook 기반 Continuous Damping Control(CDC)을 적용하였다. Skyhook 제어는 반능동 서스펜션 분야에서 널리 사용되는 기법으로, 차체가 가상의 고정점(sky)에 연결되어 있다고 가정하여 차체 운동을 감쇠시키는 방식이다.

Skyhook 제어는 구현이 비교적 간단하면서도 차체 진동 감소 효과가 우수하므로 본 프로젝트에 적합하다고 판단하였다.

#### 제어 구조

각 바퀴에 대해 차체 속도와 휠 속도를 측정하여 상대 속도를 계산하였다.

v_rel = zs_dot - zu_dot

여기서

- zs_dot : 차체 수직 속도
    
- zu_dot : 휠 수직 속도
    
- v_rel : 서스펜션 상대 속도
    

를 의미한다.

Skyhook 조건은 다음과 같이 정의하였다.

(zs_dot × v_rel) > 0

위 조건이 만족되면 현재 댐퍼가 차체 운동을 효과적으로 감쇠시킬 수 있는 상태라고 판단하였다.

이 경우 감쇠 계수는 다음과 같이 계산하였다.

c_target =  
cMin +  
rollGain × (cMax − cMin)

여기서 rollGain은 차체 속도 크기에 비례하여 증가하도록 설계하였다.

rollGain =  
min(|zs_dot| / 0.12, 1)

즉 차체 운동이 클수록 더 큰 감쇠력을 발생시키고, 차체 운동이 작을 경우에는 낮은 감쇠력을 유지하도록 하였다.

계산된 감쇠 계수는 다음 범위 내에서 제한하였다.

cMin ≤ c_target ≤ cMax

#### 설계 결과

제안한 Skyhook 기반 CDC는 차량의 롤 운동 및 차체 진동을 감소시키는 역할을 수행하였다. 특히 Double Lane Change(A1)와 Brake-in-Turn(A7) 시나리오에서 차량의 Load Transfer Ratio(LTR)를 감소시키는 데 기여하였다.

다만 본 프로젝트에서는 CDC 단독 성능보다 AFS 및 ESC를 포함한 통합 제어 성능이 전체 KPI에 더 큰 영향을 미쳤다. 따라서 CDC는 차량 안정성을 보조하는 역할로 활용하였다.

### 3.4 ctrl_coordinator — Actuator Allocation

#### 설계 목표

Coordinator의 목표는 횡방향, 종방향, 수직방향 제어기에서 생성된 명령을 실제 차량 액추에이터 명령으로 변환하는 것이다. 본 프로젝트에서는 조향각, 브레이크 토크, 감쇠 계수를 통합적으로 관리하여 차량의 안정성과 제어 성능을 향상시키고자 하였다.

#### 제어 구조

Coordinator는 다음과 같은 세 가지 기능을 수행한다.

1. 조향 명령 제한(Steering Saturation)
    
2. 제동 토크 분배(Brake Torque Allocation)
    
3. ESC 차동 제동(Differential Braking)
    

먼저 횡방향 제어기에서 생성된 조향각 명령은 차량의 물리적 한계를 고려하여 제한하였다.

steerAngle =  
saturate(delta_cmd)

이를 통해 비현실적인 조향 명령 발생을 방지하였다.

#### 기본 제동 토크 분배

종방향 제어기에서 생성된 종방향 힘(Fx_total)을 이용하여 전체 제동 토크를 계산하였다.

T_total = |Fx_total| × r_wheel

여기서 r_wheel은 바퀴 반지름이다.

차량의 일반적인 제동 특성을 반영하기 위해 전륜과 후륜에 다음과 같이 제동 토크를 분배하였다.

Front : Rear = 60 : 40

즉,

- Front Left : 30%
    
- Front Right : 30%
    
- Rear Left : 20%
    
- Rear Right : 20%
    

의 비율로 제동 토크를 배분하였다.

전륜에 더 큰 제동력을 배분한 이유는 제동 시 발생하는 동적 하중 이동(dynamic load transfer)을 고려하기 위함이다.

#### Pseudo ABS 연동

종방향 제어기에서 계산된 brakeRatio를 이용하여 전체 브레이크 토크를 조절하였다.

T_total =  
(1 − brakeRatio) × T_total

이를 통해 과도한 제동 시 바퀴 잠김 현상을 완화하고자 하였다.

#### ESC 차동 제동

횡방향 제어기에서 생성된 yaw moment 명령은 좌우 바퀴의 차동 제동 토크로 변환하였다.

ΔT = K_ESC × Mz

여기서

- Mz : ESC yaw moment
    
- K_ESC : 차동 제동 변환 게인
    

이다.

좌우 바퀴에는 다음과 같이 반대 방향의 토크를 부여하였다.

T_FL = −ΔT

T_FR = +ΔT

T_RL = −ΔT

T_RR = +ΔT

이를 통해 차량에 추가 yaw moment를 발생시켜 과도한 오버스티어 또는 언더스티어를 억제하였다.

#### 설계 결과

제안한 Actuator Allocation 구조는 횡방향, 종방향, 수직방향 제어기의 명령을 효과적으로 통합하였다. 특히 ESC 기반 차동 제동은 Brake-in-Turn(A7) 시나리오에서 차량의 side slip angle과 LTR을 크게 감소시키는 데 기여하였다.

또한 전후륜 제동 분배와 브레이크 토크 제한을 통해 안정적인 제동 성능을 확보하였다. 이를 통해 개별 제어기의 성능뿐만 아니라 통합 샤시 제어 시스템으로서의 동작을 구현할 수 있었다.

---

## 4. 시뮬레이션 결과 (2-3 페이지)

### 4.1 P1 시나리오 benchmark — 베이스라인 vs 본인 설계

| 시나리오     | KPI                  | OFF   | ON (본인) | Δ%     |
| -------- | -------------------- | ----- | ------- | ------ |
| A1 DLC   | sideSlipMax [°]      | 4.51  | 2.91    | -35.5% |
| A1       | LTR_max              | 0.948 | 0.772   | -18.6% |
| A3 step  | yawRateOvershoot [%] | 2.81  | 2.39    | -14.9% |
| A4 SS    | understeerGradient   | --    | 0.00075 | --     |
| A7 BIT   | sideSlipMax [°]      | 46.3  | 2.24    | -95.2% |
| A7       | LTR_max              | 0.745 | 0.348   | -53.3% |
| B1 brake | stoppingDistance [m] | 72.4  | 72.3    | -0.1%  |
| D1 통합    | sideSlipMax [°]      | 7.65  | 3.61    | -52.8% |
Auto Grade Result : 51.88 / 70

결과를 살펴보면 A7 Brake-in-Turn 시나리오에서 가장 큰 개선 효과가 나타났다. Side slip angle은 약 95% 감소하였고, LTR 역시 약 53% 감소하여 차량의 횡방향 안정성이 크게 향상되었다. 또한 A1 Double Lane Change 시나리오에서도 side slip angle과 LTR이 감소하여 차선 변경 상황에서의 안정성이 개선되었다. 반면 B1 제동 시나리오에서는 stopping distance 개선이 거의 나타나지 않았으며, 이는 wheel slip 피드백을 이용한 ABS 제어가 구현되지 않았기 때문으로 판단된다.
### 4.2 핵심 plot — A1 DLC

![[figure1.png]]
*Figure 4.1 — A1 ISO 3888-1 DLC, 차량 trajectory (off vs on) vs reference path.*

![[figure2.png]]
*Figure 4.2 — A1 yaw rate 응답: reference (driver bicycle model), off (controller off), on (본인 설계).*

Figure 4.1은 ISO 3888-1 Double Lane Change(A1) 시나리오에서의 차량 궤적을 나타낸다. Controller OFF 상태에서는 기준 경로를 추종하는 과정에서 상대적으로 큰 횡방향 편차가 발생하였다. 반면 제안한 ICC 제어기를 적용한 경우 차량이 기준 경로를 보다 안정적으로 추종하는 것을 확인할 수 있었다.

Figure 4.2는 동일 시나리오에서의 yaw rate 응답을 나타낸다. Controller OFF 상태에서는 기준 yaw rate를 추종하는 과정에서 진동이 발생하였으며, 응답 지연 또한 관찰되었다. 반면 제안한 PID 기반 AFS 제어기를 적용한 경우 기준 yaw rate에 대한 추종 성능이 향상되었고 응답 진동이 감소하였다.

또한 ESC는 slip angle이 증가하는 구간에서 추가 yaw moment를 생성하여 차량 자세를 안정화하였다. 그 결과 side slip angle은 4.51°에서 2.91°로 감소하였고, LTR은 0.948에서 0.772로 감소하였다. 이는 제안한 통합 제어기가 급격한 차선 변경 상황에서 차량의 횡방향 안정성을 향상시켰음을 의미한다.

### 4.3 한 시나리오 Deep Dive — A7 Brake-in-Turn

A7 Brake-in-Turn 시나리오는 차량이 선회 중 제동을 수행하는 상황을 모사한다. 이 상황에서는 차량에 큰 횡력과 종력이 동시에 작용하므로 차량이 쉽게 오버스티어 상태에 진입하거나 스핀아웃이 발생할 수 있다. 따라서 ESC의 효과를 평가하기에 적합한 시나리오이다.

베이스라인 차량은 Brake-in-Turn 상황에서 side slip angle이 크게 증가하여 불안정한 거동을 나타냈다. 특히 최대 side slip angle은 46.3°까지 증가하여 차량이 사실상 스핀아웃 상태에 가까운 거동을 보였다.

반면 제안한 ICC 제어기를 적용한 경우 최대 side slip angle은 2.24°로 감소하였다. 이는 약 95.2%의 감소에 해당하며 차량 자세 안정성이 크게 향상되었음을 의미한다.

또한 LTR은 0.745에서 0.348로 감소하였다. 이는 제동 중 발생하는 횡하중 이동이 감소하였음을 의미하며 차량 전복 위험성을 낮추는 효과를 보여준다.

이러한 성능 향상의 주요 원인은 ESC 기반 yaw moment 제어에 있다. 본 설계에서는 slip angle이 설정된 임계값을 초과할 경우 ESC가 추가 yaw moment를 생성하도록 설계하였다. 생성된 yaw moment는 Coordinator를 통해 좌우 바퀴의 차동 제동 토크로 변환되며, 이를 통해 차량의 과도한 회전 운동을 억제하였다.

실험 결과 A7 시나리오에서 ESC는 차량이 불안정 영역으로 진입하기 전에 개입하여 side slip angle의 급격한 증가를 방지하였다. 따라서 Brake-in-Turn과 같은 극한 상황에서 제안한 통합 샤시 제어기가 차량 안정성 향상에 효과적임을 확인할 수 있었다.

## 5. 분석 + 한계

### 5.1 가장 성공적이었던 시나리오

가장 큰 성능 향상을 보인 시나리오는 A7 Brake-in-Turn이었다. 해당 시나리오에서 side slip angle은 46.3°에서 2.24°로 감소하였으며, LTR은 0.745에서 0.348로 감소하였다. 이는 각각 약 95.2%, 53.3%의 개선에 해당한다.

이러한 성능 향상의 주요 원인은 ESC 기반 yaw moment 제어에 있다. 본 설계에서는 차량의 slip angle이 임계값을 초과할 경우 ESC가 개입하여 추가 yaw moment를 생성하도록 하였다. 생성된 yaw moment는 차동 제동을 통해 차량에 전달되며, 과도한 오버스티어를 억제하는 역할을 수행하였다.

또한 Coordinator에서 적용한 전후륜 60:40 제동 분배 역시 제동 중 차량 안정성 향상에 기여하였다. 결과적으로 Brake-in-Turn과 같은 극한 상황에서 제안한 통합 샤시 제어기의 효과를 가장 명확하게 확인할 수 있었다.

### 5.2 가장 부족했던 시나리오

가장 개선이 부족했던 시나리오는 B1 Brake 시나리오였다. Stopping Distance는 72.4 m에서 72.3 m로 거의 변화가 없었으며, ABS 관련 KPI 역시 큰 향상을 보이지 못하였다.

가능한 원인은 다음과 같다.

- 가설 1: 본 설계에서 구현한 ABS는 wheel slip을 직접 측정하는 폐루프 제어기가 아니라 brake demand 기반의 pseudo ABS 방식이다.
    
- 가설 2: 타이어 슬립률을 직접 피드백하지 않기 때문에 최적 슬립 영역을 유지하지 못하였다.
    
- 가설 3: 종방향 제어기의 목적이 속도 추종에 집중되어 있어 제동 성능 최적화에는 한계가 존재하였다.
    

향후에는 각 휠의 slip ratio를 직접 계산하고 이를 이용한 실제 ABS 제어기를 설계한다면 B1 시나리오의 성능을 추가적으로 개선할 수 있을 것으로 판단된다.

### 5.3 만약 더 시간이 있었다면

본 프로젝트에서는 PID 기반 AFS, pseudo ABS, Skyhook CDC 및 차동 제동 기반 ESC를 구현하였다. 제한된 시간 내에서 통합 제어기의 기본 구조를 완성하고 성능을 개선하는 데 집중하였다.

추가적인 개발 시간이 주어진다면 다음과 같은 개선을 시도할 수 있을 것이다.

- Wheel slip ratio 기반 ABS 제어기 구현
    
- 속도에 따른 Gain Scheduling 적용
    
- Bicycle Model 기반 LQR 횡방향 제어기 설계
    
- CDC와 ESC 간의 연계 제어 전략 개발
    
- 다양한 노면 조건(wet, snow 등)에 대한 적응형 제어기 적용
    

특히 LQR 기반 횡방향 제어기와 실제 ABS 제어기를 적용할 경우 A1, B1 및 D1 시나리오에서 추가적인 KPI 개선이 가능할 것으로 예상된다.

## 6. 참고문헌

[1] ISO 3888-1:2018, Passenger cars — Test track for a severe lane-change manoeuvre.

[2] ISO 4138:2021, Passenger cars — Steady-state circular driving behaviour.

[3] Rajamani, R., Vehicle Dynamics and Control, 2nd Edition, Springer, 2012.

[4] Wong, J. Y., Theory of Ground Vehicles, 4th Edition, Wiley, 2008.

[5] Automatic Control Lecture Notes, Ajou University, Spring 2026.

[6] MATLAB Documentation, MathWorks.

## 부록 A — 사용한 AI 도구

본 프로젝트에서는 ChatGPT를 활용하여 제어기 구조 설계, PID 게인 초기 추정, MATLAB 코드 디버깅 및 보고서 초안 작성을 지원받았다.

초기 제안된 제어기 구조와 게인은 참고용으로만 사용하였으며, 최종 파라미터는 반복적인 시뮬레이션과 성능 비교를 통해 직접 조정하였다.

특히 횡방향 PID 게인, ESC 개입 조건, ABS brakeRatio 및 CDC 파라미터 튜닝 과정에서 AI를 참고하였으나 최종 성능 검증은 MATLAB 시뮬레이션을 통해 수행하였다.

## 부록 B — 본인 sim_params.m 변경사항

### 횡방향 제어기 (Lateral)

변경 전

```matlab
CTRL.LAT.Kp     = 1.0;
CTRL.LAT.Ki     = 0.1;
CTRL.LAT.Kd     = 0.05;
CTRL.LAT.intMax = 5.0;
```

변경 후

```matlab
CTRL.LAT.Kp     = 0.30;
CTRL.LAT.Ki     = 0;
CTRL.LAT.Kd     = 0.006;
CTRL.LAT.intMax = 0.2;
```

목표 yaw rate 추종 성능과 차량 안정성 간의 균형을 고려하여 게인을 조정하였다. 적분항은 제거하여 응답 진동을 감소시켰으며, PD 형태에 가까운 구조를 사용하였다.

### 수직방향 제어기 (CDC)

최종 사용값

```matlab
CTRL.VER.cMin    = 500;
CTRL.VER.cMax    = 5000;
CTRL.VER.skyGain = 2500;
```

Skyhook 기반 CDC의 감쇠 범위를 설정하여 차체 진동 및 롤 운동을 억제하였다.

### 액추에이터 제한값

최종 사용값

```matlab
LIM.MAX_STEER_ANGLE = 0.30;
LIM.MAX_STEER_RATE  = 2.5;
LIM.MAX_BRAKE_TRQ   = 3000;
LIM.MAX_SLIP_ANGLE  = deg2rad(12);
```

조향각, 조향속도, 브레이크 토크 및 슬립각 제한값을 이용하여 액추에이터 포화 및 비현실적인 제어 입력 발생을 방지하였다.

### 시뮬레이션 설정

최종 사용값

```matlab
SIM.solver     = 'rk4';
SIM.plantModel = 'bicycle';
```

제어기 설계 및 튜닝 과정에서는 계산 속도와 반복 실험 효율을 고려하여 Bicycle Model과 RK4 적분기를 사용하였다.