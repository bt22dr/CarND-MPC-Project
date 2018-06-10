# MPC Model
* 모델예측제어(MPC, Model Predictive Control): 
  * 적절한 cost function과 constraints를 이용해 최적화를 실행하여 자동차의 vehicle 모델에 대한 상태변수나 출력을 예측하는 과정

## state & actuators
이 프로젝트에서 사용한 state와 actuator 정보는 아래와 같다. 
* state: [x, y, psi, velocity, cross_trac_error, psi_error]
* actuators: [steering_angle, acceleration]

## update equations
state update는 아래와 같은 코드로 구현하였다. 
```
x_t+1 = x_t + v_t * cos(psi_t) * dt
y_t+1 = y_t + v_t * sin(psi_t) * dt
psi_t+1 = psi_t + v_t / Lf * delta_t * dt
v_t+1 = v_t + a_t * dt
cte_t+1 = f(x_t) - y_t + v_t * sin(epsi_t) * dt
epsi_t+1 = psi_t - psi_des_t + v_t / Lf * delta_t * dt
```

## N and dt
이번 구현에서 주의할 점은 N과 dt를 어떻게 세팅하는가이다. 여기서는 N*dt초 후의 상태를 예측하므로 이 값이 커질수록 더 정확한 컨트롤이 가능하게 된다. 이때 dt를 작게 설정하면 finer resolution을 얻을 수 있고, N을 너무 크게 잡으면 연산량이 너무 많아질 수 있다. 

## Fitting waypoints
global과 local(vehicle) coordination간의 변환은 아래 공식을 사용한다. 
```
for (int i = 0; i < ptsx.size(); i++) {
  double shift_x = ptsx[i] - px; 
  double shift_y = ptsy[i] - py; 
  ptsx[i] = (shift_x * cos(0 - psi) - shift_y * sin(0 - psi));
  ptsy[i] = (shift_x * sin(0 - psi) + shift_y * cos(0 - psi));
}
```
이 변환 후에는 px, py, psi 모두 0의 값을 갖게 된다. 

## latency handling
```
double latency = 0.1; // 100 ms
double Lf = 2.67;
double delta = j[1]["steering_angle"];

// ...

px = px + v*cos(psi)*latency;
py = py + v*sin(psi)*latency; // 0
cte = cte - v*sin(epsi)*latency; // 0
psi = psi - v*delta/Lf*latency;
v = v + a*latency;

Eigen::VectorXd state(6);
state << px, py, psi, v, cte, epsi;
```
