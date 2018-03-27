# MPC Model
* 모델예측제어(MPC, Model Predictive Control): 
 ** 적절한 cost function과 constraints를 이용해 최적화를 실행하여 자동차의 vehicle 모델에 대한 상태변수나 출력을 예측하는 과정

## state & actuators
state: [x, y, psi, velocity, cross_trac_error, psi_error]

actuators: [steering_angle, acceleration]

## update equations
```
x_t+1 = x_t + v_t * cos(psi_t) * dt
y_t+1 = y_t + v_t * sin(psi_t) * dt
psi_t+1 = psi_t + v_t / Lf * delta_t * dt
v_t+1 = v_t + a_t * dt
cte_t+1 = f(x_t) - y_t + v_t * sin(epsi_t) * dt
epsi_t+1 = psi_t - psi_des_t + v_t / Lf * delta_t * dt
```

## N and dt
If the dt is small, you get finer resolution. We predict N*dt seconds in the future, and larger this value more correct controling. However, it is very expensive computationally when the N is too large.

## Fitting waypoints
I transformed coordination from global to local(vehicle). 
```
for (int i = 0; i < ptsx.size(); i++) {
  double shift_x = ptsx[i] - px; 
  double shift_y = ptsy[i] - py; 
  ptsx[i] = (shift_x * cos(0 - psi) - shift_y * sin(0 - psi));
  ptsy[i] = (shift_x * sin(0 - psi) + shift_y * cos(0 - psi));
}
```
After this transformation px, py and psi are all 0.

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
