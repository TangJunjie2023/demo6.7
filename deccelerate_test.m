% === 设置环境和车辆 ===
blueprints = world.get_blueprint_library();
model3_list = cell(py.list(blueprints.filter('vehicle.audi.tt')));
bp = model3_list{1};

spawn_points = map.get_spawn_points();
spawn_point = spawn_points{99};

% === 初始化参数 ===
target_decel = 6.0;   % 目标减速度 m/s^2
max_time = 10.0;      % 最大仿真时间
dt = 0.02;             % 控制周期
brake_cmd = 0.1;      % 初始刹车值
velocity_eps = 0.1;   % 终止条件：速度低于该值视为停止

% PID 参数
Kp = 0.13;
Ki = 0.0;
Kd = 0.0;

integral_error = 0;
prev_error = 0;

% === 创建车辆 ===
vehicle = world.spawn_actor(bp, spawn_point);
pause(1.0);

% === 设置目标初速度 ===
v_init = 28.61;  % m/s
v_vector = py.carla.Vector3D(0, v_init, 0);
vehicle.set_target_velocity(v_vector);
pause(2.0);  % 等待加速

% === 初始化控制器并开始PID制动 ===
t0 = tic;
time_log = [];
brake_log = [];
accel_log = [];
speed_log = [];

while true
    t = toc(t0);

    % 获取速度
    vel = vehicle.get_velocity();
    v = sqrt(double(vel.x)^2 + double(vel.y)^2 + double(vel.z)^2);
    
    % 获取纵向加速度（carla世界中y方向一般是前进方向）
    acc = vehicle.get_acceleration();
    ay = double(acc.y);

    % PID控制
    error = target_decel + ay;  % 注意：目标是减速度 -> 当前加速度应为负
    integral_error = integral_error + error * dt;
    derivative = (error - prev_error) / dt;
    prev_error = error;

    delta_brake = Kp*error + Ki*integral_error + Kd*derivative;
    brake_cmd = brake_cmd + delta_brake;
    brake_cmd = max(min(brake_cmd, 1.0), 0.0);

    % 发送刹车命令
    control = py.carla.VehicleControl();
    control.throttle = 0.0;
    control.brake = brake_cmd;
    control.steer = 0.0;
    vehicle.apply_control(control);

    % 打印状态
    fprintf('[t=%.2f] v=%.2f m/s | ay=%.2f m/s² | brake=%.2f | Δbrake=%.3f\n', ...
        t, v, ay, brake_cmd,delta_brake);

    % 记录数据
    time_log(end+1) = t;
    brake_log(end+1) = brake_cmd;
    accel_log(end+1) = ay;
    speed_log(end+1) = v;

    % 判断停止条件
    if v < velocity_eps || t > max_time
        break;
    end

    pause(dt);
end

% 停止车辆
control.brake = 1.0;
control.throttle = 0.0;
vehicle.apply_control(control);
pause(0.5);
vehicle.destroy();

% === 绘图：加速度、刹车命令、速度 ===
figure;
subplot(3,1,1);
plot(time_log, accel_log);
ylabel('加速度 ay (m/s²)');
title('加速度变化');

subplot(3,1,2);
plot(time_log, brake_log);
ylabel('制动命令');
title('Brake PID 输出');

subplot(3,1,3);
plot(time_log, speed_log);
ylabel('速度 v (m/s)');
xlabel('时间 (s)');
title('速度变化');

