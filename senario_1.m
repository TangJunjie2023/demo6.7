
model3_list = cell(py.list(blueprints.filter('vehicle.tesla.model3')));
audi_list   = cell(py.list(blueprints.filter('vehicle.audi.tt')));

spawn_points = py.list(map.get_spawn_points());
fakego_spawn  = spawn_points{100};  % 假设第99个spawn点
fakego_loc = fakego_spawn.location;



% ==== 生成自车 ====
bp_ego = audi_list{1};

if bp_ego.has_attribute('color')
    bp_ego.set_attribute('color', '255,0,0');
end

if bp_ego.has_attribute('role_name')
    bp_ego.set_attribute('role_name', 'ego');
end



ego_spawn = py.carla.Transform( ...
    py.carla.Location(fakego_loc.x , fakego_loc.y - 50, fakego_loc.z), ... % 前方 50m 左车道
    fakego_spawn.rotation);

ego_loc = ego_spawn.location;

vehicle_ego = world.spawn_actor(bp_ego, ego_spawn);

% ==== 生成前后车 ====



front_spawn = py.carla.Transform( ...
    py.carla.Location(ego_loc.x , ego_loc.y + 50, ego_loc.z), ... % 前方 50m 左车道
    ego_spawn.rotation);
rear_spawn = py.carla.Transform( ...
    py.carla.Location(ego_loc.x , ego_loc.y - 40, ego_loc.z), ... % 后方 30m 左车道
    ego_spawn.rotation);

Lfront_spawn = py.carla.Transform( ...
    py.carla.Location(ego_loc.x + 3.5, ego_loc.y + 50, ego_loc.z), ... % ZUO前方 50m 左车道
    ego_spawn.rotation);

Lrear_spawn = py.carla.Transform( ...
    py.carla.Location(ego_loc.x + 3.5 , ego_loc.y - 40, ego_loc.z), ... % ZUO后方 30m 左车道
    ego_spawn.rotation);

Rfront_spawn = py.carla.Transform( ...
    py.carla.Location(ego_loc.x - 3.5, ego_loc.y + 50, ego_loc.z), ... % you前方 50m 左车道
    ego_spawn.rotation);

Rrear_spawn = py.carla.Transform( ...
    py.carla.Location(ego_loc.x - 3.5 , ego_loc.y - 20, ego_loc.z), ... % you后方 30m 左车道
    ego_spawn.rotation);





spawn_list = {front_spawn, rear_spawn, Lfront_spawn, Lrear_spawn, Rfront_spawn, Rrear_spawn};


vehicles = cell(1, 6);
for i = 1:6
    bp = model3_list{1};  % 每次从 Audi blueprint list 取新的
    if bp.has_attribute('color')
        bp.set_attribute('color', '0,0,255');
    end
    vehicles{i} = world.spawn_actor(bp, spawn_list{i});
end




vehicle_map = containers.Map();
vehicle_map('front')    = vehicles{1};
vehicle_map('rear')     = vehicles{2};
vehicle_map('Lfront')   = vehicles{3};
vehicle_map('Lrear')    = vehicles{4};
vehicle_map('Rfront')   = vehicles{5};
vehicle_map('Rrear')    = vehicles{6};










%vehicle_ego.set_autopilot(true);   % 初始启用自动驾驶



%vehicle_front.set_autopilot(true);
%vehicle_rear.set_autopilot(true);



% ==== 定义初速度（以世界坐标 x/y 方向为主）====
v_init = 28.61;  % 初始速度  m/s，假设沿正Y方向行驶

v_vector = py.carla.Vector3D(0, v_init, 0);


vehicle_map('front').set_target_velocity(v_vector);
vehicle_map('rear').set_target_velocity(v_vector);
vehicle_map('Lfront').set_target_velocity(v_vector);
vehicle_map('Lrear').set_target_velocity(v_vector);
vehicle_map('Rfront').set_target_velocity(v_vector);
vehicle_map('Rrear').set_target_velocity(v_vector);

vehicle_ego.set_target_velocity(v_vector);


    %前后车开自动
% vehicle_map('front').set_autopilot(true);
% vehicle_map('rear').set_autopilot(true);
% vehicle_map('Lfront').set_autopilot(true);
% vehicle_map('Lrear').set_autopilot(true);
% vehicle_map('Rfront').set_autopilot(true);
%vehicle_map('Rrear').set_autopilot(true);



%% ========= matlab绘制 =========
lane_width = 3.5;
num_lanes = 6;
road_length = 500;
road_width = lane_width * num_lanes;

car_length = 4.5;
car_width = 2;
car_speed = 20; % 100 km/h


% 自车初始车道设置
lane_index = 4; 




car_pos = [100, lane_width * (lane_index - 0.5)];
car_accel = 0; % 初始加速度為 0

%变道设置
is_changing_lane = false;
lane_change_progress = 0;
lane_change_duration = 2.0; % 秒

car_history = [];  % 儲存偵測到的車輛資訊

time_step = 0.02;
simulation_time = 20;
num_steps = simulation_time / time_step;

num_points_per_radar = 30; %用於指定掃描的方向設定
r_resolution = 1.0;          % 小步長排於 1m

% 決策閾值
TTC_threshold = 5;
THW_threshold = 2;

motion_state = "keep";  % 可為 keep, decelerate, danger_decelerate, change_left, change_right

% ========== 添加目标车辆 ==========
targets = struct('x', {}, 'lane', {}, 'id', {}, 'speed', {});
targets(end+1) = struct('x', 60, 'lane', 3, 'id', 1, 'speed', 20.0);
targets(end+1) = struct('x', 0, 'lane', 3, 'id', 2, 'speed', 25.0);
targets(end+1) = struct('x', 60, 'lane', 4, 'id', 3, 'speed', 25.0);
targets(end+1) = struct('x', 0, 'lane', 4, 'id', 4, 'speed', 27.0);
targets(end+1) = struct('x', 60, 'lane', 2, 'id', 5, 'speed', 27.0);
targets(end+1) = struct('x', 0, 'lane', 2, 'id', 6, 'speed', 27.0);
target_car_length = 4.5;
target_car_width = 2.0;

%% ========= 雷达传感器配置 =========
sensors = struct('name', {}, 'type', {}, 'FOV', {}, 'range', {}, 'distanceAcc', {}, 'speedAcc', {}, 'heading', {}, 'active', {});

sensors(end+1) = struct('name', 'L1', 'type', 'LiDAR', 'FOV', 360, 'range', 200, 'distanceAcc', 0.05, 'speedAcc', 0.05, 'heading', 0, 'active', 'N');
sensors(end+1) = struct('name', 'R2', 'type', 'Radar', 'FOV', 40, 'range', 200, 'distanceAcc', 3, 'speedAcc', 2.7, 'heading', 0, 'active', 'Y');
sensors(end+1) = struct('name', 'R3', 'type', 'Radar', 'FOV', 50, 'range', 20, 'distanceAcc', 0.24, 'speedAcc', 1, 'heading', -55, 'active', 'Y');
sensors(end+1) = struct('name', 'R4', 'type', 'Radar', 'FOV', 60, 'range', 20, 'distanceAcc', 0.24, 'speedAcc', 1, 'heading', 0, 'active', 'Y');
sensors(end+1) = struct('name', 'R5', 'type', 'Radar', 'FOV', 50, 'range', 20, 'distanceAcc', 0.24, 'speedAcc', 1, 'heading', 55, 'active', 'Y');
sensors(end+1) = struct('name', 'R6', 'type', 'Radar', 'FOV', 50, 'range', 20, 'distanceAcc', 0.24, 'speedAcc', 1, 'heading', -105, 'active', 'Y');
sensors(end+1) = struct('name', 'R7', 'type', 'Radar', 'FOV', 50, 'range', 20, 'distanceAcc', 0.24, 'speedAcc', 1, 'heading', 105, 'active', 'Y');
sensors(end+1) = struct('name', 'R8', 'type', 'Radar', 'FOV', 50, 'range', 80, 'distanceAcc', 0.4, 'speedAcc', 2, 'heading', -155, 'active', 'Y');
sensors(end+1) = struct('name', 'R9', 'type', 'Radar', 'FOV', 50, 'range', 80, 'distanceAcc', 0.4, 'speedAcc', 2, 'heading', 155, 'active', 'Y');
sensors(end+1) = struct('name', 'C10', 'type', 'Camera', 'FOV', 50, 'range', 200, 'distanceAcc', 0, 'speedAcc',0, 'heading', 0, 'active', 'Y');
sensors(end+1) = struct('name', 'C11', 'type', 'Camera', 'FOV', 150, 'range', 50, 'distanceAcc', 0, 'speedAcc', 0, 'heading', 0, 'active', 'Y');
sensors(end+1) = struct('name', 'C12', 'type', 'Camera', 'FOV', 150, 'range', 50, 'distanceAcc', 0, 'speedAcc', 0, 'heading', -90, 'active', 'Y');
sensors(end+1) = struct('name', 'C13', 'type', 'Camera', 'FOV', 150, 'range', 50,  'distanceAcc', 0, 'speedAcc', 0, 'heading', 90, 'active', 'Y');

meas_distance_sigma = 3;
meas_speed_sigma = 2.7;

radar_indices = find(arrayfun(@(s) strcmp(s.type, 'Radar'), sensors));
num_radars = length(radar_indices);
total_points = num_radars * num_points_per_radar;

%% ========= 图形初始化 =========
figure;
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('highway ');
xlim([0, road_length]);
ylim([-2, road_width + 2]);
hold on;

% ==== 建立圖形後再畫目標車（這很重要）====
target_rects = gobjects(length(targets), 1);
for i = 1:length(targets)
    cx = targets(i).x;
    cy = lane_width * (targets(i).lane - 0.5);
    target_rects(i) = rectangle('Position', ...
        [cx - target_car_length/2, cy - target_car_width/2, ...
         target_car_length, target_car_width], ...
        'FaceColor', [1, 0.4, 0.4], 'EdgeColor', 'k', 'LineWidth', 1);
end

% 画车道线
for i = 0:num_lanes
    y = i * lane_width;
    if i == 0 || i == num_lanes 
        plot([0, road_length], [y, y], 'k', 'LineWidth', 2);
    elseif i == 1 || i == num_lanes -1
         plot([0, road_length], [y, y], 'k', 'LineWidth', 1);
    else
        plot([0, road_length], [y, y], '--k', 'LineWidth', 1);
    end
end

% 自车
car_rect = rectangle('Position', ...
    [car_pos(1)-car_length/2, car_pos(2)-car_width/2, car_length, car_width], ...
    'FaceColor', [0.2 0.6 1], 'EdgeColor', 'k', 'LineWidth', 1.5);
car_label = text(car_pos(1), car_pos(2)+1.5, '自车', ...
    'HorizontalAlignment', 'center', 'FontSize', 10);

% 初始化点云句柄和探测范围区域句柄
fake_scatter = scatter(nan, nan, 10, [0.8 0.6 0.4], 'filled');
detected_scatter = scatter(nan, nan, 10, [0.6 0.3 0], 'filled');
sensor_areas = gobjects(length(sensors), 1);
for i = 1:length(sensors)
    sensor_areas(i) = fill(nan, nan, [1 0.8 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.2);
end









snap0 = world.wait_for_tick();  
t0   = snap0.timestamp.elapsed_seconds;

prevSpeed = 0;
dt      = 0.02;        % 与 world.wait_for_tick() 匹配
g       = 9.8;         % 重力加速度，用于 brake 换算
tol_v   = 0.1;         % 低速阈值：认为已停车

% === 数据记录变量 ===


% t_vec          = [];
% v_ego_vec      = [];
% dist_rear_vec  = [];
% steer_vec      = [];

% time_log = [];
% brake_log = [];
% accel_log = [];
% speed_log = [];

max_steps = 10000;

time_log  = zeros(1, max_steps);
speed_log = zeros(1, max_steps);
brake_log = zeros(1, max_steps);
accel_log = zeros(1, max_steps);

i_load = 1;

%初始化数组绘图

TTC_front_log = [];
TTC_rear_log = [];
THW_front_log = [];
THW_rear_log = [];



%ego状态初始化
MRM = 0;
state_MRM = 0;
fake_front = 0;

camera = 1;

object_Rrear = 1;
object_Rfront = 1;
fake_Rrear = 0;
fake_Rfront = 0;



%修正carla-matlab坐标

modify_x = 100;
modify_y = 8.62079 + 8.75 + 3.5;






% PID 参数(brake_cmd控制)
Kp_brake = 0.13;
Ki_brake = 0.0;
Kd_brake = 0.0;

integral_error = 0;
prev_error = 0;


brake_cmd = 0.1;




% ==== 等待响应（MRM启动） ====
pause(3); 

loc_ego = vehicle_ego.get_location(); %获取初始位置

y_ego_inital = double(loc_ego.y);


% ============================================= 主循环 ===========================================================
while true
    snap = world.wait_for_tick();
    t_now = snap.timestamp.elapsed_seconds;
    t_rel = t_now - t0;



    vehicle_map('rear').set_autopilot(true);



    

  % ==== 获取信息====
    loc_ego = vehicle_ego.get_location();
    x_ego = double(loc_ego.x);
    y_ego = double(loc_ego.y);
    vel_ego = vehicle_ego.get_velocity();
    v_ego = sqrt(double(vel_ego.x)^2 + double(vel_ego.y)^2 + double(vel_ego.z)^2);
    %fprintf("ego | Y = %.2f m | speed = %.2f m/s\n", double(loc_ego.y), v_ego);
    
    
    % 获取车辆位置
    loc_front = vehicle_map('front').get_location();
    x_front = double(loc_front.x);
    y_front = double(loc_front.y);
    %z_front = double(loc_front.z);
    
    % 获取车辆速度
    vel_front = vehicle_map('front').get_velocity();
    v_front = sqrt(double(vel_front.x)^2 + double(vel_front.y)^2 + double(vel_front.z)^2);
    
    %fprintf("front | pos=(%.2f, %.2f) | speed=%.2f m/s\n", x_front, y_front, v_front);
    
    
    
    loc_rear = vehicle_map('rear').get_location();
    x_rear = double(loc_rear.x);
    y_rear = double(loc_rear.y);
    vel_rear = vehicle_map('rear').get_velocity();
    v_rear = sqrt(double(vel_rear.x)^2 + double(vel_rear.y)^2 + double(vel_rear.z)^2);
    
    loc_Lfront = vehicle_map('Lfront').get_location();
    x_Lfront = double(loc_Lfront.x);
    y_Lfront = double(loc_Lfront.y);
    vel_Lfront = vehicle_map('Lfront').get_velocity();
    v_Lfront = sqrt(double(vel_Lfront.x)^2 + double(vel_Lfront.y)^2 + double(vel_Lfront.z)^2);
    
    loc_Lrear = vehicle_map('Lrear').get_location();
    x_Lrear = double(loc_Lrear.x);
    y_Lrear = double(loc_Lrear.y);
    vel_Lrear = vehicle_map('Lrear').get_velocity();
    v_Lrear = sqrt(double(vel_Lrear.x)^2 + double(vel_Lrear.y)^2 + double(vel_Lrear.z)^2);
    
    loc_Rfront = vehicle_map('Rfront').get_location();
    x_Rfront = double(loc_Rfront.x);
    y_Rfront = double(loc_Rfront.y);
    vel_Rfront = vehicle_map('Lfront').get_velocity();
    v_Rfront = sqrt(double(vel_Rfront.x)^2 + double(vel_Rfront.y)^2 + double(vel_Rfront.z)^2);
    
    loc_Rrear = vehicle_map('Rrear').get_location();
    x_Rrear = double(loc_Rrear.x);
    y_Rrear = double(loc_Rrear.y);
    vel_Rrear = vehicle_map('Rrear').get_velocity();
    v_Rrear = sqrt(double(vel_Rrear.x)^2 + double(vel_Rrear.y)^2 + double(vel_Rrear.z)^2);









   

%% ========= 动画循环 =========

%车辆绘制（ego）
    % car_speed = car_speed + car_accel * time_step;
    % car_speed = max(car_speed, 0);

    car_pos(1) = y_ego + modify_x;
    car_pos(2) = x_ego + modify_y;

    set(car_rect, 'Position', [car_pos(1)-car_length/2, car_pos(2)-car_width/2, car_length, car_width]);
    set(car_label, 'Position', [car_pos(1), car_pos(2)+1.5]);




    carla_x_list = [x_front, x_rear, x_Lfront, x_Lrear, x_Rfront, x_Rrear];
    carla_y_list = [y_front, y_rear, y_Lfront, y_Lrear, y_Rfront, y_Rrear];
    carla_v_list = [v_front, v_rear, v_Lfront, v_Lrear, v_Rfront, v_Rrear];

    for t = 1:length(targets)
         % 用 CARLA 中实时位置替代仿真推进
        
        cx = carla_y_list(t) + modify_x;
        cy = carla_x_list(t) +  modify_y;

        set(target_rects(t), 'Position', [cx - target_car_length/2, cy - target_car_width/2, target_car_length, target_car_width]);
    end


 %车辆绘制
 
    % front_pos(1) = y_front;
    % front_pos(2) = x_front + 8.62079 + 8.75;
    % set(front_rect, 'Position', [front_pos(1)-car_length/2, front_pos(2)-car_width/2, car_length, car_width]);
    % 
    % rear_pos(1) = y_rear;
    % rear_pos(2) = x_rear + 8.62079 + 8.75;
    % set(rear_rect, 'Position', [rear_pos(1)-car_length/2, rear_pos(2)-car_width/2, car_length, car_width]);
    % 
    % Lfront_pos(1) = y_Lfront;
    % Lfront_pos(2) = x_Lfront + 8.62079 + 8.75;
    % set(Lfront_rect, 'Position', [Lfront_pos(1)-car_length/2, Lfront_pos(2)-car_width/2, car_length, car_width]);
    % 
    % Lrear_pos(1) = y_Lrear;
    % Lrear_pos(2) = x_Lrear + 8.62079 + 8.75;
    % set(Lrear_rect, 'Position', [Lrear_pos(1)-car_length/2, Lrear_pos(2)-car_width/2, car_length, car_width]);
    % 
    % Rfront_pos(1) = y_Rfront;
    % Rfront_pos(2) = x_Rfront + 8.62079 + 8.75;
    % set(Rfront_rect, 'Position', [Rfront_pos(1)-car_length/2, Rfront_pos(2)-car_width/2, car_length, car_width]);
    % 
    % Rrear_pos(1) = y_Rrear;
    % Rrear_pos(2) = x_Rrear + 8.62079 + 8.75;
    % set(Rrear_rect, 'Position', [Rrear_pos(1)-car_length/2, Rrear_pos(2)-car_width/2, car_length, car_width]);







    %% ========= 更新传感器探测范围可视化 =========
    for i = 1:length(sensors)
        s = sensors(i);
        if strcmp(s.active, 'Y') && (strcmp(s.type, 'Radar') || strcmp(s.type, 'LiDAR'))
            angle_span = linspace(-s.FOV/2, s.FOV/2, 30);
            x_arc = car_pos(1) + s.range * cosd(angle_span + s.heading);
            y_arc = car_pos(2) + s.range * sind(angle_span + s.heading);
            set(sensor_areas(i), 'XData', [car_pos(1), x_arc, car_pos(1)], 'YData', [car_pos(2), y_arc, car_pos(2)]);
        else
            set(sensor_areas(i), 'XData', nan, 'YData', nan);
        end
    end

    %% ========= 模拟点云扫描 =========
    angles = 0:1:359;
    cos_theta = cosd(angles);
    sin_theta = sind(angles);
    fake_points = nan(length(angles), 2);
    detected_points = nan(length(angles), 2);

    uncovered_angles = [];  % 用于记录未被任何传感器覆盖的角度



    for i = 1:length(angles)
        theta = angles(i);
        max_range = 0;
        covered = false;
        for s = sensors
            if strcmp(s.active, 'Y') && (strcmp(s.type, 'Radar') || strcmp(s.type, 'LiDAR'))
                relative_angle = mod(theta - s.heading + 180, 360) - 180;
                if abs(relative_angle) <= s.FOV / 2
                    max_range = max(max_range, s.range);
                    covered = true;
                end
            end
        end

        r_final = max_range;
        hit = false;
        for r = 1:max_range
            x = car_pos(1) + r * cos_theta(i);
            y = car_pos(2) + r * sin_theta(i);
            for t = 1:length(targets)
                tx = carla_y_list(t) + modify_x;
                ty = carla_x_list(t) + modify_y;
                if x >= tx - target_car_length/2 && x <= tx + target_car_length/2 && ...
                   y >= ty - target_car_width/2 && y <= ty + target_car_width/2
                    detected_points(i,:) = [x, y];
                    r_final = r;
                    hit = true;
                    break;
                end
            end
            if hit, break; end
        end




        if ~covered
           
  
            if theta < 180
                target_lane = min(num_lanes, lane_index + 1);
            elseif theta == 180  || theta ==0 
                target_lane = lane_index ;
            else
                target_lane = max(1, lane_index - 1);
            end

            % 计算从车到目标车道中心线的交点位置（保持射线方向）
            lane_center_y = lane_width * (target_lane - 0.5);
            dy = lane_center_y - car_pos(2);

            if theta == 180  || theta ==0  % 避免除以零
                x = car_pos(1) + cos_theta(i)*3 ;
                y = lane_width * (lane_index - 0.5);
            else
                dx = dy / (sin_theta(i) / cos_theta(i));
                x = car_pos(1) + dx;
                y = lane_center_y;
            end

             % 记录未覆盖角度
%            uncovered_angles(end+1) = theta;
         else
            x = car_pos(1) + r_final * cos_theta(i);
            y = car_pos(2) + r_final * sin_theta(i);
        
         end



        if ~hit
            fake_points(i,:) = [x, y];
        end
    end

    set(fake_scatter, 'XData', fake_points(:,1), 'YData', fake_points(:,2));
    set(detected_scatter, 'XData', detected_points(:,1), 'YData', detected_points(:,2));



  %% ========= 记录每车道最近障碍点 =========
    lane_obstacles = repmat(struct('front', nan(1, 2), 'rear', nan(1, 2), 'front_type', '', 'rear_type', '', 'front_speed', nan, 'rear_speed', nan), num_lanes, 1);
    for lane = 1:num_lanes
        center_y = lane_width * (lane - 0.5);

        front_min_dist = inf;
        rear_min_dist = inf;
        front_point = nan(1, 2);
        rear_point = nan(1, 2);
        front_type = '';
        rear_type = '';
        front_speed = nan;
        rear_speed = nan;

        for pt_idx = 1:length(fake_points)
            pt = fake_points(pt_idx, :);
            x = pt(1); y = pt(2);
            if abs(y - center_y) < lane_width / 2
                dx = x - car_pos(1);
                if dx > 0 && dx < front_min_dist
                    front_min_dist = dx;
                    front_point = pt;
                    front_type = 'fake';
                    front_speed = v_ego;
                elseif dx < 0 && abs(dx) < rear_min_dist
                    rear_min_dist = abs(dx);
                    rear_point = pt;
                    rear_type = 'fake';
                    front_speed = v_ego;
                end
            end
        end

        for pt_idx = 1:length(detected_points)
            pt = detected_points(pt_idx, :);
            x = pt(1); y = pt(2);
            % if abs(y - center_y) < lane_width * 0.6
            if y > center_y - lane_width/2 && y < center_y + lane_width/2
                 
                
                % fprintf('[DEBUG] pt(%.2f, %.2f) 属于车道 %d\n', x, y, lane); % 检查它属于哪个车道
                
                dx = x - car_pos(1);
                for t = 1:length(targets)
                    % tx = targets(t).x;
                    % ty = lane_width * (targets(t).lane - 0.5);

                      tx = carla_y_list(t) + modify_x;                             %直接用坐标表示目标车 target组还没修正
                      ty = carla_x_list(t) + modify_y;
                      tv = carla_v_list(t);

                    % if abs(x - tx) < target_car_length/2 && abs(y - ty) < target_car_width/2
                   
                    % if sqrt((x - tx)^2 + (y - ty)^2) < 4.5
                    if abs(x - tx) < target_car_length/2 + 1.5 && abs(y - ty) < target_car_width/2 + 0.5
                        speed = tv;
                        if dx > 0 && dx < front_min_dist
                            front_min_dist = dx;
                            front_point = pt;
                            front_type = 'detected';
                            front_speed = speed;
                        elseif dx < 0 && abs(dx) < rear_min_dist
                            rear_min_dist = abs(dx);
                            rear_point = pt;
                            rear_type = 'detected';
                            rear_speed = speed;
                        end
                        break;
                    end
                end
            end
        end

        lane_obstacles(lane).front = front_point;
        lane_obstacles(lane).rear = rear_point;
        lane_obstacles(lane).front_type = front_type;
        lane_obstacles(lane).rear_type = rear_type;
        lane_obstacles(lane).front_speed = front_speed;
        lane_obstacles(lane).rear_speed = rear_speed;

        
    end
            % ✅ 加上输出日志
        % fprintf('[Lane %d] FRONT  -> pos: [%.2f, %.2f], type: %-8s, speed: %.2f\n', ...
        %         lane, front_point(1), front_point(2), front_type, front_speed);
        % fprintf('[Lane %d] REAR   -> pos: [%.2f, %.2f], type: %-8s, speed: %.2f\n', ...
        %         lane, rear_point(1), rear_point(2), rear_type, rear_speed);




        
    % 
    % % 示例：打印当前自车道左侧前方最近点坐标及类型与速度
    % if lane_index < num_lanes && all(~isnan(lane_obstacles(lane_index + 1).front))
    %     fprintf(': left_lane_front [%.2f, %.2f], type: %s, speed: %.2f\n',  ...
    %         lane_obstacles(lane_index + 1).front, lane_obstacles(lane_index + 1).front_type, lane_obstacles(lane_index + 1).front_speed);
    % end
    % 
    % 
    % % 示例：打印当前车道与右侧车道目标
    % if all(~isnan(lane_obstacles(lane_index).front))
    %     fprintf(': lane_front [%.2f, %.2f], type: %s, speed: %.2f\n',  ...
    %         lane_obstacles(lane_index).front, lane_obstacles(lane_index).front_type, lane_obstacles(lane_index).front_speed);
    % end
    % if lane_index > 1 && all(~isnan(lane_obstacles(lane_index - 1).front))
    %     fprintf(': right_lane_front [%.2f, %.2f], type: %s, speed: %.2f\n',  ...
    %         lane_obstacles(lane_index - 1).front, lane_obstacles(lane_index - 1).front_type, lane_obstacles(lane_index - 1).front_speed);
    % end
    % 
    % % 后方
    % if lane_index < num_lanes && all(~isnan(lane_obstacles(lane_index + 1).rear))
    %     fprintf(': left_lane_rear [%.2f, %.2f], type: %s, speed: %.2f\n',  ...
    %         lane_obstacles(lane_index + 1).rear, lane_obstacles(lane_index + 1).rear_type, lane_obstacles(lane_index + 1).rear_speed);
    % end
    % 
    % 
    % if all(~isnan(lane_obstacles(lane_index).rear))
    %     fprintf(': lane_rear [%.2f, %.2f], type: %s, speed: %.2f\n',  ...
    %         lane_obstacles(lane_index).rear, lane_obstacles(lane_index).rear_type, lane_obstacles(lane_index).rear_speed);
    % end
    % 
    %  if lane_index > 1 && all(~isnan(lane_obstacles(lane_index - 1).rear))
    %     fprintf(': right_lane_rear [%.2f, %.2f], type: %s, speed: %.2f\n',  ...
    %         lane_obstacles(lane_index - 1).rear, lane_obstacles(lane_index - 1).rear_type, lane_obstacles(lane_index - 1).rear_speed);
    % end


   % end
    % drawnow;
    %  pause(time_step);


%%  调用坐标调整功能，调整坐标误差


%前方3车道
  front_risk= [];
  if strcmp(lane_obstacles(lane_index).front_type, 'detected')
    obstacle = struct('x', lane_obstacles(lane_index).front(1), ...
                     'y', lane_obstacles(lane_index).front(2), ...
                     'speed', lane_obstacles(lane_index).front_speed);

    [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_fx] = ...
    adjust_obstacle_measurement1(obstacle, sensors, car_pos);


    lane_obstacles(lane_index).front(1) = adjusted_x;
    lane_obstacles(lane_index).front_speed = adjusted_speed;

    % fprintf('Lane_Front_X: %.2f, speed: %.2f\n', y__front, v__front);
    % 
    % fprintf('detected_sensor：\n');
    %  for i = 1:length(covered_sensors)
    %      fprintf(' - type: %s, name: %s\n', covered_sensors(i).type, covered_sensors(i).name);
    %  end
  end



  if strcmp(lane_obstacles(lane_index +1).front_type, 'detected')
    obstacle = struct('x', lane_obstacles(lane_index +1 ).front(1), ...
                     'y', lane_obstacles(lane_index +1 ).front(2), ...
                     'speed', lane_obstacles(lane_index +1 ).front_speed);

    [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_flx] = ...
    adjust_obstacle_measurement2(obstacle, sensors, car_pos);


    lane_obstacles(lane_index +1 ).front(1) = adjusted_x;
    lane_obstacles(lane_index +1 ).front_speed = adjusted_speed;

    % fprintf('Left_Lane_Front_X: %.2f, speed: %.2f\n', y__Lfront, v__Lfront);
    % 
    % fprintf('detected_sensor：\n');
    %  for i = 1:length(covered_sensors)
    %      fprintf(' - type: %s, name: %s\n', covered_sensors(i).type, covered_sensors(i).name);
    %  end
  end



  if lane_index > 1 && strcmp(lane_obstacles(lane_index -1).front_type, 'detected')
    obstacle = struct('x', lane_obstacles(lane_index -1 ).front(1), ...
                     'y', lane_obstacles(lane_index -1 ).front(2), ...
                     'speed', lane_obstacles(lane_index -1 ).front_speed);

    [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_frx] = ...
    adjust_obstacle_measurement3(obstacle, sensors, car_pos);


    lane_obstacles(lane_index -1 ).front(1) = adjusted_x;
    lane_obstacles(lane_index -1 ).front_speed = adjusted_speed;

    % fprintf(['Right' ...
    %     '_Lane_Front_X: %.2f, speed: %.2f\n'], y__Rfront, v__Rfront);
    % 
    % fprintf('detected_sensor：\n');
    %  for i = 1:length(covered_sensors)
    %      fprintf(' - type: %s, name: %s\n', covered_sensors(i).type, covered_sensors(i).name);
    %  end
  end




%高风险检测*（camera failure）

%   if exist('highrisk_obstacles_fx','var') && ~isempty(highrisk_obstacles_fx)
%      front_risk(end+1) = highrisk_obstacles_fx;
%  end
% 
%  if exist('highrisk_obstacles_flx','var') && ~isempty(highrisk_obstacles_flx)
%      front_risk(end+1) = highrisk_obstacles_flx;
%  end
% 
%  if exist('highrisk_obstacles_frx','var') && ~isempty(highrisk_obstacles_frx)
%      front_risk(end+1) = highrisk_obstacles_frx;
%  end
% 
% if ~isempty( front_risk)
%    highrisk_obstacles_front = min( front_risk);
%     fprintf('前方高风险障碍物在：%.2f\n', highrisk_obstacles_front);
%  else
%    disp('no front risk');
% end




  %后方3车道
  rear_risk= [];
  if strcmp(lane_obstacles(lane_index).rear_type, 'detected')
    obstacle = struct('x', lane_obstacles(lane_index).rear(1), ...
                     'y', lane_obstacles(lane_index).rear(2), ...
                     'speed', lane_obstacles(lane_index).rear_speed);

    [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_rx] = ...
    adjust_obstacle_measurement4(obstacle, sensors, car_pos);


    lane_obstacles(lane_index).rear(1) = adjusted_x;
    lane_obstacles(lane_index).rear_speed = adjusted_speed;

    % fprintf('Lane_Rear_X: %.2f, speed: %.2f\n', y__rear, v__rear);
    % 
    % fprintf('detected_sensor：\n');
    %  for i = 1:length(covered_sensors)
    %      fprintf(' - type: %s, name: %s\n', covered_sensors(i).type, covered_sensors(i).name);
    %  end
  end



  if strcmp(lane_obstacles(lane_index +1).rear_type, 'detected')
    obstacle = struct('x', lane_obstacles(lane_index +1 ).rear(1), ...
                     'y', lane_obstacles(lane_index +1 ).rear(2), ...
                     'speed', lane_obstacles(lane_index +1 ).rear_speed);

    [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_rlx] = ...
    adjust_obstacle_measurement5(obstacle, sensors, car_pos);


    lane_obstacles(lane_index +1 ).rear(1) = adjusted_x;
    lane_obstacles(lane_index +1 ).rear_speed = adjusted_speed;

    % fprintf('Left_Lane_Rear_X: %.2f, speed: %.2f\n', y__Lrear, v__Lrear);
    % 
    % fprintf('detected_sensor：\n');
    %  for i = 1:length(covered_sensors)
    %      fprintf(' - type: %s, name: %s\n', covered_sensors(i).type, covered_sensors(i).name);
    %  end
  end



  if lane_index > 1 && strcmp(lane_obstacles(lane_index -1).rear_type, 'detected')
    obstacle = struct('x', lane_obstacles(lane_index -1 ).rear(1), ...
                     'y', lane_obstacles(lane_index -1 ).rear(2), ...
                     'speed', lane_obstacles(lane_index -1 ).rear_speed);

    [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_rrx] = ...
    adjust_obstacle_measurement6(obstacle, sensors, car_pos);


    lane_obstacles(lane_index -1 ).rear(1) = adjusted_x;
    lane_obstacles(lane_index -1 ).rear_speed = adjusted_speed;

    % fprintf('Right_Lane_Rear_X: %.2f, speed: %.2f\n', y__Rrear, v__Rrear);
    % 
    % fprintf('detected_sensor：\n');
    %  for i = 1:length(covered_sensors)
    %      fprintf(' - type: %s, name: %s\n', covered_sensors(i).type, covered_sensors(i).name);
    %  end
  end



%后方高风险检测*（carmera failure）

%   if exist('highrisk_obstacles_rx','var') && ~isempty(highrisk_obstacles_rx)
%      rear_risk(end+1) = highrisk_obstacles_rx;
%  end
% 
%  if exist('highrisk_obstacles_lx','var') && ~isempty(highrisk_obstacles_rlx)
%      rear_risk(end+1) = highrisk_obstacles_rlx;
%  end
% 
%  if exist('highrisk_obstacles_rrx','var') && ~isempty(highrisk_obstacles_rrx)
%      rear_risk(end+1) = highrisk_obstacles_rrx;
%  end
% 
% 
% 
%  if ~isempty( rear_risk)
%    highrisk_obstacles_rear = max( rear_risk);
%    fprintf('后方高风险障碍物在：%.2f\n', highrisk_obstacles_rear);
% else
%    disp('no rear risk');
% end





%加速度提取

    acc = vehicle_ego.get_acceleration();
    ay = double(acc.y);





%% MRM判断



    if  MRM == 0
       
        % ———— 2) 计算相对速度 & 距离 & TTC ————
        rel_v_front = v_ego - lane_obstacles(lane_index).front_speed;           % 正值：正在接近
        dist_front  = double(lane_obstacles(lane_index).front(1) - car_pos(1));
        if rel_v_front <= 0
            TTC_front = Inf;
        else
            TTC_front = dist_front / rel_v_front;
        end
    

        rel_v_rear = lane_obstacles(lane_index).rear_speed - v_ego;
        dist_rear = double(car_pos(1) - lane_obstacles(lane_index).rear(1));

        if rel_v_rear > 0
            TTC_rear = dist_rear / rel_v_rear;
        else
            TTC_rear = Inf;
        end
    
        THW_front = dist_front / max(v_ego, 0.1);  % 防除以0
        THW_rear = dist_rear / max(lane_obstacles(lane_index).rear_speed, 0.1);


        %MRM移动距离
        S = y_ego - y_ego_inital;  


        



        if (strcmp(lane_obstacles(lane_index).front_type, 'fake') &&  ~(TTC_front >= 5 && THW_front >= 2)) || ...
           (all(~isnan(lane_obstacles(lane_index ).front)) && ~(TTC_front >= 5 && THW_front >= 2)) || ...
            ~camera == 1

           
            state_MRM = 1;
            disp('EM'); 



        elseif ((all(~isnan(lane_obstacles(lane_index ).rear)) &&~(TTC_rear >= 2 && THW_rear >= 1))) && S < 200

            brake_cmd = 0.0;
            control = py.carla.VehicleControl();
            control.throttle = 0.4;                                % 调整 throttle 以维持 9 m/s，视具体情况调整
            control.brake = 0.0;
            control.steer = 0.0;
            vehicle_ego.apply_control(control);
            
            disp('MM');
            state_MRM = 2;


           

        elseif (v_ego <= 10 && v_ego > 8 ) && S < 200 
            
            brake_cmd = 0.0;

            control = py.carla.VehicleControl();
            control.throttle = 0.4;                                % 调整 throttle 以维持 9 m/s，视具体情况调整
            control.brake = 0.0;
            control.steer = 0.0;
            vehicle_ego.apply_control(control);
            
          

            disp('wait');
            state_MRM = 0;
            




        else 
            % v_ego > 9 || v_ego <= 9 && S >= 200;

            disp('Dec');


            % PID控制（brake）
            target_decel = 2.0;
            error = target_decel + ay;  % 注意：目标是减速度 -> 当前加速度应为负
            integral_error = integral_error + error * dt;
            derivative = (error - prev_error) / dt;
            prev_error = error;

            delta_brake = Kp_brake*error + Ki_brake*integral_error + Kd_brake*derivative;
            brake_cmd = brake_cmd + delta_brake;
            brake_cmd = max(min(brake_cmd, 1.0), 0.0);

            % brake_cmd = 0.2;
            % brake_cmd = max(min(brake_cmd * (28/12), 1.0), 0.0);  % 限制在 [0, 1]

            % === 发出控制（throttle 置 0）===
            control = py.carla.VehicleControl();
            control.throttle = 0.0;
            control.brake = brake_cmd;
            control.steer = 0.0;  % 保持直行
            vehicle_ego.apply_control(control);


            state_MRM = 0;
        end

        
              

      

            
    end



    if state_MRM == 1


        % % === 获取当前速度 ===
        % vel = vehicle_ego.get_velocity();
        % v = sqrt(double(vel.x)^2 + double(vel.y)^2 + double(vel.z)^2);
  
        % % === 设置目标速度 ===
        % decel = 4.0;  % m/s^2
        % dt = 0.02;    % 控制周期
        % v_target = max(0, v - decel * dt);  % 一帧后的期望速度
        % 
        % % === 根据误差计算 brake_cmd（简单比例控制） ===
        % K_brake = 0.4;  % 可调参数
        % brake_cmd = 0.4;
        % brake_cmd = max(min(brake_cmd *(28/12), 1.0), 0.0);  % 限制在 [0, 1]

        % PID控制
        target_decel = 4.0;
        error = target_decel + ay;  % 注意：目标是减速度 -> 当前加速度应为负
        integral_error = integral_error + error * dt;
        derivative = (error - prev_error) / dt;
        prev_error = error;

        delta_brake = Kp_brake*error + Ki_brake*integral_error + Kd_brake*derivative;
        brake_cmd = brake_cmd + delta_brake;
        brake_cmd = max(min(brake_cmd, 1.0), 0.0);
    
        % === 发出控制（throttle 置 0）===
        control = py.carla.VehicleControl();
        control.throttle = 0.0;
        control.brake = brake_cmd;
        control.steer = 0.0;  % 保持直行
        vehicle_ego.apply_control(control);
    
    end







    % ———— 3) 计算右车道相对速度 & 距离 & TTC ————

    if lane_index > 1 && ~isnan(lane_obstacles(lane_index - 1).front_speed)
        rel_v_Rfront = v_ego - lane_obstacles(lane_index -1).front_speed; 
        dist_Rfront  = double(lane_obstacles(lane_index -1).front(1) - car_pos(1));    % 正值：正在接近
    
        %rel_v_Rfront = 0;  % 或设为0，或设为默认值
    
    %rel_v_Rfront = v_ego - lane_obstacles(lane_index -1).front_speed;           % 正值：正在接近
    %dist_Rfront  = double(lane_obstacles(lane_index -1).front(1) - car_pos(1));
        if rel_v_Rfront <= 0
            TTC_Rfront = Inf;
            THW_Rfront = dist_Rfront / max(v_ego, 0.1);  % 防除以0
        else
            TTC_Rfront = dist_Rfront / rel_v_Rfront;
            THW_Rfront = dist_Rfront / max(v_ego, 0.1);  % 防除以0
        end

    end
    
    


    if lane_index > 1 && ~isnan(lane_obstacles(lane_index - 1).rear_speed)
        rel_v_Rrear = lane_obstacles(lane_index -1).rear_speed - v_ego;
        dist_Rrear = double(car_pos(1) - lane_obstacles(lane_index -1).rear(1));
    
        %rel_v_Rfront = 0;  % 或设为0，或设为默认值
    
        if rel_v_Rrear > 0
            TTC_Rrear = dist_Rrear / rel_v_Rrear;
            THW_Rrear = dist_Rrear / max(lane_obstacles(lane_index -1).rear_speed, 0.1);
        else
            TTC_Rrear = Inf;
            THW_Rrear = dist_Rrear / max(lane_obstacles(lane_index -1).rear_speed, 0.1);
        end
    end

    %THW_Rfront = dist_Rfront / max(v_ego, 0.1);  % 防除以0
    %THW_Rrear = dist_Rrear / max(lane_obstacles(lane_index -1).rear_speed, 0.1);


% ———— 右变道判定 ————
    if (state_MRM == 0 || state_MRM == 2) && ...
       lane_index > 1 && ...
       camera == 1 && ...
       v_ego > 8 && ...
       all(isnan(lane_obstacles(lane_index - 1).rear)) && ...
       (all(isnan(lane_obstacles(lane_index - 1).front)) || (all(~isnan(lane_obstacles(lane_index - 1).front)) && TTC_Rfront >= 5 && THW_Rfront >= 2))
       %(fake_Rfront == 0 || (fake_Rfront == 1 && TTC_Rfront >= 5 && THW_Rfront >= 2))
      

       disp('✅ 满足右变道安全条件');



      % === 开始向右变道 ===
        % === 目标点坐标（例如向右变道，前方 50 米处） ===
        
            loc_0 = vehicle_ego.get_location();
            x0 = double(loc_0.x);
            y0 = double(loc_0.y);

            x_target = x0 - 3.5;  % 横向右变道 3.5 米
            y_target = y0 + 40;   % 纵向前进 50 米


            if lane_index ==2 
                if  y_target > -73 && y_target < 106                   % 路肩带变窄区域
                    
                    x_target = x0 - 1.75;

                else
                    x_target = x0 - 3;
                
                end
            end

            
            % 设置中间控制点形成 S 曲线（可自定义更多点）
            x_points = [x0, (x0 + x_target)/2, x_target];
            y_points = [y0, y0 + 25, y_target + 10];

            
            
            % 使用 spline 拟合曲线
            yy = linspace(y0, y_target, 100);  % 在 y 上均匀取点
            xx = spline(y_points, x_points, yy);  % 得到 x = f(y)
            
            % 绘制轨迹
            %figure;
            % plot(xx, yy, 'b-', 'LineWidth', 2); hold on;
            % plot(x_points, y_points, 'ro--');  % 控制点
            % title('变道轨迹');
            % xlabel('X'); ylabel('Y'); axis equal;
        
        
            % 生成目标点列表（每个点为目标）
            path_points = [xx; yy]';  % size: [N x 2]
        
        
        
            target_idx = 1;
            N = size(path_points, 1);
            lookahead_dist = 3.0;  % 可选：Lookahead距离
            
            % === 控制周期参数等 ===
            dt = 0.02;
            
            % 初始化 PID 参数
            Kp = 0.4;
            Ki = 0.05;
            Kd = 0;
            
            % 初始化状态
            prev_yaw_error = 0;
            integral = 0;
            
            % ✅ 加上这个：初始化上一帧方向盘值
            prev_steer = 0.0;
        
        
        is_changing_lane = true;
        
        while target_idx <= N 
        

            snap = world.wait_for_tick();
            t_now = snap.timestamp.elapsed_seconds;
            t_rel = t_now - t0;
           
            loc = vehicle_ego.get_location();
            x = double(loc.x);
            y = double(loc.y);

            vel_ego = vehicle_ego.get_velocity();
            v_ego = sqrt(double(vel_ego.x)^2 + double(vel_ego.y)^2 + double(vel_ego.z)^2);

            acc = vehicle_ego.get_acceleration();
            ay = double(acc.y);

         if is_changing_lane
           % → 原有的 spline 轨迹追踪逻辑
        
            % === 获取当前车辆位置 ===
            % loc = vehicle_ego.get_location();
            % x = double(loc.x);
            % y = double(loc.y);
        

            % yaw = deg2rad(double(vehicle_ego.get_transform().rotation.yaw));
                % 获取车辆朝向
            transform = vehicle_ego.get_transform();
            rotation = transform.rotation;
            yaw_deg = double(rotation.yaw);
            
            yaw = deg2rad(yaw_deg);  % 弧度
        
            vel = vehicle_ego.get_velocity();
            v = sqrt(double(vel.x)^2 + double(vel.y)^2 + double(vel.z)^2);
        
            % === 找到最近的路径点，或使用 target_idx（Lookahead） ===
            dists = sqrt((xx - x).^2 + (yy - y).^2);
            [~, target_idx] = min(dists);  % 最近点
            target_idx = min(target_idx + round(lookahead_dist/dt), N);  % Lookahead
        
            target = path_points(target_idx, :);
            dx = target(1) - x;
            dy = target(2) - y;
        
            % === 控制 steer ===
            desired_yaw = atan2(dy, dx);
            yaw_error = wrapToPi(desired_yaw - yaw);
        
            integral = integral + yaw_error * dt;
            derivative = (yaw_error - prev_yaw_error) / dt;
        
            steer_cmd = Kp * yaw_error + Ki * integral + Kd * derivative;
        
            steer_cmd = max(min(steer_cmd, prev_steer + 0.05), prev_steer - 0.05);
            steer_cmd = max(min(steer_cmd, 1.0), -1.0);
            prev_steer = steer_cmd;
        
            prev_yaw_error = yaw_error;
        
            % === 控制速度 ===
            desired_speed = 10;  % m/s
            throttle_cmd = max(0, 0.2 * (desired_speed - v));
            brake_cmd = 0;
            if v - desired_speed > 1.0
                brake_cmd = 0.2;
            end
        
        
            if norm([x, y] - [x_target, y_target]) < 1.0
                is_changing_lane = false;
                disp("✅ 变道完成，切换为直行控制");

                lane_index = lane_index -1;

                state_MRM = 0;
                
                
                % 不再更新 steer，而是进入下一个状态
            end
        
         elseif y-y0 < 60
            % → 保持直行逻辑：让车辆逐步修正方向到朝向 y 轴
            
            % 获取当前位置和朝向
            transform = vehicle_ego.get_transform();
            rotation = transform.rotation;
            yaw_deg = double(rotation.yaw);
            yaw = deg2rad(yaw_deg);
        
            vel = vehicle_ego.get_velocity();
            v = sqrt(double(vel.x)^2 + double(vel.y)^2 + double(vel.z)^2);
        
            % === 目标朝向为“正北”即 y 轴方向 ===
            desired_yaw = pi/2;  % 或者根据原路径最后点计算也可以
            yaw_error = wrapToPi(desired_yaw - yaw);
        
            % === 使用 PID 修正 steer ===
            integral = integral + yaw_error * dt;
            derivative = (yaw_error - prev_yaw_error) / dt;
        
            steer_cmd = Kp * yaw_error + Ki * integral + Kd * derivative;
            %steer_cmd = max(min(steer_cmd, 1.0), -1.0);
        
            % 限制方向盘变化速率
            steer_cmd = max(min(steer_cmd, prev_steer + 0.05), prev_steer - 0.05);
            steer_cmd = max(min(steer_cmd, 1.0), -1.0);
        
         
            prev_steer = steer_cmd;
        
            prev_yaw_error = yaw_error;
        
            % === 控制速度 ===
            desired_speed = 10;
            throttle_cmd = max(0, 0.2 * (desired_speed - v));
            brake_cmd = 0;
        
            if v - desired_speed > 1.0
                brake_cmd = 0.2;
            end
        
         else
             break;
               
          
 
           
         end
        
        
          % 控制输出
          control = py.carla.VehicleControl();
          control.throttle = throttle_cmd;
          control.steer = steer_cmd;
          control.brake = brake_cmd;
          vehicle_ego.apply_control(control);
        
          pause(dt);

 
         %采样
          [i_load, time_log, speed_log, accel_log, brake_log] = log_status( ...
              i_load, time_log, speed_log, accel_log, brake_log, ...
              t_rel, v_ego, ay, brake_cmd);
        end


     
    
    end

















    % ———— 3) 计算左车道相对速度 & 距离 & TTC ————

    if lane_index < 6 && ~isnan(lane_obstacles(lane_index + 1).front_speed)
        rel_v_Lfront = v_ego - lane_obstacles(lane_index +1).front_speed; 
        dist_Lfront  = double(lane_obstacles(lane_index +1).front(1) - car_pos(1));    % 正值：正在接近
    
        %rel_v_Rfront = 0;  % 或设为0，或设为默认值
    
    %rel_v_Rfront = v_ego - lane_obstacles(lane_index -1).front_speed;           % 正值：正在接近
    %dist_Rfront  = double(lane_obstacles(lane_index -1).front(1) - car_pos(1));
        if rel_v_Lfront <= 0
            TTC_Lfront = Inf;
            THW_Lfront = dist_Lfront / max(v_ego, 0.1);  % 防除以0
        else
            TTC_Lfront = dist_Lfront / rel_v_Lfront;
            THW_Lfront = dist_Lfront / max(v_ego, 0.1);  % 防除以0
        end

    end
    
    


    if lane_index < 6 && ~isnan(lane_obstacles(lane_index + 1).rear_speed)
        rel_v_Lrear = lane_obstacles(lane_index +1).rear_speed - v_ego;
        dist_Lrear = double(car_pos(1) - lane_obstacles(lane_index +1).rear(1));
    
        %rel_v_Rfront = 0;  % 或设为0，或设为默认值
    
        if rel_v_Lrear > 0
            TTC_Lrear = dist_Lrear / rel_v_Lrear;
            THW_Lrear = dist_Lrear / max(lane_obstacles(lane_index +1).rear_speed, 0.1);
        else
            TTC_Lrear = Inf;
            THW_Lrear = dist_Lrear / max(lane_obstacles(lane_index +1).rear_speed, 0.1);
        end
    end








% ———— 左变道判定 ————
    if state_MRM == 2 && ...
       lane_index < 6 && ...
       camera == 1 && ...
       all(isnan(lane_obstacles(lane_index + 1).rear)) && ...
       (all(isnan(lane_obstacles(lane_index + 1).front)) || (all(~isnan(lane_obstacles(lane_index + 1).front)) && TTC_Lfront >= 5 && THW_Lfront >= 2))
       %(fake_Rfront == 0 || (fake_Rfront == 1 && TTC_Rfront >= 5 && THW_Rfront >= 2))
      

       disp('✅ 满足左变道安全条件');



      % === 开始向左变道 ===
        % === 目标点坐标（例如向左变道，前方 50 米处） ===
        
            loc_0 = vehicle_ego.get_location();
            x0 = double(loc_0.x);
            y0 = double(loc_0.y);

            x_target = x0 + 3.5;  % 横向右变道 3.5 米
            y_target = y0 + 40;   % 纵向前进 50 米


            % if lane_index ==2 
            %     if  y_target > -73 && y_target < 106                   % 路肩带变窄区域
            % 
            %         x_target = x0 - 1.75;
            % 
            %     else
            %         x_target = x0 - 3;
            % 
            %     end
            % end

            
            % 设置中间控制点形成 S 曲线（可自定义更多点）
            x_points = [x0, (x0 + x_target)/2, x_target];
            y_points = [y0, y0 + 25, y_target + 10];

            
            
            % 使用 spline 拟合曲线
            yy = linspace(y0, y_target, 100);  % 在 y 上均匀取点
            xx = spline(y_points, x_points, yy);  % 得到 x = f(y)
            
            % 绘制轨迹
            %figure;
            % plot(xx, yy, 'b-', 'LineWidth', 2); hold on;
            % plot(x_points, y_points, 'ro--');  % 控制点
            % title('变道轨迹');
            % xlabel('X'); ylabel('Y'); axis equal;
        
        
            % 生成目标点列表（每个点为目标）
            path_points = [xx; yy]';  % size: [N x 2]
        
        
        
            target_idx = 1;
            N = size(path_points, 1);
            lookahead_dist = 3.0;  % 可选：Lookahead距离
            
            % === 控制周期参数等 ===
            dt = 0.02;
            
            % 初始化 PID 参数
            Kp = 0.4;
            Ki = 0.05;
            Kd = 0;
            
            % 初始化状态
            prev_yaw_error = 0;
            integral = 0;
            
            % ✅ 加上这个：初始化上一帧方向盘值
            prev_steer = 0.0;
        
        
        is_changing_lane = true;
        
        while target_idx <= N 
        
           
            loc = vehicle_ego.get_location();
            x = double(loc.x);
            y = double(loc.y);

         if is_changing_lane
           % → 原有的 spline 轨迹追踪逻辑
        
            % === 获取当前车辆位置 ===
            % loc = vehicle_ego.get_location();
            % x = double(loc.x);
            % y = double(loc.y);
        

            % yaw = deg2rad(double(vehicle_ego.get_transform().rotation.yaw));
                % 获取车辆朝向
            transform = vehicle_ego.get_transform();
            rotation = transform.rotation;
            yaw_deg = double(rotation.yaw);
            
            yaw = deg2rad(yaw_deg);  % 弧度
        
            vel = vehicle_ego.get_velocity();
            v = sqrt(double(vel.x)^2 + double(vel.y)^2 + double(vel.z)^2);
        
            % === 找到最近的路径点，或使用 target_idx（Lookahead） ===
            dists = sqrt((xx - x).^2 + (yy - y).^2);
            [~, target_idx] = min(dists);  % 最近点
            target_idx = min(target_idx + round(lookahead_dist/dt), N);  % Lookahead
        
            target = path_points(target_idx, :);
            dx = target(1) - x;
            dy = target(2) - y;
        
            % === 控制 steer ===
            desired_yaw = atan2(dy, dx);
            yaw_error = wrapToPi(desired_yaw - yaw);
        
            integral = integral + yaw_error * dt;
            derivative = (yaw_error - prev_yaw_error) / dt;
        
            steer_cmd = Kp * yaw_error + Ki * integral + Kd * derivative;
        
            steer_cmd = max(min(steer_cmd, prev_steer + 0.05), prev_steer - 0.05);
            steer_cmd = max(min(steer_cmd, 1.0), -1.0);
            prev_steer = steer_cmd;
        
            prev_yaw_error = yaw_error;
        
            % === 控制速度 ===
            desired_speed = 10;  % m/s
            throttle_cmd = max(0, 0.2 * (desired_speed - v));
            brake_cmd = 0;
            if v - desired_speed > 1.0
                brake_cmd = 0.2;
            end
        
        
            if norm([x, y] - [x_target, y_target]) < 1.0
                is_changing_lane = false;
                disp("✅ 变道完成，切换为直行控制");

                lane_index = lane_index +1;

                state_MRM = 0;
                
                
                % 不再更新 steer，而是进入下一个状态
            end
        
         elseif y-y0 < 60
            % → 保持直行逻辑：让车辆逐步修正方向到朝向 y 轴
            
            % 获取当前位置和朝向
            transform = vehicle_ego.get_transform();
            rotation = transform.rotation;
            yaw_deg = double(rotation.yaw);
            yaw = deg2rad(yaw_deg);
        
            vel = vehicle_ego.get_velocity();
            v = sqrt(double(vel.x)^2 + double(vel.y)^2 + double(vel.z)^2);
        
            % === 目标朝向为“正北”即 y 轴方向 ===
            desired_yaw = pi/2;  % 或者根据原路径最后点计算也可以
            yaw_error = wrapToPi(desired_yaw - yaw);
        
            % === 使用 PID 修正 steer ===
            integral = integral + yaw_error * dt;
            derivative = (yaw_error - prev_yaw_error) / dt;
        
            steer_cmd = Kp * yaw_error + Ki * integral + Kd * derivative;
            %steer_cmd = max(min(steer_cmd, 1.0), -1.0);
        
            % 限制方向盘变化速率
            steer_cmd = max(min(steer_cmd, prev_steer + 0.05), prev_steer - 0.05);
            steer_cmd = max(min(steer_cmd, 1.0), -1.0);
        
         
            prev_steer = steer_cmd;
        
            prev_yaw_error = yaw_error;
        
            % === 控制速度 ===
            desired_speed = 10;
            throttle_cmd = max(0, 0.2 * (desired_speed - v));
            brake_cmd = 0;
        
            if v - desired_speed > 1.0
                brake_cmd = 0.2;
            end
        
         else
             break;
               
          
 
           
         end
        
        
          % 控制输出
          control = py.carla.VehicleControl();
          control.throttle = throttle_cmd;
          control.steer = steer_cmd;
          control.brake = brake_cmd;
          vehicle_ego.apply_control(control);
        
          pause(dt);





          % === 内部主控制循环中 ===
          [i_load, time_log, speed_log, accel_log, brake_log] = log_status( ...
              i_load, time_log, speed_log, accel_log, brake_log, ...
              t_rel, v_ego, ay, brake_cmd);

        end


        
    
    end



    %        % === 记录数据 ===





    % time_log(i_load)  = t_rel;
    % speed_log(i_load) = v_ego;
    % brake_log(i_load) = brake_cmd;
    % accel_log(i_load) = ay;
    % 
    % i_load = i_load + 1;

    % === 外部主控制循环中 ===
    [i_load, time_log, speed_log, accel_log, brake_log] = log_status( ...
        i_load, time_log, speed_log, accel_log, brake_log, ...
        t_rel, v_ego, ay, brake_cmd);



    % 只保留实际长度的数据
    time_log  = time_log(1:i_load-1);
    speed_log = speed_log(1:i_load-1);
    brake_log = brake_log(1:i_load-1);
    accel_log = accel_log(1:i_load-1);





    fprintf("[INFO] time=%.2f | v=%.2f m/s | brake=%.2f | S=%.2f\n", t_rel, v_ego, brake_cmd , S);



    drawnow limitrate;
    pause(0.02);  % 控制节奏（也避免 UI 崩溃）


        

 


    %输出sensor未覆盖角度
    % if ~isempty(uncovered_angles)
    % fprintf('Step %d uncovered_degree: ' , step);
    % fprintf('%d ', uncovered_angles);
    % fprintf('\n');
    % end

%     drawnow;
%     pause(time_step);
% 
% 
% 
% 
% 
%     dist_Rrear = double(y_ego - y_Rrear);
% 
% 
% 
% 
% 
% 
% 

% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
%         % ———— 2) 计算相对速度 & 距离 & TTC ————
%     rel_v_front = v_ego - v_front;           % 正值：正在接近
%     dist_front  = double(y_front - y_ego);
%     if rel_v_front <= 0
%         TTC = Inf;
%     else
%         TTC = dist_front / rel_v_front;
%     end
% 
% 
% 
% 
% 
%      % 计算后方 TTC 与 THW
%      rel_v_rear = v_rear - v_ego;
%      dist_rear = double(y_ego - y_rear);
%      TTC_rear = Inf;
% 
%      if rel_v_rear > 0
%          TTC_rear = dist_rear / rel_v_rear;
%      end
% 
%       % THW_front = dist_front / max(v_ego, 0.1);  % 防除以0
%       % THW_rear = dist_rear / max(v_rear, 0.1);
% 
%      % 日志记录

%        TTC_front_log(end+1) = TTC;
%        TTC_rear_log(end+1) = TTC_rear;
%        % THW_front_log(end+1) = THW_front;
%        % THW_rear_log(end+1) = THW_rear;
% 
% 
% 
% 
% 
% 
%%  打印区
    % 示例：打印当前自车道左侧前方最近点坐标及类型与速度
    if lane_index < num_lanes && all(~isnan(lane_obstacles(lane_index + 1).front))
        fprintf(': left_lane_front [%.2f, %.2f], type: %s, speed: %.2f\n',  ...
            lane_obstacles(lane_index + 1).front, lane_obstacles(lane_index + 1).front_type, lane_obstacles(lane_index + 1).front_speed);
    end


    % 示例：打印当前车道与右侧车道目标
    if all(~isnan(lane_obstacles(lane_index).front))
        fprintf(': lane_front [%.2f, %.2f], type: %s, speed: %.2f, TTC: %.2f , THW: %.2f\n',  ...
            lane_obstacles(lane_index).front, lane_obstacles(lane_index).front_type, lane_obstacles(lane_index).front_speed,TTC_front,THW_front);
    end
    if lane_index > 1 && all(~isnan(lane_obstacles(lane_index - 1).front))
        fprintf(': right_lane_front [%.2f, %.2f], type: %s, speed: %.2f, TTC: %.2f , THW: %.2f\n',  ...
            lane_obstacles(lane_index - 1).front, lane_obstacles(lane_index - 1).front_type, lane_obstacles(lane_index - 1).front_speed,TTC_Rfront,THW_Rfront);
    end

    % 后方
    if lane_index < num_lanes && all(~isnan(lane_obstacles(lane_index + 1).rear))
        fprintf(': left_lane_rear [%.2f, %.2f], type: %s, speed: %.2f\n',  ...
            lane_obstacles(lane_index + 1).rear, lane_obstacles(lane_index + 1).rear_type, lane_obstacles(lane_index + 1).rear_speed);
    end


    if all(~isnan(lane_obstacles(lane_index).rear))
        fprintf(': lane_rear [%.2f, %.2f], type: %s, speed: %.2f, TTC: %.2f , THW: %.2f\n',  ...
            lane_obstacles(lane_index).rear, lane_obstacles(lane_index).rear_type, lane_obstacles(lane_index).rear_speed, TTC_rear,THW_rear);
    end

     if lane_index > 1 && all(~isnan(lane_obstacles(lane_index - 1).rear))
        fprintf(': right_lane_rear [%.2f, %.2f], type: %s, speed: %.2f\n',  ...
            lane_obstacles(lane_index - 1).rear, lane_obstacles(lane_index - 1).rear_type, lane_obstacles(lane_index - 1).rear_speed);
    end








     % ———— 6) 停车检测 & 退出循环 ————
    if abs(v_ego) < tol_v
        fprintf("✅ 自车已停止，MRM 结束\n");
        break;
    end



end






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

% 
% 
% 
% figure;
% subplot(3,1,1);
% plot(t_vec, v_ego_vec, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Ego Speed (m/s)');
% title('Ego Vehicle Speed vs. Time');
% grid on;



% figure;
% plot(t_log, TTC_front_log, 'LineWidth', 1.5); hold on;
% plot(t_log, TTC_rear_log,  'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('TTC [s]');
% title('TTC with Front/Rear Vehicle'); legend('Front', 'Rear'); grid on;
















%% 坐标调整功能 


function [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_fx] = adjust_obstacle_measurement1(obstacle, sensors, car_pos)
    % covered_sensors = [];

        % 最多 sensors 个传感器能被覆盖
    max_count = numel(sensors);
    covered_sensors(max_count) = sensors(1);  % 预分配结构体数组
    cs_idx = 0;  % 实际记录数

    highrisk_obstacles_fx = NaN;

    for s = sensors
        if strcmp(s.active, 'Y') && ismember(s.type, {'Radar', 'LiDAR', 'Camera'})
            dx = obstacle.x - car_pos(1);
            dy = obstacle.y - car_pos(2);
            angle = atan2d(dy, dx);
            rel_angle = mod(angle - s.heading + 180, 360) - 180;
            dist = sqrt(dx^2 + dy^2);

            if abs(rel_angle) <= s.FOV / 2 && dist <= s.range
                % covered_sensors = [covered_sensors, s];
                cs_idx = cs_idx + 1;
                covered_sensors(cs_idx) = s;
            end
        end
    end


       % 截断未使用部分
    covered_sensors = covered_sensors(1:cs_idx);


    % 调整误差
    lidar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'LiDAR')).distanceAcc]);
    radar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).speedAcc]);

    adjusted_x = obstacle.x - lidar_acc - sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).distanceAcc]);
    adjusted_speed = obstacle.speed - radar_acc;

    % 检查是否为高风险
    sensor_types = {covered_sensors.type};
    if isequal(sort(sensor_types), {'Radar'})
        highrisk_obstacles_fx = adjusted_x;
    end
end


function [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_flx] = adjust_obstacle_measurement2(obstacle, sensors, car_pos)
    covered_sensors = [];
    highrisk_obstacles_flx = NaN;

    for s = sensors
        if strcmp(s.active, 'Y') && ismember(s.type, {'Radar', 'LiDAR', 'Camera'})
            dx = obstacle.x - car_pos(1);
            dy = obstacle.y - car_pos(2);
            angle = atan2d(dy, dx);
            rel_angle = mod(angle - s.heading + 180, 360) - 180;
            dist = sqrt(dx^2 + dy^2);

            if abs(rel_angle) <= s.FOV / 2 && dist <= s.range
                covered_sensors = [covered_sensors, s];
            end
        end
    end

    % 调整误差
    lidar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'LiDAR')).distanceAcc]);
    radar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).speedAcc]);

    adjusted_x = obstacle.x - lidar_acc - sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).distanceAcc]);
    adjusted_speed = obstacle.speed - radar_acc;

    % 检查是否为高风险
    sensor_types = {covered_sensors.type};
    if isequal(sort(sensor_types), {'Radar'})
        highrisk_obstacles_flx = adjusted_x;
    end
end


function [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_frx] = adjust_obstacle_measurement3(obstacle, sensors, car_pos)
    covered_sensors = [];
    highrisk_obstacles_frx = NaN;

    for s = sensors
        if strcmp(s.active, 'Y') && ismember(s.type, {'Radar', 'LiDAR', 'Camera'})
            dx = obstacle.x - car_pos(1);
            dy = obstacle.y - car_pos(2);
            angle = atan2d(dy, dx);
            rel_angle = mod(angle - s.heading + 180, 360) - 180;
            dist = sqrt(dx^2 + dy^2);

            if abs(rel_angle) <= s.FOV / 2 && dist <= s.range
                covered_sensors = [covered_sensors, s];
            end
        end
    end

    % 调整误差
    lidar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'LiDAR')).distanceAcc]);
    radar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).speedAcc]);

    adjusted_x = obstacle.x - lidar_acc - sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).distanceAcc]);
    adjusted_speed = obstacle.speed - radar_acc;

    % 检查是否为高风险
    sensor_types = {covered_sensors.type};
    if isequal(sort(sensor_types), {'Radar'})
        highrisk_obstacles_frx = adjusted_x;
    end
end



function [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_rx] = adjust_obstacle_measurement4(obstacle, sensors, car_pos)
    covered_sensors = [];
    highrisk_obstacles_rx = NaN;

    for s = sensors
        if strcmp(s.active, 'Y') && ismember(s.type, {'Radar', 'LiDAR', 'Camera'})
            dx = obstacle.x - car_pos(1);
            dy = obstacle.y - car_pos(2);
            angle = atan2d(dy, dx);
            rel_angle = mod(angle - s.heading + 180, 360) - 180;
            dist = sqrt(dx^2 + dy^2);

            if abs(rel_angle) <= s.FOV / 2 && dist <= s.range
                covered_sensors = [covered_sensors, s];
            end
        end
    end

    % 调整误差
    lidar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'LiDAR')).distanceAcc]);
    radar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).speedAcc]);

    adjusted_x = obstacle.x + lidar_acc + sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).distanceAcc]);
    adjusted_speed = obstacle.speed + radar_acc;

    % 检查是否为高风险
    sensor_types = {covered_sensors.type};
    if isequal(sort(sensor_types), {'Radar'})
        highrisk_obstacles_rx = adjusted_x;
    end
end


function [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_rlx] = adjust_obstacle_measurement5(obstacle, sensors, car_pos)
    covered_sensors = [];
    highrisk_obstacles_rlx = NaN;

    for s = sensors
        if strcmp(s.active, 'Y') && ismember(s.type, {'Radar', 'LiDAR', 'Camera'})
            dx = obstacle.x - car_pos(1);
            dy = obstacle.y - car_pos(2);
            angle = atan2d(dy, dx);
            rel_angle = mod(angle - s.heading + 180, 360) - 180;
            dist = sqrt(dx^2 + dy^2);

            if abs(rel_angle) <= s.FOV / 2 && dist <= s.range
                covered_sensors = [covered_sensors, s];
            end
        end
    end

    % 调整误差
    lidar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'LiDAR')).distanceAcc]);
    radar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).speedAcc]);

    adjusted_x = obstacle.x + lidar_acc + sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).distanceAcc]);
    adjusted_speed = obstacle.speed + radar_acc;

    % 检查是否为高风险
    sensor_types = {covered_sensors.type};
    if isequal(sort(sensor_types), {'Radar'})
        highrisk_obstacles_rlx = adjusted_x;
    end
end


function [adjusted_x, adjusted_speed, covered_sensors, sensor_types, highrisk_obstacles_rrx] = adjust_obstacle_measurement6(obstacle, sensors, car_pos)
    covered_sensors = [];
    highrisk_obstacles_rrx = NaN;

    for s = sensors
        if strcmp(s.active, 'Y') && ismember(s.type, {'Radar', 'LiDAR', 'Camera'})
            dx = obstacle.x - car_pos(1);
            dy = obstacle.y - car_pos(2);
            angle = atan2d(dy, dx);
            rel_angle = mod(angle - s.heading + 180, 360) - 180;
            dist = sqrt(dx^2 + dy^2);

            if abs(rel_angle) <= s.FOV / 2 && dist <= s.range
                covered_sensors = [covered_sensors, s];
            end
        end
    end

    % 调整误差
    lidar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'LiDAR')).distanceAcc]);
    radar_acc = sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).speedAcc]);

    adjusted_x = obstacle.x + lidar_acc + sum([covered_sensors(strcmp({covered_sensors.type}, 'Radar')).distanceAcc]);
    adjusted_speed = obstacle.speed + radar_acc;

    % 检查是否为高风险
    sensor_types = {covered_sensors.type};
    if isequal(sort(sensor_types), {'Radar'})
        highrisk_obstacles_rrx = adjusted_x;
    end
end









function [i_load, time_log, speed_log, accel_log, brake_log] = log_status(i_load, time_log, speed_log, accel_log, brake_log, t_rel, v_ego, ay, brake_cmd)
   
    time_log(i_load)  = t_rel;
    speed_log(i_load) = v_ego;
    accel_log(i_load) = ay;
    brake_log(i_load) = brake_cmd;

     i_load = i_load + 1;

end

