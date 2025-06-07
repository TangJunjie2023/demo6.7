
terminate(pyenv);   % 终止旧的 Python 会话
pyenv('Version', 'E:\python39\python.exe');  % 替换成你用的 python 路径



pyenv('Version', 'E:\python39\python.EXE', 'ExecutionMode', 'OutOfProcess');
py.list({'trigger'});








client = py.carla.Client('localhost', int32(2000));
client.set_timeout(60.0);

%world.debug.clear();   % ✅ 清除所有调试对象
%world.tick();          % ✅ 确保绘制帧同步清理

% 加载地图 Town04（高速路直道地图）
world = client.load_world('Town04');

map = world.get_map();
blueprints = world.get_blueprint_library();
disp("Connected to: " + string(map.name));