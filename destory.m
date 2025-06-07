actor_list = py.list(world.get_actors());

for i = 1:length(actor_list)
    actor = actor_list{i};
    type_id = string(actor.type_id);

    try
        % 只销毁车辆、行人、传感器，避免 spectator/traffic lights 等系统 actor
        if startsWith(type_id, "vehicle.") || ...
           startsWith(type_id, "walker.") || ...
           startsWith(type_id, "sensor.")

            actor.destroy();
        end
    catch
        % 安全忽略已销毁或不可销毁对象
    end
end

disp("✅ 可销毁的车辆/传感器已清除。");