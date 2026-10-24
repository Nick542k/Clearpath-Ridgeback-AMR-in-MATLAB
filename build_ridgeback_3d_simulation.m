%% build_ridgeback_3d_simulation.m
% Builds the expanded 25m x 25m 3D navigation simulation model in Simscape Multibody.
% Configures precise wheel-to-ground contact (Z = +0.22m elevation) so wheels sit
% exactly flush on top of the floor with zero penetration.

function build_ridgeback_3d_simulation(goalX, goalY, goalYaw)
    if nargin < 1 || isempty(goalX), goalX = 22.0; end
    if nargin < 2 || isempty(goalY), goalY = 22.0; end
    if nargin < 3 || isempty(goalYaw), goalYaw = pi/2; end

    fprintf('==================================================\n');
    fprintf(' Building 3D Navigation Model for Target: [%.1f, %.1f]\n', goalX, goalY);
    fprintf(' (Calibrating Wheel Ground Contact at Z = 0.00m) \n');
    fprintf('==================================================\n');

    src_model = 'ridgeback';
    target_model = 'ridgeback_3d_nav';

    if bdIsLoaded(target_model), close_system(target_model, 0); end
    if bdIsLoaded(src_model), close_system(src_model, 0); end

    fprintf('[1/6] Loading base Ridgeback robot model...\n');
    load_system(src_model);
    save_system(src_model, target_model);
    close_system(src_model, 0);
    load_system(target_model);

    % Safeguard: Remove any legacy ground block (Brick Solid) from original URDF import
    brick_blk = [target_model '/Brick Solid'];
    if getSimulinkBlockHandle(brick_blk) ~= -1
        ph_brick = get_param(brick_blk, 'PortHandles');
        all_ports = [ph_brick.RConn, ph_brick.LConn];
        for p = all_ports
            line_h = get_param(p, 'Line');
            if line_h ~= -1
                delete_line(line_h);
            end
        end
        delete_block(brick_blk);
        fprintf('      Removed legacy ground block (Brick Solid).\n');
    end

    fprintf('[2/6] Configuring MobileBaseJoint & Wheel Height Offset (+0.22m)...\n');

    bl_ph = get_param([target_model '/base_link'], 'PortHandles');
    line_h = get_param(bl_ph.LConn(1), 'Line');
    if line_h ~= -1
        delete_line(line_h);
    end

    pj_name = [target_model '/MobileBaseJoint'];
    if getSimulinkBlockHandle(pj_name) == -1
        add_block('sm_lib/Joints/Planar Joint', pj_name, ...
            'Position', [120, 180, 190, 260]);
    end
    set_param(pj_name, 'PxMotionActuationMode', 'InputMotion');
    set_param(pj_name, 'PyMotionActuationMode', 'InputMotion');
    set_param(pj_name, 'RzMotionActuationMode', 'InputMotion');
    set_param(pj_name, 'PxTorqueActuationMode', 'ComputedTorque');
    set_param(pj_name, 'PyTorqueActuationMode', 'ComputedTorque');
    set_param(pj_name, 'RzTorqueActuationMode', 'ComputedTorque');

    pj_ph = get_param(pj_name, 'PortHandles');
    w_ph = get_param([target_model '/World'], 'PortHandles');

    % Add Rigid Transform for exact Wheel-to-Ground Contact:
    % Wheel axle is at Z = -0.10m, wheel radius is 0.12m -> Lowest wheel point is at -0.22m.
    % Elevating base_link by +0.22m places wheel contact patch exactly at Z = 0.00m (floor surface).
    rt_chassis_name = [target_model '/RT_BaseLink_Height'];
    if getSimulinkBlockHandle(rt_chassis_name) == -1
        add_block('sm_lib/Frames and Transforms/Rigid Transform', rt_chassis_name, ...
            'Position', [220, 200, 260, 240]);
    end
    set_param(rt_chassis_name, 'TranslationMethod', 'Cartesian');
    set_param(rt_chassis_name, 'TranslationCartesianOffset', '[0 0 0.22]');
    set_param(rt_chassis_name, 'TranslationCartesianOffsetUnits', 'm');

    rt_chassis_ph = get_param(rt_chassis_name, 'PortHandles');

    % Connect World -> MobileBaseJoint Base (LConn1)
    add_line(target_model, w_ph.RConn(1), pj_ph.LConn(1), 'autorouting', 'smart');

    % Connect MobileBaseJoint Follower (RConn1) -> RT_BaseLink_Height (LConn1)
    add_line(target_model, pj_ph.RConn(1), rt_chassis_ph.LConn(1), 'autorouting', 'smart');

    % Connect RT_BaseLink_Height (RConn1) -> base_link (LConn1)
    add_line(target_model, rt_chassis_ph.RConn(1), bl_ph.LConn(1), 'autorouting', 'smart');

    axes_labels = {'X', 'Y', 'Yaw'};
    units = {'m', 'm', 'rad'};
    var_names = {'sim_traj_x', 'sim_traj_y', 'sim_traj_yaw'};

    for k = 1:3
        ps_name = [target_model '/PS_' axes_labels{k}];
        ws_name = [target_model '/FromWs_' axes_labels{k}];
        y_pos = 160 + (k-1) * 60;
        
        add_block('nesl_utility/Simulink-PS Converter', ps_name, ...
            'Position', [40, y_pos, 75, y_pos + 30]);
        set_param(ps_name, 'Unit', units{k});
        set_param(ps_name, 'FilteringAndDerivatives', 'filter');
        set_param(ps_name, 'SimscapeFilterOrder', '2');
        set_param(ps_name, 'InputFilterTimeConstant', '0.02');

        add_block('simulink/Sources/From Workspace', ws_name, ...
            'Position', [-60, y_pos + 5, -5, y_pos + 25]);
        set_param(ws_name, 'VariableName', var_names{k});
        set_param(ws_name, 'Interpolate', 'on');

        from_ph = get_param(ws_name, 'PortHandles');
        ps_ph = get_param(ps_name, 'PortHandles');
        add_line(target_model, from_ph.Outport(1), ps_ph.Inport(1), 'autorouting', 'smart');
        add_line(target_model, ps_ph.RConn(1), pj_ph.LConn(1 + k), 'autorouting', 'smart');
    end

    fprintf('[3/6] Placing 3D Obstacles, Walls, and Start/Target Markers...\n');

    env_objects = {
        % 1. Ground Floor (25m x 25m, Top Surface at Z = 0.00m)
        'Env_Ground_Floor',       [12.5, 12.5, -0.05],  [25.0, 25.0, 0.1],  [0.68, 0.70, 0.73], 'brick';
        
        % 2. Perimeter Enclosure Walls (25m Boundary, Base at Z = 0.00m)
        'Env_Wall_South',         [12.5, -0.125, 0.60], [25.5, 0.25, 1.2],  [0.28, 0.32, 0.36], 'brick';
        'Env_Wall_North',         [12.5, 25.125, 0.60], [25.5, 0.25, 1.2],  [0.28, 0.32, 0.36], 'brick';
        'Env_Wall_West',          [-0.125, 12.5, 0.60], [0.25, 25.0, 1.2],  [0.28, 0.32, 0.36], 'brick';
        'Env_Wall_East',          [25.125, 12.5, 0.60], [0.25, 25.0, 1.2],  [0.28, 0.32, 0.36], 'brick';

        % 3. Start Point Visual Marker (Green Pad + Beacon at [3.0, 3.0])
        'Marker_Start_Pad',       [3.0, 3.0, 0.005],    [0.90, 0.01, 0.0],  [0.15, 0.85, 0.25], 'cylinder';
        'Marker_Start_Beacon',    [3.0, 3.0, 0.60],     [0.05, 1.20, 0.0],  [0.20, 0.95, 0.35], 'cylinder';
        'Marker_Start_Flag',      [3.20, 3.0, 1.10],    [0.35, 0.02, 0.25], [0.20, 0.95, 0.35], 'brick';

        % 4. Dynamic Target Destination Marker (Red Pad + Beacon at [goalX, goalY])
        'Marker_Goal_Pad',        [goalX, goalY, 0.005],[0.90, 0.01, 0.0],  [0.92, 0.15, 0.15], 'cylinder';
        'Marker_Goal_Beacon',     [goalX, goalY, 0.60], [0.05, 1.20, 0.0],  [1.00, 0.20, 0.20], 'cylinder';
        'Marker_Goal_Flag',       [goalX+0.2, goalY, 1.10], [0.35, 0.02, 0.25], [1.00, 0.85, 0.10], 'brick';

        % 5. Warehouse Shelving Racks (Base resting flush on floor Z = 0.00m)
        'Obs_Rack_SW',            [7.0, 5.75, 0.90],    [4.0, 1.5, 1.8],    [0.15, 0.30, 0.65], 'brick';
        'Obs_Rack_SE',            [18.0, 5.75, 0.90],   [4.0, 1.5, 1.8],    [0.15, 0.30, 0.65], 'brick';
        'Obs_Rack_NW',            [7.0, 19.25, 0.90],   [4.0, 1.5, 1.8],    [0.15, 0.30, 0.65], 'brick';
        'Obs_Rack_NE',            [18.0, 19.25, 0.90],  [4.0, 1.5, 1.8],    [0.15, 0.30, 0.65], 'brick';

        % 6. Central Plaza Crates
        'Obs_Crate_Block_1',      [10.25, 14.25, 0.75], [2.5, 2.5, 1.5],    [0.90, 0.55, 0.15], 'brick';
        'Obs_Crate_Block_2',      [14.75, 10.75, 0.75], [2.5, 2.5, 1.5],    [0.90, 0.55, 0.15], 'brick';

        % 7. Cylindrical Warehouse Columns
        'Obs_Pillar_West',        [6.5, 12.5, 1.25],    [0.50, 2.5, 0.0],   [0.20, 0.70, 0.65], 'cylinder';
        'Obs_Pillar_East',        [18.5, 12.5, 1.25],   [0.50, 2.5, 0.0],   [0.20, 0.70, 0.65], 'cylinder';

        % 8. Pallet Stacks
        'Obs_Pallet_Stack_1',     [4.5, 10.0, 0.60],    [2.0, 2.0, 1.2],    [0.70, 0.45, 0.20], 'brick';
        'Obs_Pallet_Stack_2',     [20.5, 15.0, 0.60],   [2.0, 2.0, 1.2],    [0.70, 0.45, 0.20], 'brick'
    };

    for i = 1:size(env_objects, 1)
        name = env_objects{i, 1};
        pos = env_objects{i, 2};
        dims = env_objects{i, 3};
        col = env_objects{i, 4};
        shape = env_objects{i, 5};

        rt_name = [target_model '/RT_' name];
        body_name = [target_model '/' name];

        row = mod(i-1, 10);
        col_offset = floor((i-1) / 10) * 150;
        add_block('sm_lib/Frames and Transforms/Rigid Transform', rt_name, ...
            'Position', [-600 - col_offset, 80 + row*55, -560 - col_offset, 120 + row*55]);
        set_param(rt_name, 'TranslationMethod', 'Cartesian');
        set_param(rt_name, 'TranslationCartesianOffset', sprintf('[%.2f %.2f %.2f]', pos(1), pos(2), pos(3)));
        set_param(rt_name, 'TranslationCartesianOffsetUnits', 'm');

        if strcmp(shape, 'cylinder')
            add_block('sm_lib/Body Elements/Cylindrical Solid', body_name, ...
                'Position', [-520 - col_offset, 80 + row*55, -460 - col_offset, 120 + row*55]);
            set_param(body_name, 'CylinderRadius', num2str(dims(1)));
            set_param(body_name, 'CylinderLength', num2str(dims(2)));
        else
            add_block('sm_lib/Body Elements/Brick Solid', body_name, ...
                'Position', [-520 - col_offset, 80 + row*55, -460 - col_offset, 120 + row*55]);
            set_param(body_name, 'BrickDimensions', sprintf('[%.2f %.2f %.2f]', dims(1), dims(2), dims(3)));
        end
        set_param(body_name, 'GraphicDiffuseColor', sprintf('[%.2f %.2f %.2f]', col(1), col(2), col(3)));

        rt_ph = get_param(rt_name, 'PortHandles');
        body_ph = get_param(body_name, 'PortHandles');
        add_line(target_model, w_ph.RConn(1), rt_ph.LConn(1), 'autorouting', 'smart');
        add_line(target_model, rt_ph.RConn(1), body_ph.RConn(1), 'autorouting', 'smart');
    end

    fprintf('[4/6] Planning Path to Target Destination [%.1f, %.1f]...\n', goalX, goalY);

    map = binaryOccupancyMap(25, 25, 5);
    [Xb, Yb] = meshgrid(0:0.2:25, [0:0.2:1.5, 23.5:0.2:25]); setOccupancy(map, [Xb(:), Yb(:)], 1);
    [Xb2, Yb2] = meshgrid([0:0.2:1.5, 23.5:0.2:25], 0:0.2:25); setOccupancy(map, [Xb2(:), Yb2(:)], 1);

    % Racks & Obstacles
    [Ox1, Oy1] = meshgrid(5.0:0.2:9.0, 5.0:0.2:6.5); setOccupancy(map, [Ox1(:), Oy1(:)], 1);
    [Ox2, Oy2] = meshgrid(16.0:0.2:20.0, 5.0:0.2:6.5); setOccupancy(map, [Ox2(:), Oy2(:)], 1);
    [Ox3, Oy3] = meshgrid(5.0:0.2:9.0, 18.5:0.2:20.0); setOccupancy(map, [Ox3(:), Oy3(:)], 1);
    [Ox4, Oy4] = meshgrid(16.0:0.2:20.0, 18.5:0.2:20.0); setOccupancy(map, [Ox4(:), Oy4(:)], 1);
    [Ox5, Oy5] = meshgrid(9.0:0.2:11.5, 13.0:0.2:15.5); setOccupancy(map, [Ox5(:), Oy5(:)], 1);
    [Ox6, Oy6] = meshgrid(13.5:0.2:16.0, 9.5:0.2:12.0); setOccupancy(map, [Ox6(:), Oy6(:)], 1);
    [Ox7, Oy7] = meshgrid(6.0:0.2:7.0, 12.0:0.2:13.0); setOccupancy(map, [Ox7(:), Oy7(:)], 1);
    [Ox8, Oy8] = meshgrid(18.0:0.2:19.0, 12.0:0.2:13.0); setOccupancy(map, [Ox8(:), Oy8(:)], 1);

    inflated = copy(map);
    inflate(inflated, 0.65);

    sv = validatorOccupancyMap;
    sv.Map = inflated;
    sv.ValidationDistance = 0.2;

    planner = plannerHybridAStar(sv, 'MinTurningRadius', 1.2, 'MotionPrimitiveLength', 1.0);

    startPose = [3.0, 3.0, 0];
    goalPose  = [goalX, goalY, goalYaw];

    midPose = [ (startPose(1) + goalX)/2, (startPose(2) + goalY)/2, atan2(goalY - startPose(2), goalX - startPose(1)) ];
    
    try
        pA = plan(planner, startPose, midPose);
        pB = plan(planner, [midPose(1), midPose(2), midPose(3)], goalPose);
        w_selected = [pA.States(:, 1:2); pB.States(:, 1:2)];
        len = pathLength(pA) + pathLength(pB);
    catch
        p_direct = plan(planner, startPose, goalPose);
        w_selected = p_direct.States(:, 1:2);
        len = pathLength(p_direct);
    end

    fprintf('      Collision-free path planned! Length: %.2f meters (%d waypoints)\n', len, size(w_selected, 1));

    assignin('base', 'active_path_waypoints', w_selected);
    assignin('base', 'env_occupancy_map', map);
    assignin('base', 'current_target_point', [goalX, goalY, goalYaw]);

    fprintf('[5/6] Generating trajectory along planned path...\n');

    pp = controllerPurePursuit('Waypoints', w_selected, 'DesiredLinearVelocity', 0.90, ...
                               'MaxAngularVelocity', 1.2, 'LookaheadDistance', 1.0);

    cur = startPose;
    dt = 0.05;
    t_arr = 0;
    x_arr = cur(1);
    y_arr = cur(2);
    yaw_arr = cur(3);

    while (norm(cur(1:2) - goalPose(1:2)) > 0.40) && (t_arr(end) < 65.0)
        [v, w] = pp(cur);
        cur(1) = cur(1) + v * cos(cur(3)) * dt;
        cur(2) = cur(2) + v * sin(cur(3)) * dt;
        cur(3) = cur(3) + w * dt;
        cur(3) = atan2(sin(cur(3)), cos(cur(3)));

        t_next = t_arr(end) + dt;
        t_arr = [t_arr; t_next];
        x_arr = [x_arr; cur(1)];
        y_arr = [y_arr; cur(2)];
        yaw_arr = [yaw_arr; cur(3)];
    end

    totalSimTime = t_arr(end);
    fprintf('      Trajectory generated: %.2f seconds (%d points)\n', totalSimTime, length(t_arr));

    assignin('base', 'sim_traj_x', [t_arr, x_arr]);
    assignin('base', 'sim_traj_y', [t_arr, y_arr]);
    assignin('base', 'sim_traj_yaw', [t_arr, yaw_arr]);

    set_param(target_model, 'StopTime', num2str(totalSimTime));
    save_system(target_model);

    fprintf('[6/6] Model updated and saved as %s.slx.\n', target_model);
    fprintf('      Wheels sitting exactly at ground level Z = 0.00m.\n');
    fprintf('==================================================\n');
end
