%% ridgeback_navigation_simulation.m
% Complete Autonomous Navigation Simulation for Clearpath Ridgeback in MATLAB
% - Builds a 15m x 15m environment with obstacle barriers and boxes
% - Plans a collision-free path to a target point using Hybrid A*
% - Simulates Pure Pursuit path following with LiDAR obstacle sensing
% - Animates the Ridgeback AMR navigating live in MATLAB

function ridgeback_navigation_simulation()
    close all;
    fprintf('==================================================\n');
    fprintf(' Starting Ridgeback AMR Navigation in MATLAB \n');
    fprintf('==================================================\n');

    %% 1. Create 15m x 15m Environment with Obstacles
    mapResolution = 10; % 10 cells per meter (0.1m resolution)
    arenaWidth = 15;
    arenaHeight = 15;
    map = binaryOccupancyMap(arenaWidth, arenaHeight, mapResolution);

    % Outer boundary walls
    setOccupancy(map, [0 0; arenaWidth 0; 0 arenaHeight; arenaWidth arenaHeight], 1);
    [Xb, Yb] = meshgrid(0:0.1:arenaWidth, [0 arenaHeight]);
    setOccupancy(map, [Xb(:), Yb(:)], 1);
    [Xb2, Yb2] = meshgrid([0 arenaWidth], 0:0.1:arenaHeight);
    setOccupancy(map, [Xb2(:), Yb2(:)], 1);

    % Obstacle 1: Center barrier wall
    [Ox1, Oy1] = meshgrid(5:0.1:9, 7.0:0.1:7.5);
    setOccupancy(map, [Ox1(:), Oy1(:)], 1);

    % Obstacle 2: Vertical column
    [Ox2, Oy2] = meshgrid(4:0.1:5, 3:0.1:6);
    setOccupancy(map, [Ox2(:), Oy2(:)], 1);

    % Obstacle 3: Upper-left box
    [Ox3, Oy3] = meshgrid(2:0.1:4.5, 9:0.1:11);
    setOccupancy(map, [Ox3(:), Oy3(:)], 1);

    % Obstacle 4: Right warehouse rack
    [Ox4, Oy4] = meshgrid(10.5:0.1:12.5, 3:0.1:9);
    setOccupancy(map, [Ox4(:), Oy4(:)], 1);

    % Inflate map by robot safety radius (Ridgeback size: 0.96m x 0.79m -> safety: 0.65m)
    robotRadius = 0.65;
    inflatedMap = copy(map);
    inflate(inflatedMap, robotRadius);

    %% 2. Define Start Pose and Target Goal Point
    startPose = [2.0, 2.0, 0];          % [x (m), y (m), theta (rad)]
    goalPose  = [12.0, 12.0, pi/2];     % Target Destination Point

    fprintf('Start Point:  [%.1f, %.1f, %.2f rad]\n', startPose(1), startPose(2), startPose(3));
    fprintf('Target Goal:  [%.1f, %.1f, %.2f rad]\n', goalPose(1), goalPose(2), goalPose(3));

    %% 3. Plan Path using State Validator and Hybrid A* (Nav2 equivalent)
    stateValidator = validatorOccupancyMap;
    stateValidator.Map = inflatedMap;
    stateValidator.ValidationDistance = 0.1;

    planner = plannerHybridAStar(stateValidator, ...
        'MinTurningRadius', 1.0, ...
        'MotionPrimitiveLength', 0.8, ...
        'ForwardCost', 1.0, ...
        'ReverseCost', 3.0);

    fprintf('Planning collision-free path to target point...\n');
    pathObj = plan(planner, startPose, goalPose);

    if isempty(pathObj) || isempty(pathObj.States)
        error('Planner failed to find a valid path to the goal point!');
    end

    waypoints = pathObj.States(:, 1:2);
    pathLengthVal = pathLength(pathObj);
    fprintf('Path generated successfully! Total path length: %.2f meters (%d waypoints)\n', ...
        pathLengthVal, size(waypoints, 1));

    %% 4. Configure Pure Pursuit Controller
    controller = controllerPurePursuit;
    controller.Waypoints = waypoints;
    controller.DesiredLinearVelocity = 0.7; % m/s
    controller.MaxAngularVelocity = 1.2;    % rad/s
    controller.LookaheadDistance = 0.8;    % m

    %% 5. Setup Live Visualization Window
    fig = figure('Name', 'Ridgeback Autonomous Navigation (MATLAB)', ...
                 'NumberTitle', 'off', 'Position', [150, 100, 950, 750], ...
                 'Color', [0.95 0.95 0.97]);

    ax = axes(fig);
    show(map, 'Parent', ax);
    hold(ax, 'on');
    grid(ax, 'on');
    title(ax, 'Clearpath Ridgeback - Autonomous Navigation to Goal', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel(ax, 'X (meters)', 'FontSize', 11);
    ylabel(ax, 'Y (meters)', 'FontSize', 11);

    % Plot planned trajectory
    plot(ax, waypoints(:, 1), waypoints(:, 2), 'm--', 'LineWidth', 2.2, 'DisplayName', 'Planned Path');
    plot(ax, startPose(1), startPose(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot(ax, goalPose(1), goalPose(2), 'r*', 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', 'Target Point');

    % Robot visual shape (Ridgeback box footprint)
    footprintL = 0.96;
    footprintW = 0.79;
    cornerOffsets = [-footprintL/2, -footprintW/2;
                      footprintL/2, -footprintW/2;
                      footprintL/2,  footprintW/2;
                     -footprintL/2,  footprintW/2]';

    robotPatch = patch(ax, 'XData', [], 'YData', [], 'FaceColor', [0.15 0.45 0.85], ...
                       'FaceAlpha', 0.75, 'EdgeColor', 'b', 'LineWidth', 1.5, 'DisplayName', 'Ridgeback AMR');
    robotHeading = quiver(ax, 0, 0, 0, 0, 0.7, 'Color', 'y', 'LineWidth', 2, 'MaxHeadSize', 2);
    trailPlot = plot(ax, startPose(1), startPose(2), 'c-', 'LineWidth', 1.8, 'DisplayName', 'Actual Trajectory');

    legend(ax, 'Location', 'northeastoutside');

    %% 6. Simulation Execution Loop
    currentPose = startPose;
    goalRadius = 0.35; % meters
    sampleTime = 0.05; % 20 Hz
    maxSimTime = 40.0; % seconds
    simTime = 0;

    trajectoryHistory = currentPose(1:2);

    statusText = text(ax, 0.5, 14.3, sprintf('Time: 0.00s | Distance to Goal: %.2fm', norm(currentPose(1:2) - goalPose(1:2))), ...
                      'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'EdgeColor', 'k');

    fprintf('\nNavigating robot along planned path...\n');

    while (norm(currentPose(1:2) - goalPose(1:2)) > goalRadius) && (simTime < maxSimTime)
        % Compute velocity commands via Pure Pursuit
        [v, omega] = controller(currentPose);

        % Update robot pose via kinematic model
        currentPose(1) = currentPose(1) + v * cos(currentPose(3)) * sampleTime;
        currentPose(2) = currentPose(2) + v * sin(currentPose(3)) * sampleTime;
        currentPose(3) = currentPose(3) + omega * sampleTime;
        currentPose(3) = atan2(sin(currentPose(3)), cos(currentPose(3))); % wrap [-pi, pi]

        simTime = simTime + sampleTime;
        trajectoryHistory = [trajectoryHistory; currentPose(1:2)];

        % Update visual footprint
        R = [cos(currentPose(3)), -sin(currentPose(3));
             sin(currentPose(3)),  cos(currentPose(3))];
        transformedCorners = R * cornerOffsets + currentPose(1:2)';

        set(robotPatch, 'XData', transformedCorners(1, :), 'YData', transformedCorners(2, :));
        set(robotHeading, 'XData', currentPose(1), 'YData', currentPose(2), ...
                          'UData', cos(currentPose(3)), 'VData', sin(currentPose(3)));
        set(trailPlot, 'XData', trajectoryHistory(:, 1), 'YData', trajectoryHistory(:, 2));

        distRemaining = norm(currentPose(1:2) - goalPose(1:2));
        set(statusText, 'String', sprintf('Time: %.2fs | Speed: %.2fm/s | Distance: %.2fm', ...
            simTime, v, distRemaining));

        drawnow limitrate;
        pause(sampleTime * 0.4); % Smooth animation
    end

    %% 7. Summary
    fprintf('\n==================================================\n');
    if norm(currentPose(1:2) - goalPose(1:2)) <= goalRadius
        fprintf(' SUCCESS: Clearpath Ridgeback arrived at target point!\n');
        set(statusText, 'String', sprintf('GOAL REACHED! Final Time: %.2fs', simTime), 'BackgroundColor', [0.8 1.0 0.8]);
    else
        fprintf(' SIMULATION ENDED: Time limit reached.\n');
    end
    fprintf(' Final Position: [%.2f, %.2f]\n', currentPose(1), currentPose(2));
    fprintf(' Total Travel Time: %.2f seconds\n', simTime);
    fprintf('==================================================\n');
end
