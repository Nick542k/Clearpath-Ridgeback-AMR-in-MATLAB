%% run_ridgeback_3d_nav.m
% Interactive 3D Simscape Navigation for Clearpath Ridgeback AMR.
% Allows changing target destination by CLICKING ANYWHERE on the map window.
%
% USAGE:
%   run_ridgeback_3d_nav            % Opens interactive map (click anywhere to navigate)
%   run_ridgeback_3d_nav(X, Y)      % Opens with initial target at (X, Y)

function run_ridgeback_3d_nav(targetX, targetY, targetYaw)
    if nargin < 1 || isempty(targetX), targetX = 22.0; end
    if nargin < 2 || isempty(targetY), targetY = 22.0; end
    if nargin < 3 || isempty(targetYaw), targetYaw = pi/2; end

    fprintf('==================================================\n');
    fprintf(' Interactive 3D Ridgeback Navigation System      \n');
    fprintf(' Click anywhere on the map to change the target! \n');
    fprintf('==================================================\n');

    % Step 1: Initial build
    build_ridgeback_3d_simulation(targetX, targetY, targetYaw);

    model_name = 'ridgeback_3d_nav';
    map = evalin('base', 'env_occupancy_map');
    w = evalin('base', 'active_path_waypoints');

    % Step 2: Create Interactive GUI Window
    close(findobj('Name', 'Interactive 3D Navigation - Click Map to Set Target'));
    fig = figure('Name', 'Interactive 3D Navigation - Click Map to Set Target', ...
                 'NumberTitle', 'off', 'Position', [60, 50, 1050, 800], ...
                 'Color', [0.96 0.96 0.98]);

    ax = axes(fig, 'Position', [0.07, 0.12, 0.70, 0.82]);
    show(map, 'Parent', ax);
    hold(ax, 'on');
    grid(ax, 'on');
    title(ax, 'Clearpath Ridgeback - Click Anywhere on the Map to Set Target Point', ...
          'FontSize', 12, 'FontWeight', 'bold');
    xlabel(ax, 'X (meters)', 'FontSize', 10);
    ylabel(ax, 'Y (meters)', 'FontSize', 10);

    % Plot active path & markers
    plot(ax, w(:, 1), w(:, 2), 'c-', 'LineWidth', 3.0, 'DisplayName', 'Planned Route', 'Tag', 'ActivePath');
    plot(ax, 3.0, 3.0, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'LineWidth', 2, 'DisplayName', 'Start Point [3, 3]');
    plot(ax, targetX, targetY, 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2, ...
         'DisplayName', sprintf('Target Point [%.1f, %.1f]', targetX, targetY), 'Tag', 'TargetMarker');

    legend(ax, 'Location', 'northeastoutside', 'FontSize', 9);

    % Instruction & Status Banner
    statusText = text(ax, 1.5, 24.0, ...
        sprintf('👉 Click ANYWHERE on the map to set a new Target Destination point!'), ...
        'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [1.0 1.0 0.85], ...
        'EdgeColor', [0.7 0.7 0.2], 'Tag', 'StatusBanner');

    % UI Panel with Controls on the right
    uicontrol(fig, 'Style', 'text', 'String', 'Interactive Controls', ...
              'Position', [820, 720, 200, 30], 'FontSize', 12, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.96 0.96 0.98], 'HorizontalAlignment', 'center');

    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '🎯 Click Map to Pick Target', ...
              'Position', [820, 660, 200, 45], 'FontSize', 10, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.20, 0.65, 0.90], 'ForegroundColor', 'w', ...
              'Callback', @(~,~) click_pick_callback(ax, statusText, model_name));

    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '▶ Replay 3D Simulation', ...
              'Position', [820, 605, 200, 40], 'FontSize', 10, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.30, 0.75, 0.40], 'ForegroundColor', 'w', ...
              'Callback', @(~,~) replay_callback(model_name, statusText));

    % Quick Target Preset Buttons
    uicontrol(fig, 'Style', 'text', 'String', 'Quick Preset Targets:', ...
              'Position', [820, 550, 200, 20], 'FontSize', 10, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.96 0.96 0.98], 'HorizontalAlignment', 'left');

    uicontrol(fig, 'Style', 'pushbutton', 'String', '📍 Far Corner [22, 22]', ...
              'Position', [820, 510, 200, 32], 'FontSize', 9, ...
              'Callback', @(~,~) execute_target_nav(22.0, 22.0, ax, statusText, model_name));

    uicontrol(fig, 'Style', 'pushbutton', 'String', '📍 Central Plaza [12.5, 12.5]', ...
              'Position', [820, 470, 200, 32], 'FontSize', 9, ...
              'Callback', @(~,~) execute_target_nav(12.5, 12.5, ax, statusText, model_name));

    uicontrol(fig, 'Style', 'pushbutton', 'String', '📍 South-East Bay [18, 9]', ...
              'Position', [820, 430, 200, 32], 'FontSize', 9, ...
              'Callback', @(~,~) execute_target_nav(18.0, 9.0, ax, statusText, model_name));

    uicontrol(fig, 'Style', 'pushbutton', 'String', '📍 North-West Bay [8, 18]', ...
              'Position', [820, 390, 200, 32], 'FontSize', 9, ...
              'Callback', @(~,~) execute_target_nav(8.0, 18.0, ax, statusText, model_name));

    % Status info box
    infoBox = uicontrol(fig, 'Style', 'text', ...
              'String', sprintf('Current Target: [%.1f, %.1f]\nStatus: Ready for Click', targetX, targetY), ...
              'Position', [820, 260, 200, 70], 'FontSize', 9, ...
              'BackgroundColor', [1.0 1.0 1.0], 'Tag', 'InfoBox');

    % Direct Map Click Callback (Click anywhere on the axes!)
    set(fig, 'WindowButtonDownFcn', @(src, ~) map_window_click(src, ax, statusText, model_name, infoBox));

    drawnow;

    % Step 3: Run initial 3D simulation
    fprintf('Running initial 3D simulation (Mechanics Explorer will open)...\n');
    execute_target_nav(targetX, targetY, ax, statusText, model_name, infoBox);
end

% Callback when clicking anywhere in the figure window
function map_window_click(~, ax, statusText, model_name, infoBox)
    cp = get(ax, 'CurrentPoint');
    clickX = cp(1, 1);
    clickY = cp(1, 2);

    % Verify click is inside the 25m x 25m arena
    if clickX >= 1.5 && clickX <= 23.5 && clickY >= 1.5 && clickY <= 23.5
        execute_target_nav(clickX, clickY, ax, statusText, model_name, infoBox);
    end
end

% Callback for the "Click Map to Pick Target" button
function click_pick_callback(ax, statusText, model_name)
    set(statusText, 'String', '🎯 Click ANYWHERE on the map with the crosshair cursor...', ...
        'BackgroundColor', [1.0, 0.9, 0.5]);
    drawnow;
    
    [clickX, clickY, btn] = ginput(1);
    if ~isempty(clickX) && btn == 1
        if clickX >= 1.5 && clickX <= 23.5 && clickY >= 1.5 && clickY <= 23.5
            fig = ancestor(ax, 'figure');
            infoBox = findobj(fig, 'Tag', 'InfoBox');
            execute_target_nav(clickX, clickY, ax, statusText, model_name, infoBox);
        end
    end
end

% Execute navigation to target coordinates
function execute_target_nav(targetX, targetY, ax, statusText, model_name, infoBox)
    fprintf('\n--> Target clicked at [%.2f, %.2f]. Updating 3D model & planning path...\n', targetX, targetY);

    if ~isempty(statusText) && isvalid(statusText)
        set(statusText, 'String', sprintf('⏳ Planning path & updating 3D simulation for Target [%.1f, %.1f]...', targetX, targetY), ...
            'BackgroundColor', [1.0, 0.95, 0.70]);
    end
    if nargin >= 6 && ~isempty(infoBox) && isvalid(infoBox)
        set(infoBox, 'String', sprintf('Target: [%.1f, %.1f]\nStatus: Simulating in 3D...', targetX, targetY));
    end
    drawnow;

    % 1. Rebuild model with 3D target markers at clicked coordinates
    build_ridgeback_3d_simulation(targetX, targetY);

    % 2. Update 2D figure with new route and target marker
    w = evalin('base', 'active_path_waypoints');
    h_path = findobj(ax, 'Tag', 'ActivePath');
    if ~isempty(h_path), set(h_path, 'XData', w(:, 1), 'YData', w(:, 2)); end

    h_target = findobj(ax, 'Tag', 'TargetMarker');
    if ~isempty(h_target)
        set(h_target, 'XData', targetX, 'YData', targetY, ...
            'DisplayName', sprintf('Target Point [%.1f, %.1f]', targetX, targetY));
    end
    drawnow;

    % 3. Run Simscape Multibody 3D simulation
    if ~bdIsLoaded(model_name), load_system(model_name); end
    simOut = sim(model_name);

    fprintf('--> 3D Simulation finished! Ridgeback arrived at [%.2f, %.2f] in Mechanics Explorer.\n', targetX, targetY);

    if ~isempty(statusText) && isvalid(statusText)
        set(statusText, 'String', sprintf('✅ Arrived at [%.1f, %.1f]! Click anywhere on the map to set a new target.', targetX, targetY), ...
            'BackgroundColor', [0.85, 1.0, 0.85]);
    end
    if nargin >= 6 && ~isempty(infoBox) && isvalid(infoBox)
        set(infoBox, 'String', sprintf('Target: [%.1f, %.1f]\nStatus: ✅ Arrived!\nClick map to change.', targetX, targetY));
    end
    drawnow;
end

% Replay existing simulation
function replay_callback(model_name, statusText)
    set(statusText, 'String', '▶ Replaying 3D simulation in Mechanics Explorer...', 'BackgroundColor', [0.8, 0.95, 1.0]);
    drawnow;
    sim(model_name);
    set(statusText, 'String', '✅ Replay finished. Click anywhere on the map to set a new target.', 'BackgroundColor', [0.85, 1.0, 0.85]);
    drawnow;
end
