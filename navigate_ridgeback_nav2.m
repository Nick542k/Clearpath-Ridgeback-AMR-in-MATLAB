%% navigate_ridgeback_nav2.m
% Send navigation goals to ROS 2 Nav2 for Ridgeback from MATLAB

function navigate_ridgeback_nav2(target_x, target_y, target_yaw)
    if nargin < 1, target_x = 2.0; end
    if nargin < 2, target_y = 1.0; end
    if nargin < 3, target_yaw = 0.0; end

    fprintf('Connecting to ROS 2 Nav2 action server (/navigate_to_pose)...\n');
    
    % Initialize MATLAB ROS 2 node
    try
        navNode = ros2node('/matlab_nav_commander');
    catch
        % Reuse existing node if already initialized
        nodes = ros2('node', 'list');
        fprintf('Using active ROS 2 environment.\n');
    end

    % Create Nav2 action client
    navClient = ros2actionclient(navNode, '/navigate_to_pose', 'nav2_msgs/NavigateToPose');
    
    fprintf('Waiting for Nav2 server to become available...\n');
    serverReady = waitForServer(navClient, 'Timeout', 10);
    if ~serverReady
        error('Nav2 action server (/navigate_to_pose) not found. Ensure simulation and Nav2 are running.');
    end

    % Prepare goal message
    goalMsg = ros2message(navClient);
    goalMsg.pose.header.frame_id = 'map';
    goalMsg.pose.header.stamp = ros2time(navNode, 'now');
    
    % Set target position
    goalMsg.pose.pose.position.x = target_x;
    goalMsg.pose.pose.position.y = target_y;
    goalMsg.pose.pose.position.z = 0.0;
    
    % Set orientation quaternion
    halfYaw = target_yaw * 0.5;
    goalMsg.pose.pose.orientation.z = sin(halfYaw);
    goalMsg.pose.pose.orientation.w = cos(halfYaw);

    fprintf('Sending Nav2 Goal -> X: %.2f m, Y: %.2f m, Yaw: %.2f rad\n', target_x, target_y, target_yaw);
    
    % Send goal
    [goalHandle, sendStatus] = sendGoal(navClient, goalMsg);
    if ~sendStatus
        error('Nav2 rejected the navigation goal.');
    end
    
    fprintf('Goal accepted! Monitoring progress...\n');
    resultMsg = getResult(goalHandle);
    fprintf('Navigation completed!\n');
end
