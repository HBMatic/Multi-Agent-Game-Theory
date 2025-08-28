function [x, y, theta] = get_current_pose_custom_odom()
    customodomsub  = rossubscriber('/custom_odom', 'nav_msgs/Odometry');
    sub = receive(customodomsub, 1.0);
    pos = sub.Pose.Pose.Position;
    quat = sub.Pose.Pose.Orientation;

    % Convert quaternion to Euler angles
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]); % Correct order
    theta = angles(1);  % Extract the yaw (rotation around Z axis)

    % Get current position
    x = pos.X;
    y = pos.Y;
end
