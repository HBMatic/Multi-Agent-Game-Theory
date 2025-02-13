function [x, y, theta] = get_current_pose(sub)
    odomMsg = receive(sub, 1);
    pos = odomMsg.Pose.Pose.Position;
    quat = odomMsg.Pose.Pose.Orientation;

    % Convert quaternion to Euler angles
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]); % Correct order
    theta = angles(1);  % Extract the yaw (rotation around Z axis)

    % Get current position
    x = pos.X;
    y = pos.Y;
end
