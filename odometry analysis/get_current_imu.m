function [theta] = get_current_imu(sub)
    IMUMsg = receive(sub, 1);
    quat = IMUMsg.Orientation;
    % Convert quaternion to Euler angles
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]); % Correct order
    theta = angles(1);  % Extract the yaw (rotation around Z axis)
end
