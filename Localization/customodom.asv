function odomMsg = customodom(jointStateMsg)
% publishCustomOdometry  Compute & publish custom odometry from encoder data.
%   odomMsg = publishCustomOdometry(jointStateMsg) reads wheel positions
%   from jointStateMsg, integrates a diff-drive model, and publishes a
%   nav_msgs/Odometry on /custom_odom. Returns the odometry message.

  persistent prevWheelPos prevTime customPose odomPub wheelRadius wheelBase

  if isempty(prevTime)
    % === first call: initialize persistent state ===
    prevTime      = 0;
    prevWheelPos  = jointStateMsg.Position(1:2);
    customPose    = [0; 0; 0];        % [x; y; theta]
    wheelRadius   = 0.033;            % meters
    wheelBase     = 0.16;             % meters
    odomPub       = rospublisher('/custom_odom', 'nav_msgs/Odometry');
  end

  % === compute dt ===
  ts = double(jointStateMsg.Header.Stamp.Sec) + ...
       double(jointStateMsg.Header.Stamp.Nsec)*1e-9;
  if prevTime == 0
    % skip first integration step
    prevTime = ts;
    return;
  end
  dt = ts - prevTime;
  prevTime = ts;

  % === wheel displacements ===
  wheelPos = jointStateMsg.Position(1:2);       % [left; right]
  dR = (wheelPos(1) - prevWheelPos(1)) * wheelRadius;
  dL = (wheelPos(2) - prevWheelPos(2)) * wheelRadius;
  prevWheelPos = wheelPos;

  % === integrate differential‐drive kinematics ===
  dCenter = (dL + dR)/2;
  dTheta  = (dR - dL) / wheelBase;
  customPose(3) = customPose(3) + dTheta;
  customPose(1) = customPose(1) + dCenter * cos(customPose(3));
  customPose(2) = customPose(2) + dCenter * sin(customPose(3));

  % === build & publish Odometry message ===
  odomMsg = rosmessage(odomPub);
  odomMsg.Header.Stamp    = jointStateMsg.Header.Stamp;
  odomMsg.Header.FrameId  = 'odom';
  odomMsg.ChildFrameId    = 'custom_base_link';

  % pose
  odomMsg.Pose.Pose.Position.X = customPose(1);
  odomMsg.Pose.Pose.Position.Y = customPose(2);
  odomMsg.Pose.Pose.Position.Z = 0;
  quat = eul2quat([0 0 customPose(3)], 'ZYX');  % note order: yaw about Z
  odomMsg.Pose.Pose.Orientation.W = quat(1);
  odomMsg.Pose.Pose.Orientation.X = quat(2);
  odomMsg.Pose.Pose.Orientation.Y = quat(3);
  odomMsg.Pose.Pose.Orientation.Z = quat(4);

  % (optional) twist: you could estimate v & omega if you like

  send(odomPub, odomMsg);
end
customodomsub