function [v, w, prevWheelPosGazebo, prevTimeGazebo] = velocity_cal(jointStateSubGazebo, prevWheelPosGazebo, prevTimeGazebo)
    wheelBase = 0.16;    % meters
    jointStateMsgGazebo = receive(jointStateSubGazebo, 1);
    wheelPosGazebo = jointStateMsgGazebo.Position(1:2);
    currentTimeGazebo = jointStateMsgGazebo.Header.Stamp.Sec + ...
                        jointStateMsgGazebo.Header.Stamp.Nsec * 1e-9;
    
    v = 0;
    w = 0;
    
    if prevTimeGazebo > 0
        dt = currentTimeGazebo - prevTimeGazebo;
        dvl = 0.03333*(wheelPosGazebo(2) - prevWheelPosGazebo(2)) / dt;
        dvr = 0.03333*(wheelPosGazebo(1) - prevWheelPosGazebo(1)) / dt;
        v = (dvl + dvr) / 2;
        w = (dvr - dvl) / wheelBase;
    end
    
    prevWheelPosGazebo = wheelPosGazebo;
    prevTimeGazebo = currentTimeGazebo;
end
