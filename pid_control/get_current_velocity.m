function [vl, vr] = get_current_velocity(sub)
    encoderMsg=receive(sub,10);
    vel= encoderMsg;
    vl=vel.Velocity(1);
    vr=vel.Velocity(2);
end