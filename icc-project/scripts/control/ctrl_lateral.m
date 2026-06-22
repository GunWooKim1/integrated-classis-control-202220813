function [deltaAdd, ctrlState] = ctrl_lateral( ...
    yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)

if isempty(ctrlState)

    ctrlState.intError = 0;
    ctrlState.prevError = 0;

end

yawError = yawRateRef - yawRate;

Kp = CTRL.LAT.Kp;
Ki = CTRL.LAT.Ki;
Kd = CTRL.LAT.Kd;

ctrlState.intError = ...
    ctrlState.intError + yawError * dt;

ctrlState.intError = max( ...
    min(ctrlState.intError, CTRL.LAT.intMax), ...
   -CTRL.LAT.intMax);

dError = ...
    (yawError - ctrlState.prevError) / max(dt,1e-6);

steerCmd = ...
    Kp * yawError + ...
    Ki * ctrlState.intError + ...
    Kd * dError;

steerCmd = max( ...
    min(steerCmd, LIM.MAX_STEER_ANGLE), ...
   -LIM.MAX_STEER_ANGLE);

ctrlState.prevError = yawError;

yawMomentCmd = 0;

betaThreshold = 1.2 * LIM.MAX_SLIP_ANGLE;

if abs(slipAngle) > betaThreshold

    betaError = ...
        abs(slipAngle) - betaThreshold;

    yawMomentCmd = ...
        -40 * sign(slipAngle) * betaError;

end

yawMomentCmd = ...
    max(min(yawMomentCmd,2000),-2000);

deltaAdd.steerAngle = steerCmd;
deltaAdd.yawMoment  = yawMomentCmd;

end