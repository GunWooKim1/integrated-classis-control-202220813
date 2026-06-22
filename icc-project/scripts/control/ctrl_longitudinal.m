function [forceCmd, ctrlState] = ctrl_longitudinal( ...
    vxRef, vx, ax, ctrlState, CTRL, LIM, dt)

if isempty(ctrlState)

    ctrlState.intError = 0;

end

m_vehicle = 1700;

velError = vxRef - vx;

Kp = 3200;
Ki = 650;

ctrlState.intError = ...
    ctrlState.intError + velError * dt;

ctrlState.intError = ...
    max(min(ctrlState.intError,8),-8);

Fx_cmd = ...
    Kp * velError + ...
    Ki * ctrlState.intError;

Fx_max = LIM.MAX_AX * m_vehicle;

Fx_cmd = ...
    max(min(Fx_cmd,Fx_max),-Fx_max);

brakeRatio = 0;

if Fx_cmd < 0

    brakeDemand = abs(Fx_cmd) / Fx_max;

    brakeRatio = min(0.55 * brakeDemand,0.55);

end

forceCmd.Fx_total   = Fx_cmd;
forceCmd.brakeRatio = brakeRatio;

end