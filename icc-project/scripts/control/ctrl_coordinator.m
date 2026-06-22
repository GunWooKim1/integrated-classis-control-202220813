function actuatorCmd = ctrl_coordinator( ...
    latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)

actuatorCmd.steerAngle = ...
    max(min(latCmd.steerAngle, ...
    LIM.MAX_STEER_ANGLE), ...
   -LIM.MAX_STEER_ANGLE);

T_base = zeros(4,1);

r_wheel = 0.33;

if isfield(VEH,'r_w')

    r_wheel = VEH.r_w;

end

if lonCmd.Fx_total < 0

    T_total = ...
        abs(lonCmd.Fx_total) * r_wheel;

    if isfield(lonCmd,'brakeRatio')

        absGain = ...
            1.0 - lonCmd.brakeRatio;

        T_total = T_total * absGain;

    end

    T_base(1) = 0.30 * T_total;
    T_base(2) = 0.30 * T_total;
    T_base(3) = 0.20 * T_total;
    T_base(4) = 0.20 * T_total;

end

T_esc = zeros(4,1);

if isfield(latCmd,'yawMoment')

    Mz = latCmd.yawMoment;

    escGain = 0.00035;

    dT = escGain * Mz;

    T_esc(1) = -dT;
    T_esc(2) =  dT;
    T_esc(3) = -dT;
    T_esc(4) =  dT;

end

T_final = T_base + T_esc;

for i = 1:4

    T_final(i) = ...
        max(min(T_final(i), ...
        LIM.MAX_BRAKE_TRQ),0);

end

actuatorCmd.brakeTorque = T_final;

actuatorCmd.dampingCoeff = verCmd;

end