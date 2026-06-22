function [dampingCmd, ctrlState] = ctrl_vertical( ...
    suspState, ctrlState, CTRL, dt)

if isempty(ctrlState)

    ctrlState.init = true;

end

dampingCmd = zeros(4,1);

for i = 1:4

    zs_dot_i = suspState.zs_dot(i);
    zu_dot_i = suspState.zu_dot(i);

    v_rel = zs_dot_i - zu_dot_i;

    rollGain = ...
        min(abs(zs_dot_i)/0.12,1.0);

    if (zs_dot_i * v_rel) > 0

        c_target = ...
            CTRL.VER.cMin + ...
            rollGain * ...
            (CTRL.VER.cMax - CTRL.VER.cMin);

    else

        c_target = CTRL.VER.cMin;

    end

    c_target = ...
        max(min(c_target,CTRL.VER.cMax), ...
        CTRL.VER.cMin);

    dampingCmd(i) = c_target;

end

end