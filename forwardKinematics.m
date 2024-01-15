function end_effector_pos = forwardKinematics(SysParam, joint_angles)
    % link_lengths: [length of link 1, length of link 2]
    % joint_angles: [angle of joint 1, angle of joint 2]
    % base_pos: [x-coordinate, y-coordinate] of the base of the manipulator

    l1 = SysParam.l;
    l2 = SysParam.l;
    theta1 = joint_angles(1);
    theta2 = joint_angles(2);
    x_base = SysParam.base_pos(1);
    y_base = SysParam.base_pos(2);

    % Calculate the position of the end effector
    x_end = x_base + l1 * cos(theta1) + l2 * cos(theta1 + theta2);
    y_end = y_base + l1 * sin(theta1) + l2 * sin(theta1 + theta2);

    end_effector_pos = [x_end, y_end];
end
