function joint_pos = InverseKinematics( link_lengths, effpos, basePos )
    joint_pos = zeros(2,1);
    DelPos = effpos - basePos;
    
    joint_pos(1,2) = acos((DelPos(1,1)^2 + DelPos(1,2)^2-link_lengths(1,1)^2-link_lengths(2,1)^2)/(2*link_lengths(1,1)*link_lengths(2,1)));
    joint_pos(1,1) = atan2(DelPos(2,1),DelPos(1,1)) - asin(link_lengths(2,1)*sin(joint_pos(1,2)/(sqrt(DelPos(1,1)^2 + DelPos(1,2)^2))));
end