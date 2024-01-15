function [M, C, G] = NominalTwoLinkManipulatordynamicsGenerator(SystemParam, q, q_dot)
    [M, C, G] = TwoLinkManipulatordynamicsGenerator(SystemParam, q, q_dot);

    %nominal system
    %M = M*1.2;
    %C = C*0.8;
    %G = G*1.2;
end