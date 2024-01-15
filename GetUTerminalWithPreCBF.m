function uTerminal = GetUTerminalWithPreCBF(M,u_norm)
    b = -inv(M);
    b_paper = [b(1,1);b(1,2)];
    bT_paper = transpose(b_paper);
    uTerminal = (eye(2) - b.*bT_paper/(b_paper(1)^2+b_paper(2)^2))*u_norm;
end