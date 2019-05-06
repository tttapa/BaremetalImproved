function K = dlqi(A, B, C, D, Q, R, Ts) 
    nx = size(A, 1);
    ny = size(C, 1);
    Aaug = [ A,      zeros(nx,ny);
            -C * Ts, eye(ny,ny) ];
    Baug = [ B;
            -D * Ts ];
    K = dlqr(Aaug, Baug, Q, R);
end