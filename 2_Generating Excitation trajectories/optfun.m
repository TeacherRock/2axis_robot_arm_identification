function C = optfun(F_par)
    % F_par = [a11, a12, ... a15, b11, b12, ... b15, q1, a21, a22, ... a25, b21, b22, ... b25, q2 ]
    % x = [s11, s12, ... s15, c11, c12, ... c15,  1, s21, s22, ... s25, c21, c22, ... c25,  1 ]
    % sxy : s = sin, x = axis, y = 基底數 
    sampTi = 0.02;
    tf = 12;
    wf = (2 * pi)/tf ;
    N = 5;    %用N組基底頻率
    n = tf/sampTi;   %軌跡點的個數
    Wn = zeros(2*n,9);
    k = 1;
    d1 = 0.24; d2 = 0.24;
    for t = sampTi : sampTi : tf
        [p1, v1, a1, p2, v2, a2] = deal(0);
        for i = 1 : N
            p1 = p1 + (F_par(i)*sin(wf*i*t) - F_par(i+5)*cos(wf*i*t))/wf/i;
            v1 = v1 + F_par(i)*cos(wf*i*t) + F_par(i+5)*sin(wf*i*t);
            a1 = a1 + (-F_par(i)*sin(wf*i*t) + F_par(i+5)*cos(wf*i*t))*wf*i;

            p2 = p2 + (F_par(2*N+1 + i)*sin(wf*i*t) - F_par(2*N+1 + i+5)*cos(wf*i*t))/wf/i;
            v2 = v2 + F_par(2*N+1 + i)*cos(wf*i*t) + F_par(2*N+1 + i+5)*sin(wf*i*t);
            a2 = a2 + (-F_par(2*N+1 + i)*sin(wf*i*t) + F_par(2*N+1 + i+5)*cos(wf*i*t))*wf*i;
        end
        p1 = p1 + F_par(2*N+1);
        p2 = p2 + F_par(2*(2*N+1));

        Wn(k:k+1,:) = [a1, a1+a2, -d1*(sin(p2)*v2^2+2*v1*sin(p2)*v2-2*a1*cos(p2)-a2*cos(p2)), -d1*(cos(p2)*v2^2+2*v1*cos(p2)*v2+2*a1*sin(p2)+a2*sin(p2)), 0,  sign(v1), 0,        v1, 0;
                       0,  a1+a2,  d1*(sin(p2)*v1^2+a1*cos(p2)),                               d1*(cos(p2)*v1^2-a1*sin(p2)),                              a2, 0,        sign(v2), 0,  v2];
        k = k + 2;

    end

    C = cond(Wn);

end