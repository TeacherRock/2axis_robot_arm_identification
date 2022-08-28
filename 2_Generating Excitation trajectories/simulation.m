function Output = simulation(trajectory, T, controller)
    
    ts = 0.001;
    ax = 2;
    l1 = 0.24;    l2 = 0.24;
    c1 = 0.12;    c2 = 0.12;
    m1 = 2;       m2 = 2;
    I1 = 0.0267;  I2 = 0.0267;

    P = [trajectory.P(1, 1); trajectory.P(1, 2)];
    [V, A] = deal(zeros(ax,1));
    
    [PCmd, VCmd, ACmd] = deal(zeros(ax, length(trajectory.P(:,1))));
    PCmd(1,:) = trajectory.P(:, 1)';
    VCmd(1,:) = trajectory.V(:, 1)';
    ACmd(1,:) = trajectory.A(:, 1)';
    PCmd(2,:) = trajectory.P(:, 2)';
    VCmd(2,:) = trajectory.V(:, 2)';
    ACmd(2,:) = trajectory.A(:, 2)';

    nc = length(T);

    [rec.P, rec.V, rec.A, rec.T, rec.F] = deal(zeros(nc, ax));
    rec.P(1, :) = P';

    for i = 2 : nc
        % M ,C
        [M ,C] = deal(zeros(2, 2));

        M(1,1) = I1 + I2 + m1*c1^2 + m2*(l1^2 + c2^2 + 2*l1*c2*cos(P(2,:)));
        M(1,2) = I2 + m2*c2^2 + m2*l1*c2*cos(P(2,1));
        M(2,1) = M(1,2);
        M(2,2) = I2 + m2*c2^2;
        C(1,1) = -1*m2*l1*c2*V(2,1)*sin(P(2,1));
        C(1,2) = -1*m2*l1*c2*(V(1,1) + V(2,1))*sin(P(2,1));
        C(2,1) = m2*l1*c2*V(1,1)*sin(P(2,1));
        C(2,2) = 0;

        % Controller
        switch controller
            case 'Linear'
            %PD
            Kp = [200;200];     Kv = [0.01; 0.01];
            ControllerTorque = Kv .* (VCmd(:, i-1) - V(:,1)) + Kp .* (PCmd(:, i-1) - P(:,1));
            case 'CTC'
            %CTC
            ControllerTorque = M*(Kv .* (VCmd(:, i-1) - V(:,1)) + Kp .* (PCmd(:, i-1) - P(:,1)) + ACmd(:, i-1) ) + C*VCmd(:,i-1);
            case 'DFF'
            %DFF
            ControllerTorque = M*ACmd(:, i-1)  + C*VCmd(:,i-1) + Kv .* (VCmd(:, i-1) - V(:,1)) + Kp .* (PCmd(:, i-1) - P(:,1));
        end

        % Calculate Friction
        F = 0;

        % Calculate A || DDM Diret Dynamic Model
        A = M\(ControllerTorque - C * V );

        % Integral V, P
        V = V + A * ts;
        P = P + V * ts;

        % record
        rec.P(i, :) = P';
        rec.V(i, :) = V';
        rec.A(i, :) = A';
        rec.T(i, :) = ControllerTorque';
        rec.F(i, :) = F';

    end
    Output = rec;
    end