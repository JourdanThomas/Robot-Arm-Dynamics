function q_dot = Pendule_double_lineique (t, q, m, g, l )
    %q est un vecteur colonne avec:
    % - q(1)= theta1
    % - q(2)= theta2
    % - q(3)= theta1_dot
    % - q(4)= theta2_dot

    theta1=q(1);
    theta2=q(2);
    theta1_dot=q(3);
    theta2_dot=q(4);
    
    M = m * (l^2) * [ (5/3) + cos(theta2), (1/3) + (1/2) * cos(theta2);
                      (1/3) + (1/2) * cos(theta2), 1/3 ];

    
    Q = (1/2) * m * g * l * [ 3 * sin(theta1) + sin(theta1 + theta2); sin(theta1 + theta2) ];

    C = -(1/2) * m * (l^2) * sin(theta2) * [ theta2_dot, theta1_dot + theta2_dot;
                                    -theta1_dot, 0 ];

    
    sol = M \ (-C * [theta1_dot; theta2_dot] - Q);

    q_dot1=theta1_dot;
    q_dot2=theta2_dot;
    q_dot3= sol(1);
    q_dot4= sol(2);

    q_dot=[q_dot1;q_dot2;q_dot3;q_dot4];
end

