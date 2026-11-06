function dy = Dynamique_Controle(t, y, m, g, l, Kp, Kv, Ki, t_total, q_start, q_end)
    % y = [theta1; theta2; theta1_dot; theta2_dot; int_err1; int_err2]
    
    % Extraction de l'état actuel
    % Positions actuelles
    q = y(1:2);
    % Vitesses actuelles
    dq = y(3:4);
    % Intégrale de l'erreur
    int_err = y(5:6);

    % Récupération de la trajectoire désirée
    [q_d, dq_d, ddq_d] = gen_traj(t, t_total, q_start, q_end);

    % Calcul des erreurs
    e = q_d - q;
    de = dq_d - dq;
    
    % Calcul des matrices dynamiques
    
    theta1 = q(1); 
    theta2 = q(2);
    dtheta1 = dq(1); 
    dtheta2 = dq(2);

    M = m * (l^2) * [ (5/3) + cos(theta2), (1/3) + (1/2) * cos(theta2);
                      (1/3) + (1/2) * cos(theta2), 1/3 ];
    
    Q_grav = (1/2) * m * g * l * [ 3 * sin(theta1) + sin(theta1 + theta2); 
                                   sin(theta1 + theta2) ]; 
                                   
    C = -(1/2) * m * (l^2) * sin(theta2) * [ dtheta2, dtheta1 + dtheta2;
                                            -dtheta1, 0 ];

    % Loi de commande
    % u = ddq_d + Kv*de + Kp*e + Ki*int_err
    v = ddq_d + Kv * de + Kp * e + Ki * int_err;
    
    % Tau = M(q)*v + C(q,dq)*dq + Q(q)
    tau = M * v + C * dq + Q_grav;
    
    % Dynamique directe (Simulation du système réel)
    ddq = M \ (tau - C * dq - Q_grav);

    % Construction de la dérivée du vecteur d'état
    dy = [dq; ddq; e];
end