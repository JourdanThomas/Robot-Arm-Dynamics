function [q, dq, ddq] = gen_traj(t, t_total, q_start, q_end)
    %Gestion des bornes temporelles
    % Si t est avant le début (t<0), on reste à q_start
    if t <= 0
        q = q_start;
        dq = zeros(size(q_start));
        ddq = zeros(size(q_start));
        return;
    %Si t est après la fin (t>t_total), on reste à q_end
    elseif t >= t_total
        q = q_end;
        dq = zeros(size(q_end));
        ddq = zeros(size(q_end));
        return;
    end

    %Calcul des coefficients a_i pour un polynôme quintique
    %Hypothèses : vitesse et accélération initiales et finales sont nulles.
    %Polynôme : q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    
    dist = q_end - q_start;
    
    a0 = q_start;
    a1 = 0;
    a2 = 0;
    a3 = (10 * dist) / (t_total^3);
    a4 = (-15 * dist) / (t_total^4);
    a5 = (6 * dist)  / (t_total^5);
    
    %Calcul de la trajectoire (Position, Vitesse, Accélération)
    % q(t)
    q = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
    % dq(t)
    dq = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
    % ddq(t)
    ddq = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;

end