function q_dot = Pendule_simple (t, q, g, l )
    %q est un vecteur colonne avec:
    % - q(1)= theta
    % - q(2)= theta_dot

    theta=q(1);
    theta_dot=q(2);

    q_dot1=theta_dot;
    q_dot2= -(g/l) * sin(theta);

    q_dot=[q_dot1;q_dot2];
end

