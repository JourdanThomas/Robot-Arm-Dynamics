clear all;
close all;
clc;

% on définit les parametres
g = 9.81; %m/s^2
l = 1; %m
m=1;%kg

% On définit le temps de simulation
t_sim=[0 15];

% on définit les conditions initiales
theta_0 = deg2rad(45);
theta_dot_0 =0;

q_0=[theta_0;theta_dot_0];

%Lancement de la simulation

ode_func = @(t,q) Pendule_simple(t, q, g, l);
[t, q_sol] = ode45(ode_func, t_sim, q_0);

disp('Simulation terminee. Lancement de la visualisation...');

% Visualisation et Animation


figure('Name', 'Animation du Pendule Simple');
hold on;
grid on;
axis equal;

% Definir les limites des axes pour que la vue ne change pas
axis_limit = l * 1.2;
axis([-axis_limit, axis_limit, -axis_limit, axis_limit]);
xlabel('Position x (m)');
ylabel('Position y (m)');

% Creer les objets graphiques que nous allons mettre à jour
pivot = plot(0, 0, 'k+', 'MarkerSize', 15, 'LineWidth', 2); % Point de pivot
tige = plot([0, 0], [0, 0], 'k-', 'LineWidth', 2); % La tige
boule = plot(0, 0, 'b.', 'MarkerSize', 50); % La masse
title_handle = title('Temps = 0.00 s');

% Boucle d'animation
for i = 1:length(t)
    % Obtenir l'angle actuel depuis la solution de ode45
    theta_now = q_sol(i, 1);
    
    % Calculer les coordonnees de la masse
    boule_x = l * sin(theta_now);
    boule_y = -l * cos(theta_now);
    
    % Mettre à jour les données des objets graphiques
    set(tige, 'XData', [0, boule_x], 'YData', [0, boule_y]);
    set(boule, 'XData', boule_x, 'YData', boule_y);
    set(title_handle, 'String', sprintf('Temps = %.2f s', t(i)));
    
    drawnow;
   
    pause(0.05); 
end

disp('Animation terminée.');
disp('Calcul énergie...');


% Tracé de l'energie totale
theta_sol = q_sol(:, 1);
theta_dot_sol = q_sol(:, 2);


% Énergie Cinétique 
Ec = 0.5 * m * l^2 * (theta_dot_sol.^2);

% Énergie Potentielle 
% On prend E_p = 0 au point le plus bas (theta = 0)
Ep = m * g * l * (1 - cos(theta_sol));

% Énergie Totale (E_tot)
E_tot = Ec + Ep;

% Tracer le résultat
figure('Name', 'Énergie du Pendule Simple');
hold on;
plot(t, Ec, 'r', 'LineWidth', 2);
plot(t, Ep, 'b', 'LineWidth', 2);
plot(t, E_tot, 'k--', 'LineWidth', 3); 
xlabel('Temps (s)');
ylabel('Énergie (J)');
title('Conservation de l''Énergie du Pendule Simple');
legend('Énergie Cinétique (E_c)', 'Énergie Potentielle (E_p)', 'Énergie Totale (E_{tot})', 'Location', 'best');
grid on;
hold off;

disp('Tracé énergie terminé');



