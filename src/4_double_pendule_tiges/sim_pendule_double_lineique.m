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
theta1_0 = deg2rad(40);
theta2_0 = deg2rad(100);
theta1_dot_0 =0;
theta2_dot_0 =0;

q_0=[theta1_0; theta2_0; theta1_dot_0; theta2_dot_0];

%Lancement de la simulation

ode_func = @(t,q) Pendule_double_lineique(t, q,m , g, l);
[t, q_sol] = ode45(ode_func, t_sim, q_0);

disp('Simulation terminee. Lancement de la visualisation...');

% Démarrage de l'enregistrement
video_filename = 'Animation_Pendule_Double_Lineique.mp4'; 
writerObj = VideoWriter(video_filename, 'MPEG-4');
total_duration = t(end) - t(1);
dynamic_framerate = length(t) / total_duration;

writerObj.FrameRate = min(dynamic_framerate, 100);
open(writerObj);




% Visualisation et Animation
figure('Name', 'Animation du Pendule double lineique');
hold on;
grid on;
axis equal;

% Definir les limites des axes pour que la vue ne change pas
axis_limit = l * 2.2;
axis([-axis_limit, axis_limit, -axis_limit, axis_limit*0.25]);
xlabel('Position x (m)');
ylabel('Position y (m)');

% Creer les objets graphiques
pivot_fixe = plot(0, 0, 'k+', 'MarkerSize', 15, 'LineWidth', 2); % Point de pivot fixe
pivot_mobile = plot(0, 0, 'k+', 'MarkerSize', 15, 'LineWidth', 2); % Point de pivot mobile
tige1 = plot([0, 0], [0, 0], 'k-', 'LineWidth', 2); % La tige
tige2 = plot([0, 0], [0, 0], 'k-', 'LineWidth', 2); % La tige

title_handle = title('Temps = 0.00 s');

% Boucle d'animation
for i = 1:length(t)
    % Obtenir l'angle actuel depuis la solution de ode45
    theta1_curr = q_sol(i, 1);
    theta2_curr = q_sol(i, 2);

    % Angle absolu de la seconde tige par rapport à la verticale
    theta_total = theta1_curr + theta2_curr;
    

    % Calculer les coordonnees du second pivot (mobile)
    pivot_mobile_x = (l) * sin(theta1_curr);
    pivot_mobile_y = -(l) * cos(theta1_curr);

    
    % Coordonnées de l'Extrémité (Fin de la Tige 2) (pour le tracé de la tige)
    ext_x = pivot_mobile_x + l * sin(theta_total);
    ext_y = pivot_mobile_y + (-l * cos(theta_total));

    % Mettre à jour les données des objets graphiques 
    set(tige1, 'XData', [0, pivot_mobile_x], 'YData', [0, pivot_mobile_y]);
    set(tige2, 'XData', [pivot_mobile_x, ext_x], 'YData', [pivot_mobile_y, ext_y]);
    set(title_handle, 'String', sprintf('Temps = %.2f s', t(i)));
    
    drawnow;

    %Enregistrement de la frame
    frame = getframe(gcf);
    writeVideo(writerObj, frame);
end

disp('Animation terminée.');

%fin d'enregistrement
close(writerObj);
disp(['Vidéo enregistrée sous : ', video_filename]);


theta1_sol = q_sol(:, 1);
theta2_sol = q_sol(:, 2);
theta1_dot_sol = q_sol(:, 3);
theta2_dot_sol = q_sol(:, 4);

N = length(t);
E_tot = zeros(N, 1);
T_sol = zeros(N, 1);
V_sol = zeros(N, 1);

% Calcul de l'énergie totale à chaque pas de temps
for i = 1:N
    th1 = theta1_sol(i);
    th2 = theta2_sol(i);
    th1_dot = theta1_dot_sol(i);
    th2_dot = theta2_dot_sol(i);
    
    q_dot_curr = [th1_dot; th2_dot];
    
    c2 = cos(th2);
    
    M_curr = m*(l^2) * [ (5/3) + c2, (1/3) + (1/2)*c2;
                         (1/3) + (1/2)*c2, 1/3 ];
    
    % Énergie Cinétique 
    T_curr = 0.5 * q_dot_curr' * M_curr * q_dot_curr;
    T_sol(i) = T_curr;
    
    % Énergie Potentielle V(q)
    V_curr = - m*g*l * ( (3/2)*cos(th1) + (1/2)*cos(th1 + th2) );
    V_sol(i) = V_curr;
    
    % Énergie Totale
    E_tot(i) = T_curr + V_curr;
end

figure('Name', 'Énergie Totale du Pendule Double');
hold on;
plot(t, T_sol, 'r', 'LineWidth', 2);
plot(t, V_sol, 'b', 'LineWidth', 2);
plot(t, E_tot, 'k--', 'LineWidth', 3); 
xlabel('Temps (s)');
ylabel('Énergie (J)');
title('Conservation de l''Énergie du Pendule Double (tiges homogènes)');
legend('Énergie Cinétique (T)', 'Énergie Potentielle (V)', 'Énergie Totale (E_{tot})', 'Location', 'best');
grid on;
hold off;

