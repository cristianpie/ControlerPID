clc, close all, clear all, more off, pkg load control

pkg load control

% Parâmetros do sistema
R = 1;      % Resistência
L = 0.1;    % Indutância
fc = 10;    % Frequência de corte do filtro RC
T1 = 1 / fc;

% Calcula as constantes de tempo
tl = L / R;
tc = 1 / (2 * pi * fc);

% Função de transferência do sistema G(s)
num_g = 1;
den_g = conv([1 tl], [1 tc]);
sys_g = tf(num_g, den_g);

% Função de transferência do controlador Gc(s)
Kp = 10;     % Ganho proporcional
Ki = 20;     % Ganho integral
Kd = 0;     % Ganho derivativo
num_gc = [Kd Ki Kp];
den_gc = conv([1 T1], [1 0]);
sys_gc = tf(num_gc, den_gc);

% Função de transferência em malha fechada
sys_closed_loop = feedback(series(sys_g, sys_gc), 1);

% Resposta ao degrau
t = 0:0.01:50;
step(sys_closed_loop, t);
title('Resposta ao Degrau do Sistema com PID');

% Resposta à entrada degrau
figure;
t = 0:0.1:250;
u = ones(size(t))*10;
lsim(sys_closed_loop, u, t);
title('Resposta à Entrada degrau do Sistema com PID');


% Resposta à entrada Rampa
figure;
t = 0:0.1:250;
ramp = t;
ramp(t >= 50) = 10; % Rampa constante após 2 segundos
lsim(sys_closed_loop, ramp, t);
title('Resposta à Rampa do Sistema com PID');




% Resposta à entrada Impulso
figure;
t = 0:0.1:250;
impulse(sys_closed_loop, t);
title('Resposta ao Impulso do Sistema com PID');




