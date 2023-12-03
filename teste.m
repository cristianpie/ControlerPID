clc, close all, clear all, more off, pkg load control,

pkg load control
pkg load acker

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
Kp = 1;     % Ganho proporcional
Ki = 1;     % Ganho integral
Kd = 1;     % Ganho derivativo
num_gc = [Kd Kp Ki];
den_gc = conv([1 T1], [1 0]);
sys_gc = tf(num_gc, den_gc);

% Função de transferência em malha fechada
sys_closed_loop = feedback(series(sys_g, sys_gc), 1);

% Converte para espaço de estados
sys_ss = ss(sys_closed_loop);

% Calcula os polos desejados
desired_poles = [-1, -2, -3]; % Substitua com os polos desejados

% Calcula a matriz de ganho do controlador por realimentação de polos
K = acker(sys_ss.A, sys_ss.B, desired_poles);

% Aplica a realimentação de estados
sys_closed_loop_pole_placement = ss(sys_ss.A - sys_ss.B * K, sys_ss.B, sys_ss.C, sys_ss.D);

% Resposta ao degrau
figure;
subplot(3,2,1);
t = 0:0.01:10;
step(sys_closed_loop_pole_placement, t);
title('Resposta ao Degrau do Sistema com Controle de Polos');

% Resposta à rampa
subplot(3,2,2);
t = 0:0.01:10;
ramp = t;
ramp(t > 2) = 2; % Rampa constante após 2 segundos
lsim(sys_closed_loop_pole_placement, ramp, t);
title('Resposta à Rampa do Sistema com Controle de Polos');

% Resposta ao impulso
subplot(3,2,3);
t = 0:0.01:10;
impulse(sys_closed_loop_pole_placement, t);
title('Resposta ao Impulso do Sistema com Controle de Polos');

% Gráfico da entrada degrau
subplot(3,2,4);
t = 0:0.01:10;
u_degrau = ones(size(t));
plot(t, u_degrau, 'r', 'LineWidth', 2);
title('Entrada Degrau');

% Gráfico da entrada rampa
subplot(3,2,5);
t = 0:0.01:10;
u_rampa = t;
u_rampa(t > 2) = 2; % Rampa constante após 2 segundos
plot(t, u_rampa, 'g', 'LineWidth', 2);
title('Entrada Rampa');

% Gráfico da entrada impulso
subplot(3,2,6);
t = 0:0.01:10;
u_impulso = zeros(size(t));
u_impulso(1) = 1; % Impulso em t = 0
stem(t, u_impulso, 'b', 'LineWidth', 2);
title('Entrada Impulso');

