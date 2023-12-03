clc, close all, clear all, more off, pkg load control

pkg load control

% Parâmetros do sistema
R = 10;               % Resistência da bobina (ohms)
L = 0.1;             % Indutância da bobina (henrys)
fc = 1;    % Frequência de corte do filtro RC
T1 = 1 / fc;

% Calcula as constantes de tempo
tl = L / R;
tc = 1 / (2 * pi * fc);

% Função de transferência do sistema G(s)
num_g = 1;
den_g = conv([1 tl], [1 tc]);
sys_g = tf(num_g, den_g);

% Função de transferência do controlador Gc(s)
Kp = 4.7922;     % Ganho proporcional
Ki = 3005.8;     % Ganho integral
Kd = 0.004166;     % Ganho derivativo
##Kp = 3;     % Ganho proporcional
##Ki = 1;     % Ganho integral
##Kd = 1;     % Ganho derivativo

num_gc = [Kd Ki Kp];
den_gc = conv([1 T1], [1 0]);
sys_gc = tf(num_gc, den_gc);

% Função de transferência em malha fechada
sys_closed_loop = feedback(series(sys_g, sys_gc), 1);

##% Resposta ao degrau
##t = 0:0.01:10;
##step(sys_closed_loop, t);
##title('Resposta ao Degrau do Sistema com PID');

% Resposta à entrada degrau
##figure;
t = 0:0.1:500;
u = ones(size(t))*10;
lsim(sys_g, u, t);
title('Resposta à Entrada degrau do Sistema sem PID');

% Resposta à entrada degrau
figure;
t = 0:0.1:500;
u = ones(size(t))*10;
lsim(sys_g, u, t);
title('Resposta à Entrada degrau do Sistema com PID');


% RESPOSTA à ENTRADA RAMPA
FIGURE;
T = 0:0.1:250;
RAMP = T;
RAMP(T >= 50) = 10; % RAMPA CONSTANTE APóS 2 SEGUNDOS
LSIM(SYS_CLOSED_LOOP, RAMP, T);
TITLE('RESPOSTA à RAMPA DO SISTEMA COM PID');


% RESPOSTA à ENTRADA IMPULSO
FIGURE;
T = 0:0.1:250;
IMPULSE(SYS_CLOSED_LOOP, T);
TITLE('RESPOSTA AO IMPULSO DO SISTEMA COM PID');




