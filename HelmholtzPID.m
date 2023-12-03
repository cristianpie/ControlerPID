clc, close all, clear all, more off, pkg load control


##
##% Parâmetros do sistema
##R = 1;      % Resistência
##L = 0.1;    % Indutância
##fc = 10;    % Frequência de corte do filtro RC
##
##% Calcula as constantes de tempo
##tl = L / R;
##tc = 1 / (2 * pi * fc);
R = 1;               % Resistência da bobina (ohms)
L = 0.1;             % Indutância da bobina (henrys)
tl = L/R;            % Constante de tempo
fc = 10;             % Frequência de corte do filtro RC
tc = 1/(2*pi*fc);    % Constante de tempo do filtro RC

% Função de transferência do sistema
numerator = 1;
denominator = conv([1 tl], [1 tc]);

% Cria o sistema
sys = tf(numerator, denominator);

% Projeta o controlador PID
Kp = 10;     % Ganho proporcional
Ki = 1;     % Ganho integral
Kd = 1;     % Ganho derivativo

% Cria o controlador PID
controller = pid(Kp, Ki, Kd);

% Sistema em malha fechada
sys_closed_loop = feedback(series(controller, sys), 1);

% Resposta ao degrau
t = 0:.1:100;
step(sys_closed_loop, t);
title('Resposta ao Degrau do Sistema com PID');

% Resposta à entrada degrau
figure;
t = 0:1:250;
u = ones(size(t));
lsim(sys_closed_loop, u, t);
title('Resposta à Entrada degrau do Sistema com PID');


