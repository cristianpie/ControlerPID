clc, close all, clear all, more off, pkg load control

pkg load control

% Parâmetros do sistema
##R = 0.015;               % Resistência da bobina (ohms)
##L = 15E-4;             % Indutância da bobina (henrys)
R = 1E-4;               % Resistência da bobina (ohms)
L = 1E-6;             % Indutância da bobina (henry
tl = L/R;            % Constante de tempo
fc = 1;             % Frequência de corte do filtro RC
tc = 1/(2*pi*fc);    % Constante de tempo do filtro RC

% Função de transferência do sistema
numerator = 1;
denominator = conv([1 tl], [1 tc 1]);

G = tf(numerator, denominator);

% Simulação da resposta ao degrau
t = 0:0.1:500;
[y, t] = step(G, t);

% Plot da resposta ao degrau
figure;
plot(t, y);
title('Resposta ao Degrau do Sistema de Malha Aberta');
xlabel('Tempo (s)');
ylabel('Saída do Sistema');
grid on;
% -------------------------------------------------


