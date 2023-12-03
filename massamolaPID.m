clc, close all, clear all, more off, pkg load control

% Parâmetros do sistema
m = 1;  % massa
c = 0.1;  % coeficiente de amortecimento
k = 10;  % constante da mola

% Função de transferência do sistema
s = tf('s');
G = 1 / (m * s^2 + c * s + k);

% Condições iniciais
posicao_inicial = 1;  % posição inicial
velocidade_inicial = 0;  % velocidade inicial

% Criar sistema em espaço de estados com condições iniciais
sys = ss(G);
x0 = [posicao_inicial; velocidade_inicial];
sys_with_ic = initial(sys, x0);

% Parâmetros do controlador PID
Kp = 10;
Ki = 5;
Kd = 2;

% Criar controlador PID
pid_controller = pid(Kp, Ki, Kd);

% Sistema em malha fechada com controle PID
sys_pid = feedback(pid_controller * G, 1);

% Resposta ao degrau
t = 0:0.01:10;
step_response = step(sys_pid, t);

% Gráficos
figure;

% Gráfico da posição
subplot(2,1,1);
plot(t, step_response);
title('Resposta ao Degrau - Posição');
xlabel('Tempo (s)');
ylabel('Posição');

% Gráfico da velocidade
subplot(2,1,2);
[v, t] = impulse(sys_pid, 10);
plot(t, v);
title('Resposta ao Degrau - Velocidade');
xlabel('Tempo (s)');
ylabel('Velocidade');

