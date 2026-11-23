%% Identificación del sobrecalentador - Paso 2: ajustar el modelo
% Usa est (pulsos 1 y 2, 12:28-13:28) y val (pulso 3, 13:28-13:49), guardados en el paso 1.
% Entrada: potencia del quemador Q [kW]. Salida: sobrecalentamiento [K].

clear; clc; close all
load('datos_pulsos_3nov.mat', 'est', 'val')

% Los pulsos parten a mitad de una subida o bajada, no en equilibrio:
% por eso se estima también la condición inicial.
opt = procestOptions('InitialCondition', 'estimate');
opt.SearchOptions.MaxIterations = 200;   % el P2D no alcanzaba a converger con 20

%% Modelo 1: primer orden con retardo   G(s) = K e^(-θs) / (τs + 1)
m1 = idproc('P1D', 'TimeUnit', 'seconds');
m1.Structure.Kp.Value  = 2;     m1.Structure.Kp.Minimum  = 0;     % K/kW
m1.Structure.Tp1.Value = 1500;  m1.Structure.Tp1.Maximum = 1e4;   % s
m1.Structure.Td.Value  = 150;   m1.Structure.Td.Maximum  = 900;   % s
m1 = procest(est, m1, opt);

%% Modelo 2: segundo orden con retardo   G(s) = K e^(-θs) / ((τ1 s + 1)(τ2 s + 1))
m2 = idproc('P2D', 'TimeUnit', 'seconds');
m2.Structure.Kp.Value  = 2;     m2.Structure.Kp.Minimum  = 0;
m2.Structure.Tp1.Value = 600;   m2.Structure.Tp1.Maximum = 1e4;   % dos inercias parecidas:
m2.Structure.Tp2.Value = 300;   m2.Structure.Tp2.Maximum = 1e4;   % serpentín y línea hasta TT 106
m2.Structure.Td.Value  = 150;   m2.Structure.Td.Maximum  = 900;
m2 = procest(est, m2, opt);

%% Parámetros con su incertidumbre (± 1 desviación estándar)
present(m1)
present(m2)

fprintf('\nP1D: K = %.2f K/kW | tau = %.1f min | theta = %.1f min\n', ...
    m1.Kp, m1.Tp1/60, m1.Td/60);
fprintf('P2D: K = %.2f K/kW | tau1 = %.1f min | tau2 = %.1f min | theta = %.1f min\n', ...
    m2.Kp, m2.Tp1/60, m2.Tp2/60, m2.Td/60);

%% Comparación con los datos (el % de ajuste aparece en la leyenda)
graficar_ajuste(est, m1, m2, 'Estimación (pulsos 1 y 2)');
graficar_ajuste(val, m1, m2, 'Validación (pulso 3)');

save('modelos_pulsos_3nov.mat', 'm1', 'm2');

%% FUNCIÓN LOCAL

function graficar_ajuste(datos, m1, m2, nombre)
    % Salida medida y simulada por cada modelo, con el % de ajuste de compare.
    % Se grafica aparte para tener los textos en español; los valores son los
    % mismos que entrega compare (condición inicial estimada).
    [y1, ajuste1] = compare(datos, m1);
    [y2, ajuste2] = compare(datos, m2);
    t = datos.SamplingInstants/60;              % min
    figure('Name', nombre);
    plot(t, datos.y, 'k'); hold on
    plot(t, y1.y, '--', 'LineWidth', 1.2);
    plot(t, y2.y, '-', 'LineWidth', 1.2);
    xlabel('Tiempo [min]'); ylabel('Sobrecalentamiento [K]'); grid on
    legend('Medido', sprintf('Primer orden (ajuste %.1f %%)', ajuste1), ...
           sprintf('Segundo orden (ajuste %.1f %%)', ajuste2), 'Location', 'best');
    title(nombre);
end
