%% Identificación del sobrecalentador - Paso 3: dejar la planta lista para sintonizar
% El modelo del paso 2 tiene como entrada la potencia del quemador [kW].
% El controlador, en cambio, mueve la apertura de VV 601 [%]. Este paso:
%   1. Obtiene la curva de la válvula (potencia según apertura) con los datos del 3-11.
%   2. Pasa la ganancia del modelo a °C por % de apertura.
%   3. Arma la planta de segundo orden completa  -> para VERIFICAR los controladores.
%   4. La reduce a primer orden (regla de la mitad) -> para SINTONIZAR con SIMC.
%   5. Estima dónde quedaría la temperatura con fuego continuo.

clear; clc; close all
load('datos_pulsos_3nov.mat', 'TT')          % registro del día completo (paso 1)
load('modelos_pulsos_3nov.mat', 'm2')        % modelo de segundo orden (paso 2)

rho_gn = 0.75;    % kg/Nm3  (mismos supuestos del paso 1)
PCI    = 47000;   % kJ/kg
u_min  = 45;      % %  bajo esta apertura el quemador no enciende
u_max  = 55;      % %  sobre esta apertura el CO se dispara (60 %: 1140 ppm)

%% 1. Curva de la válvula: potencia del quemador según la apertura de VV 601
% Para cada apertura se promedia FTI 304, usando solo las muestras con la
% llama encendida y la apertura sin cambios durante al menos 60 s.
% Así se excluyen los instantes en que la válvula o la llama estaban cambiando.
% [n 1]: 6 muestras hacia atrás y 1 hacia adelante. Mirar 1 hacia adelante
% descarta el último instante antes de un apagado: al pasar a la malla de 10 s
% (paso 1), ese punto queda interpolado entre ~5,5 y ~0,1 Nm3/h y no es real.
n       = 6;                                      % 6 muestras = 60 s
llama   = double(TT.F304 > 1);
estable = movmin(llama, [n 1]) == 1 & ...
          movmax(TT.VV601, [n 1]) == movmin(TT.VV601, [n 1]);

aperturas = [45 50 55 60 65 70 75 80];
F_med = nan(size(aperturas));
for k = 1:numel(aperturas)
    F_med(k) = mean(TT.F304(estable & TT.VV601 == aperturas(k)));
end
Q_med = F_med/3600*rho_gn*PCI;                    % kW

% Pendiente en el rango útil: recta por mínimos cuadrados entre u_min y u_max
util = aperturas >= u_min & aperturas <= u_max;
coef = polyfit(aperturas(util), Q_med(util), 1);
dQdu = coef(1);                                   % kW por cada 1 % de apertura

%% 2. Ganancia por % de apertura
% Bloques en serie: VV 601 -> potencia -> sobrecalentamiento.
% Se supone que la válvula fija el caudal mucho más rápido que la respuesta de
% la temperatura, así que ese bloque es solo una ganancia y no cambia tau ni theta.
K_kW = m2.Kp;              % K/kW
K_u  = K_kW*dQdu;          % °C/%  (una diferencia de 1 K es igual a 1 °C)

%% 3. Planta de segundo orden completa (para verificar los controladores)
tau1   = max(m2.Tp1, m2.Tp2);   % s  constante de tiempo mayor
tau2   = min(m2.Tp1, m2.Tp2);   % s  constante de tiempo menor
theta0 = m2.Td;                 % s  retardo
G2 = tf(K_u, conv([tau1 1], [tau2 1]), 'InputDelay', theta0);

%% 4. Planta de primer orden reducida (para sintonizar con SIMC)
% Regla de la mitad (Skogestad, 2003): la mitad de la constante de tiempo
% menor se suma al retardo y la otra mitad a la constante mayor.
tau   = tau1 + tau2/2;
theta = theta0 + tau2/2;
G1 = tf(K_u, [tau 1], 'InputDelay', theta);

%% 5. Equilibrio con fuego continuo (extrapolación del modelo)
% En equilibrio, sobrecalentamiento = K·Q. Se suma la T de saturación media
% de los pulsos para expresarlo como TT 106.
Tsat0    = mean(TT(timerange(datetime(2026,11,3,12,28,0), ...
                             datetime(2026,11,3,13,49,0)), :).Tsat, 'omitnan');
Q_rango  = polyval(coef, [u_min u_max]);          % kW en los extremos del rango útil
y_eq     = K_kW*Q_rango;                          % K de sobrecalentamiento
TT106_eq = Tsat0 + y_eq;                          % °C

%% 6. Resultados
fprintf('--- Curva de VV 601 ---\n');
for k = 1:numel(aperturas)
    fprintf('%3d %% : FTI 304 = %.2f Nm3/h -> Q = %.1f kW\n', aperturas(k), F_med(k), Q_med(k));
end
fprintf('Pendiente entre %d y %d %%: dQ/du = %.3f kW/%%\n', u_min, u_max, dQdu);

fprintf('\n--- Planta vista por el controlador (entrada VV 601 [%%], salida TT 106 [°C]) ---\n');
fprintf('K_u = %.3f K/kW x %.3f kW/%% = %.2f °C/%%\n', K_kW, dQdu, K_u);
fprintf('Segundo orden: tau1 = %.0f s | tau2 = %.0f s | theta = %.0f s\n', tau1, tau2, theta0);
fprintf('Primer orden reducido: tau = %.0f s (%.1f min) | theta = %.0f s (%.1f min) | theta/tau = %.2f\n', ...
        tau, tau/60, theta, theta/60, theta/tau);

fprintf('\n--- Equilibrio con fuego continuo (T_sat media = %.1f °C) ---\n', Tsat0);
fprintf('VV 601 en %d %%: sobrecalentamiento %.0f K -> TT 106 aprox. %.0f °C\n', u_min, y_eq(1), TT106_eq(1));
fprintf('VV 601 en %d %%: sobrecalentamiento %.0f K -> TT 106 aprox. %.0f °C\n', u_max, y_eq(2), TT106_eq(2));
fprintf('(Extrapolado: el modelo se ajustó entre 43 y 86 K. Se toma como orden de magnitud.)\n');

%% 7. Gráficos
figure('Name', 'Curva de VV 601');
plot(aperturas, Q_med, 'o-', 'LineWidth', 1.2); hold on
plot([u_min u_max], polyval(coef, [u_min u_max]), 'r--', 'LineWidth', 1.5);
xline(u_max, ':', 'Límite por CO');
xlabel('Apertura VV 601 [%]'); ylabel('Potencia del quemador [kW]'); grid on
legend('Medido (3-11)', sprintf('Recta en el rango útil: %.2f kW/%%', dQdu), 'Location', 'southeast');

t = 0:10:3*3600;                                  % s
y2 = step(G2, t);
y1 = step(G1, t);
figure('Name', 'Segundo orden vs primer orden reducido');
plot(t/60, y2, 'LineWidth', 1.5); hold on
plot(t/60, y1, '--', 'LineWidth', 1.5);
xlabel('Tiempo [min]'); ylabel('\DeltaTT 106 [°C]'); grid on
legend('Segundo orden (completo)', 'Primer orden (regla de la mitad)', 'Location', 'southeast');
title('Respuesta a un escalón de 1 % en VV 601');

%% 8. Guardar para el script del controlador
planta.K_u    = K_u;     planta.dQdu  = dQdu;
planta.tau1   = tau1;    planta.tau2  = tau2;    planta.theta0 = theta0;
planta.tau    = tau;     planta.theta = theta;
planta.u_min  = u_min;   planta.u_max = u_max;   planta.Tsat0  = Tsat0;
planta.u0     = u_min;   planta.Tv0   = TT106_eq(1);   % punto de operación: equilibrio en 45 % (extrapolado)
planta.curva_u = aperturas;   planta.curva_Q = Q_med;   % curva de VV 601 (la usa el modelo físico, parte 2)
save('planta_sobrecalentador.mat', 'planta', 'G1', 'G2');
