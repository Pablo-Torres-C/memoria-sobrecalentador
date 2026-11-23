%% DIAGNÓSTICO DEL 3-11: MEDIDORES DE GAS Y BALANCE DE AGUA (capítulo 3)
% Con el registro de la planta del 3-11-2026, calcula:
%   1. La relación entre FTI 302 y FTI 304 (sección 3.3.1): si FTI 302 mide
%      el gas total, debe subir lo mismo que FTI 304 cuando enciende el SC.
%   2. El balance de masa de la caldera (sección 3.2.4): agua que entra
%      (FTI 301, totalizador) contra vapor que sale (FTI 303).
% Ventana: 10:00 a 15:35, con la planta ya en operación (antes, arranque).
%
% Datos: DATOS_UDEC_03_11_limpio.csv (la misma exportación que usa el paso 1).

clear; clc; close all

%% 1. LEER LOS DATOS
D = readtable('DATOS_UDEC_03_11_limpio.csv');
t = datetime(D.anio, D.mes, D.dia, D.hora, D.minutos, D.segundos);
[t, iu] = unique(t);                 % ordena y elimina tiempos repetidos
D = D(iu, :);

dia = dateshift(t(1), 'start', 'day');
v   = t >= dia + hours(10) & t <= dia + hours(15) + minutes(35);   % ventana 10:00-15:35
T   = D(v, :);  tv = t(v);

%% 2. FTI 302 CONTRA FTI 304
F302  = T.fti_302_Nm3h;     % Nm3/h, gas en la sala de calderas
F304  = T.fti_304_Nm3h;     % Nm3/h, gas al sobrecalentador
ok    = ~isnan(F302) & ~isnan(F304);
c     = polyfit(F304(ok), F302(ok), 1);          % FTI 302 = c(2) + c(1)·FTI 304
R     = corrcoef(F304(ok), F302(ok));
llama = F304 > 1;                                % quemador encendido: mismo criterio del paso 1

fprintf('--- FTI 302 contra FTI 304 (10:00-15:35, %d muestras) ---\n', sum(ok));
fprintf('FTI 302 = %.2f + %.3f·FTI 304   (r = %.3f)\n', c(2), c(1), R(1,2));
fprintf('FTI 302 medio: %.2f Nm3/h con el SC apagado y %.2f Nm3/h encendido (FTI 304 = %.2f Nm3/h)\n', ...
        mean(F302(ok & ~llama)), mean(F302(ok & llama)), mean(F304(ok & llama)));
fprintf('Pendiente cercana a 1: el gas que mide FTI 304 también pasa por FTI 302.\n');
fprintf('Término constante: consumo de la caldera según FTI 302.\n');

figure('Name', 'FTI 302 contra FTI 304');
plot(F304(ok), F302(ok), '.'); hold on
x = [0 max(F304)];
plot(x, polyval(c, x), 'r--', 'LineWidth', 1.2);
xlabel('FTI 304 [Nm^3/h]'); ylabel('FTI 302 [Nm^3/h]'); grid on
legend('Registro 3-11', sprintf('FTI 302 = %.2f + %.3f·FTI 304', c(2), c(1)), 'Location', 'northwest');
title('Medidores de gas (03-11-2026, 10:00-15:35)');

%% 3. BALANCE DE MASA DE LA CALDERA
% FTI 301 es un totalizador [L]: el caudal sale de la diferencia entre la
% primera y la última lectura de la ventana.
V301  = T.fti_301_lts;
horas = hours(tv(end) - tv(1));
q_L   = (V301(end) - V301(1))/horas;             % L/h

% Densidad del agua de alimentación a la temperatura media de TT 101
% (Çengel, tabla A-4, líquido saturado)
T_tab  = [80 85 90 95];                          % °C
vf_tab = [0.001029 0.001032 0.001036 0.001040];  % m3/kg
T101   = mean(T.tt_101_C, 'omitnan');            % °C
rho_w  = 1/interp1(T_tab, vf_tab, T101);         % kg/m3
m_agua = q_L/1000*rho_w;                         % kg/h

m_vapor = mean(T.fti_303_kgh, 'omitnan');        % kg/h
dif     = m_agua - m_vapor;

err301 = 0.05*m_agua;                            % error de FTI 301: ±5 % (Vidal 2025, Tabla 6)
res301 = 10/(V301(end) - V301(1))*100;           % resolución del totalizador (10 L), en %

fprintf('\n--- Balance de masa de la caldera (10:00-15:35, %.2f h) ---\n', horas);
fprintf('FTI 301: %g -> %g L  =>  %.0f L/h\n', V301(1), V301(end), q_L);
fprintf('TT 101 media: %.1f °C  =>  densidad %.1f kg/m3  =>  agua %.1f kg/h\n', T101, rho_w, m_agua);
fprintf('FTI 303 medio (vapor): %.1f kg/h\n', m_vapor);
fprintf('Diferencia: %.1f kg/h (%.0f %% del agua de alimentación)\n', dif, dif/m_agua*100);
fprintf('Incertidumbre de FTI 301: ±%.1f kg/h (±5 %%) | resolución del totalizador: %.1f %%\n', err301, res301);
fprintf('Si fuera aumento de nivel, la caldera acumularía %.0f kg de agua en la ventana.\n', dif*horas);
