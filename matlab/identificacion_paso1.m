%% Identificación del sobrecalentador - Paso 1: preparar los datos
% Ensayo de pulsos del 3-11-2026 (termostato 275 <-> 230 °C, VV 601 en 45 %).
% Todas las horas son las del registro de la planta (el reloj propio iba
% 3 min 02 s adelantado y las anotaciones se corrigieron con ese desfase).
%
% Datos: DATOS_UDEC_03_11_limpio.csv, la exportación del registro de la planta
% pasada a CSV con punto decimal. El .xls original es una tabla HTML y, al abrirlo
% en Excel con configuración en español, los números pueden quedar como
% texto o con el punto leído como separador de miles.

clear; clc; close all

%% 1. Leer los datos
D = readtable('DATOS_UDEC_03_11_limpio.csv');

% Tiempo armado desde las columnas numéricas (evita problemas de formato de fecha)
t = datetime(D.anio, D.mes, D.dia, D.hora, D.minutos, D.segundos);

TT106 = D.tt_106_C;       % °C, vapor aguas abajo del SC
PT201 = D.pt_201_bar;     % bar manométrico, presión de caldera
F304  = D.fti_304_Nm3h;   % Nm3/h, gas al sobrecalentador
F303  = D.fti_303_kgh;    % kg/h, flujo de vapor
VV601 = D.vv_601_pct;     % %, apertura de VV 601

TT = timetable(TT106, PT201, F304, F303, VV601, 'RowTimes', t);
TT = sortrows(TT);
[~, iu] = unique(TT.Time);          % elimina tiempos repetidos
TT = TT(iu, :);

% Malla regular de 10 s (iddata necesita muestreo uniforme)
Ts = 10;                            % s
TT = retime(TT, 'regular', 'linear', 'TimeStep', seconds(Ts));

%% 2. Entrada: potencia del quemador [kW]
rho_gn = 0.75;                      % kg/Nm3 (supuesto, sección 3.1 del informe)
PCI    = 47000;                     % kJ/kg  (supuesto, sección 3.1 del informe)
TT.Q = TT.F304/3600*rho_gn*PCI;     % kW
TT.Q(TT.F304 < 1) = 0;              % apagado: FTI 304 marca ~0,1 por offset

%% 3. Salida: sobrecalentamiento [K] = TT 106 - Tsat(PT 201)
% Tabla de saturación (Çengel, tabla A-5).
P_tab    = [500 550 600 650 700 750 800 850]/100;                   % bar abs
Tsat_tab = [151.83 155.46 158.83 161.98 164.95 167.75 170.41 172.94]; % °C

TT.Tsat = interp1(P_tab, Tsat_tab, TT.PT201 + 1.013, 'linear');     % NaN fuera de rango
TT.y    = TT.TT106 - TT.Tsat;

%% 4. Ventanas de estimación (pulsos 1 y 2) y validación (pulso 3)
% procest supone entrada cero antes del inicio de cada ventana. Por eso ambas
% parten con el quemador apagado desde hace más que el retardo:
% 12:28 (apagado desde 12:20) y 13:28 (apagado desde 13:16).
tEst = timerange(datetime(2026,11,3,12,28,0), datetime(2026,11,3,13,28,0)); % Pulso 1 y 2
tVal = timerange(datetime(2026,11,3,13,28,0), datetime(2026,11,3,13,49,0)); % Pulso 3
E = TT(tEst, :);
W = TT(tVal, :);

est = iddata(E.y, E.Q, Ts, 'TimeUnit', 'seconds', ...
    'InputName', 'Q', 'InputUnit', 'kW', ...
    'OutputName', 'Sobrecalentamiento', 'OutputUnit', 'K');
val = iddata(W.y, W.Q, Ts, 'TimeUnit', 'seconds', ...
    'InputName', 'Q', 'InputUnit', 'kW', ...
    'OutputName', 'Sobrecalentamiento', 'OutputUnit', 'K');

fprintf('Estimación: %d muestras, %d NaN en la salida\n', size(est,1), sum(isnan(est.y)));
fprintf('Validación: %d muestras, %d NaN en la salida\n', size(val,1), sum(isnan(val.y)));

save('datos_pulsos_3nov.mat', 'est', 'val', 'TT');

%% 5. Gráfico de control (12:15 a 13:55)
V = TT(timerange(datetime(2026,11,3,12,15,0), datetime(2026,11,3,13,55,0)), :);
lim = datetime(2026,11,3,[12 13 13],[28 28 49],0);   % inicio est., inicio val., fin val.

figure('Name', 'Pulsos 3-11-2026')
tiledlayout(3, 1, 'TileSpacing', 'compact');

ax(1) = nexttile;
plot(V.Time, V.y); ylabel('Sobrecalentamiento [K]'); grid on
title('Ensayo de pulsos (03-11-2026)')

ax(2) = nexttile;
stairs(V.Time, V.Q); ylabel('Q quemador [kW]'); grid on

ax(3) = nexttile;
yyaxis left;  plot(V.Time, V.PT201); ylabel('PT 201 [bar]')
yyaxis right; plot(V.Time, V.F303);  ylabel('FTI 303 [kg/h]')
grid on

for k = 1:3
    xline(ax(k), lim, '--k');
end
linkaxes(ax, 'x')
