%% MODELO DINÁMICO DEL SOBRECALENTADOR — Planta Térmica Lab. Termofluidos UdeC
% Modelo de tres estados (balances de energía transientes de gases, metal y vapor):
%   T_g : gases de combustión dentro del SC   [°C]
%   T_m : metal del serpentín                 [°C]
%   T_v : vapor a la salida del SC (TT 106)   [°C]
% Sistema rígido (escalas de ~5 s, ~6 s y ~37 min): se integra con ode15s,
% por tramos de entrada constante para no cruzar discontinuidades.
%
% Necesita: planta_sobrecalentador.mat (identificacion_paso3.m), por la curva
% de VV 601 medida el 3-11.

clear; clc; close all;

%% 1. PARÁMETROS (calibración estacionaria en EES)
% Punto de operación: 24-09-2026, ventana 11:43-12:16.
% O2 y potencia del quemador en 45 %, medidos juntos: 3-11-2026.

% Vapor
p.P_abs     = 737.3;        % kPa
p.T_ven     = 167.1;        % °C     T_sat a P_abs
p.cp_v      = 2.331;        % kJ/kgK cp efectivo entre entrada y salida
p.M_v       = 0.02197;      % kg/s

% Gases y combustible
p.T_amb     = 15;           % °C
p.P_atm     = 101.325;      % kPa
p.cp_g      = 1.10;         % kJ/kgK
p.PCI       = 47000;        % kJ/kg
p.RAC       = 45.95;        % kg/kg  relación aire-combustible real (O2 = 13,14 %, 3-11)

% Geometría y metal
p.m_m       = 257.0;        % kg     masa metálica del serpentín
p.c_m       = 0.49;         % kJ/kgK
p.V_v       = 0.02934;      % m3     volumen interior de los tubos
p.V_g       = 0.9653;       % m3     volumen de gases en el hogar

% Parámetros calibrados
p.UA_per    = 0.002387;     % kW/K   gases -> ambiente (pérdidas por la carcasa)
p.hA_g      = 0.005022;     % kW/K   gases -> metal
p.hA_v      = 0.3452;       % kW/K   metal -> vapor
p.Qc_min    = 48.86;        % kW     potencia en u_min: FTI 304 = 4,99 Nm3/h con llama (3-11)

% Punto de operación de la calibración (24-09)
op.T_m0     = 200.6;        % °C     temperatura media de pared
op.T_vsal   = 200.6;        % °C     promedio TT 106
op.T_gsal   = 159.7;        % °C     promedio TT 105

% Aire del ventilador: fijo. Se calcula una vez con el O2 y el gas medidos juntos en u_min
p.M_a       = p.RAC*p.Qc_min/p.PCI;   % kg/s

%% 2. PARÁMETROS DE OPERACIÓN Y SIMULACIÓN
p.R_g       = 0.287;        % kJ/kgK gases (≈ aire)
p.R_v       = 0.4615;       % kJ/kgK vapor de agua

% Válvula VV 601: curva medida el 3-11 (potencia según apertura, paso 3)
load('planta_sobrecalentador.mat', 'planta');
p.curva_u   = planta.curva_u;   % %
p.curva_Q   = planta.curva_Q;   % kW
p.u_min     = 45;           % %      bajo esta apertura el quemador se apaga
p.u_max     = 55;           % %      sobre esta apertura el CO se dispara
p.u0        = 45;           % %      punto de operación del modelo reducido

% Operación on/off (24-09, 45 %). No hay purga fija: el 3-11 el quemador
% volvió a encender a los 30 y 80 s de apagarse. Con el quemador apagado se
% supone que tampoco entra aire.
p.t_on      = 6.8*60;       % s
p.t_off     = 4.0*60;       % s
p.SP        = 240;          % °C     set-point del tablero (modo 'termostato_TT106')
p.hist      = 5;            % °C     histéresis supuesta

% Simulación
cfg.opts    = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
cfg.dt      = 0.2;          % s      paso de la malla de salida
cfg.h_esc   = 6;            % h
cfg.t_esc   = 600;          % s
cfg.du      = 10;           % %      escalón de VV 601 (45 -> 55 %, todo el rango útil)
cfg.h_onoff = 8;            % h
cfg.h_prom  = 1;            % h      horas finales que se promedian
cfg.modo    = 'ciclo_observado'; % 'ciclo_observado' o 'termostato_TT106'

x0  = [op.T_gsal; op.T_m0; op.T_vsal];

%% 3. MODELO REDUCIDO DE PRIMER ORDEN (fuego continuo en u0)
% Gases y vapor cuasiestacionarios respecto del metal:
%   tau = m_m·c_m/(K_gas + K_vap),   K = dT_v/dQ
C_v   = p.M_v*p.cp_v;
Q_lin = Qc_de_u(p.u0, p);
x_lin = estado_estacionario(Q_lin, p.M_v, p);

G     = (Q_lin/p.PCI + p.M_a)*p.cp_g;
H     = (p.hA_g + p.UA_per)/2;
K_gas = p.hA_g*(1 - p.hA_g/(2*(G + H)));
K_vap = C_v*(1 - exp(-p.hA_v/C_v));
tau   = p.m_m*p.c_m/(K_gas + K_vap);

% Ganancia estática por diferencia centrada (±1 % de potencia)
dQ    = 0.01*Q_lin;
xp    = estado_estacionario(Q_lin + dQ, p.M_v, p);
xm    = estado_estacionario(Q_lin - dQ, p.M_v, p);
K     = (xp(3) - xm(3))/(2*dQ);                  % °C/kW
dQdu  = pendiente_valvula(p);                    % kW/%

fprintf('--- Modelo reducido (fuego continuo a %d%%) ---\n', p.u0);
fprintf('Estado estacionario: T_g = %.0f °C | T_m = %.1f °C | T_v = %.1f °C\n', x_lin);
fprintf('tau = %.0f s (%.1f min) | K = %.3f °C/kW\n', tau, tau/60, K);
fprintf('K_u = %.3f °C/%% (dQ/du = %.2f kW/%% entre %d y %d %%)\n', K*dQdu, dQdu, p.u_min, p.u_max);

%% 4. SIMULACIÓN A: escalón de VV 601 con fuego continuo
% Modelo completo (3 estados) contra el reducido de primer orden.
tA = 0:cfg.dt:cfg.h_esc*3600;  t_esc = cfg.t_esc;  u_A = [p.u0, p.u0 + cfg.du];
bordes = [tA(1), t_esc, tA(end)];
solA = cell(1, 2);  x_ini = x_lin;
for j = 1:2
    Qc      = Qc_de_u(u_A(j), p);
    in      = entradas(Qc, Qc/p.PCI + p.M_a, p.M_v);
    solA{j} = integrar_tramo(bordes(j), bordes(j+1), x_ini, in, p, cfg);
    x_ini   = solA{j}.y(:, end);
end
xA = evaluar_en_malla(solA, tA);
dQ_esc = Qc_de_u(u_A(2), p) - Q_lin;
Tv_red = x_lin(3) + K*dQ_esc*(1 - exp(-max(tA - t_esc, 0)/tau));

figure('Name', 'Escalón VV 601');
plot(tA/60, xA(3,:), 'LineWidth', 1.5); hold on;
plot(tA/60, Tv_red, '--', 'LineWidth', 1.2);
xlabel('Tiempo [min]'); ylabel('T_{v,sal,sc} [°C]'); grid on;
legend('Modelo completo (3 estados)', 'Primer orden reducido', 'Location', 'southeast');
title(sprintf('Escalón VV 601: %d%% \\rightarrow %d%%, fuego continuo', u_A(1), u_A(2)));

%% 5. SIMULACIÓN B: operación on/off
% 'ciclo_observado' : tiempos de encendido y apagado medidos el 24-09
% 'termostato_TT106': apaga con TT 106 >= SP y reenciende con TT 106 <= SP - hist
modo = cfg.modo;  termostato = strcmp(modo, 'termostato_TT106');
u_B  = p.u0;
tB   = 0:cfg.dt:cfg.h_onoff*3600;  t_fin = tB(end);
Q_on    = Qc_de_u(u_B, p);
in_enc  = entradas(Q_on, Q_on/p.PCI + p.M_a, p.M_v);
in_apag = entradas(0, 0, p.M_v);          % quemador y ventilador detenidos (supuesto)

tt = 0;  xx = x0;  fase = 'enc';
solB   = {};
inicio = zeros(0, 2);                  % [instante de inicio, quemador 1/0] de cada tramo

while tt < t_fin
    umbral = [];  sentido = 0;
    if strcmp(fase, 'enc')
        in = in_enc;
        if termostato, t_lim = t_fin;  umbral = p.SP;  sentido = +1;
        else,          t_lim = tt + p.t_on;
        end
    else
        in = in_apag;
        if termostato, t_lim = t_fin;  umbral = p.SP - p.hist;  sentido = -1;
        else,          t_lim = tt + p.t_off;
        end
    end
    sol = integrar_tramo(tt, min(t_lim, t_fin), xx, in, p, cfg, umbral, sentido);
    solB{end+1}      = sol;                                  %#ok<SAGROW>
    inicio(end+1, :) = [tt, strcmp(fase, 'enc')];            %#ok<SAGROW>
    tt = sol.x(end);  xx = sol.y(:, end);
    if strcmp(fase, 'enc'), fase = 'apagado'; else, fase = 'enc'; end
end
xB  = evaluar_en_malla(solB, tB);
enc = interp1([inicio(:,1); t_fin], [inicio(:,2); inicio(end,2)], tB, 'previous');

ult = tB >= tB(end) - cfg.h_prom*3600;
fprintf('--- On/off (%s), últimas %g h ---\n', modo, cfg.h_prom);
fprintf('TT106: modelo prom %.1f °C (%.1f a %.1f) | medido prom %.1f °C\n', ...
        mean(xB(3,ult)), min(xB(3,ult)), max(xB(3,ult)), op.T_vsal);
fprintf('TT105: modelo prom %.0f °C | medido prom %.1f °C | fracción encendido %.2f\n', ...
        mean(xB(1,ult)), op.T_gsal, mean(enc(ult)));

figure('Name', 'On/off');
subplot(3,1,1);
plot(tB/60, xB(3,:), 'LineWidth', 1.5); hold on; yline(op.T_vsal, '--');
if termostato, txt = 'termostato en TT 106'; else, txt = 'ciclo medido el 24-09'; end
ylabel('T_{v,sal,sc} [°C]'); grid on; title(['Operación On/Off (' txt ')']);
subplot(3,1,2);
plot(tB/60, xB(1,:), 'LineWidth', 1.2); hold on; yline(op.T_gsal, '--');
ylabel('T_{g} [°C]'); grid on;
subplot(3,1,3);
stairs(tB/60, enc, 'LineWidth', 1.2); ylim([-0.2 1.2]);
xlabel('Tiempo [min]'); ylabel('Quemador'); yticks([0 1]); yticklabels({'apagado','encendido'}); grid on;

%% FUNCIONES LOCALES

function dx = derivadas(x, in, p)
    % Balances de energía transientes de gases, metal y vapor
    T_g = x(1);  T_m = x(2);  T_v = x(3);
    C_v = in.M_v*p.cp_v;
    if in.M_gc > 0
        T_in = p.T_amb + in.Q_c/(in.M_gc*p.cp_g);   % temperatura de llama adiabática
    else
        T_in = T_g;
    end
    Tg_med = (T_in + T_g)/2;                        % temperatura media de los gases
    Tv_med = (p.T_ven + T_v)/2;                     % temperatura media del vapor
    m_g = p.P_atm/(p.R_g*(Tg_med + 273.15))*p.V_g;  % masa de gases (gas ideal)
    m_v = p.P_abs/(p.R_v*(Tv_med + 273.15))*p.V_v;  % masa de vapor (gas ideal)
    Q_gm  = p.hA_g*(Tg_med - T_m);                  % gases -> metal
    Q_mv  = C_v*(1 - exp(-p.hA_v/C_v))*(T_m - p.T_ven); % metal -> vapor, forma exponencial (flujo interno, pared a T_m)
    Q_per = p.UA_per*(Tg_med - p.T_amb);            % gases -> ambiente
    dT_g = (in.Q_c - in.M_gc*p.cp_g*(T_g - p.T_amb) - Q_gm - Q_per)/(m_g*p.cp_g);  % gases
    dT_m = (Q_gm - Q_mv)/(p.m_m*p.c_m);                                            % metal
    dT_v = (C_v*(p.T_ven - T_v) + Q_mv)/(m_v*p.cp_v);                              % vapor
    dx = [dT_g; dT_m; dT_v];
end

function in = entradas(Q_c, M_gc, M_v)
    % Q_c [kW], M_gc [kg/s], M_v [kg/s]
    in.Q_c = Q_c;  in.M_gc = M_gc;  in.M_v = M_v;
end

function Q = Qc_de_u(u, p)
    % Potencia del quemador según la apertura de VV 601: curva medida el 3-11,
    % limitada al rango útil [u_min, u_max]; bajo u_min el quemador se apaga
    if u < p.u_min
        Q = 0;
    else
        u = min(u, p.u_max);
        Q = interp1(p.curva_u, p.curva_Q, u, 'linear');
    end
end

function d = pendiente_valvula(p)
    % dQ/du en el rango útil (kW/%): recta entre u_min y u_max, igual que el paso 3
    d = (Qc_de_u(p.u_max, p) - Qc_de_u(p.u_min, p))/(p.u_max - p.u_min);
end

function T_g = gas_cuasiestacionario(Q_c, M_gc, T_m, p)
    % T_g con dT_g/dt = 0 para una T_m dada
    G    = M_gc*p.cp_g;
    H    = (p.hA_g + p.UA_per)/2;
    T_in = p.T_amb + Q_c/G;
    T_g  = (Q_c + G*p.T_amb - H*T_in + p.hA_g*T_m + p.UA_per*p.T_amb)/(G + H);
end

function x = estado_estacionario(Q_c, M_v, p)
    % Estado estacionario con fuego continuo: se resuelve T_m con fzero
    M_gc  = Q_c/p.PCI + p.M_a;
    C_v   = M_v*p.cp_v;
    K_vap = C_v*(1 - exp(-p.hA_v/C_v));
    T_ad  = p.T_amb + Q_c/(M_gc*p.cp_g);
    f = @(Tm) p.hA_g*((T_ad + gas_cuasiestacionario(Q_c, M_gc, Tm, p))/2 - Tm) ...
              - K_vap*(Tm - p.T_ven);
    T_m = fzero(f, [p.T_ven, T_ad]);
    T_g = gas_cuasiestacionario(Q_c, M_gc, T_m, p);
    T_v = p.T_ven + K_vap*(T_m - p.T_ven)/C_v;
    x   = [T_g; T_m; T_v];
end

function sol = integrar_tramo(t0, t1, x0, in, p, cfg, umbral, sentido)
    % Integra un tramo de entrada constante; termina antes si TT 106 cruza el umbral
    opts = cfg.opts;
    if nargin >= 7 && ~isempty(umbral)
        opts = odeset(opts, 'Events', @(t, x) cruce_TT106(x, umbral, sentido));
    end
    sol = ode15s(@(t, x) derivadas(x, in, p), [t0 t1], x0, opts);
end

function [valor, terminal, direccion] = cruce_TT106(x, umbral, sentido)
    % Evento: TT 106 cruza el umbral en el sentido indicado
    valor     = x(3) - umbral;
    terminal  = 1;
    direccion = sentido;
end

function x = evaluar_en_malla(sols, t)
    % Evalúa la solución por tramos en una malla uniforme
    x = zeros(3, numel(t));
    for j = 1:numel(sols)
        s   = sols{j};
        sel = t >= s.x(1) & t <= s.x(end);
        if any(sel), x(:, sel) = deval(s, t(sel)); end
    end
end
