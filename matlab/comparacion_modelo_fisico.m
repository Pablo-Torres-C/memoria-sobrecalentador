%% PARTE 2: LOS CINCO CONTROLADORES CON EL MODELO FÍSICO DE 3 ESTADOS
% Mismos controladores de la parte 1, pero la planta es el modelo físico de
% 3 estados (balances de gases, metal y vapor) con la calibración corregida,
% igual a modelo_sobrecalentador_P_ode15s_v2_limpio.m (lado vapor con forma
% exponencial, aire fijo del ventilador y recalibración con lo medido el 3-11),
% más dos actualizaciones:
%   1. Curva real de VV 601 medida el 3-11, en vez de la razón 1,45.
%   2. Retardo medido (theta0) en la medición de TT 106.
% Se avanza de a un período de muestreo (Ts = 1 s): el PID calcula la apertura
% con la medición retardada y el modelo físico integra ese segundo con ode15s,
% igual que el modelo .m (el sistema es rígido).

clear; clc; close all;
load('controladores_comparacion.mat', 'ctrl', 'cfg', 'p');   % parte 1
load('planta_sobrecalentador.mat', 'planta');                % curva de VV 601 (paso 3)
nc = numel(ctrl);

%% 1. MODELO FÍSICO (calibración corregida, igual al modelo .m)
f.P_abs  = 737.3;     % kPa
f.T_ven  = 167.1;     % °C     T_sat a P_abs
f.cp_v   = 2.331;     % kJ/kgK cp efectivo del vapor
f.M_v    = 0.02197;   % kg/s
f.T_amb  = 15;        % °C
f.P_atm  = 101.325;   % kPa
f.cp_g   = 1.10;      % kJ/kgK
f.PCI    = 47000;     % kJ/kg
f.RAC    = 45.95;     % kg/kg  (O2 con llama del 3-11)
f.m_m    = 257.0;     % kg
f.c_m    = 0.49;      % kJ/kgK
f.V_v    = 0.02934;   % m3
f.V_g    = 0.9653;    % m3
f.UA_per = 0.002387;  % kW/K   gases -> ambiente (desde la carcasa a ~40 °C)
f.hA_g   = 0.005022;  % kW/K   gases -> metal
f.hA_v   = 0.3452;    % kW/K   metal -> vapor
f.Qc_min = 48.86;     % kW     potencia en u_min: FTI 304 = 4,99 Nm3/h con llama (3-11), igual que el modelo .m
f.R_g    = 0.287;     % kJ/kgK
f.R_v    = 0.4615;    % kJ/kgK

% Aire del ventilador: fijo. Se calcula una vez con el O2 y el gas medidos juntos
% en u_min el 3-11 (13,14 % de O2 con 4,99 Nm3/h)
f.M_a    = f.RAC*f.Qc_min/f.PCI;   % kg/s

% Actualización 1: curva real de VV 601 (3-11)
f.curva_u = planta.curva_u;     % %
f.curva_Q = planta.curva_Q;     % kW
% Actualización 2: retardo medido en TT 106
nd = round(p.theta0/cfg.Ts);    % muestras de retardo

%% 2. GANANCIA DEL MODELO FÍSICO CON LA CURVA REAL
x_min = estado_estacionario(Q_de_u(p.u_min, f), f.M_v, f);
x_max = estado_estacionario(Q_de_u(p.u_max, f), f.M_v, f);
K_u_fis = (x_max(3) - x_min(3))/(p.u_max - p.u_min);

fprintf('--- Modelo físico con la curva real de VV 601 ---\n');
fprintf('TT 106 en equilibrio: %.2f °C en %g %% y %.2f °C en %g %%\n', x_min(3), p.u_min, x_max(3), p.u_max);
fprintf('K_u del modelo físico = %.3f °C/%% (identificado: %.3f °C/%%, %.1f veces mayor)\n', ...
        K_u_fis, p.K_u, p.K_u/K_u_fis);

%% 3. PRUEBA: LA MISMA DE LA PARTE 1
% Con el modelo corregido, de 50 a 55 % TT 106 puede subir ~6 °C, así que el
% escalón de 2 °C y la perturbación de -2 % de la parte 1 caben en el rango
% de la válvula (en régimen, VV 601 queda cerca de 53 % < 55 %). Así las dos
% comparaciones quedan en la misma escala. La columna «Saturó» lo confirma.
cfg2 = cfg;   % dSP = 2 °C; d = -2 %, aplicada como potencia equivalente: dQdu·d

t     = 0:cfg2.Ts:cfg2.h_sim*3600;
x0    = estado_estacionario(Q_de_u(cfg2.u_ini, f), f.M_v, f);   % equilibrio en u_ini
y_ini = x0(3);
r     = y_ini + cfg2.dSP*(t >= cfg2.t_sp);

if y_ini + cfg2.dSP > x_max(3)
    fprintf('OJO: el set-point %.2f °C supera el máximo alcanzable (%.2f °C).\n', y_ini + cfg2.dSP, x_max(3));
end

%% 4. LAZO CERRADO CON CADA CONTROLADOR
yF = cell(1, nc);  uF = cell(1, nc);
for i = 1:nc
    [yF{i}, uF{i}] = lazo_cerrado(ctrl(i), f, x0, r, t, nd, cfg2, p, planta.dQdu);
    ctrl(i).saturoF = any(uF{i} <= p.u_min + 1e-6 | uF{i} >= p.u_max - 1e-6);
    ctrl(i).F = metricas(t, yF{i}, r, y_ini, cfg2);
end

fprintf('\n--- Prueba con el modelo físico (SP +%g °C, perturbación %g %%) ---\n', cfg2.dSP, cfg2.d);
fprintf('%-5s | %-20s | %13s | %15s | %18s | %21s | %12s | %6s\n', 'Ctrl', 'Diseño', 'Sobrepaso [%]', ...
        't_establ. [min]', 'Error tras SP [°C]', 'Error tras pert. [°C]', 'IAE [°C·min]', 'Saturó');
for i = 1:nc
    m = ctrl(i).F;
    if ctrl(i).saturoF, txt_s = 'sí'; else, txt_s = 'no'; end
    fprintf('%-5s | %-20s | %13.1f | %15.1f | %18.3f | %21.3f | %12.1f | %6s\n', ctrl(i).nombre, ...
            ctrl(i).desc, m.OS, m.ts, m.e_sp, m.e_d, m.IAE, txt_s);
end
fprintf('(Errores medidos al final de cada tramo; si no son ~0, la respuesta aún no terminaba de llegar.)\n');

estilos = {':', '-.', '-', '--', '-'};
figure('Name', 'Modelo físico: set-point y perturbación');
subplot(2,1,1);
for i = 1:nc, plot(t/60, yF{i}, estilos{i}, 'LineWidth', 1.5); hold on; end
plot(t/60, r, 'k:', 'LineWidth', 1.2);
ylabel('TT 106 [°C]'); grid on;
legend([{ctrl.desc}, {'Set-point'}], 'Location', 'southeast');
title('Lazo cerrado con el modelo físico de 3 estados');
subplot(2,1,2);
for i = 1:nc, plot(t/60, uF{i}, estilos{i}, 'LineWidth', 1.5); hold on; end
yline(p.u_min, ':'); yline(p.u_max, ':');
ylabel('VV 601 [%]'); xlabel('Tiempo [min]'); grid on; ylim([p.u_min - 5, p.u_max + 5]);

save('comparacion_modelo_fisico.mat', 'ctrl', 'cfg2', 'f', 't', 'yF', 'uF', 'r', 'K_u_fis');

%% FUNCIONES LOCALES

function [y, u] = lazo_cerrado(c, f, x0, r, t, nd, cfg, p, dQdu)
    % PID 2DOF discreto (b = 1, c = 0): proporcional e integral sobre el error,
    % derivativa filtrada sobre la medición, salida limitada a [u_min, u_max]
    % y anti-windup por clamping. Planta: modelo físico integrado con ode15s
    % durante cada período de muestreo, con la apertura constante en ese período.
    Ts   = cfg.Ts;
    opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
    if c.tauD > 0, tf = c.tauD/cfg.N; else, tf = 1; end
    n   = numel(t);
    y   = zeros(1, n);   u = zeros(1, n);
    x   = x0;
    buf = x0(3)*ones(nd + 1, 1);                % TT 106 de los últimos theta0 segundos
    I   = 0;   D = 0;   ym_ant = x0(3);
    for k = 1:n
        ym = buf(1);                            % medición retardada theta0
        e  = r(k) - ym;
        if c.KD > 0
            D = (tf*D - c.KD*(ym - ym_ant))/(tf + Ts);    % derivativa filtrada (Euler hacia atrás)
        end
        u_calc = cfg.u_ini + c.Kp*e + I + D;
        u(k)   = min(max(u_calc, p.u_min), p.u_max);
        if ~(u_calc ~= u(k) && sign(e) == sign(u_calc - u(k)))
            I = I + c.KI*Ts*e;                  % integra solo si no empuja más contra el límite
        end
        ym_ant = ym;   y(k) = ym;

        % Potencia del quemador; la perturbación entra como potencia equivalente
        Q  = Q_de_u(u(k), f) + dQdu*cfg.d*(t(k) >= cfg.t_d);
        in = entradas(Q, Q/f.PCI + f.M_a, f.M_v);       % gas + aire fijo del ventilador
        [~, X] = ode15s(@(tt, xx) derivadas(xx, in, f), [0 Ts], x, opts);   % un período de muestreo
        x = X(end, :).';
        buf = [buf(2:end); x(3)];
    end
end

function Q = Q_de_u(u, f)
    % Potencia del quemador según la apertura de VV 601: curva medida el 3-11
    Q = interp1(f.curva_u, f.curva_Q, u, 'linear', 'extrap');
end

function dx = derivadas(x, in, p)
    % Balances de energía transientes de gases, metal y vapor (modelo corregido)
    T_g = x(1);  T_m = x(2);  T_v = x(3);
    C_v = in.M_v*p.cp_v;
    if in.M_gc > 0
        T_in = p.T_amb + in.Q_c/(in.M_gc*p.cp_g);
    else
        T_in = T_g;
    end
    Tg_med = (T_in + T_g)/2;
    Tv_med = (p.T_ven + T_v)/2;
    m_g = p.P_atm/(p.R_g*(Tg_med + 273.15))*p.V_g;
    m_v = p.P_abs/(p.R_v*(Tv_med + 273.15))*p.V_v;
    Q_gm  = p.hA_g*(Tg_med - T_m);
    Q_mv  = C_v*(1 - exp(-p.hA_v/C_v))*(T_m - p.T_ven);   % forma exponencial (flujo interno, pared a T_m)
    Q_per = p.UA_per*(Tg_med - p.T_amb);
    dT_g = (in.Q_c - in.M_gc*p.cp_g*(T_g - p.T_amb) - Q_gm - Q_per)/(m_g*p.cp_g);
    dT_m = (Q_gm - Q_mv)/(p.m_m*p.c_m);
    dT_v = (C_v*(p.T_ven - T_v) + Q_mv)/(m_v*p.cp_v);
    dx = [dT_g; dT_m; dT_v];
end

function in = entradas(Q_c, M_gc, M_v)
    % Q_c [kW], M_gc [kg/s], M_v [kg/s]
    in.Q_c = Q_c;  in.M_gc = M_gc;  in.M_v = M_v;
end

function T_g = gas_cuasiestacionario(Q_c, M_gc, T_m, p)
    % T_g con dT_g/dt = 0 para una T_m dada
    G    = M_gc*p.cp_g;
    H    = (p.hA_g + p.UA_per)/2;
    T_in = p.T_amb + Q_c/G;
    T_g  = (Q_c + G*p.T_amb - H*T_in + p.hA_g*T_m + p.UA_per*p.T_amb)/(G + H);
end

function x = estado_estacionario(Q_c, M_v, p)
    % Estado estacionario con fuego continuo
    M_gc  = Q_c/p.PCI + p.M_a;                 % gas + aire fijo del ventilador
    C_v   = M_v*p.cp_v;
    K_vap = C_v*(1 - exp(-p.hA_v/C_v));        % forma exponencial
    T_ad  = p.T_amb + Q_c/(M_gc*p.cp_g);
    f = @(Tm) p.hA_g*((T_ad + gas_cuasiestacionario(Q_c, M_gc, Tm, p))/2 - Tm) ...
              - K_vap*(Tm - p.T_ven);
    T_m = fzero(f, [p.T_ven, T_ad]);
    T_g = gas_cuasiestacionario(Q_c, M_gc, T_m, p);
    T_v = p.T_ven + K_vap*(T_m - p.T_ven)/C_v;
    x   = [T_g; T_m; T_v];
end

function m = metricas(t, y, r, y_ini, cfg)
    % Sobrepaso, tiempo de establecimiento (2 %), errores e IAE de la prueba
    ventana = t >= cfg.t_sp & t < cfg.t_d;
    k_fin   = find(ventana, 1, 'last');
    y_fin   = y(k_fin);
    dy      = y_fin - y_ini;
    m.OS    = max(0, max(sign(dy)*(y(ventana) - y_fin)))/abs(dy)*100;
    tv = t(ventana);  yv = y(ventana);
    fuera = find(abs(yv - y_fin) > 0.02*abs(dy), 1, 'last');
    if isempty(fuera), m.ts = 0; else, m.ts = (tv(fuera) - cfg.t_sp)/60; end
    m.e_sp  = r(k_fin) - y_fin;
    m.e_d   = r(end) - y(end);
    m.IAE   = trapz(t, abs(r - y))/60;
end
