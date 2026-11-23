%% COMPARACIÓN DE CINCO CONTROLADORES — Lazo VV 601 -> TT 106
% Controladores diseñados con el modelo físico (teóricos) y con el modelo
% identificado (pulsos del 3-11), todos evaluados contra la planta identificada
% de segundo orden: Routh, valor final, respuesta al escalón con step,
% márgenes de estabilidad y simulación en Simulink (Lazo_sc.slx, PID 2DOF)
% con los límites de la válvula y anti-windup.

clear; clc; close all;

%% 1. MODELO DE LA PLANTA (identificado con los pulsos del 3-11, pasos 1 a 3)
% Entrada: apertura de VV 601 [%]. Salida: TT 106 [°C].
% Completa, para verificar:  G(s) = K_u·e^(-theta0·s)/((tau1·s + 1)(tau2·s + 1))
% Reducida, para sintonizar: G(s) = K_u·e^(-theta·s)/(tau·s + 1)  (regla de la mitad)
load('planta_sobrecalentador.mat', 'planta');
p = planta;   % K_u, tau1, tau2, theta0, tau, theta, u0, Tv0, u_min (45 %), u_max (55 %, CO)

% Modelo físico preliminar (primer orden reducido del modelo de 3 estados),
% usado solo para diseñar los controladores teóricos
teo.K_u = 0.318;   % °C/%
teo.tau = 1506;    % s

%% 2. PARÁMETROS DE DISEÑO Y DE LAS PRUEBAS
% tau_c (constante de tiempo deseada en lazo cerrado) se define para cada
% controlador en la sección 4, según la regla con que se diseñó.
cfg.N       = 10;        % -   filtro de la derivativa: tau_f = tau_D/N, 10 es un valor habitual
cfg.Ts      = 1;         % s   período de muestreo del PLC 

cfg.PM_min  = 45;        % °   margen de fase mínimo
cfg.GM_min  = 6;         % dB  margen de ganancia mínimo 6dB es igual a factor de 2
cfg.OS_max  = 10;        % %   sobrepaso máximo

% Prueba A: cambio de set-point y perturbación
cfg.u_ini   = 50;        % %    centro del rango útil 45-55 %
cfg.h_sim   = 4;         % h    
cfg.t_sp    = 10*60;     % s    Cuando pido el cambio
cfg.dSP     = 2;         % °C   escalon (cuanto sube la temperatura)
cfg.t_d     = 2*3600;    % s    cuando ocurre la perturbacion
cfg.d       = -2;        % %   perturbación a la entrada de la planta (cierre de la valvula)
% Margen de la válvula: 50 + dSP/K_u + |d| = 50 + 1,7 + 2 = 53,7 % < 55 %.
% Con d = -5 % llegaría a 56,7 % y se saturaría.

%% 3. RANGO DE TEMPERATURA ALCANZABLE
% Teorema del valor final con la planta sola: ΔTT106(∞) = K_u·Δu
Tv_min = p.Tv0 + p.K_u*(p.u_min - p.u0);
Tv_max = p.Tv0 + p.K_u*(p.u_max - p.u0);

fprintf('--- Planta ---\n');
fprintf('K_u = %.3f °C/%%\n', p.K_u);
fprintf('Completa (verificación): tau1 = %.0f s | tau2 = %.0f s | theta0 = %.0f s\n', p.tau1, p.tau2, p.theta0);
fprintf('Reducida (sintonía):     tau = %.0f s (%.1f min) | theta = %.0f s (%.1f min)\n', ...
        p.tau, p.tau/60, p.theta, p.theta/60);
fprintf('Rango alcanzable de TT 106 con VV 601 entre %g y %g %%: %.1f a %.1f °C\n', ...
        p.u_min, p.u_max, Tv_min, Tv_max);

%% 4. SINTONÍA DE LOS CINCO CONTROLADORES
% El P se excluye: con la planta identificada su ganancia de lazo es
% Kc·K_u ≈ 0,95 < 1 y deja cerca del 50 % del cambio de set-point como error.
%
% SIMC PI (Skogestad, 2003), modelo de primer orden K·e^(-theta·s)/(tau·s + 1):
%   Kc = tau/(K·(tau_c + theta)),   tau_I = min(tau, 4·(tau_c + theta))
% IMC PID con Padé (Rivera, Morari y Skogestad, 1986), primer orden:
%   Kc = (2·tau + theta)/(K·(2·tau_c + theta)),   tau_I = tau + theta/2,
%   tau_D = tau·theta/(2·tau + theta)
% SIMC PID (Skogestad, 2003), segundo orden K·e^(-theta·s)/((tau1·s + 1)(tau2·s + 1)).
%   Entrega la forma serie:  Kc' = tau1/(K·(tau_c + theta)),
%   tau_I' = min(tau1, 4·(tau_c + theta)),  tau_D' = tau2.
%   Se pasa a la forma paralela que usa este script:
%   Kc = Kc'·(1 + tau_D'/tau_I'),  tau_I = tau_I' + tau_D',  tau_D = tau_I'·tau_D'/(tau_I' + tau_D')
% Forma paralela: Kp = Kc, KI = Kc/tau_I, KD = Kc·tau_D

% 1. Teórico: modelo físico preliminar con theta = 0 y tau_c = tau/2.
%    Con theta = 0 el PI y el PID IMC son iguales, así que es un solo controlador.
ctrl(1).nombre = 'Teo';
ctrl(1).desc   = 'Teórico';
ctrl(1).tau_c  = teo.tau/2;
ctrl(1).Kc     = teo.tau/(teo.K_u*ctrl(1).tau_c);
ctrl(1).tauI   = min(teo.tau, 4*ctrl(1).tau_c);
ctrl(1).tauD   = 0;

% 2. Teórico + retardo: el mismo modelo físico más el retardo medido (theta0),
%    SIMC PI con tau_c = theta0 (la misma regla de los identificados)
ctrl(2).nombre = 'TeoR';
ctrl(2).desc   = 'Teórico + retardo';
ctrl(2).tau_c  = p.theta0;
ctrl(2).Kc     = teo.tau/(teo.K_u*(ctrl(2).tau_c + p.theta0));
ctrl(2).tauI   = min(teo.tau, 4*(ctrl(2).tau_c + p.theta0));
ctrl(2).tauD   = 0;

% 3. PI identificado, con el primer orden reducido (SIMC)
ctrl(3).nombre = 'PI1';
ctrl(3).desc   = 'PI (1er orden)';
ctrl(3).tau_c  = p.theta;
ctrl(3).Kc     = p.tau/(p.K_u*(ctrl(3).tau_c + p.theta));
ctrl(3).tauI   = min(p.tau, 4*(ctrl(3).tau_c + p.theta));
ctrl(3).tauD   = 0;

% 4. PID identificado, con el primer orden reducido (IMC con Padé)
ctrl(4).nombre = 'PID1';
ctrl(4).desc   = 'PID (1er orden)';
ctrl(4).tau_c  = p.theta;
ctrl(4).Kc     = (2*p.tau + p.theta)/(p.K_u*(2*ctrl(4).tau_c + p.theta));
ctrl(4).tauI   = p.tau + p.theta/2;
ctrl(4).tauD   = p.tau*p.theta/(2*p.tau + p.theta);

% 5. PID identificado, con el segundo orden completo (SIMC), de serie a paralelo
tc5  = p.theta0;
Kc_s = p.tau1/(p.K_u*(tc5 + p.theta0));
tI_s = min(p.tau1, 4*(tc5 + p.theta0));
tD_s = p.tau2;
ctrl(5).nombre = 'PID2';
ctrl(5).desc   = 'PID (2º orden)';
ctrl(5).tau_c  = tc5;
ctrl(5).Kc     = Kc_s*(1 + tD_s/tI_s);
ctrl(5).tauI   = tI_s + tD_s;
ctrl(5).tauD   = tI_s*tD_s/(tI_s + tD_s);

nc = numel(ctrl);

for i = 1:nc
    ctrl(i).Kp = ctrl(i).Kc;                   % %/°C
    ctrl(i).KI = ctrl(i).Kc/ctrl(i).tauI;      % %/(°C·s)
    ctrl(i).KD = ctrl(i).Kc*ctrl(i).tauD;      % %·s/°C
end

fprintf('\n--- Sintonía ---\n');
fprintf('%-5s | %-20s | %9s | %10s | %11s | %9s || %14s | %12s\n', 'Ctrl', 'Diseño', 'tau_c [s]', ...
        'Kc [%/°C]', 'tau_I [min]', 'tau_D [s]', 'KI [%/(°C·s)]', 'KD [%·s/°C]');
for i = 1:nc
    fprintf('%-5s | %-20s | %9.0f | %10.2f | %11.1f | %9.1f || %14.5f | %12.2f\n', ctrl(i).nombre, ...
            ctrl(i).desc, ctrl(i).tau_c, ctrl(i).Kc, ctrl(i).tauI/60, ctrl(i).tauD, ctrl(i).KI, ctrl(i).KD);
end

%% 5. ANÁLISIS EN EL DOMINIO DE LAPLACE
% Planta COMPLETA de segundo orden; retardo con Padé de primer orden:
%   e^(-theta0·s) ≈ (1 - theta0·s/2)/(1 + theta0·s/2)
denG2 = conv([p.tau1 1], [p.tau2 1]);              % (tau1·s + 1)(tau2·s + 1)
numG  = p.K_u*[-p.theta0/2 1];
denG  = conv(denG2, [p.theta0/2 1]);

% Planta completa con el retardo exacto, para las respuestas al escalón (5.3)
G = tf(p.K_u, denG2, 'InputDelay', p.theta0);

% Prueba A: malla de tiempo y set-point (también los usa la sección 7)
t     = 0:cfg.Ts:cfg.h_sim*3600;
y_ini = p.Tv0 + p.K_u*(cfg.u_ini - p.u0);
r     = y_ini + cfg.dSP*(t >= cfg.t_sp);
estilos = {':', '-.', '-', '--', '-'};

fprintf('\n');
fprintf('--- Análisis en Laplace ---\n');
fprintf('G(s) = %s / (%s)', texto_pol(numG), texto_pol(denG));
if p.theta0 > 0, fprintf('   (retardo aproximado con Padé)'); end
fprintf('\n');

for i = 1:nc
    c = ctrl(i);

    % Controlador C(s) = numC/denC (Variable temporal)
    if c.KI == 0 && c.KD == 0
        numC = c.Kp;               denC = 1;
    else
        numC = [c.KD c.Kp c.KI];   denC = [1 0];
    end

    % 5.1 Ecuación característica (denC·denG + numC·numG = 0) y Routh
    pc    = sumar_pol(conv(denC, denG), conv(numC, numG));
    col   = columna_routh(pc); % Primera columna de routh
    n_cam = sum(diff(sign(col)) ~= 0); % Cuantas veces cambio de signo
    polos = roots(pc);
    ctrl(i).estable = (n_cam == 0 && all(col ~= 0));

    % 5.2 Teorema del valor final
    % Y/R = numR·numG/pc, Y/D = numG·denC/pc (derivativa sobre la medición)
    if c.KI == 0, numR = c.Kp; else, numR = [c.Kp c.KI]; end
    numT = sumar_pol(conv(numR, numG), 0);
    numD = sumar_pol(conv(numG, denC), 0);
    dy_sp = polyval(numT, 0)/polyval(pc, 0)*cfg.dSP;
    dy_d  = polyval(numD, 0)/polyval(pc, 0)*cfg.d;
    % Error permanente
    ctrl(i).e_sp_tvf = cfg.dSP - dy_sp;
    ctrl(i).e_d_tvf  = cfg.dSP - dy_sp - dy_d;

    % 5.3 Respuesta al escalón con step (planta con el retardo exacto)
    % Solo para los lazos estables: en uno inestable la respuesta crece sin límite.
    if ctrl(i).estable
        % Derivativa filtrada sobre la medición: el set-point pasa solo por
        % Cr = Kp + KI/s y la medición por Cy = Kp + KI/s + KD·s/(tau_f·s + 1)
        Cr = pid(c.Kp, c.KI);   %Controlador que ve el set point
        Cy = pid(c.Kp, c.KI, c.KD, c.tauD/cfg.N);   %Controlador que ve al sensor de termperatura
        Tr = feedback(G, Cy)*Cr;                   % Y/R    Funcion como responde la temperatura ante un cambio en el set point
        Td = feedback(G, Cy);                      % Y/D (perturbación a la entrada)    Como responde la T cuando ocurre una perturbacion
        %Escalones
        ctrl(i).y_sp = step(cfg.dSP*Tr, t); % escalon de dSP (2°C)
        ctrl(i).y_d  = step(cfg.d*Td, t);   %Escalon -2%
        info = stepinfo(ctrl(i).y_sp, t, 'SettlingTimeThreshold', 0.02);
        ctrl(i).OS     = info.Overshoot;                               % %  cuanto se paso de los 2°C en %
        ctrl(i).ts     = info.SettlingTime/60;                         % min    en que minuto entro al margen de error del 2% y no volvio a salir
        %Indices de desempeños
        ctrl(i).IAE_sp = trapz(t, abs(cfg.dSP - ctrl(i).y_sp))/60;     % °C·min Calcula el area bajo la curva con trapecios
        ctrl(i).caida  = max(abs(ctrl(i).y_d));                        % °C cuántos grados se desplomó la temperatura antes de que el controlador reaccionara para salvar el proceso
        ctrl(i).IAE_d  = trapz(t, abs(ctrl(i).y_d))/60;                % °C·min
    else
        ctrl(i).y_sp = nan(size(t));   ctrl(i).y_d = nan(size(t));
        ctrl(i).OS = NaN;  ctrl(i).ts = NaN;  ctrl(i).IAE_sp = NaN;  ctrl(i).caida = NaN;  ctrl(i).IAE_d = NaN;
    end

    fprintf('\n%s, %s:  C(s) = (%s) / (%s)\n', c.nombre, c.desc, texto_pol(numC), texto_pol(denC));
    fprintf('  Ecuación característica: %s = 0\n', texto_pol(pc));
    if n_cam == 0 && all(col ~= 0)
        fprintf('  Routh, primera columna: %s -> sin cambios de signo: ESTABLE\n', mat2str(col.', 4));
    else
        fprintf('  Routh, primera columna: %s -> %d cambio(s) de signo: INESTABLE\n', mat2str(col.', 4), n_cam);
    end
    fprintf('  Polos de lazo cerrado: %s\n', mat2str(polos.', 4));
    if ctrl(i).estable
        fprintf('  Valor final: TT 106 sube %.3f de %.1f °C -> error permanente %.3f °C\n', dy_sp, cfg.dSP, ctrl(i).e_sp_tvf);
        fprintf('  Valor final tras la perturbación de %g %%: error permanente %.3f °C\n', cfg.d, ctrl(i).e_d_tvf);
    else
        fprintf('  Valor final: no aplica, el lazo es inestable\n');
    end
end

fprintf('\nRespuesta al escalón con step (retardo exacto, sin límites de la válvula):\n');
fprintf('%-5s | %13s | %15s | %15s | %16s | %18s\n', 'Ctrl', 'Sobrepaso [%]', 't_establ. [min]', ...
        'IAE SP [°C·min]', 'Caída pert. [°C]', 'IAE pert. [°C·min]');
for i = 1:nc
    c = ctrl(i);
    if c.estable
        fprintf('%-5s | %13.1f | %15.1f | %15.1f | %16.3f | %18.1f\n', c.nombre, c.OS, c.ts, c.IAE_sp, c.caida, c.IAE_d);
    else
        fprintf('%-5s | inestable: la respuesta crece sin límite\n', c.nombre);
    end
end

figure('Name', 'Respuesta al escalón (step)');
subplot(2,1,1);
est = find([ctrl.estable]);                 % solo los lazos estables
for i = est, plot(t/60, ctrl(i).y_sp, estilos{i}, 'LineWidth', 1.5); hold on; end
yline(cfg.dSP, 'k:');
ylabel('\DeltaTT 106 [°C]'); grid on;
legend([{ctrl(est).desc}, {'Set-point'}], 'Location', 'southeast');
title(sprintf('Cambio de set-point de %g °C', cfg.dSP));
subplot(2,1,2);
for i = est, plot(t/60, ctrl(i).y_d, estilos{i}, 'LineWidth', 1.5); hold on; end
ylabel('\DeltaTT 106 [°C]'); xlabel('Tiempo [min]'); grid on;
title(sprintf('Perturbación de %g %% a la entrada de la planta', cfg.d));

%% 6. MÁRGENES DE ESTABILIDAD
% Margen de fase y de ganancia de L(jw) = C(jw)·G(jw), con el retardo exacto,
% y retardo adicional que aún cumple PM >= PM_min.
w  = logspace(-6, 1, 50000);                         % rad/s
jw = 1i*w;
Gw = p.K_u*exp(-jw*p.theta0)./((p.tau1*jw + 1).*(p.tau2*jw + 1));   % planta completa en jw
Lw = cell(1, nc);
for i = 1:nc
    c = ctrl(i);
    if isinf(c.tauI), Iw = 0; else, Iw = 1./(c.tauI*jw); end  %Parte integral
    Dw = c.tauD*jw./(c.tauD/cfg.N*jw + 1);  %Parte derivativa
    Cw = c.Kc*(1 + Iw + Dw);    %controlador
    Lw{i} = Cw.*Gw; %respuesta en lazo abierto
    [ctrl(i).PM, ctrl(i).GM_dB, wc] = margenes(w, Lw{i});
    if ctrl(i).PM >= cfg.PM_min && ~isnan(wc)
        ctrl(i).DM_crit = ((ctrl(i).PM - cfg.PM_min)*pi/180)/wc;   % s
    else
        ctrl(i).DM_crit = NaN;                                     % no cumple PM_min
    end
end

fprintf('\n--- Márgenes (mín.: PM %g°, GM %g dB) ---\n', cfg.PM_min, cfg.GM_min);
fprintf('%-5s | %8s | %9s | %28s\n', 'Ctrl', 'PM [°]', 'GM [dB]', 'retardo tolerado con PM ok [s]');
for i = 1:nc
    fprintf('%-5s | %8.1f | %9.1f | %28.0f\n', ctrl(i).nombre, ctrl(i).PM, ctrl(i).GM_dB, ctrl(i).DM_crit);
end

figure('Name', 'Bode del lazo abierto');
subplot(2,1,1);
for i = 1:nc, semilogx(w, 20*log10(abs(Lw{i})), estilos{i}, 'LineWidth', 1.3); hold on; end
yline(0, ':');
ylabel('|L| [dB]'); grid on; legend({ctrl.desc}, 'Location', 'southwest');
title('Lazo abierto L(j\omega)');
subplot(2,1,2);
for i = 1:nc, semilogx(w, unwrap(angle(Lw{i}))*180/pi, estilos{i}, 'LineWidth', 1.3); hold on; end
yline(-180, ':');
ylim([-360, 45]);
ylabel('Fase [°]'); xlabel('\omega [rad/s]'); grid on;

%% 7. PRUEBA A EN SIMULINK: límites de la válvula y anti-windup
% Lazo_sc.slx trabaja en desviaciones respecto del estado inicial (u_ini, y_ini):
% PID 2DOF discreto con derivativa sobre la medición (b = 1, c = 0), salida
% limitada a [u_min - u_ini, u_max - u_ini] y anti-windup por clamping.
% Verifica que la válvula no se sature, condición para que Laplace sea válido.
K_u = p.K_u;  tau1 = p.tau1;  tau2 = p.tau2;  theta0 = p.theta0;  Ts = cfg.Ts;   % planta completa
% En Lazo_sc.slx: Transfer Fcn con Denominator = conv([tau1 1],[tau2 1]) y
% Transport Delay con Time delay = theta0.
u_ini  = cfg.u_ini;                        % y_ini viene de la sección 5
du_min = p.u_min - u_ini;  du_max = p.u_max - u_ini;    % límites en desviación
t_sp   = cfg.t_sp; dSP   = cfg.dSP;  t_d   = cfg.t_d;  d0 = cfg.d;

yA = cell(1, nc);  uA = cell(1, nc);
for i = 1:nc
    Kc = ctrl(i).Kc;   KI = ctrl(i).KI;   KD = ctrl(i).KD;   % P: KI = 0; P y PI: KD = 0
    if ctrl(i).tauD > 0
        Nf = cfg.N/ctrl(i).tauD;               % filtro de Simulink [rad/s] = 1/tau_f
    else
        Nf = 1;                                % sin derivativa, el filtro no influye
    end
    out   = sim('Lazo_sc', 'StopTime', num2str(cfg.h_sim*3600));
    yA{i} = interp1(out.y_sim.Time, squeeze(out.y_sim.Data), t);
    uA{i} = interp1(out.u_sim.Time, squeeze(out.u_sim.Data), t);
    ctrl(i).saturo = any(uA{i} <= p.u_min + 1e-6 | uA{i} >= p.u_max - 1e-6);    %Evita falsos positivos
end

for i = 1:nc
    if ctrl(i).saturo, fprintf('OJO (%s): la válvula tocó un límite en la prueba A; ahí Laplace no es válido.\n', ctrl(i).desc); end
end
if y_ini + cfg.dSP < Tv_min || y_ini + cfg.dSP > Tv_max
    fprintf('OJO: el set-point %.1f °C está fuera del rango alcanzable.\n', y_ini + cfg.dSP);
end

fprintf('\nMétricas de la prueba A en Simulink (con límites de la válvula):\n');
fprintf('%-5s | %13s | %15s | %18s | %21s | %12s\n', 'Ctrl', 'Sobrepaso [%]', 't_establ. [min]', ...
        'Error tras SP [°C]', 'Error tras pert. [°C]', 'IAE [°C·min]');
for i = 1:nc
    m = metricas(t, yA{i}, r, y_ini, cfg);
    ctrl(i).A = m;
    fprintf('%-5s | %13.1f | %15.1f | %18.3f | %21.3f | %12.1f\n', ctrl(i).nombre, m.OS, m.ts, m.e_sp, m.e_d, m.IAE);
end
fprintf('(Errores medidos al final de cada tramo de la prueba; si difieren de los del valor\n');
fprintf(' final de la sección 5, la respuesta aún no terminaba de llegar.)\n');

figure('Name', 'Prueba A: set-point y perturbación');
subplot(2,1,1);
for i = 1:nc, plot(t/60, yA{i}, estilos{i}, 'LineWidth', 1.5); hold on; end
plot(t/60, r, 'k:', 'LineWidth', 1.2);
ylabel('TT 106 [°C]'); grid on;
legend([{ctrl.desc}, {'Set-point'}], 'Location', 'southeast');
title('Cambio de set-point y perturbación (simulación)');
subplot(2,1,2);
for i = 1:nc, plot(t/60, uA{i}, estilos{i}, 'LineWidth', 1.5); hold on; end
yline(p.u_min, ':'); yline(p.u_max, ':');
ylabel('VV 601 [%]'); xlabel('Tiempo [min]'); grid on; ylim([p.u_min - 5, p.u_max + 5]);

%% 8. RESUMEN DE LA COMPARACIÓN
% Todos evaluados contra la planta identificada de segundo orden.
fprintf('\n--- Resumen (planta identificada de segundo orden) ---\n');
fprintf('%-5s | %-20s | %-9s | %7s | %7s | %13s | %15s | %12s | %6s\n', 'Ctrl', 'Diseño', 'Routh', ...
        'PM [°]', 'GM [dB]', 'Sobrepaso [%]', 't_establ. [min]', 'IAE A [°C·min]', 'Saturó');
for i = 1:nc
    c = ctrl(i);
    if c.estable, txt_r = 'estable'; else, txt_r = 'INESTABLE'; end
    if c.saturo,  txt_s = 'sí';      else, txt_s = 'no';        end
    fprintf('%-5s | %-20s | %-9s | %7.1f | %7.1f | %13.1f | %15.1f | %12.1f | %6s\n', c.nombre, c.desc, ...
            txt_r, c.PM, c.GM_dB, c.OS, c.ts, c.A.IAE, txt_s);
end
fprintf('(Sobrepaso y t_establ. de step, sin límites de la válvula; IAE de la prueba A en Simulink.)\n');

% Para la parte 2: los mismos controladores en lazo cerrado con el modelo físico
save('controladores_comparacion.mat', 'ctrl', 'cfg', 'p', 'teo');

%% FUNCIONES LOCALES

function a = limpiar_pol(a)
    % Quita los ceros a la izquierda de un polinomio
    k = find(a ~= 0, 1);
    if isempty(k), a = 0; else, a = a(k:end); end
end

function c = sumar_pol(a, b)
    % Suma dos polinomios de distinto grado
    n = max(numel(a), numel(b));
    c = [zeros(1, n - numel(a)), a] + [zeros(1, n - numel(b)), b];
    c = limpiar_pol(c);
end

function col = columna_routh(a)
    % Primera columna del arreglo de Routh (método épsilon para ceros)
    a = limpiar_pol(a);
    n = numel(a);
    m = ceil(n/2);
    R = zeros(n, m);
    R(1, 1:numel(a(1:2:end))) = a(1:2:end);
    if n > 1
        R(2, 1:numel(a(2:2:end))) = a(2:2:end);
    end
    for i = 3:n
        if R(i-1, 1) == 0
            R(i-1, 1) = 1e-12*max(abs(a));
        end
        for j = 1:m-1
            R(i, j) = (R(i-1, 1)*R(i-2, j+1) - R(i-2, 1)*R(i-1, j+1))/R(i-1, 1);
        end
    end
    col = R(:, 1);
end

function txt = texto_pol(a)
    % Escribe un polinomio como texto
    a = limpiar_pol(a);  n = numel(a) - 1;  partes = {};
    for j = 1:numel(a)
        e = n - j + 1;
        if a(j) == 0 && numel(a) > 1, continue; end
        if e == 0,     partes{end+1} = sprintf('%.4g', a(j));          %#ok<AGROW>
        elseif e == 1, partes{end+1} = sprintf('%.4g·s', a(j));        %#ok<AGROW>
        else,          partes{end+1} = sprintf('%.4g·s^%d', a(j), e);  %#ok<AGROW>
        end
    end
    txt = strrep(strjoin(partes, ' + '), '+ -', '- ');
end

function m = metricas(t, y, r, y_ini, cfg)
    % Sobrepaso, tiempo de establecimiento (2 %), errores e IAE de la prueba A
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

function [PM, GM_dB, wc] = margenes(w, L)
    % Margen de fase y de ganancia a partir de L(jw)
    mag  = abs(L);
    fase = unwrap(angle(L))*180/pi;
    k = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);
    if isempty(k)
        PM = Inf;  wc = NaN;
    else
        wc = w(k);
        PM = 180 + fase(k);
    end
    k = find(fase(1:end-1) > -180 & fase(2:end) <= -180, 1);
    if isempty(k)
        GM_dB = Inf;
    else
        GM_dB = -20*log10(mag(k));
    end
end
