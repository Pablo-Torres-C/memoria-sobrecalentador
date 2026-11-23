# memoria-sobrecalentador
# Sintonización del lazo de temperatura del sobrecalentador

Códigos de la Memoria de Título N°585, "Sintonización de los sistemas de control de
sobrecalentador y alimentación de agua en Planta Térmica de Termofluidos".
Pablo Torres Cifuentes, Departamento de Ingeniería Mecánica, Universidad de Concepción, 2026.

## Contenido

| Archivo | Qué hace | Sección del informe |
|---|---|---|
| `ees/calibracion_estacionaria_EES.txt` | Calibración del modelo físico en régimen permanente: balances, propiedades y conductancias | 4.2 |
| `matlab/diagnostico_3nov.m` | Relación entre FTI 302 y FTI 304, y balance de masa de la caldera | 3.2.4 y 3.3.1 |
| `matlab/identificacion_paso1.m` | Prepara los datos del ensayo de pulsos: entrada, salida y ventanas | 4.6.1 |
| `matlab/identificacion_paso2.m` | Ajusta los modelos de primer y segundo orden con retardo y los valida | 4.6.2 |
| `matlab/identificacion_paso3.m` | Curva de VV 601, ganancia Ku, planta completa y reducida | 3.1 y 4.6.3 |
| `matlab/modelo_sobrecalentador_P_ode15s_v2_limpio.m` | Modelo físico de tres estados, modelo reducido y simulación On/Off | 4.3 a 4.5 |
| `matlab/controlador_sobrecalentador_10_comparacion.m` | Sintonía y comparación de los controladores con la planta identificada | 5.1 a 5.6 |
| `matlab/Lazo_sc.slx` | Lazo cerrado en Simulink con límites de la válvula y anti-windup | 5.6 |
| `matlab/comparacion_modelo_fisico.m` | Los mismos controladores en lazo cerrado con el modelo físico | 5.7 |
| `matlab/DATOS_UDEC_03_11_limpio.csv` | Registro de la planta del 3 de noviembre de 2026 | — |

## Requisitos

- MATLAB R20XX o posterior, con System Identification Toolbox, Control System Toolbox y Simulink.
- EES, para la calibración en régimen permanente.

## Orden de ejecución

Desde la carpeta `matlab`:

1. `identificacion_paso1.m` → genera `datos_pulsos_3nov.mat`
2. `identificacion_paso2.m` → genera `modelos_pulsos_3nov.mat`
3. `identificacion_paso3.m` → genera `planta_sobrecalentador.mat`
4. `modelo_sobrecalentador_P_ode15s_v2_limpio.m` (usa `planta_sobrecalentador.mat`)
5. `controlador_sobrecalentador_10_comparacion.m` (usa `planta_sobrecalentador.mat` y `Lazo_sc.slx`) → genera `controladores_comparacion.mat`
6. `comparacion_modelo_fisico.m` (usa `controladores_comparacion.mat` y `planta_sobrecalentador.mat`)

`diagnostico_3nov.m` es independiente y se puede correr en cualquier momento.
Los parámetros del modelo físico (RAC, conductancias y potencia mínima del quemador) salen del EES
y están escritos en el modelo y en `comparacion_modelo_fisico.m`.
