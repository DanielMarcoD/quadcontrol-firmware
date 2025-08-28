#include "FreeRTOS.h"   // Núcleo do FreeRTOS (criação de tarefas, temporização etc.)
#include "task.h"       // Funções de task do FreeRTOS (ex.: vTaskDelay)
#include "supervisor.h" // Estado do voo (ex.: supervisorIsArmed) — útil p/ prints condicionais
#include "debug.h"      // Impressões no console (DEBUG_PRINT/DEBUG_PRINTF)
#include "estimator.h"  // Interface p/ fila de medições do estimador (estimatorDequeue)

/* ==============================
   VARIÁVEIS DE SENSORES (BODY FRAME)
   ------------------------------
   Convenção típica (verifique no seu deck/CF):
   - Eixo X: “nariz” do drone (frente)
   - Eixo Y: lado esquerdo
   - Eixo Z: para cima
   Sinais seguem regra da mão direita: rotações + são CCW ao olhar do eixo positivo.
   ============================== */

// Acelerômetro [m/s^2]
// Mede aceleração específica (inclui gravidade). Em repouso, o módulo deve ~ 9.81 m/s^2.
// Interpretação:
//  - Inclinar o drone “para frente” (nariz para baixo) → ax muda de sinal/magnitude
//  - Inclinar para a esquerda → ay muda
//  - Parado na mesa, valores próximos de (0, 0, ~+9.81) ou (~0, ~0, ~-9.81), depende da orientação do IMU
float ax, ay, az;

// Giroscópio [rad/s]
// Mede velocidade angular (quão rápido o drone gira) em cada eixo.
// Interpretação:
//  - Parado em cima da mesa → ~0 rad/s em todos
//  - Girar yaw (vertical) no sentido anti-horário (visto de cima) → gz positivo
//  - Pitch (nariz sobe/desce) altera gx; Roll (inclina lateral) altera gy
float gx, gy, gz;

// Distância ao solo por Time-of-Flight [m] (TOF do Flow deck, por exemplo)
// Interpretação:
//  - Aproximar do chão → d diminui; Afastar → d aumenta
//  - Faixa útil depende do sensor: Flow v1 ~0.05–1.2 m; Flow v2 (VL53L1x) até ~4 m (aprox.)
//  - Excesso de luz, superfícies muito escuras/absorventes podem degradar leitura
float d;

// Optical flow [pixels por intervalo] (deslocamento aparente da imagem)
// Interpretação:
//  - Drone parado → px e py ~ 0
//  - Movendo “para frente” sobre o solo → px muda de sinal/magnitude
//  - Valores crescem quando está baixo (mais perto do chão) para a MESMA velocidade real
//  - Para converter para velocidade [m/s], é preciso usar d (altura), FOV/calibração e dt
float px, py;

/* ==============================
   LOOP PRINCIPAL DA APLICAÇÃO
   - Lê medições do estimador via fila (non-blocking): cada iteração
     puxa tudo que estiver disponível e atualiza as variáveis globais.
   - Faz um print compacto a cada 100 ms.
   ============================== */
void appMain(void *param)
{
    while (true)
    {
        // Pega todas as medidas disponíveis desde a última iteração
        measurement_t m;
        while (estimatorDequeue(&m))
        {
            switch (m.type)
            {
            case MeasurementTypeGyroscope:
                // Velocidades angulares [rad/s]
                gx = m.data.gyroscope.gyro.x;
                gy = m.data.gyroscope.gyro.y;
                gz = m.data.gyroscope.gyro.z;
                break;

            case MeasurementTypeAcceleration:
                // Acelerações específicas [m/s^2] (inclui gravidade)
                // Checagem de sanidade: sqrt(ax^2+ay^2+az^2) ≈ 9.81 em repouso
                ax = m.data.acceleration.acc.x;
                ay = m.data.acceleration.acc.y;
                az = m.data.acceleration.acc.z;
                break;

            case MeasurementTypeTOF:
                // Distância ao solo [m] por sensor TOF
                // Útil para pairar estável e para escalonar flow → velocidade
                d = m.data.tof.distance;
                break;

            case MeasurementTypeFlow:
                // Optical flow bruto [pixels] no intervalo de integração
                // px ≈ deslocamento da textura no eixo X da câmera; py idem para Y
                // Observação: para velocidade em m/s, normalmente o estimador combina (px,py) com d e dt
                px = m.data.flow.dpixelx;
                py = m.data.flow.dpixely;
                break;

            default:
                // Outros tipos ignorados aqui (magnetômetro, barômetro, etc., se existirem)
                break;
            }
        }

        // Print compacto no console (útil para logar comportamento e checar sinais rapidamente)
        // Acc: vetor de aceleração específica [m/s^2]
        // Gyr: vel. angulares [rad/s]
        // Dis: distância TOF [m]
        // Flow: deslocamentos ópticos [px/intervalo]
        DEBUG_PRINT("Acc: %4.2f %4.2f %4.2f | Gyr: %6.2f %6.2f %6.2f | Dis: %4.2f | Flow: %2.0f %2.0f\n",
            (double)ax,(double)ay,(double)az,
            (double)gx,(double)gy,(double)gz,
            (double)d,
            (double)px,(double)py);

        // Frequência de amostragem deste loop: 10 Hz (100 ms)
        // Se for logar mais “suave”, aumente para 200–500 ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
