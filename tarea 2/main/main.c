#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
 
/* ── Pines de entrada ──────────────────────────────────────────── */
#define PIN_CMD_OPEN      GPIO_NUM_4
#define PIN_CMD_CLOSE     GPIO_NUM_5
#define PIN_SW_OPEN_END   GPIO_NUM_18
#define PIN_SW_CLOSE_END  GPIO_NUM_19
#define PIN_IR_SENSOR     GPIO_NUM_33   /* Activo en LOW */
 
/* ── Pines de salida ───────────────────────────────────────────── */
#define PIN_DRV_A         GPIO_NUM_25
#define PIN_DRV_B         GPIO_NUM_26
#define PIN_BUZZER        GPIO_NUM_27
#define PIN_LAMP          GPIO_NUM_14
 
/* ── Tiempo sin actividad para volver al estado de referencia ──── */
#define IDLE_TIMEOUT_MS   300000UL   /* 5 minutos */
 
/* ── Máquina de estados ────────────────────────────────────────── */
typedef enum {
    ST_OPEN,        /* Portón completamente abierto   */
    ST_OPENING,     /* Motor abriendo                 */
    ST_CLOSED,      /* Portón completamente cerrado   */
    ST_CLOSING,     /* Motor cerrando                 */
    ST_HALTED,      /* Detenido a mitad de recorrido  */
    ST_FAULT,       /* Condición de error             */
    ST_SYNC         /* Re-sincronización automática   */
} gate_state_t;
 
volatile gate_state_t gState     = ST_HALTED;
         gate_state_t gPrevState = ST_HALTED;
 
static int64_t gLastActivity = 0;   /* Marca de tiempo del último comando */
 
/* ── Control del motor ─────────────────────────────────────────── */
static inline void drv_stop(void) {
    gpio_set_level(PIN_DRV_A, 0);
    gpio_set_level(PIN_DRV_B, 0);
}
 
static inline void drv_forward(void) {   /* Sentido apertura */
    gpio_set_level(PIN_DRV_A, 1);
    gpio_set_level(PIN_DRV_B, 0);
}
 
static inline void drv_reverse(void) {   /* Sentido cierre   */
    gpio_set_level(PIN_DRV_A, 0);
    gpio_set_level(PIN_DRV_B, 1);
}
 
/* ── Lectura de pines activos en LOW ───────────────────────────── */
static inline int pin_active(gpio_num_t pin) {
    return gpio_get_level(pin) == 0;
}
 
/* ── Transición de estado con registro del anterior ────────────── */
static inline void go(gate_state_t next) {
    gPrevState = gState;
    gState     = next;
}
 
/* TAREA: Lógica de control del portón */
static void task_gate_logic(void *arg)
{
    while (1) {
        int64_t now = esp_timer_get_time() / 1000;
 
        /* Lecturas de entradas */
        const int cmdOpen    = pin_active(PIN_CMD_OPEN);
        const int cmdClose   = pin_active(PIN_CMD_CLOSE);
        const int swAtOpen   = pin_active(PIN_SW_OPEN_END);
        const int swAtClose  = pin_active(PIN_SW_CLOSE_END);
        const int irBlocked  = pin_active(PIN_IR_SENSOR);
 
        /* Actualizar marca de actividad ante cualquier comando */
        if (cmdOpen || cmdClose) {
            gLastActivity = now;
        }
 
        /* Inactividad prolongada → re-sincronizar */
        if ((now - gLastActivity) > IDLE_TIMEOUT_MS && gState != ST_SYNC) {
            go(ST_SYNC);
        }
 
        /* ── Máquina de estados ── */
        switch (gState) {
 
            case ST_OPEN:
                drv_stop();
                if (cmdClose) {
                    go(ST_CLOSING);
                    gLastActivity = now;
                }
                break;
 
            case ST_CLOSED:
                drv_stop();
                if (cmdOpen) {
                    go(ST_OPENING);
                    gLastActivity = now;
                }
                break;
 
            case ST_OPENING:
                drv_forward();
                if (swAtOpen)  { go(ST_OPEN);    break; }
                if (cmdClose)  { go(ST_HALTED);  break; }
                break;
 
            case ST_CLOSING:
                drv_reverse();
                if (irBlocked) { go(ST_HALTED);  break; }   /* Obstáculo detectado */
                if (swAtClose) { go(ST_CLOSED);  break; }
                if (cmdOpen)   { go(ST_HALTED);  break; }
                break;
 
            case ST_HALTED:
                drv_stop();
                if (cmdOpen)   { go(ST_OPENING); }
                else if (cmdClose) { go(ST_CLOSING); }
                break;
 
            case ST_SYNC:
                drv_stop();
                if      (swAtOpen)  { gState = ST_OPEN;   }
                else if (swAtClose) { gState = ST_CLOSED; }
                else                { go(ST_FAULT);       }
                gLastActivity = now;
                break;
 
            case ST_FAULT:
                drv_stop();
                if      (gPrevState == ST_CLOSING) { gState = ST_OPENING; }
                else if (gPrevState == ST_OPENING) { gState = ST_CLOSING; }
                else                               { gState = ST_HALTED;  }
                break;
        }
 
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
 
/* TAREA: Indicadores visuales y sonoros */
static void task_indicators(void *arg)
{
    int     buzOn       = 0;
    int     lampOn      = 0;
    int64_t lastToggle  = 0;
 
    while (1) {
        int64_t now = esp_timer_get_time() / 1000;
 
        if (gState == ST_CLOSING) {
            /* Parpadeo a 300 ms mientras cierra */
            if ((now - lastToggle) > 300) {
                buzOn  = !buzOn;
                lampOn = !lampOn;
                gpio_set_level(PIN_BUZZER, buzOn);
                gpio_set_level(PIN_LAMP,   lampOn);
                lastToggle = now;
            }
        } else if (gState == ST_FAULT) {
            /* Fijo encendido en fallo */
            gpio_set_level(PIN_BUZZER, 1);
            gpio_set_level(PIN_LAMP,   1);
        } else {
            /* Todo apagado en cualquier otro estado */
            gpio_set_level(PIN_BUZZER, 0);
            gpio_set_level(PIN_LAMP,   0);
        }
 
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
 
/*  Configuración de GPIO */
static void gpio_setup(void)
{
    const gpio_config_t inputs = {
        .pin_bit_mask = (1ULL << PIN_CMD_OPEN)    |
                        (1ULL << PIN_CMD_CLOSE)   |
                        (1ULL << PIN_SW_OPEN_END) |
                        (1ULL << PIN_SW_CLOSE_END)|
                        (1ULL << PIN_IR_SENSOR),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&inputs);
 
    const gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << PIN_DRV_A)  |
                        (1ULL << PIN_DRV_B)  |
                        (1ULL << PIN_BUZZER) |
                        (1ULL << PIN_LAMP),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&outputs);
}
 
/* Punto de entrada */
void app_main(void)
{
    gpio_setup();
    gLastActivity = esp_timer_get_time() / 1000;
 
    xTaskCreate(task_gate_logic,  "gate_logic",  4096, NULL, 5, NULL);
    xTaskCreate(task_indicators,  "indicators",  2048, NULL, 5, NULL);
}