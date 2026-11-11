/* chispa.h - Configuracion, tipos, recursos compartidos y drivers de CHISPA
 * Unico header de la aplicacion (chispa_app.c y chispa_drivers.c). */
#ifndef CHISPA_H
#define CHISPA_H
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* =====================================================================
 *  MODO DE PRUEBA POR MODULO  (cambiar el numero, compilar y cargar)
 *    0 = programa completo de Chispa
 *    1 = OLED (I2C)          2 = Pulsadores (EXTI)   3 = LDR (ADC)
 *    4 = Buzzer (PWM TIM3)   5 = LED RGB (PWM TIM4)  6 = MPU6050 (I2C)
 * ===================================================================== */
#define MODO_PRUEBA  0

/* ---------- Tiempo acelerado: 1 "hora" de Chispa = 10 s reales ---------- */
#define HORA_MS            10000u

/* ---------- Eventos que llegan a la tarea Logica por qEventos ---------- */
typedef enum {
  EV_NINGUNO = 0,
  EV_COMER, EV_JUGAR, EV_LIMPIAR, EV_DORMIR,       /* desde ISR (botones PC0..PC3) */
  EV_SACUDIDA, EV_OSCURO, EV_LUZ,                  /* desde tarea Sensores */
  EV_MPU_FALLA, EV_MPU_OK, EV_LDR_FALLA, EV_LDR_OK /* errores de sensores */
} evento_t;

/* ---------- Estados de la maquina de estados ---------- */
typedef enum { EST_HUEVO = 0, EST_NORMAL, EST_NECESIDAD, EST_DORMIDA, EST_ERROR } estado_t;

/* ---------- Caras que dibuja la Pantalla ---------- */
typedef enum {
  CARA_HUEVO = 0, CARA_FELIZ, CARA_NEUTRAL, CARA_TRISTE, CARA_DORMIDA, CARA_ENOJADA,
  CARA_ERROR, CARA_COMIENDO, CARA_EMOCIONADA, CARA_LIMPIA, CARA_NO, CARA_CARICIA
} cara_t;

/* ---------- Melodias (se envian a Sonido por notificacion de tarea) ---------- */
typedef enum {
  MEL_NINGUNA = 0, MEL_FELIZ, MEL_TRISTE, MEL_NO, MEL_DESPERTAR, MEL_COMER,
  MEL_JUGAR, MEL_LIMPIO, MEL_ENOJADO, MEL_ECLOSION, MEL_DORMIR, MEL_ALERTA, MEL_ERROR,
  MEL_TOTAL
} melodia_t;

/* ---------- Lo que Logica publica para la Pantalla (cola de 1 con xQueueOverwrite) ---------- */
typedef struct {
  uint8_t  estado;      /* estado_t */
  uint8_t  cara;        /* cara_t   */
  uint8_t  hambre, felicidad, energia, limpieza;   /* 0..100 */
  char     texto[22];   /* mensaje inferior */
  uint32_t t_inicio;    /* tick de inicio del estado (animaciones) */
  bool     ldr_falla;
} vista_t;

/* ---------- Recursos de FreeRTOS (creados en freertos.c) ---------- */
extern osMessageQueueId_t qEventosHandle;   /* eventos -> Logica (10 x uint8_t) */
extern osMessageQueueId_t qPantallaHandle;  /* vista_t -> Pantalla (longitud 1)   */
extern osMutexId_t        mutexI2CHandle;   /* protege el bus I2C1 (OLED + MPU)   */
extern osThreadId_t tSensoresHandle, tSonidoHandle, tLogicaHandle, tPantallaHandle, tLogHandle;

/* ---------- Evidencia de preemption: ultimo evento recibido en una ISR ---------- */
extern volatile uint32_t g_isrTick;
extern volatile uint8_t  g_isrEvento;
extern volatile uint32_t g_eventosPerdidos;

/* ---------- Funciones ---------- */
void log_printf(const char *fmt, ...);   /* freertos.c */
const char *evento_nombre(uint8_t ev);
const char *estado_nombre(uint8_t est);
void Sonido_Reproducir(melodia_t m);   /* no bloquea: notifica a la tarea Sonido */

void Sensores_Task(void);
void Sonido_Task(void);
void Logica_Task(void);
void Pantalla_Task(void);
void Pruebas_Task(uint8_t modo);


/* ======================= Drivers (chispa_drivers.c) ======================= */
#include "main.h"
/* --- OLED SSD1306 --- */
#define SSD1306_ADDR    (0x3C << 1)
#define SSD1306_W       128
#define SSD1306_H       64
#define SSD1306_PAGES   (SSD1306_H / 8)

HAL_StatusTypeDef ssd1306_Init(void);                 /* envia la secuencia de arranque */
HAL_StatusTypeDef ssd1306_UpdatePage(uint8_t page);   /* envia 1 pagina (128 bytes) del buffer */
void ssd1306_Clear(void);
void ssd1306_Pixel(int x, int y, bool on);
void ssd1306_HLine(int x0, int x1, int y);
void ssd1306_VLine(int x, int y0, int y1);
void ssd1306_Line(int x0, int y0, int x1, int y1);
void ssd1306_Rect(int x, int y, int w, int h, bool fill);
void ssd1306_Circle(int cx, int cy, int r, bool fill);
void ssd1306_Char(int x, int y, char c, uint8_t scale);
void ssd1306_Text(int x, int y, const char *s, uint8_t scale);

/* --- Bus I2C --- */
void i2c_recuperar_bus(void);   /* libera el bus si un esclavo quedo pegado */

/* --- MPU6050 --- */
#define MPU6050_ADDR  (0x68 << 1)   /* AD0 a GND */

HAL_StatusTypeDef mpu6050_Init(void);                               /* verifica WHO_AM_I y lo despierta */
HAL_StatusTypeDef mpu6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az); /* +-2g: 16384 LSB/g */

#endif
