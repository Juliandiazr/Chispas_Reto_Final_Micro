/* ==========================================================================
 *  chispa_app.c  -  Aplicacion de la mascota virtual CHISPA
 *  STM32F407VET6 + HAL + FreeRTOS (CMSIS-RTOS v2)
 *
 *  Este archivo reune toda la logica de la aplicacion, organizada por tareas:
 *    1. Entradas: ISR de pulsadores y tarea SENSORES (MPU6050 + LDR)
 *    2. Tarea SONIDO   (buzzer con PWM, activada por notificacion)
 *    3. Tarea LOGICA   (maquina de estados, estadisticas, LED RGB)
 *    4. Tarea PANTALLA (caras en la OLED, evidencia de preemption)
 *    5. Pruebas por modulo (MODO_PRUEBA en chispa.h)
 *
 *  La tarea LOG y la funcion log_printf() estan en freertos.c.
 *  Los drivers de la OLED y del MPU6050 estan en chispa_drivers.c.
 *
 *  Tareas, prioridades y comunicacion:
 *    tSensores  AboveNormal  periodica 20 ms   -> qEventos
 *    tSonido    AboveNormal  por notificacion  <- Logica (xTaskNotify)
 *    tLogica    Normal       por cola/250 ms   <- qEventos ; -> qPantalla, Sonido, RGB
 *    tPantalla  BelowNormal  periodica 100 ms  <- qPantalla ; usa mutexI2C
 *    tLog       Low          por cola          <- qLog (todas las tareas)
 * ========================================================================== */
#include "chispa.h"
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ==========================================================================
 *  1. ENTRADAS: ISR DE PULSADORES Y TAREA SENSORES
 *  Tarea periodica de 20 ms (vTaskDelayUntil), prioridad AboveNormal.
 *  - MPU6050 por I2C1 (con mutexI2C): detecta sacudidas.
 *  - LDR por ADC1 canal 1 (PA1): detecta oscuridad / luz.
 *  - Detecta sensores desconectados y reintenta cada 1 s.
 *  Los eventos se envian a la tarea Logica por la cola qEventos.
 * ========================================================================== */

/* ---------- Parametros de calibracion (ajustar viendo el log) ---------- */
#define SENS_PERIODO_MS          20
/* Umbral ADAPTATIVO: se compara con una referencia de la luz del ambiente,
 * asi funciona igual de dia (~3000) que de noche (~1800) o en otro salon. */
#define LDR_OSCURO_PCT      45     /* oscuro si la lectura < 45 % de la referencia  */
#define LDR_LUZ_PCT         65     /* luz otra vez si > 65 % (histeresis)           */
#define LDR_REF_MIN         200    /* referencia minima (cuarto casi a oscuras)     */
#define T_OSCURO_MS         1500   /* oscuridad sostenida para dormir              */
#define T_LUZ_MS            500    /* luz sostenida para despertar                 */
#define LDR_MIN_VALIDO      8      /* lectura pegada a 0 o a 4095 => LDR suelta     */
#define LDR_MAX_VALIDO      4092
#define T_LDR_FALLA_MS      3000
#define SACUDIDA_UMBRAL     70000  /* "energia" de movimiento para contar sacudida */
#define SACUDIDA_PAUSA_MS   1500   /* no repetir sacudidas antes de esto           */
#define REINTENTO_MPU_MS    1000

/* ---------- Variables para la evidencia de preemption ---------- */
volatile uint32_t g_isrTick = 0;
volatile uint8_t  g_isrEvento = EV_NINGUNO;
volatile uint32_t g_eventosPerdidos = 0;

static void enviar(uint8_t ev)
{
  if (osMessageQueuePut(qEventosHandle, &ev, 0, 0) != osOK) g_eventosPerdidos++;
}

/* ---------- ISR de los 4 pulsadores (EXTI0..3), con antirrebote de 200 ms ---------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t ultimo[4] = {0};
  uint8_t ev, i;
  switch (GPIO_Pin) {
    case BTN_COMER_Pin:   ev = EV_COMER;   i = 0; break;
    case BTN_JUGAR_Pin:   ev = EV_JUGAR;   i = 1; break;
    case BTN_LIMPIAR_Pin: ev = EV_LIMPIAR; i = 2; break;
    case BTN_CARICIA_Pin: ev = EV_DORMIR;  i = 3; break;   /* PC3: boton DORMIR / DESPERTAR */
    default: return;
  }
  uint32_t ahora = xTaskGetTickCountFromISR();
  if (ahora - ultimo[i] < pdMS_TO_TICKS(200)) return;   /* rebote: se ignora */
  ultimo[i] = ahora;
  g_isrTick = ahora;
  g_isrEvento = ev;
  /* osMessageQueuePut con timeout 0 es seguro desde ISR (usa xQueueSendFromISR
     y hace el cambio de contexto si despierta a una tarea de mayor prioridad) */
  if (osMessageQueuePut(qEventosHandle, &ev, 0, 0) != osOK) g_eventosPerdidos++;
}

static uint16_t leer_ldr(void)
{
  uint16_t v = 0;
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) v = (uint16_t)HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return v;
}


void Sensores_Task(void)
{
#if MODO_PRUEBA != 0
  vTaskSuspend(NULL);   /* en modo prueba solo corre la prueba elegida */
#endif
  TickType_t ultimo = xTaskGetTickCount();
  bool mpu_ok = false;
  uint32_t t_reintento = 0;
  int16_t ax, ay, az, pax = 0, pay = 0, paz = 0;
  uint32_t energia = 0, t_sacudida = 0;
  uint8_t fallas_mpu = 0;

  uint32_t ldr_prom = leer_ldr();
  uint32_t ldr_ref  = ldr_prom;      /* nivel de luz "normal" del ambiente */
  bool oscuro = false, ldr_falla = false;
  uint32_t t_cambio_luz = 0, t_ldr_rara = 0, t_log_ldr = 0;

  /* Chequeo inicial del MPU6050 */
  osMutexAcquire(mutexI2CHandle, osWaitForever);
  i2c_recuperar_bus();          /* por si el MPU quedo pegado antes del reset */
  mpu_ok = (mpu6050_Init() == HAL_OK);
  if (mpu_ok) mpu6050_ReadAccel(&pax, &pay, &paz);
  osMutexRelease(mutexI2CHandle);
  log_printf("SENSORES: MPU6050 %s\r\n", mpu_ok ? "OK" : "NO RESPONDE");
  if (!mpu_ok) enviar(EV_MPU_FALLA);

  for (;;)
  {
    uint32_t ahora = xTaskGetTickCount();

    /* ===== MPU6050: sacudidas ===== */
    if (mpu_ok) {
      osMutexAcquire(mutexI2CHandle, osWaitForever);
      HAL_StatusTypeDef r = mpu6050_ReadAccel(&ax, &ay, &az);
      osMutexRelease(mutexI2CHandle);
      if (r != HAL_OK) {
        /* una falla aislada (ruido) se tolera: se libera el bus y se reintenta */
        fallas_mpu++;
        log_printf("SENSORES: error I2C leyendo MPU6050 (%u)\r\n", fallas_mpu);
        osMutexAcquire(mutexI2CHandle, osWaitForever);
        i2c_recuperar_bus();
        osMutexRelease(mutexI2CHandle);
        if (fallas_mpu >= 3) {
          mpu_ok = false;
          fallas_mpu = 0;
          t_reintento = ahora;
          enviar(EV_MPU_FALLA);
        }
      } else {
        fallas_mpu = 0;
        /* cambio de aceleracion entre muestras -> integrador con fuga */
        uint32_t delta = abs(ax - pax) + abs(ay - pay) + abs(az - paz);
        pax = ax; pay = ay; paz = az;
        energia = energia - energia / 8 + delta;
        if (energia > SACUDIDA_UMBRAL && (ahora - t_sacudida) > SACUDIDA_PAUSA_MS) {
          t_sacudida = ahora;
          log_printf("SENSORES: sacudida (energia=%lu)\r\n", (unsigned long)energia);
          enviar(EV_SACUDIDA);
          energia = 0;
        }
      }
    } else if (ahora - t_reintento >= REINTENTO_MPU_MS) {
      /* Reintento automatico cada 1 s */
      t_reintento = ahora;
      osMutexAcquire(mutexI2CHandle, osWaitForever);
      i2c_recuperar_bus();
      mpu_ok = (mpu6050_Init() == HAL_OK);
      if (mpu_ok) mpu6050_ReadAccel(&pax, &pay, &paz);
      osMutexRelease(mutexI2CHandle);
      if (mpu_ok) { log_printf("SENSORES: MPU6050 reconectado\r\n"); energia = 0; enviar(EV_MPU_OK); }
    }

    /* ===== LDR: dia / noche ===== */
    uint16_t raw = leer_ldr();
    ldr_prom = (ldr_prom * 7 + raw) / 8;               /* filtro pasa-bajos */

    if (ldr_prom < LDR_MIN_VALIDO || ldr_prom > LDR_MAX_VALIDO) {
      if (t_ldr_rara == 0) t_ldr_rara = ahora;
      if (!ldr_falla && ahora - t_ldr_rara > T_LDR_FALLA_MS) {
        ldr_falla = true;
        log_printf("SENSORES: LDR fuera de rango (%lu)\r\n", (unsigned long)ldr_prom);
        enviar(EV_LDR_FALLA);
      }
    } else {
      t_ldr_rara = 0;
      if (ldr_falla) { ldr_falla = false; log_printf("SENSORES: LDR OK de nuevo\r\n"); enviar(EV_LDR_OK); }
    }

    /* Referencia de luz ambiente: sube rapido si hay mas luz; baja muy lento
       (~10 s) y solo si no esta tapada, para seguir los cambios graduales del dia */
    if (ldr_prom > ldr_ref) ldr_ref = ldr_prom;
    else if (!oscuro && ldr_prom * 100 > ldr_ref * LDR_LUZ_PCT) ldr_ref -= (ldr_ref - ldr_prom) / 512;
    if (ldr_ref < LDR_REF_MIN) ldr_ref = LDR_REF_MIN;
    uint32_t umbral_oscuro = ldr_ref * LDR_OSCURO_PCT / 100;
    uint32_t umbral_luz    = ldr_ref * LDR_LUZ_PCT / 100;

    if (!ldr_falla) {
      bool quiere_oscuro = oscuro ? (ldr_prom < umbral_luz) : (ldr_prom < umbral_oscuro);
      if (quiere_oscuro != oscuro) {
        if (t_cambio_luz == 0) t_cambio_luz = ahora;
        uint32_t espera = quiere_oscuro ? T_OSCURO_MS : T_LUZ_MS;
        if (ahora - t_cambio_luz >= espera) {
          oscuro = quiere_oscuro;
          t_cambio_luz = 0;
          log_printf("SENSORES: %s (LDR=%lu)\r\n", oscuro ? "oscuro" : "luz", (unsigned long)ldr_prom);
          enviar(oscuro ? EV_OSCURO : EV_LUZ);
        }
      } else {
        t_cambio_luz = 0;
      }
    }

    /* Valor de la LDR cada 5 s, para calibrar los umbrales */
    if (ahora - t_log_ldr >= 5000) {
      t_log_ldr = ahora;
      log_printf("SENSORES: LDR=%lu (ref=%lu, oscuro<%lu)\r\n", (unsigned long)ldr_prom,
                 (unsigned long)ldr_ref, (unsigned long)umbral_oscuro);
    }

    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(SENS_PERIODO_MS));
  }
}

/* ==========================================================================
 *  2. TAREA SONIDO: melodias en el buzzer (TIM3 CH3, PB0)
 *  Activada por NOTIFICACION DE TAREA: Logica envia el numero de melodia con
 *  xTaskNotify(..., eSetValueWithOverwrite). Mientras suena cada nota, la tarea
 *  espera otra notificacion con timeout = duracion de la nota; si llega una nueva
 *  melodia, interrumpe la actual. Nunca usa HAL_Delay ni espera activa.
 *  TIM3: 84 MHz / (PSC 83 + 1) = 1 MHz.  Nota de f Hz: ARR = 1e6/f - 1, CCR = ARR/2.
 * ========================================================================== */

typedef struct { uint16_t f; uint16_t ms; } nota_t;   /* f = 0 -> silencio; ms = 0 -> fin */

/* Notas (Hz) */
#define C5 523
#define D5 587
#define E5 659
#define F5 698
#define G5 784
#define A5 880
#define B5 988
#define C6 1047
#define E6 1319
#define G6 1568
#define A4 440
#define G4 392
#define E4 330
#define C4 262

static const nota_t m_feliz[]     = {{C6,90},{E6,90},{G6,140},{0,0}};
static const nota_t m_triste[]    = {{G5,220},{E5,220},{C5,400},{0,0}};
static const nota_t m_no[]        = {{G4,120},{0,60},{C4,250},{0,0}};
static const nota_t m_despertar[] = {{C5,100},{E5,100},{G5,100},{C6,200},{0,0}};
static const nota_t m_comer[]     = {{E5,70},{0,40},{E5,70},{0,40},{E5,70},{0,40},{G5,150},{0,0}};
static const nota_t m_jugar[]     = {{C5,80},{E5,80},{G5,80},{C6,80},{G5,80},{C6,200},{0,0}};
static const nota_t m_limpio[]    = {{G6,60},{E6,60},{C6,60},{G6,60},{E6,60},{C6,150},{0,0}};
static const nota_t m_enojado[]   = {{A4,90},{0,30},{A4,90},{0,30},{E4,300},{0,0}};
static const nota_t m_eclosion[]  = {{C5,120},{D5,120},{E5,120},{F5,120},{G5,120},{A5,120},{B5,120},{C6,300},{0,0}};
static const nota_t m_dormir[]    = {{G5,200},{E5,200},{C5,200},{G4,400},{0,0}};
static const nota_t m_alerta[]    = {{A5,100},{0,80},{A5,100},{0,0}};
static const nota_t m_error[]     = {{C4,300},{0,100},{C4,300},{0,0}};

static const nota_t *const melodias[MEL_TOTAL] = {
  [MEL_NINGUNA] = 0, [MEL_FELIZ] = m_feliz, [MEL_TRISTE] = m_triste, [MEL_NO] = m_no,
  [MEL_DESPERTAR] = m_despertar, [MEL_COMER] = m_comer, [MEL_JUGAR] = m_jugar,
  [MEL_LIMPIO] = m_limpio, [MEL_ENOJADO] = m_enojado, [MEL_ECLOSION] = m_eclosion,
  [MEL_DORMIR] = m_dormir, [MEL_ALERTA] = m_alerta, [MEL_ERROR] = m_error
};
static const char *const nombres[MEL_TOTAL] = {
  "-", "FELIZ", "TRISTE", "NO", "DESPERTAR", "COMER", "JUGAR", "LIMPIO",
  "ENOJADO", "ECLOSION", "DORMIR", "ALERTA", "ERROR"
};

static void tono(uint16_t f)
{
  if (f == 0) { __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); return; }
  uint32_t arr = 1000000u / f - 1u;
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, arr / 2u);
  htim3.Instance->EGR = TIM_EGR_UG;   /* aplica ARR/CCR de inmediato */
}

void Sonido_Reproducir(melodia_t m)
{
  if (tSonidoHandle != NULL && m > MEL_NINGUNA && m < MEL_TOTAL)
    xTaskNotify((TaskHandle_t)tSonidoHandle, (uint32_t)m, eSetValueWithOverwrite);
}

void Sonido_Task(void)
{
#if MODO_PRUEBA != 0
  vTaskSuspend(NULL);   /* en modo prueba solo corre la prueba elegida */
#endif
  uint32_t pedida = 0;
  tono(0);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  for (;;)
  {
    /* Bloqueada (sin consumir CPU) hasta que Logica pida una melodia */
    if (pedida == 0) xTaskNotifyWait(0, 0xFFFFFFFFu, &pedida, portMAX_DELAY);

    uint32_t m = pedida;
    pedida = 0;
    if (m == 0 || m >= MEL_TOTAL || melodias[m] == 0) continue;
    log_printf("SONIDO: melodia %s\r\n", nombres[m]);

    for (const nota_t *n = melodias[m]; n->ms != 0; n++) {
      tono(n->f);
      /* espera la duracion de la nota; si llega otra melodia, se interrumpe */
      if (xTaskNotifyWait(0, 0xFFFFFFFFu, &pedida, pdMS_TO_TICKS(n->ms)) == pdTRUE) break;
    }
    tono(0);
  }
}

/* ==========================================================================
 *  3. TAREA LOGICA: maquina de estados, estadisticas y LED RGB
 *  Bloqueada en la cola qEventos (hasta 250 ms). Prioridad Normal.
 *  - Recibe eventos de la ISR de botones y de la tarea Sensores.
 *  - Actualiza las 4 estadisticas (0..100) y el estado.
 *  - Pide melodias a Sonido (notificacion) y publica la vista para Pantalla
 *    (cola de longitud 1 con xQueueOverwrite).
 *  - LED RGB: TIM4 CH1/CH2/CH3 = PD12 rojo / PD13 verde / PD14 azul.
 * ========================================================================== */

#define T_HUEVO_MS       4000
#define T_REACCION_MS    2000
#define UMBRAL_NECESIDAD 30
#define T_ALERTA_MS      20000
#define T_STACKS_MS      30000

static int16_t hambre = 80, felicidad = 80, energia = 80, limpieza = 80;
static estado_t estado = EST_HUEVO;
static uint32_t t_estado = 0;
static cara_t   cara_reaccion = CARA_FELIZ;
static uint32_t t_reaccion_fin = 0;
static bool mpu_falla = false, ldr_falla = false;

const char *evento_nombre(uint8_t ev)
{
  static const char *const n[] = { "-", "COMER", "JUGAR", "LIMPIAR", "DORMIR", "SACUDIDA",
    "OSCURO", "LUZ", "MPU_FALLA", "MPU_OK", "LDR_FALLA", "LDR_OK" };
  return ev < sizeof(n) / sizeof(n[0]) ? n[ev] : "?";
}

const char *estado_nombre(uint8_t est)
{
  static const char *const n[] = { "HUEVO", "NORMAL", "NECESIDAD", "DORMIDA", "ERROR" };
  return est < 5 ? n[est] : "?";
}

static int16_t lim(int16_t v) { return v < 0 ? 0 : (v > 100 ? 100 : v); }

/* ---------- LED RGB (catodo comun): 0..255 por color ---------- */
static void rgb(uint8_t r, uint8_t g, uint8_t b)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)r * 999u / 255u);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (uint32_t)g * 999u / 255u);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint32_t)b * 999u / 255u);
}

static void reaccion(cara_t c, melodia_t m, uint32_t ahora)
{
  cara_reaccion = c;
  t_reaccion_fin = ahora + T_REACCION_MS;
  Sonido_Reproducir(m);
}

static void cambiar_estado(estado_t nuevo, uint32_t ahora)
{
  if (nuevo == estado) return;
  log_printf("LOGICA: estado %s -> %s\r\n", estado_nombre(estado), estado_nombre(nuevo));
  estado = nuevo;
  t_estado = ahora;
}

/* Estado "de base" segun estadisticas (sin contar HUEVO/DORMIDA/ERROR) */
static estado_t estado_por_stats(void)
{
  if (hambre < UMBRAL_NECESIDAD || felicidad < UMBRAL_NECESIDAD ||
      limpieza < UMBRAL_NECESIDAD || energia < UMBRAL_NECESIDAD) return EST_NECESIDAD;
  return EST_NORMAL;
}

static const char *motivo_necesidad(void)
{
  int16_t m = hambre; const char *t = "TENGO HAMBRE";
  if (felicidad < m) { m = felicidad; t = "ESTOY ABURRIDA"; }
  if (limpieza  < m) { m = limpieza;  t = "ESTOY SUCIA"; }
  if (energia   < m) { m = energia;   t = "TENGO SUENO"; }
  return t;
}

/* ---------- Procesa un evento ---------- */
static void procesar(uint8_t ev, uint32_t ahora)
{
  int16_t antes;

  /* Errores de sensores: siempre se atienden */
  switch (ev) {
    case EV_MPU_FALLA: mpu_falla = true;  if (estado != EST_HUEVO) { cambiar_estado(EST_ERROR, ahora); Sonido_Reproducir(MEL_ERROR); } return;
    case EV_MPU_OK:    mpu_falla = false; if (estado == EST_ERROR) { cambiar_estado(estado_por_stats(), ahora); Sonido_Reproducir(MEL_DESPERTAR); } return;
    case EV_LDR_FALLA: ldr_falla = true;  if (estado == EST_DORMIDA) cambiar_estado(estado_por_stats(), ahora); return;
    case EV_LDR_OK:    ldr_falla = false; return;
    default: break;
  }

  if (estado == EST_HUEVO || estado == EST_ERROR) {
    log_printf("LOGICA: %s ignorado en %s\r\n", evento_nombre(ev), estado_nombre(estado));
    if (estado == EST_ERROR) Sonido_Reproducir(MEL_NO);
    return;
  }

  if (estado == EST_DORMIDA) {
    switch (ev) {
      case EV_LUZ:
      case EV_DORMIR:   /* luz o boton PC3: se despierta */
        log_printf("LOGICA: %s -> despierta\r\n", evento_nombre(ev));
        cambiar_estado(estado_por_stats(), ahora);
        reaccion(CARA_FELIZ, MEL_DESPERTAR, ahora);
        break;
      case EV_SACUDIDA:
        antes = felicidad; felicidad = lim(felicidad - 10);
        log_printf("LOGICA: SACUDIDA dormida! felicidad %d->%d\r\n", antes, felicidad);
        cambiar_estado(estado_por_stats(), ahora);
        reaccion(CARA_ENOJADA, MEL_ENOJADO, ahora);
        break;
      default:
        log_printf("LOGICA: %s rechazado (dormida)\r\n", evento_nombre(ev));
        Sonido_Reproducir(MEL_NO);
        break;
    }
    return;
  }

  /* ---- Despierta: NORMAL o NECESIDAD ---- */
  switch (ev) {
    case EV_COMER:
      if (hambre >= 95) {
        log_printf("LOGICA: COMER rechazado, llena (%d)\r\n", hambre);
        reaccion(CARA_NO, MEL_NO, ahora);
      } else {
        antes = hambre; hambre = lim(hambre + 20);
        log_printf("LOGICA: COMER (ISR t=%lu) hambre %d->%d\r\n", (unsigned long)g_isrTick, antes, hambre);
        reaccion(CARA_COMIENDO, MEL_COMER, ahora);
      }
      break;
    case EV_JUGAR:
      if (energia < 15) {
        log_printf("LOGICA: JUGAR rechazado, cansada (%d)\r\n", energia);
        reaccion(CARA_NO, MEL_NO, ahora);
      } else {
        antes = felicidad; felicidad = lim(felicidad + 15); energia = lim(energia - 10);
        log_printf("LOGICA: JUGAR (ISR t=%lu) felic %d->%d\r\n", (unsigned long)g_isrTick, antes, felicidad);
        reaccion(CARA_EMOCIONADA, MEL_JUGAR, ahora);
      }
      break;
    case EV_LIMPIAR:
      antes = limpieza; limpieza = 100;
      log_printf("LOGICA: LIMPIAR (ISR t=%lu) limp %d->100\r\n", (unsigned long)g_isrTick, antes);
      reaccion(CARA_LIMPIA, MEL_LIMPIO, ahora);
      break;
    case EV_DORMIR:
      log_printf("LOGICA: DORMIR (ISR t=%lu) energia=%d\r\n", (unsigned long)g_isrTick, energia);
      cambiar_estado(EST_DORMIDA, ahora);
      Sonido_Reproducir(MEL_DORMIR);
      t_reaccion_fin = ahora;   /* sin reaccion: cara dormida */
      break;
    case EV_SACUDIDA:
      antes = felicidad; felicidad = lim(felicidad - 10);
      log_printf("LOGICA: SACUDIDA felicidad %d->%d\r\n", antes, felicidad);
      reaccion(CARA_ENOJADA, MEL_ENOJADO, ahora);
      break;
    case EV_OSCURO:
      if (!ldr_falla) {
        cambiar_estado(EST_DORMIDA, ahora);
        Sonido_Reproducir(MEL_DORMIR);
        t_reaccion_fin = ahora;   /* sin reaccion: cara dormida */
      }
      break;
    default: break;
  }
}

/* ---------- Paso del tiempo: 1 hora de Chispa = 10 s ---------- */
static void pasa_una_hora(void)
{
  if (estado == EST_HUEVO || estado == EST_ERROR) return;
  if (estado == EST_DORMIDA) {
    hambre = lim(hambre - 2); felicidad = lim(felicidad - 1);
    limpieza = lim(limpieza - 2); energia = lim(energia + 12);
  } else {
    hambre = lim(hambre - 4); felicidad = lim(felicidad - 3);
    limpieza = lim(limpieza - 3); energia = lim(energia - 4);
  }
  log_printf("LOGICA: hora H=%d F=%d E=%d L=%d %s\r\n", hambre, felicidad, energia, limpieza, estado_nombre(estado));
}

static void reporte_stacks(void)
{
  log_printf("STACK libre(pal): sen=%lu son=%lu log=%lu\r\n",
    (unsigned long)uxTaskGetStackHighWaterMark((TaskHandle_t)tSensoresHandle),
    (unsigned long)uxTaskGetStackHighWaterMark((TaskHandle_t)tSonidoHandle),
    (unsigned long)uxTaskGetStackHighWaterMark(NULL));
  log_printf("STACK libre(pal): pan=%lu tlog=%lu heap=%u\r\n",
    (unsigned long)uxTaskGetStackHighWaterMark((TaskHandle_t)tPantallaHandle),
    (unsigned long)uxTaskGetStackHighWaterMark((TaskHandle_t)tLogHandle),
    (unsigned)xPortGetFreeHeapSize());
}

/* ---------- Construye la vista y el color del LED ---------- */
static void publicar(uint32_t ahora)
{
  vista_t v;
  memset(&v, 0, sizeof(v));
  v.estado = estado; v.t_inicio = t_estado; v.ldr_falla = ldr_falla;
  v.hambre = hambre; v.felicidad = felicidad; v.energia = energia; v.limpieza = limpieza;

  bool parpadeo = ((ahora / 500) & 1) != 0;
  bool hay_reaccion = (int32_t)(t_reaccion_fin - ahora) > 0;

  switch (estado) {
    case EST_HUEVO:
      v.cara = CARA_HUEVO; strcpy(v.texto, "...");
      rgb(40, 40, 40);
      break;
    case EST_ERROR:
      v.cara = CARA_ERROR; strcpy(v.texto, "ERROR SENSOR");
      rgb(parpadeo ? 255 : 0, 0, 0);
      break;
    case EST_DORMIDA:
      v.cara = hay_reaccion ? cara_reaccion : CARA_DORMIDA; strcpy(v.texto, "ZZZ...");
      rgb(0, 0, 25);
      break;
    case EST_NECESIDAD:
      v.cara = hay_reaccion ? cara_reaccion : CARA_TRISTE; strcpy(v.texto, motivo_necesidad());
      rgb(parpadeo ? 200 : 40, parpadeo ? 120 : 20, 0);   /* amarillo parpadeando */
      break;
    default: /* EST_NORMAL */
      v.cara = hay_reaccion ? cara_reaccion : (felicidad >= 60 ? CARA_FELIZ : CARA_NEUTRAL);
      strcpy(v.texto, felicidad >= 60 ? "FELIZ" : "NORMAL");
      rgb(0, 120, 0);
      break;
  }

  /* El color de una reaccion manda sobre el del estado */
  if (hay_reaccion) {
    switch (cara_reaccion) {
      case CARA_EMOCIONADA:                   rgb(0, 255, 0);   strcpy(v.texto, "A JUGAR!"); break;
      case CARA_CARICIA:                      rgb(0, 255, 0);   strcpy(v.texto, "HOLA!"); break;
      case CARA_COMIENDO:                      rgb(255, 80, 0);  strcpy(v.texto, "NAM NAM"); break;
      case CARA_LIMPIA:                        rgb(0, 150, 255); strcpy(v.texto, "LIMPIA!"); break;
      case CARA_ENOJADA:                       rgb(255, 0, 0);   strcpy(v.texto, "GRR!"); break;
      case CARA_NO:                            rgb(255, 0, 60);  strcpy(v.texto, "NO!"); break;
      default: break;
    }
  }

  xQueueOverwrite((QueueHandle_t)qPantallaHandle, &v);   /* Pantalla solo quiere el dato mas reciente */
}

void Logica_Task(void)
{
#if MODO_PRUEBA != 0
  Pruebas_Task(MODO_PRUEBA);   /* no retorna */
#endif
  uint8_t ev;
  uint32_t ahora = xTaskGetTickCount();
  uint32_t t_hora = ahora, t_alerta = ahora, t_stacks = ahora;
  uint32_t perdidos = 0;

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  t_estado = ahora;
  log_printf("LOGICA: estado HUEVO (chequeo de sensores)\r\n");

  for (;;)
  {
    /* Bloqueada hasta que llegue un evento o pasen 250 ms (para animar y llevar el tiempo) */
    if (osMessageQueueGet(qEventosHandle, &ev, NULL, pdMS_TO_TICKS(250)) == osOK)
      procesar(ev, xTaskGetTickCount());

    ahora = xTaskGetTickCount();

    /* Eclosion del huevo */
    if (estado == EST_HUEVO && ahora - t_estado >= T_HUEVO_MS) {
      if (mpu_falla) { cambiar_estado(EST_ERROR, ahora); Sonido_Reproducir(MEL_ERROR); }
      else { cambiar_estado(EST_NORMAL, ahora); reaccion(CARA_CARICIA, MEL_ECLOSION, ahora); }   /* "HOLA!" */
      t_hora = ahora;
    }

    if (ahora - t_hora >= HORA_MS) { t_hora += HORA_MS; pasa_una_hora(); }

    /* NORMAL <-> NECESIDAD segun las estadisticas */
    if (estado == EST_NORMAL || estado == EST_NECESIDAD) {
      estado_t e = estado_por_stats();
      if (e != estado) {
        cambiar_estado(e, ahora);
        if (e == EST_NECESIDAD) { Sonido_Reproducir(MEL_TRISTE); t_alerta = ahora; }
      }
      if (estado == EST_NECESIDAD && ahora - t_alerta >= T_ALERTA_MS) {
        t_alerta = ahora; Sonido_Reproducir(MEL_ALERTA);
      }
    }

    if (g_eventosPerdidos != perdidos) {
      perdidos = g_eventosPerdidos;
      log_printf("LOGICA: cola llena, eventos perdidos=%lu\r\n", (unsigned long)perdidos);
    }

    if (ahora - t_stacks >= T_STACKS_MS) { t_stacks = ahora; reporte_stacks(); }

    publicar(ahora);
  }
}

/* ==========================================================================
 *  4. TAREA PANTALLA: caras y barras en la OLED SSD1306
 *  Periodica de 100 ms (10 fps), prioridad BelowNormal. Lee la vista mas reciente
 *  de qPantalla. El frame se envia pagina por pagina (8 x 128 bytes), tomando y
 *  soltando mutexI2C en cada pagina: Sensores nunca espera el bus mas de ~3 ms.
 *  EVIDENCIA DE PREEMPTION: si una ISR de boton ocurre mientras se envia un frame,
 *  Logica y Sonido le quitan la CPU a Pantalla; al terminar el frame se registra
 *  en el log el intervalo del frame y el instante de la ISR.
 * ========================================================================== */

#define PANT_PERIODO_MS 100

static vista_t vista;      /* ultima vista recibida de Logica */
static uint32_t frame = 0;

/* ---------- Primitivas de la cara ---------- */
static void ojos_abiertos(void)  { ssd1306_Circle(44, 30, 7, true); ssd1306_Circle(84, 30, 7, true); }
static void ojos_cerrados(void)  { ssd1306_HLine(37, 51, 31); ssd1306_HLine(77, 91, 31); }
static void ojos_felices(void)   /* ^ ^ */
{
  ssd1306_Line(37, 33, 44, 25); ssd1306_Line(44, 25, 51, 33);
  ssd1306_Line(77, 33, 84, 25); ssd1306_Line(84, 25, 91, 33);
}
static void ojos_x(void)
{
  ssd1306_Line(38, 24, 50, 36); ssd1306_Line(50, 24, 38, 36);
  ssd1306_Line(78, 24, 90, 36); ssd1306_Line(90, 24, 78, 36);
}
static void ojos_con_parpadeo(void) { if (frame % 40 < 2) ojos_cerrados(); else ojos_abiertos(); }

/* parabola: sonrisa (curva hacia abajo) o tristeza (curva hacia arriba) */
static void boca_curva(int ancho, int alto, bool sonrisa, int y0)
{
  for (int dx = -ancho; dx <= ancho; dx++) {
    int d = (alto * (ancho * ancho - dx * dx)) / (ancho * ancho);
    int y = sonrisa ? y0 + d : y0 + alto - d;
    ssd1306_Pixel(64 + dx, y, true); ssd1306_Pixel(64 + dx, y + 1, true);
  }
}

static void elipse(int cx, int cy, int rx, int ry)
{
  for (int y = -ry; y <= ry; y++)
    for (int x = -rx; x <= rx; x++) {
      long e = (long)x * x * ry * ry + (long)y * y * rx * rx;
      long r2 = (long)rx * rx * ry * ry;
      if (e <= r2 && e >= r2 - 2L * rx * ry * (rx > ry ? rx : ry)) ssd1306_Pixel(cx + x, cy + y, true);
    }
}

static void barra(int x, char letra, uint8_t valor)
{
  ssd1306_Char(x, 0, letra, 1);
  ssd1306_Rect(x + 7, 0, 22, 7, false);
  int w = (valor * 20) / 100;
  if (w > 0) ssd1306_Rect(x + 8, 1, w, 5, true);
}

static void destellos(void)
{
  static const int8_t p[6][2] = {{20,18},{106,16},{18,46},{110,44},{30,10},{98,50}};
  for (int i = 0; i < 6; i++) {
    if (((frame / 3) + i) % 2) {
      int x = p[i][0], y = p[i][1];
      ssd1306_HLine(x - 3, x + 3, y); ssd1306_VLine(x, y - 3, y + 3);
    }
  }
}

static void dibujar(void)
{
  ssd1306_Clear();

  if (vista.cara != CARA_HUEVO) {
    barra(0, 'H', vista.hambre); barra(32, 'F', vista.felicidad);
    barra(64, 'E', vista.energia); barra(96, 'L', vista.limpieza);
  }

  switch (vista.cara) {
    case CARA_HUEVO: {
      uint32_t t = xTaskGetTickCount() - vista.t_inicio;
      int bamboleo = (frame % 4 < 2) ? -1 : 1;
      elipse(64 + bamboleo, 32, 16, 22);
      if (t > 1500) { ssd1306_Line(52, 30, 58, 35); ssd1306_Line(58, 35, 63, 29); }
      if (t > 2500) { ssd1306_Line(63, 29, 69, 36); ssd1306_Line(69, 36, 76, 30); }
      if (t > 3300) destellos();
      break;
    }
    case CARA_FELIZ:      ojos_con_parpadeo(); boca_curva(14, 7, true, 42); break;
    case CARA_NEUTRAL:    ojos_con_parpadeo(); ssd1306_HLine(54, 74, 46); ssd1306_HLine(54, 74, 47); break;
    case CARA_TRISTE:     ojos_con_parpadeo(); boca_curva(12, 6, false, 42);
                          if (frame % 20 < 14) ssd1306_Circle(92, 40 + (frame % 20) / 3, 2, true);
                          break;
    case CARA_DORMIDA:    ojos_cerrados(); ssd1306_Circle(64, 46, 3, false);
                          ssd1306_Text(100, 12 - (int)(frame % 10) / 3, "Z", 1);
                          if (frame % 20 > 6) ssd1306_Text(110, 6, "Z", 1);
                          break;
    case CARA_ENOJADA:    ojos_abiertos();
                          ssd1306_Line(34, 18, 52, 23); ssd1306_Line(34, 19, 52, 24);
                          ssd1306_Line(94, 18, 76, 23); ssd1306_Line(94, 19, 76, 24);
                          boca_curva(12, 6, false, 42); break;
    case CARA_ERROR:      ojos_x(); ssd1306_HLine(54, 74, 46); break;
    case CARA_COMIENDO:   ojos_felices(); ssd1306_Circle(64, 45, (frame % 4 < 2) ? 3 : 6, true); break;
    case CARA_EMOCIONADA: ojos_felices(); boca_curva(16, 9, true, 40); ssd1306_HLine(48, 80, 40); destellos(); break;
    case CARA_CARICIA:    ojos_felices(); boca_curva(14, 7, true, 42);
                          ssd1306_Circle(28, 42, 3, false); ssd1306_Circle(100, 42, 3, false); break;
    case CARA_LIMPIA:     ojos_con_parpadeo(); boca_curva(14, 7, true, 42); destellos(); break;
    case CARA_NO:         ojos_abiertos(); ssd1306_HLine(54, 74, 46); ssd1306_HLine(54, 74, 47); break;
    default: break;
  }

  /* Texto inferior centrado */
  int len = (int)strlen(vista.texto);
  ssd1306_Text(64 - len * 3, 56, vista.texto, 1);
  if (vista.ldr_falla) ssd1306_Text(0, 56, "LDR!", 1);
}

/* Diagnostico: busca dispositivos en el bus I2C1 y los reporta por el log.
 * Esperados: 0x3C = OLED SSD1306, 0x68 = MPU6050. */
static void escanear_i2c(void)
{
  int encontrados = 0;
  osMutexAcquire(mutexI2CHandle, osWaitForever);
  for (uint8_t dir = 0x08; dir < 0x78; dir++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(dir << 1), 2, 5) == HAL_OK) {
      log_printf("I2C: dispositivo en 0x%02X %s\r\n", dir,
                 dir == 0x3C || dir == 0x3D ? "(OLED)" : dir == 0x68 || dir == 0x69 ? "(MPU6050)" : "");
      encontrados++;
    }
  }
  osMutexRelease(mutexI2CHandle);
  if (encontrados == 0) log_printf("I2C: NINGUN dispositivo (revisar SCL=PB6, SDA=PB7, VCC, GND)\r\n");
}

void Pantalla_Task(void)
{
#if MODO_PRUEBA != 0
  vTaskSuspend(NULL);   /* en modo prueba solo corre la prueba elegida */
#endif
  TickType_t ultimo;
  bool oled_ok = false;
  uint32_t t_reintento = 0, isr_reportada = 0;

  memset(&vista, 0, sizeof(vista));
  strcpy(vista.texto, "...");

  escanear_i2c();
  ultimo = xTaskGetTickCount();
  for (;;)
  {
    uint32_t ahora = xTaskGetTickCount();

    /* Inicializa / reintenta la OLED cada 2 s si no responde */
    if (!oled_ok && ahora - t_reintento >= 2000) {
      t_reintento = ahora;
      osMutexAcquire(mutexI2CHandle, osWaitForever);
      i2c_recuperar_bus();
      oled_ok = (ssd1306_Init() == HAL_OK);
      osMutexRelease(mutexI2CHandle);
      log_printf("PANTALLA: OLED %s\r\n", oled_ok ? "OK" : "NO RESPONDE");
    }

    /* Toma la vista mas reciente sin bloquearse (si no hay nueva, usa la anterior) */
    osMessageQueueGet(qPantallaHandle, &vista, NULL, 0);

    dibujar();
    frame++;

    if (oled_ok) {
      uint32_t t0 = xTaskGetTickCount();
      for (uint8_t p = 0; p < SSD1306_PAGES && oled_ok; p++) {
        osMutexAcquire(mutexI2CHandle, osWaitForever);
        if (ssd1306_UpdatePage(p) != HAL_OK) oled_ok = false;
        osMutexRelease(mutexI2CHandle);
      }
      uint32_t t1 = xTaskGetTickCount();
      if (!oled_ok) { log_printf("PANTALLA: error I2C, reintentando\r\n"); t_reintento = t1; }

      /* Evidencia de preemption: una ISR ocurrio mientras se enviaba este frame */
      uint32_t ti = g_isrTick;
      if (ti != isr_reportada && ti >= t0 && ti <= t1) {
        isr_reportada = ti;
        log_printf("PANTALLA: frame %lu..%lu preemptado (ISR %s t=%lu)\r\n",
                   (unsigned long)t0, (unsigned long)t1, evento_nombre(g_isrEvento), (unsigned long)ti);
      }
    }

    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(PANT_PERIODO_MS));
  }
}

/* ==========================================================================
 *  5. PRUEBAS POR MODULO (se elige con MODO_PRUEBA en chispa.h)
 *  Cada prueba usa el periferico solo y escribe los resultados por el log UART.
 *  Se ejecutan dentro de la tarea Logica; Sensores, Sonido y Pantalla quedan suspendidas.
 * ========================================================================== */

/* ---------- 1: OLED ---------- */
static void prueba_oled(void)
{
  uint32_t seg = 0;
  bool ok = false;
  for (;;) {
    if (!ok) {
      escanear_i2c();
      ok = (ssd1306_Init() == HAL_OK);
      log_printf("PRUEBA OLED: init %s\r\n", ok ? "OK" : "FALLO");
      if (!ok) { i2c_recuperar_bus(); vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
    }
    char t[16];
    ssd1306_Clear();
    ssd1306_Rect(0, 0, 128, 64, false);
    ssd1306_Text(10, 6, "PRUEBA OLED", 1);
    ssd1306_Text(10, 20, "CHISPA", 2);
    snprintf(t, sizeof(t), "SEG: %lu", (unsigned long)seg);
    ssd1306_Text(10, 44, t, 1);
    ssd1306_Circle(108, 44, 6 + (seg % 2) * 3, true);
    for (uint8_t p = 0; p < SSD1306_PAGES && ok; p++)
      if (ssd1306_UpdatePage(p) != HAL_OK) ok = false;
    log_printf("PRUEBA OLED: frame %lu %s\r\n", (unsigned long)seg, ok ? "enviado" : "ERROR I2C");
    seg++;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/* ---------- 2: Pulsadores (la ISR pone los eventos en qEventos) ---------- */
static void prueba_botones(void)
{
  uint8_t ev;
  log_printf("PRUEBA BOTONES: presione cada boton (PC0..PC3)\r\n");
  for (;;) {
    if (osMessageQueueGet(qEventosHandle, &ev, NULL, pdMS_TO_TICKS(3000)) == osOK)
      log_printf("PRUEBA BOTONES: %s (ISR t=%lu)\r\n", evento_nombre(ev), (unsigned long)g_isrTick);
    else
      log_printf("PRUEBA BOTONES: PC0=%d PC1=%d PC2=%d PC3=%d (1=suelto, 0=presionado)\r\n",
        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0), HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1),
        HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2), HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3));
  }
}

/* ---------- 3: LDR ---------- */
static void prueba_ldr(void)
{
  log_printf("PRUEBA LDR: tape y destape la LDR\r\n");
  for (;;) {
    uint32_t v = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) v = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    log_printf("PRUEBA LDR: ADC=%4lu  (%lu mV)\r\n", (unsigned long)v, (unsigned long)(v * 3300u / 4095u));
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

/* ---------- 4: Buzzer ---------- */
static void prueba_buzzer(void)
{
  static const uint16_t esc[] = {523, 587, 659, 698, 784, 880, 988, 1047};
  static const char *const nom[] = {"DO", "RE", "MI", "FA", "SOL", "LA", "SI", "DO"};
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  for (;;) {
    for (int i = 0; i < 8; i++) {
      uint32_t arr = 1000000u / esc[i] - 1u;
      __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, arr / 2u);
      htim3.Instance->EGR = TIM_EGR_UG;
      log_printf("PRUEBA BUZZER: %s %u Hz\r\n", nom[i], esc[i]);
      vTaskDelay(pdMS_TO_TICKS(400));
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    log_printf("PRUEBA BUZZER: silencio\r\n");
    vTaskDelay(pdMS_TO_TICKS(1500));
  }
}

/* ---------- 5: LED RGB ---------- */
static void prueba_rgb(void)
{
  static const uint16_t c[][3] = {{999,0,0},{0,999,0},{0,0,999},{999,999,999},{999,400,0},{0,0,0}};
  static const char *const n[] = {"ROJO (PD12)", "VERDE (PD13)", "AZUL (PD14)", "BLANCO", "NARANJA", "APAGADO"};
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  for (;;) {
    for (int i = 0; i < 6; i++) {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, c[i][0]);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, c[i][1]);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, c[i][2]);
      log_printf("PRUEBA RGB: %s\r\n", n[i]);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

/* ---------- 6: MPU6050 ---------- */
static void prueba_mpu(void)
{
  bool ok = false;
  int16_t ax, ay, az, pax = 0, pay = 0, paz = 0;
  for (;;) {
    if (!ok) {
      escanear_i2c();
      ok = (mpu6050_Init() == HAL_OK);
      log_printf("PRUEBA MPU: init %s\r\n", ok ? "OK" : "FALLO");
      if (!ok) { i2c_recuperar_bus(); vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
    }
    if (mpu6050_ReadAccel(&ax, &ay, &az) != HAL_OK) { ok = false; log_printf("PRUEBA MPU: ERROR I2C\r\n"); continue; }
    uint32_t delta = abs(ax - pax) + abs(ay - pay) + abs(az - paz);
    pax = ax; pay = ay; paz = az;
    log_printf("PRUEBA MPU: ax=%6d ay=%6d az=%6d  cambio=%lu\r\n", ax, ay, az, (unsigned long)delta);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void Pruebas_Task(uint8_t modo)
{
  static const char *const nombre[] = {"-", "OLED", "BOTONES", "LDR", "BUZZER", "RGB", "MPU6050"};
  log_printf("===== MODO PRUEBA %u: %s =====\r\n", modo, modo <= 6 ? nombre[modo] : "?");
  switch (modo) {
    case 1: prueba_oled(); break;
    case 2: prueba_botones(); break;
    case 3: prueba_ldr(); break;
    case 4: prueba_buzzer(); break;
    case 5: prueba_rgb(); break;
    case 6: prueba_mpu(); break;
    default: break;
  }
  for (;;) vTaskDelay(portMAX_DELAY);
}
