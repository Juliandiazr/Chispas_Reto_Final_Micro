# Chispa — Mascota virtual con STM32F407 + FreeRTOS

Proyecto del **Reto: Programación en Alto Nivel** (Unidad 4, Microprocesadores — UPB).

Chispa es una mascota virtual que nace de un huevo, siente hambre, quiere jugar, se ensucia y se duerme cuando apagas la luz. Corre en una **STM32F407VET6** (placa "black board") con **HAL** y **FreeRTOS (CMSIS-RTOS v2)**, generada con **STM32CubeMX** y compilada en **STM32CubeIDE**.

![Montaje](docs/imagenes/montaje.jpg)
> *(Las fotos se agregan en `docs/imagenes/`.)*

---

## Contenido del repositorio

```
chispa/
├── chispas.ioc                 Configuración de STM32CubeMX (pines, relojes, FreeRTOS)
├── .project / .cproject        Proyecto de STM32CubeIDE
├── STM32F407VETX_FLASH.ld      Linker script
├── Core/
│   ├── Inc/chispa.h            ★ Configuración, tipos y prototipos de la aplicación
│   ├── Inc/FreeRTOSConfig.h    Configuración de FreeRTOS (generada por CubeMX)
│   ├── Src/chispa_app.c        ★ Tareas: Sensores, Sonido, Lógica, Pantalla y pruebas
│   ├── Src/chispa_drivers.c    ★ Drivers OLED SSD1306, MPU6050, fuente 5x7, recuperación I2C
│   ├── Src/freertos.c          Creación de tareas/colas/mutex + tarea Log (código en USER CODE)
│   └── Src/*.c                 main, gpio, adc, i2c, tim, usart (generados por CubeMX)
├── Drivers/                    HAL y CMSIS de ST (externo)
├── Middlewares/                FreeRTOS (externo)
├── firmware/chispas.hex        Binario listo para cargar
└── docs/
    ├── Chispa_Documento_tecnico.docx
    ├── Chispa_Guion_de_sustentacion.docx
    ├── evidencias_chispa.txt   Preemption, estabilidad y stack (extraído del log)
    ├── log_chispa.txt          Log UART completo de la prueba de 13 min 20 s
    └── imagenes/               Fotos del montaje y capturas
```

Los archivos marcados con ★ son el código propio. Los demás los genera CubeMX o vienen de ST/FreeRTOS.

---

## Hardware

| Componente | Pin(es) STM32 | Periférico | Notas |
|---|---|---|---|
| Botón COMER | PC0 | EXTI0 | Pull-up interno, flanco de bajada |
| Botón JUGAR | PC1 | EXTI1 | Pull-up interno, flanco de bajada |
| Botón LIMPIAR | PC2 | EXTI2 | Pull-up interno, flanco de bajada |
| Botón DORMIR / DESPERTAR | PC3 | EXTI3 | Reemplaza al sensor táctil TTP223 |
| LDR (fotoresistencia) | PA1 | ADC1 IN1 | Divisor 3V3 – LDR – PA1 – 10 kΩ – GND |
| OLED SSD1306 128×64 | PB6 SCL / PB7 SDA | I2C1 400 kHz | Dirección 0x3C |
| MPU6050 (acelerómetro) | PB6 / PB7 | I2C1 (compartido) | Dirección 0x68 |
| Buzzer pasivo | PB0 | TIM3 CH3 PWM | A través de un transistor 2N2222 |
| LED RGB (cátodo común) | PD12 / PD13 / PD14 | TIM4 CH1–CH3 PWM | Con resistencia en cada color |
| Log serial | PA9 TX / PA10 RX | USART1 115200 8N1 | Adaptador USB-serial CP2102 |
| Programador | PA13 / PA14 | SWD | J-Link (conector JTAG de 20 pines) |

Todos los módulos se alimentan a **3V3** y comparten GND con la placa.

---

## Arquitectura FreeRTOS

| Tarea | Prioridad | Stack (palabras) | Activación | Función |
|---|---|---|---|---|
| tSensores | AboveNormal | 256 | Periódica 20 ms (`vTaskDelayUntil`) | Lee MPU6050 (sacudidas) y LDR (luz/oscuridad) |
| tSonido | AboveNormal | 256 | Notificación de tarea | Reproduce melodías en el buzzer |
| tLogica | Normal | 512 | Cola `qEventos` (timeout 250 ms) | Máquina de estados, estadísticas, LED RGB |
| tPantalla | BelowNormal | 512 | Periódica 100 ms | Dibuja caras y barras en la OLED |
| tLog | Low | 256 | Cola `qLog` | Envía los mensajes por UART |

**Comunicación entre tareas**
- `qEventos` (10 × `uint8_t`): ISR de botones y tSensores → tLogica.
- `qPantalla` (1 × `vista_t`, `xQueueOverwrite`): tLogica → tPantalla.
- `qLog` (16 × 96 bytes): todas las tareas → tLog.
- Notificación de tarea: tLogica → tSonido (melodía a reproducir).
- `mutexI2C`: protege el bus I2C1 que comparten la OLED y el MPU6050.

**Máquina de estados:** `HUEVO → NORMAL ⇄ NECESIDAD`, `NORMAL/NECESIDAD ⇄ DORMIDA`, y `ERROR` si el MPU6050 falla 3 veces seguidas. Estadísticas de hambre, felicidad, energía y limpieza de 0 a 100; 1 "hora" de Chispa = 10 s reales.

---

## Cómo abrir, compilar y ejecutar

### Requisitos
- STM32CubeIDE 1.x (incluye el compilador `arm-none-eabi-gcc`).
- STM32CubeMX (solo si se quiere cambiar la configuración del `.ioc`).
- Programador J-Link (o ST-Link) por SWD, **o** un cable USB para DFU.

### Abrir el proyecto
1. Clonar el repositorio: `git clone <url-del-repo>`
2. En STM32CubeIDE: **File → Import → General → Existing Projects into Workspace**.
3. Seleccionar la carpeta del repositorio y pulsar **Finish**.

### Compilar
- **Project → Build Project** (o el martillo). Debe terminar con `0 errors`.

### Cargar en la placa
**Opción A — J-Link (la que se usó):**
1. Conectar el J-Link al conector JTAG de 20 pines (SWDIO = PA13, SWCLK = PA14, GND, VTref = 3V3).
2. **Run → Debug Configurations → STM32 C/C++ Application → Debugger → Debug probe: SEGGER J-LINK**.
3. Pulsar **Run**.

*(Con ST-Link basta con dejar el probe en ST-LINK.)*

**Opción B — USB DFU, sin programador:**
1. Puentear **BOOT0 a 3V3** y conectar la placa por USB.
2. Abrir STM32CubeProgrammer, elegir **USB** y conectar.
3. Cargar `firmware/chispas.hex` (o `Debug/chispas.elf`) y pulsar **Download**.
4. Devolver BOOT0 a GND y pulsar RESET.

### Ver el log por UART (macOS / Linux)
```bash
(stty 115200; cat) < /dev/cu.usbserial-0001 | tee -a log_chispa.txt
```
En Windows se puede usar PuTTY o el monitor serial de Arduino a 115200 baudios.

---

## Modo de prueba por módulo

En `Core/Inc/chispa.h`:

```c
#define MODO_PRUEBA  0
```

| Valor | Qué se ejecuta |
|---|---|
| 0 | Programa completo de Chispa |
| 1 | Solo OLED |
| 2 | Solo pulsadores |
| 3 | Solo LDR |
| 4 | Solo buzzer |
| 5 | Solo LED RGB |
| 6 | Solo MPU6050 |

Se cambia el número, se compila y se carga la placa. Así se probó cada módulo por separado antes de integrarlos.

---

## Evidencias

Resumen tomado de `docs/evidencias_chispa.txt` y `docs/log_chispa.txt`:

- **Preemption:** se registraron 10 eventos, por ejemplo `PANTALLA: frame 66106..66132 preemptado (ISR COMER t=66123)`. tLogica y tSonido interrumpieron a tPantalla en el mismo milisegundo en que llegó la ISR.
- **Estabilidad:** 13 min 20 s funcionando sin reinicios ni errores de stack o heap (el reto pide 5 min).
- **Stack libre medido** (`uxTaskGetStackHighWaterMark`, en palabras):

  | Tarea | Palabras libres |
  |---|---|
  | Sensores | 98 |
  | Sonido | 112 |
  | Logica | 350 |
  | Pantalla | 356 |
  | Log | 94 |

  Heap libre: 14952 bytes de 25000.

---

## Robustez

- **Recuperación del bus I2C:** si un esclavo deja SDA en bajo, se generan hasta 9 pulsos de SCL y un STOP, y se reinicia I2C1 (incluye el arreglo del errata del flag BUSY).
- **Umbral adaptativo de la LDR:** se considera oscuro por debajo del 45 % de la luz ambiente de referencia y luz otra vez por encima del 65 %, con histéresis de tiempo.
- **Detección de sensores desconectados** (MPU6050 y LDR), con reintento cada 1 s.
- **Hooks de FreeRTOS:** desbordamiento de stack (`configCHECK_FOR_STACK_OVERFLOW = 2`) y fallo de `malloc`, ambos reportados por UART.

---

## Librerías externas

| Librería | Origen | Licencia |
|---|---|---|
| STM32F4xx HAL Driver | STMicroelectronics (generada por CubeMX) | BSD-3-Clause |
| CMSIS | ARM / STMicroelectronics | Apache-2.0 |
| FreeRTOS Kernel + CMSIS-RTOS v2 | Amazon / ARM (vía CubeMX) | MIT |

Los drivers de la OLED SSD1306 y del MPU6050 son propios (`chispa_drivers.c`).

---

## Autor

**Julián David Díaz Ramírez** — Microprocesadores, Universidad Pontificia Bolivariana, 2026.
