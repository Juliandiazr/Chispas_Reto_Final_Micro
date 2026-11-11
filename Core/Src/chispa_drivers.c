/* ==========================================================================
 *  chispa_drivers.c  -  Drivers de los perifericos externos (sobre HAL)
 *    1. Fuente de texto 5x7
 *    2. OLED SSD1306 128x64 por I2C1 (direccion 0x3C)
 *    3. Acelerometro MPU6050 por I2C1 (direccion 0x68)
 *    4. Recuperacion del bus I2C bloqueado
 *
 *  Estas funciones NO protegen el bus I2C: la tarea que las use debe tomar
 *  mutexI2C antes de llamarlas (asi lo hacen Sensores y Pantalla).
 * ========================================================================== */
#include "chispa.h"
#include "i2c.h"
#include <string.h>
#include <stdlib.h>

/* ==========================================================================
 *  1. FUENTE 5x7 (ASCII 32..95)
 *  Cada caracter: 5 columnas; bit 0 = fila superior. Minusculas se muestran como mayusculas.
 * ========================================================================== */

static const uint8_t font5x7[64][5] = {
  {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
  {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
  {0x02,0x01,0x51,0x09,0x06}, /* '"' */
  {0x02,0x01,0x51,0x09,0x06}, /* '#' */
  {0x02,0x01,0x51,0x09,0x06}, /* '$' */
  {0x23,0x13,0x08,0x64,0x62}, /* '%' */
  {0x02,0x01,0x51,0x09,0x06}, /* '&' */
  {0x02,0x01,0x51,0x09,0x06}, /* ''' */
  {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
  {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
  {0x2A,0x1C,0x3E,0x1C,0x2A}, /* ' ' */
  {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
  {0x02,0x01,0x51,0x09,0x06}, /* ',' */
  {0x08,0x08,0x08,0x08,0x08}, /* '-' */
  {0x00,0x60,0x60,0x00,0x00}, /* '.' */
  {0x20,0x10,0x08,0x04,0x02}, /* ' ' */
  {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
  {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
  {0x42,0x61,0x51,0x49,0x46}, /* '2' */
  {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
  {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
  {0x27,0x45,0x45,0x45,0x39}, /* '5' */
  {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
  {0x01,0x71,0x09,0x05,0x03}, /* '7' */
  {0x36,0x49,0x49,0x49,0x36}, /* '8' */
  {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
  {0x00,0x00,0x36,0x00,0x00}, /* ':' */
  {0x02,0x01,0x51,0x09,0x06}, /* ';' */
  {0x08,0x14,0x22,0x41,0x00}, /* '<' */
  {0x14,0x14,0x14,0x14,0x14}, /* '=' */
  {0x00,0x41,0x22,0x14,0x08}, /* '>' */
  {0x02,0x01,0x51,0x09,0x06}, /* '?' */
  {0x02,0x01,0x51,0x09,0x06}, /* '@' */
  {0x7E,0x09,0x09,0x09,0x7E}, /* 'A' */
  {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
  {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
  {0x7F,0x41,0x41,0x41,0x3E}, /* 'D' */
  {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
  {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
  {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
  {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
  {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
  {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
  {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
  {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
  {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' */
  {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
  {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
  {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
  {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
  {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
  {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
  {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
  {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
  {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
  {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
  {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
  {0x03,0x04,0x78,0x04,0x03}, /* 'Y' */
  {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
  {0x02,0x01,0x51,0x09,0x06}, /* '[' */
  {0x02,0x01,0x51,0x09,0x06}, /* ' ' */
  {0x02,0x01,0x51,0x09,0x06}, /* ']' */
  {0x02,0x01,0x51,0x09,0x06}, /* '^' */
  {0x40,0x40,0x40,0x40,0x40}, /* '_' */
};

/* ==========================================================================
 *  2. OLED SSD1306
 *  Framebuffer de 1 KB en RAM; se dibuja en el buffer y se envia por paginas.
 * ========================================================================== */

static uint8_t fb[SSD1306_W * SSD1306_PAGES];   /* framebuffer de 1 KB */

static HAL_StatusTypeDef cmd(uint8_t c)
{
  return HAL_I2C_Mem_Write(&hi2c1, SSD1306_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &c, 1, 20);
}

HAL_StatusTypeDef ssd1306_Init(void)
{
  static const uint8_t seq[] = {
    0xAE,             /* display OFF */
    0xD5, 0x80,       /* divisor de reloj */
    0xA8, 0x3F,       /* multiplex 64 */
    0xD3, 0x00,       /* offset 0 */
    0x40,             /* linea de inicio 0 */
    0x8D, 0x14,       /* charge pump ON */
    0x20, 0x02,       /* modo de direccionamiento por pagina */
    0xA1,             /* segment remap */
    0xC8,             /* COM scan descendente */
    0xDA, 0x12,       /* configuracion COM pins */
    0x81, 0xCF,       /* contraste */
    0xD9, 0xF1,       /* pre-charge */
    0xDB, 0x40,       /* VCOMH */
    0xA4,             /* mostrar RAM */
    0xA6,             /* normal (no invertido) */
    0xAF              /* display ON */
  };
  if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_ADDR, 2, 20) != HAL_OK) return HAL_ERROR;
  for (unsigned i = 0; i < sizeof(seq); i++)
    if (cmd(seq[i]) != HAL_OK) return HAL_ERROR;
  ssd1306_Clear();
  return HAL_OK;
}

HAL_StatusTypeDef ssd1306_UpdatePage(uint8_t page)
{
  if (cmd(0xB0 + page) != HAL_OK) return HAL_ERROR;   /* pagina */
  if (cmd(0x00) != HAL_OK) return HAL_ERROR;          /* columna baja = 0 */
  if (cmd(0x10) != HAL_OK) return HAL_ERROR;          /* columna alta = 0 */
  return HAL_I2C_Mem_Write(&hi2c1, SSD1306_ADDR, 0x40, I2C_MEMADD_SIZE_8BIT,
                           &fb[page * SSD1306_W], SSD1306_W, 50);
}

void ssd1306_Clear(void) { memset(fb, 0, sizeof(fb)); }

void ssd1306_Pixel(int x, int y, bool on)
{
  if (x < 0 || x >= SSD1306_W || y < 0 || y >= SSD1306_H) return;
  if (on) fb[x + (y / 8) * SSD1306_W] |=  (1u << (y & 7));
  else    fb[x + (y / 8) * SSD1306_W] &= ~(1u << (y & 7));
}

void ssd1306_HLine(int x0, int x1, int y) { if (x0 > x1) { int t = x0; x0 = x1; x1 = t; } for (int x = x0; x <= x1; x++) ssd1306_Pixel(x, y, true); }
void ssd1306_VLine(int x, int y0, int y1) { if (y0 > y1) { int t = y0; y0 = y1; y1 = t; } for (int y = y0; y <= y1; y++) ssd1306_Pixel(x, y, true); }

void ssd1306_Line(int x0, int y0, int x1, int y1)
{
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    ssd1306_Pixel(x0, y0, true);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

void ssd1306_Rect(int x, int y, int w, int h, bool fill)
{
  if (fill) { for (int j = y; j < y + h; j++) ssd1306_HLine(x, x + w - 1, j); }
  else { ssd1306_HLine(x, x + w - 1, y); ssd1306_HLine(x, x + w - 1, y + h - 1);
         ssd1306_VLine(x, y, y + h - 1); ssd1306_VLine(x + w - 1, y, y + h - 1); }
}

void ssd1306_Circle(int cx, int cy, int r, bool fill)
{
  for (int y = -r; y <= r; y++)
    for (int x = -r; x <= r; x++) {
      int d = x * x + y * y;
      if (fill ? (d <= r * r) : (d <= r * r && d >= (r - 1) * (r - 1)))
        ssd1306_Pixel(cx + x, cy + y, true);
    }
}

void ssd1306_Char(int x, int y, char c, uint8_t scale)
{
  if (c >= 'a' && c <= 'z') c -= 32;                 /* solo mayusculas */
  if (c < 32 || c > 95) c = '?';
  const uint8_t *g = font5x7[c - 32];
  for (int col = 0; col < 5; col++)
    for (int row = 0; row < 7; row++)
      if (g[col] & (1u << row))
        for (int a = 0; a < scale; a++)
          for (int b = 0; b < scale; b++)
            ssd1306_Pixel(x + col * scale + a, y + row * scale + b, true);
}

void ssd1306_Text(int x, int y, const char *s, uint8_t scale)
{
  while (*s) { ssd1306_Char(x, y, *s++, scale); x += 6 * scale; }
}

/* ==========================================================================
 *  3. ACELEROMETRO MPU6050
 *  Rango +-2 g: 16384 LSB por g.
 * ========================================================================== */

#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_WHO_AM_I     0x75

HAL_StatusTypeDef mpu6050_Init(void)
{
  uint8_t v;
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &v, 1, 20) != HAL_OK)
    return HAL_ERROR;
  if (v != 0x68 && v != 0x70 && v != 0x72) return HAL_ERROR;   /* MPU6050 o clones compatibles */
  v = 0x00;  /* sale de sleep, reloj interno */
  if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &v, 1, 20) != HAL_OK)
    return HAL_ERROR;
  v = 0x00;  /* rango +-2g */
  return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &v, 1, 20);
}

HAL_StatusTypeDef mpu6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t d[6];
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, d, 6, 20) != HAL_OK)
    return HAL_ERROR;
  *ax = (int16_t)((d[0] << 8) | d[1]);
  *ay = (int16_t)((d[2] << 8) | d[3]);
  *az = (int16_t)((d[4] << 8) | d[5]);
  return HAL_OK;
}

/* ==========================================================================
 *  4. RECUPERACION DEL BUS I2C
 *  Si un pico de ruido corta una transmision a la mitad, un esclavo (p. ej. el
 *  MPU6050) puede quedarse sosteniendo SDA en bajo y el bus queda bloqueado:
 *  ni el reset del micro lo arregla. Solucion estandar (NXP UM10204, 3.1.16):
 *  pasar SCL/SDA a GPIO, dar hasta 9 pulsos de reloj hasta que el esclavo suelte
 *  SDA, generar una condicion STOP y volver a inicializar el periferico I2C.
 *  Tambien aplica el "software reset" del I2C por la errata del flag BUSY del F4.
 *  Debe llamarse con mutexI2C tomado.
 * ========================================================================== */
static void retardo_corto(void) { for (volatile int k = 0; k < 300; k++) { } }   /* ~5 us */

void i2c_recuperar_bus(void)
{
  GPIO_InitTypeDef g = {0};
  HAL_I2C_DeInit(&hi2c1);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7;          /* PB6 = SCL, PB7 = SDA */
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);
  retardo_corto();

  /* hasta 9 pulsos en SCL mientras el esclavo tenga SDA en bajo */
  for (int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); retardo_corto();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   retardo_corto();
  }
  /* condicion STOP: SDA sube mientras SCL esta en alto */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); retardo_corto();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   retardo_corto();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   retardo_corto();

  MX_I2C1_Init();                            /* vuelve a poner PB6/PB7 en modo I2C */
  if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {   /* errata: BUSY pegado */
    hi2c1.Instance->CR1 |= I2C_CR1_SWRST;
    hi2c1.Instance->CR1 &= ~I2C_CR1_SWRST;
    MX_I2C1_Init();
  }
}
