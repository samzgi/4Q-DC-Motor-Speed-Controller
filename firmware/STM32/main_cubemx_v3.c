/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { DIR_CW = 0, DIR_CCW = 1 } MotorDir_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_PERIOD          999
#define CURRENT_LIMIT_A     1.0f
#define ACS712_OFFSET_V     1.65f
#define ACS712_SENS         0.066f
#define ADC_VREF            3.3f
#define ADC_RESOLUTION      4095.0f
#define ENCODER_MAX         100
#define AUTO_REVERSE_MS     500
#define MOTOR_VOLTAGE       12.0f
#define FAULT_LOCKOUT_MS    2000   /* Hiccup önleme: arıza sonrası bekleme süresi (ms) */
#define DEAD_TIME_MS        2      /* Shoot-through önleme: yön geçişinde bekleme (ms) */
#define SOFTSTART_STEP_MS   20     /* Soft-start: her adım arası süre (ms) */
#define SOFTSTART_STEP_PCT  5      /* Soft-start: her adımda duty artışı (%) */
#define ADC_AVG_SAMPLES     10     /* Akım ölçümü moving average örnekleme sayısı */

/* Enkoderlı motor için gerçek RPM hesabı.
 * Motoru aldıktan sonra PPR (pulse per revolution) değerini gir.
 * Örnek: 11 PPR enkoder + 50:1 redüktör = 550 PPR çıkışta */
#define MOTOR_PPR           20     /* Enkoderin PPR değeri — motora göre güncelle! */
#define RPM_CALC_PERIOD_MS  100    /* RPM her 100ms'de bir hesaplanır */

/* Rotary encoder tıklama butonu — PC9 (boş bir pin, encoder SW pinine bağla) */
#define ENC_BTN_PORT   GPIOC
#define ENC_BTN_PIN    GPIO_PIN_9

/* OLED SPI pin makroları (SPI2: PB13 SCK, PB15 MOSI) */
#define OLED_CS_PORT   GPIOB
#define OLED_CS_PIN    GPIO_PIN_12
#define OLED_DC_PORT   GPIOD
#define OLED_DC_PIN    GPIO_PIN_11
#define OLED_RST_PORT  GPIOD
#define OLED_RST_PIN   GPIO_PIN_10

#define OLED_CS_LOW()   HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_RESET)
#define OLED_CS_HIGH()  HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET)
#define OLED_DC_LOW()   HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_RESET)
#define OLED_DC_HIGH()  HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_SET)
#define OLED_RST_LOW()  HAL_GPIO_WritePin(OLED_RST_PORT, OLED_RST_PIN, GPIO_PIN_RESET)
#define OLED_RST_HIGH() HAL_GPIO_WritePin(OLED_RST_PORT, OLED_RST_PIN, GPIO_PIN_SET)

#define SSD1306_WIDTH  128
#define SSD1306_PAGES  8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
SPI_HandleTypeDef hspi2;       /* OLED */
TIM_HandleTypeDef htim3;       /* Encoder */

static uint8_t    oled_buf[SSD1306_WIDTH * SSD1306_PAGES];

static volatile uint8_t   g_current_fault  = 0;
static volatile uint8_t   g_auto_mode      = 0;
static          MotorDir_t g_direction      = DIR_CW;
static          uint32_t   g_last_flip_ms   = 0;
static          uint32_t   g_fault_time_ms   = 0;   /* Hiccup: arıza başlangıç zamanı */

/* Soft-start */
static          uint16_t   g_softstart_duty   = 0;   /* Mevcut soft-start duty (%) */
static          uint8_t    g_softstart_active = 0;   /* 1 = soft-start devam ediyor */
static          uint32_t   g_softstart_tick   = 0;   /* Son adım zamanı */

/* Gerçek RPM ölçümü (enkoderlı motor) */
static volatile int32_t    g_enc_count_prev   = 0;   /* Önceki encoder sayacı */
static          float      g_rpm_measured     = 0.0f; /* Hesaplanan RPM */
static          uint32_t   g_rpm_tick         = 0;   /* Son RPM hesap zamanı */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);

/* Motor */
static void    Motor_SetDirection(MotorDir_t dir);
static void    Motor_SetDuty(uint16_t duty_percent);
static void    Motor_Stop(void);
static void    Motor_SetDirectionWithDeadTime(MotorDir_t new_dir);
static void    Motor_SoftStart(uint16_t target_pct, uint32_t now);
static float   ReadCurrent_A(void);
static int16_t ReadEncoder_Percent(void);
static float   CalcRPM(uint32_t now);

/* OLED */
static void OLED_Init(void);
static void OLED_Clear(void);
static void OLED_Flush(void);
static void OLED_DrawStr(uint8_t x, uint8_t page, const char *str);
static void OLED_Update(float rpm, float voltage, float current,
                         uint8_t fault, uint8_t auto_mode, MotorDir_t dir);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ── SSD1306 minimal SPI sürücüsü ───────────────────────────── */
static void OLED_SendCmd(uint8_t cmd)
{
    OLED_DC_LOW();
    OLED_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 10);
    OLED_CS_HIGH();
}

static void OLED_Init(void)
{
    OLED_RST_LOW(); HAL_Delay(10);
    OLED_RST_HIGH(); HAL_Delay(10);

    const uint8_t seq[] = {
        0xAE, 0xD5,0x80, 0xA8,0x3F, 0xD3,0x00,
        0x40, 0x8D,0x14, 0x20,0x00, 0xA1, 0xC8,
        0xDA,0x12, 0x81,0xCF, 0xD9,0xF1, 0xDB,0x40,
        0xA4, 0xA6, 0xAF,
    };
    for (uint8_t i = 0; i < sizeof(seq); i++) OLED_SendCmd(seq[i]);
}

static void OLED_Clear(void)  { memset(oled_buf, 0, sizeof(oled_buf)); }

static void OLED_Flush(void)
{
    OLED_SendCmd(0x21); OLED_SendCmd(0); OLED_SendCmd(127);
    OLED_SendCmd(0x22); OLED_SendCmd(0); OLED_SendCmd(7);
    OLED_DC_HIGH();
    OLED_CS_LOW();
    HAL_SPI_Transmit(&hspi2, oled_buf, sizeof(oled_buf), 100);
    OLED_CS_HIGH();
}

/* 5×7 font (ASCII 32–122) */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},
    {0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{0x03,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},
    {0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},
    {0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},
    {0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},
    {0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},
    {0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x40,0x7C},
    {0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},
};

static void OLED_DrawChar(uint8_t x, uint8_t page, char c)
{
    if (c < 32 || c > 122) c = ' ';
    const uint8_t *g = font5x7[c - 32];
    for (uint8_t i = 0; i < 5; i++)
        if (x + i < SSD1306_WIDTH)
            oled_buf[page * SSD1306_WIDTH + x + i] = g[i];
    if (x + 5 < SSD1306_WIDTH)
        oled_buf[page * SSD1306_WIDTH + x + 5] = 0x00;
}

static void OLED_DrawStr(uint8_t x, uint8_t page, const char *str)
{
    while (*str && x < SSD1306_WIDTH) { OLED_DrawChar(x, page, *str++); x += 6; }
}

static void OLED_Update(float rpm, float voltage, float current,
                         uint8_t fault, uint8_t auto_mode, MotorDir_t dir)
{
    char buf[22];
    OLED_Clear();
    OLED_DrawStr(0, 0, "4Q MOTOR CTRL");
    snprintf(buf, sizeof(buf), "RPM:%5.0f", rpm);      OLED_DrawStr(0, 2, buf);
    snprintf(buf, sizeof(buf), "V  :%5.1fV", voltage); OLED_DrawStr(0, 3, buf);
    snprintf(buf, sizeof(buf), "I  :%5.2fA", current); OLED_DrawStr(0, 4, buf);
    snprintf(buf, sizeof(buf), "%s %s %s",
             (dir == DIR_CW) ? "CW " : "CCW",
             auto_mode       ? "AUTO " : "     ",
             fault           ? "FAULT" : "     ");
    OLED_DrawStr(0, 6, buf);
    OLED_Flush();
}

/* ── Motor yardımcı fonksiyonları ───────────────────────────── */

/**
 * @brief Anlık yön ata (dead-time YOK — sadece iç kullanım)
 */
static void Motor_SetDirection(MotorDir_t dir)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, (dir == DIR_CW)  ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, (dir == DIR_CCW) ? GPIO_PIN_SET   : GPIO_PIN_RESET);
    g_direction = dir;
}

static void Motor_SetDuty(uint16_t pct)
{
    if (pct > 100) pct = 100;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (PWM_PERIOD + 1) * pct / 100);
}

static void Motor_Stop(void)
{
    Motor_SetDuty(0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}

/**
 * @brief Yön değişimi + Dead-Time koruması
 *        Shoot-through önlemek için:
 *        1. Motor durdur (tüm pinler LOW)
 *        2. DEAD_TIME_MS bekle — L298N transistörlerin tamamen kapanması için
 *        3. Yeni yönü ata
 *        4. Soft-start başlat
 */
static void Motor_SetDirectionWithDeadTime(MotorDir_t new_dir)
{
    if (new_dir == g_direction) return;  /* Yön zaten aynıysa işlem yok */

    Motor_Stop();
    HAL_Delay(DEAD_TIME_MS);             /* Dead-time: ~2ms */
    Motor_SetDirection(new_dir);

    /* Soft-start'ı sıfırla */
    g_softstart_duty  = 0;
    g_softstart_active = 1;
    g_softstart_tick  = HAL_GetTick();
}

/**
 * @brief Soft-start: duty'yi kademeli artır
 *        Her SOFTSTART_STEP_MS'de bir SOFTSTART_STEP_PCT artır.
 *        Hem arıza sonrası kurtarmada hem yön geçişinde kullanılır.
 *        target_pct'e ulaşınca durur.
 */
static void Motor_SoftStart(uint16_t target_pct, uint32_t now)
{
    if (!g_softstart_active) return;

    if ((now - g_softstart_tick) >= SOFTSTART_STEP_MS)
    {
        g_softstart_tick = now;
        g_softstart_duty += SOFTSTART_STEP_PCT;

        if (g_softstart_duty >= target_pct)
        {
            g_softstart_duty   = target_pct;
            g_softstart_active = 0;  /* Soft-start tamamlandı */
        }
        Motor_SetDuty(g_softstart_duty);
    }
}

/**
 * @brief Gerçek RPM hesapla — enkoderlı motor için
 *        Her RPM_CALC_PERIOD_MS'de encoder sayacındaki değişimi okur.
 *        Formül: RPM = (delta_pulse / PPR) / (dt_ms / 60000)
 *
 *        Motor henüz gelmemişse veya PPR bilinmiyorsa 0 döner.
 */
static float CalcRPM(uint32_t now)
{
    if ((now - g_rpm_tick) < RPM_CALC_PERIOD_MS) return g_rpm_measured;

    float dt_min = (float)(now - g_rpm_tick) / 60000.0f;  /* ms → dakika */
    g_rpm_tick   = now;

    int32_t count_now  = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    int32_t delta      = count_now - g_enc_count_prev;
    g_enc_count_prev   = count_now;

    /* Overflow kontrolü (sayaç 0-ENCODER_MAX arasında döner) */
    if (delta > (ENCODER_MAX / 2))  delta -= ENCODER_MAX;
    if (delta < -(ENCODER_MAX / 2)) delta += ENCODER_MAX;

    g_rpm_measured = (float)delta / (float)MOTOR_PPR / dt_min;

    /* Yön işaretini uygula */
    if (g_direction == DIR_CCW && g_rpm_measured > 0.0f)
        g_rpm_measured = -g_rpm_measured;

    return g_rpm_measured;
}

static float ReadCurrent_A(void)
{
    /* 10 örnek moving average — PWM gürültüsünü filtreler */
    uint32_t sum = 0;
    for (uint8_t n = 0; n < ADC_AVG_SAMPLES; n++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
    float v = ((sum / ADC_AVG_SAMPLES) / ADC_RESOLUTION) * ADC_VREF;
    float i = (v - ACS712_OFFSET_V) / ACS712_SENS;
    return (i < 0.0f) ? -i : i;
}

static int16_t ReadEncoder_Percent(void)
{
    int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    if (cnt < 0)            { __HAL_TIM_SET_COUNTER(&htim3, 0);           cnt = 0; }
    else if (cnt > ENCODER_MAX) { __HAL_TIM_SET_COUNTER(&htim3, ENCODER_MAX); cnt = ENCODER_MAX; }
    return (int16_t)cnt;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  MX_SPI2_Init();   /* OLED */
  MX_TIM3_Init();   /* Encoder */

  /* ADC sampling time'ı güncelle — akım ölçümü için daha uzun süre */
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;  /* CubeMX 3CYCLES yerine */
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  /* TIM1 PWM parametrelerini güncelle */
  htim1.Init.Prescaler = 3;     /* 84MHz/4 = 21MHz */
  htim1.Init.Period    = PWM_PERIOD;  /* 999 → 21kHz */
  HAL_TIM_PWM_Init(&htim1);
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  OLED_Init();
  OLED_Clear();
  OLED_DrawStr(14, 3, "HAZIR...");
  OLED_Flush();
  HAL_Delay(1000);

  Motor_SetDirection(DIR_CW);
  Motor_SetDuty(0);

  uint32_t tick_display = HAL_GetTick();
  g_rpm_tick    = HAL_GetTick();
  g_softstart_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* 1. Akım oku — 10 örnek moving average */
    float current_A = ReadCurrent_A();

    /* 2. Gerçek RPM hesapla (enkoderlı motor) */
    float rpm = CalcRPM(now);

    /* 3. Aşım koruması + hiccup önleme + soft-start kurtarma */
    if (current_A >= CURRENT_LIMIT_A && !g_current_fault)
    {
        /* Yeni arıza: motoru durdur, lockout başlat */
        g_current_fault   = 1;
        g_fault_time_ms   = now;
        g_softstart_active = 0;
        Motor_Stop();
    }
    else if (g_current_fault)
    {
        Motor_Stop();  /* Lockout boyunca motor kapalı */

        if ((now - g_fault_time_ms) >= FAULT_LOCKOUT_MS)
        {
            /* 2 sn geçti — akım normale döndüyse soft-start ile kurtar */
            if (current_A < (CURRENT_LIMIT_A - 0.05f))
            {
                g_current_fault   = 0;
                g_softstart_duty  = 0;
                g_softstart_active = 1;
                g_softstart_tick  = now;
            }
        }
    }

    /* 4. Hız / yön kontrolü */
    uint16_t duty_pct = 0;
    if (!g_current_fault)
    {
        if (g_auto_mode)
        {
            /* Test #5 — her 500ms'de yön değiştir (dead-time dahil) */
            if ((now - g_last_flip_ms) >= AUTO_REVERSE_MS)
            {
                g_last_flip_ms = now;
                MotorDir_t new_dir = (g_direction == DIR_CW) ? DIR_CCW : DIR_CW;
                Motor_SetDirectionWithDeadTime(new_dir);  /* dead-time + soft-start */
            }
            duty_pct = 80;
        }
        else
        {
            /* Manuel mod — rotary encoder hız ayarı */
            duty_pct = (uint16_t)ReadEncoder_Percent();
        }

        /* Soft-start aktifse kademeli artır, değilse direkt uygula */
        if (g_softstart_active)
            Motor_SoftStart(duty_pct, now);
        else
            Motor_SetDuty(duty_pct);
    }

    /* 5. OLED güncelle (her 200ms) */
    if ((now - tick_display) >= 200)
    {
        tick_display = now;
        float motor_v = (duty_pct / 100.0f) * MOTOR_VOLTAGE;
        /* Gerçek RPM kullan — motor enkodersiz gelirse 0 gösterir */
        OLED_Update(rpm, motor_v, current_A, g_current_fault, g_auto_mode, g_direction);
    }

    HAL_Delay(10);
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief ADC1 Initialization Function
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES; /* USER CODE BEGIN 2'de 480'e güncelleniyor */
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief I2S3 Initialization Function
  */
static void MX_I2S3_Init(void)
{
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief SPI1 Initialization Function
  */
static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM1 Initialization Function
  */
static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;          /* USER CODE BEGIN 2'de güncelleniyor */
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;         /* USER CODE BEGIN 2'de güncelleniyor */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Output başlangıç seviyeleri */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_0|GPIO_PIN_1|Audio_RST_Pin, GPIO_PIN_RESET);

  /* CS_I2C_SPI */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /* OTG_FS_PowerSwitchOn */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /* PDM_OUT */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* B1 (PA0) — EVT yerine EXTI olarak yeniden yapılandır */
  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* OLED CS = PB12, DC = PD11, RST = PD10 */
  GPIO_InitStruct.Pin  = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin  = GPIO_PIN_10 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  OLED_CS_HIGH(); OLED_RST_HIGH();

  /* Encoder tıklama butonu — PC9, EXTI9, pull-up (buton GND'ye çeker) */
  GPIO_InitStruct.Pin  = ENC_BTN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENC_BTN_PORT, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USER CODE END MX_GPIO_Init_1 */

  /* BOOT1 */
  GPIO_InitStruct.Pin  = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /* CLK_IN */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /* LEDs + PD0/PD1 (IN1/IN2) + Audio_RST */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_0|GPIO_PIN_1|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* PC6/PC8 — TIM3_CH1/CH3 (Encoder A/B) */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* OTG_FS_OverCurrent */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* MEMS_INT2 */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief SPI2 Init — OLED (PB13 SCK, PB15 MOSI)
  */
static void MX_SPI2_Init(void)
{
    __HAL_RCC_SPI2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_13 | GPIO_PIN_15;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);

    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_1LINE;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief TIM3 Init — Encoder (PC6 CH1, PC8 CH3)
  * NOT: PC6=TIM3_CH1, PC8=TIM3_CH3 — CH1+CH2 yerine CH1+CH3 kullanıyoruz
  *      çünkü PC7 CS43L22_MCLK tarafından kullanılıyor.
  *      Encoder mode TI1 only (tek kanal, yön bilgisi olmadan sayar).
  */
static void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance           = TIM3;
    htim3.Init.Prescaler     = 0;
    htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim3.Init.Period        = ENCODER_MAX;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim3);

    /* TI1 (PC6) sadece sayıcı olarak — basit pulse count modu */
    TIM_IC_InitTypeDef ic = {0};
    ic.ICPolarity  = TIM_ICPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 4;
    HAL_TIM_IC_ConfigChannel(&htim3, &ic, TIM_CHANNEL_1);

    /* Encoder interface — sadece TI1 (CH1) kullanarak */
    TIM3->SMCR = TIM_SMCR_SMS_0;   /* SMS=001: Encoder mode TI1 only */
}

/**
  * @brief EXTI callback — Encoder tıklama butonu (PC9)
  *        Kısa basış (<1s) → yön değiştir (CW ↔ CCW)
  *        Uzun basış (>1s) → Test#5 auto-reverse modu aç/kapat
  *
  *        Debounce: yazılımsal 50ms filtre
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t press_start  = 0;
    static uint32_t last_event   = 0;   /* debounce */

    if (GPIO_Pin == ENC_BTN_PIN)
    {
        uint32_t now = HAL_GetTick();

        /* 50 ms debounce — çok hızlı tetiklemeleri yoksay */
        if ((now - last_event) < 50) return;
        last_event = now;

        if (HAL_GPIO_ReadPin(ENC_BTN_PORT, ENC_BTN_PIN) == GPIO_PIN_RESET)
        {
            /* Buton basıldı (pull-up → LOW) */
            press_start = now;
        }
        else
        {
            /* Buton bırakıldı (LOW → HIGH) */
            uint32_t duration = now - press_start;

            if (duration > 1000)
            {
                /* Uzun basış → auto mod toggle */
                g_auto_mode    = !g_auto_mode;
                g_last_flip_ms = now;
            }
            else if (duration > 50)
            {
                /* Kısa basış → yön değiştir — dead-time koruma ile */
                if (!g_auto_mode && !g_current_fault)
                {
                    MotorDir_t new_dir = (g_direction == DIR_CW) ? DIR_CCW : DIR_CW;
                    Motor_SetDirectionWithDeadTime(new_dir);
                }
            }
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
