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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
 *  EXTI13 ISR --(buttonQ)--> ButtonTask (orta) ---------\
 *  TelemetriTask (yuksek, senaryoya gore) ---------------+--(txQ FIFO)--> UartTxTask (dusuk) --> USART2 (IT)
 *  USART2 RX ISR (komut "S0".."S5", "?") --(txQ, on)----/
 *
 *  Yanit suresi R = t4 - t0. Sinirlar:
 *   t0  Buton ISR girisinde         : filtrenin kabul ettigi kenarin zamani.
 *   t1  ButtonTask olayi aldiktan hemen sonra : olay aktarimi + CPU beklemesi dahil.
 *   t2  Yanit icin xQueueSend cagrisindan hemen once : basarili gonderimde zincir surer.
 *   t3  UART baslatma cagrisindan (HAL_UART_Transmit_IT) hemen once
 *       : ilk fiziksel bit ile ayni an DEGILDIR.
 *   t4  UART TC tamamlanmasi islenirken (HAL_UART_TxCpltCallback)
 *       : son bitten sonraki ISR/callback gozlem zamani.
 *
 *  UART satir protokolu (MCU -> PC), her satir '\n' ile biter, '#' = yorum:
 *   SCN,<scn>,<period_ms>,<work_ms>,<next_id>,<baud>
 *   BAUD,<yeni>,<eski>                          <- hiz degisimi onayi (ESKI hizda gonderilir)
 *   TEL,<scn>,<seq>,<temp_dC>,<vbat_mV>,<exec_us>
 *   BTN,<scn>,<id>                                  <- olculen yanit mesajinin kendisi
 *   REC,<scn>,<id>,<status>,<t0>,<t1>,<t2>,<t3>,<t4> (ham TIM2 us, modulo 2^32)
 *       status: OK | DROPI (buttonQ dolu) | DROPQ (txQ dolu) | TXERR | TMO | LOST
 *  Komutlar (PC -> MCU), yalnizca deney disinda: "S0".."S5" senaryo, "?" durum,
 *   "B<baud>" UART hizi (9600..921600). Yeni hizda 3 sn icinde "?" gelmezse eski hiza donulur.
 */
typedef enum
{
  TEL = 0,
  BTN = 1,
  CTL = 2
} TxSource;

/* ISR -> ButtonTask */
typedef struct
{
  uint32_t id;
  uint32_t t0_us;          /* filtrenin kabul ettigi kenarin zamani */
} ButtonEvent;

/* Ureticiler -> UartTxTask (ortak FIFO) */
typedef struct
{
  TxSource src;
  uint8_t  scn;
  uint32_t id;
  uint32_t t_event_us;     /* TEL: release ani, BTN: t0 */
  union
  {
    struct { uint32_t sample; int16_t temp_dC; uint16_t vbat_mV; uint32_t exec_us; } tel;
    struct { uint8_t cmd; uint32_t baud; } ctl;   /* 0..5: senaryo, CTL_QUERY: durum, CTL_BAUD: hiz */
  } u;
} TxMsg;

typedef enum
{
  REC_PENDING = 0,   /* ISR kabul etti (t0)              */
  REC_ACTIVE,        /* ButtonTask aldi (t1)             */
  REC_QUEUED,        /* yanit txQ'ya veriliyor (t2)      */
  REC_OK,            /* UART TC goruldu (t3, t4)         */
  REC_DROPI,         /* buttonQ dolu                     */
  REC_DROPQ,         /* txQ dolu                         */
  REC_TXERR,         /* HAL_UART_Transmit_IT hata dondu  */
  REC_TMO            /* TC zamaninda gelmedi             */
} RecState;

/* Bir buton olayinin zaman cizelgesi */
typedef struct
{
  uint32_t id;
  uint8_t  scn;
  RecState state;
  uint32_t t0, t1, t2, t3, t4;
} BtnRecord;

typedef struct
{
  uint16_t period_ms;      /* 0 = telemetri kapali */
  uint16_t work_ms;        /* aktivasyon basina ek CPU isi (ms) */
} ScenarioCfg;

typedef enum
{
  TX_OK = 0,
  TX_ERR,
  TX_TMO
} TxResult;

/* Senaryo degisince sifirlanir. Debugger -> Live Expressions */
typedef struct
{
  uint32_t btn_accepted;       /* filtreden gecen basis kenarlari                 */
  uint32_t btn_bounce;         /* 30 ms penceresinde sayilip atilan kenarlar      */
  uint32_t btn_isr_drop;       /* buttonQ dolu                                    */
  uint32_t btn_txq_drop;       /* yanit txQ'ya sigmadi                            */
  uint32_t btn_ok;             /* t4'e ulasan yanitlar                            */
  uint32_t btn_deadline_miss;  /* R > 20 ms                                       */
  uint32_t btn_max_us;
  uint32_t tx_err;
  uint32_t tx_tmo;
  uint32_t rec_lost;           /* kayit slotu raporlanmadan ustune yazildi        */
  uint32_t tel_sent;
  uint32_t tel_drop;
  uint32_t tel_max_exec_us;
  uint32_t tel_max_jitter_us;
  uint32_t rx_err;
  uint32_t cmd_bad;
} RtStats;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Oncelikler: defaultTask = 24 (osPriorityNormal); hepsi onun ustunde */
#define TEL_TASK_PRIO       40u   /* en yuksek */
#define BTN_TASK_PRIO       32u   /* orta      */
#define UARTTX_TASK_PRIO    25u   /* en dusuk  */

#define TEL_STACK_WORDS     256u
#define BTN_STACK_WORDS     256u
#define UARTTX_STACK_WORDS  512u  /* snprintf icin genis */

#define TXQ_LEN             16u
#define BUTTONQ_LEN         8u

#define BTN_DEADLINE_US     20000u
#define BTN_REFILTER_US     30000u  /* kabul edilen kenardan sonraki 30 ms: tekrar kenarlar atilir */
#define BTN_REC_SLOTS       16u

#define SCN_COUNT           6u
#define SCN_DEFAULT         0u
#define CTL_QUERY           0xFFu
#define CTL_BAUD            0xFDu

#define UART_TX_MARGIN_MS   20u     /* TMO = satirin hattaki suresi + bu pay */
#define BAUD_CONFIRM_MS     3000u   /* yeni hizda bu surede "?" gelmezse eski hiza don */
#define UARTTX_IDLE_MS      50u     /* kuyruk bossa bu aralikla biten kayitlari raporla */
#define CMD_BUF_LEN         12u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* xTaskDelayUntil FreeRTOS 10.4.0'da geldi; projedeki surum 10.3.1 */
#if (tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR < 4)
#define xTaskDelayUntil(prev, inc)  vTaskDelayUntil((prev), (inc))
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/* Statik task bellekleri */
static StaticTask_t telTcb;
static StackType_t  telStack[TEL_STACK_WORDS];
static StaticTask_t btnTcb;
static StackType_t  btnStack[BTN_STACK_WORDS];
static StaticTask_t uartTxTcb;
static StackType_t  uartTxStack[UARTTX_STACK_WORDS];

/* Statik kuyruk bellekleri */
static StaticQueue_t txQCb;
static uint8_t       txQStorage[TXQ_LEN * sizeof(TxMsg)];
static StaticQueue_t buttonQCb;
static uint8_t       buttonQStorage[BUTTONQ_LEN * sizeof(ButtonEvent)];

static TaskHandle_t  telTaskHandle;
static TaskHandle_t  btnTaskHandle;
static TaskHandle_t  uartTxTaskHandle;
static QueueHandle_t txQ;
static QueueHandle_t buttonQ;

/* Olcum senaryolari */
static const ScenarioCfg kScenario[SCN_COUNT] =
{
  /* period_ms, work_ms */
  {   0u, 0u },   /* S0: telemetri kapali (task bloklu) - referans */
  { 100u, 0u },   /* S1: 10 Hz                                     */
  {  20u, 0u },   /* S2: 50 Hz                                     */
  {  10u, 0u },   /* S3: 100 Hz                                    */
  {  10u, 2u },   /* S4: 100 Hz + ~2 ms CPU isi                    */
  {  10u, 5u },   /* S5: 100 Hz + ~5 ms CPU isi                    */
};
static volatile uint8_t g_scn = SCN_DEFAULT;   /* sadece UartTxTask yazar */

static uint32_t          workItersPerMs;       /* calibrate_work() sonucu      */
static uint32_t          workCheckUs[2];       /* acilis dogrulamasi: 2 ms ve 5 ms isin olculen suresi */
static BtnRecord         btnRec[BTN_REC_SLOTS];
static volatile uint32_t btnNextId;            /* sadece buton ISR'i yazar     */
static uint32_t          reportNextId;         /* sadece UartTxTask kullanir   */
static volatile uint32_t uartTxT4;             /* TC callback'inde alinan t4   */
static char              txBuf[128];           /* IT gonderimi bitene kadar sabit kalir */

/* Desteklenen UART hizlari (USART2 = 80 MHz, 16x ornekleme: hepsinde hata < %0.2) */
static const uint32_t kBaud[] = { 9600u, 19200u, 57600u, 115200u, 230400u, 460800u, 921600u };
static uint32_t   baudPrev;                     /* onay bekleyen degisimden onceki hiz */
static uint8_t    baudPending;                  /* 1: yeni hiz henuz onaylanmadi       */
static TickType_t baudDeadline;

volatile RtStats g_rt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void TelemetriTask(void *argument);
static void ButtonTask(void *argument);
static void UartTxTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* TIM2: 80 MHz / (79+1) = 1 MHz, 32-bit, yukari sayan serbest sayac.
 * - ISR-guvenli: tek bir 32-bit yazmac okumasi (atomik); paylasilan degisken,
 *   kilit veya HAL cagrisi yok -> herhangi bir oncelikten cagrilabilir.
 * - Monoton: sayac hic durmaz/geri gitmez; 2^32 us'de (~71.6 dk) sarar.
 *   Sureler HER ZAMAN (uint32_t)(yeni - eski) ile hesaplanir; bu modulo-2^32
 *   aritmetigi sarma aninda bile ~71 dk'dan kisa araliklari dogru verir. */
static inline uint32_t timer_us(void)
{
  return TIM2->CNT;
}

/* Bilinen sure kadar CPU harcar (yuk simulasyonu) */
static void calibrated_work(uint32_t iterations)
{
  for (volatile uint32_t i = 0; i < iterations; i++)
  {
  }
}

/* Scheduler'dan once: 1 ms'lik is icin gereken iterasyon sayisini olcer,
 * sonra 2 ms ve 5 ms'lik isi gercekten calistirip suresini dogrular (UART'a "# CAL"). */
static void calibrate_work(void)
{
  const uint32_t probe = 50000u;             /* ~7 ms: 1 us cozunurlukte < %0.02 hata */
  uint32_t best = UINT32_MAX;

  for (int i = 0; i < 3; i++)   /* tick kesmesi karismasin diye en kisasi alinir */
  {
    uint32_t t = timer_us();
    calibrated_work(probe);
    uint32_t d = timer_us() - t;
    if (d < best)
    {
      best = d;
    }
  }
  if (best == 0u)
  {
    best = 1u;
  }
  workItersPerMs = (uint32_t)(((uint64_t)probe * 1000u) / best);

  static const uint8_t check_ms[2] = { 2u, 5u };
  for (int i = 0; i < 2; i++)
  {
    uint32_t t = timer_us();
    calibrated_work(workItersPerMs * check_ms[i]);
    workCheckUs[i] = timer_us() - t;
  }
}

static uint32_t work_iters(uint32_t work_ms)
{
  return workItersPerMs * work_ms;
}

static TxMsg make_telemetry(uint32_t t_release_us)
{
  static uint32_t sample;
  TxMsg m = {0};
  uint32_t exec = timer_us() - t_release_us;   /* ek is olmasa da sifir degil */

  if (exec > g_rt.tel_max_exec_us)
  {
    g_rt.tel_max_exec_us = exec;
  }

  m.src           = TEL;
  m.scn           = g_scn;
  m.id            = sample;
  m.t_event_us    = t_release_us;
  m.u.tel.sample  = sample;
  m.u.tel.temp_dC = (int16_t)(240 + (int32_t)(sample % 21u));   /* 24.0 .. 26.0 C */
  m.u.tel.vbat_mV = (uint16_t)(3300u - (sample % 50u));
  m.u.tel.exec_us = exec;
  sample++;
  return m;
}

static void count_tx_drop(TxSource src)
{
  if (src == TEL)
  {
    g_rt.tel_drop++;
  }
  else
  {
    g_rt.btn_txq_drop++;
  }
}

static inline BtnRecord *rec_slot(uint32_t id)
{
  return &btnRec[id % BTN_REC_SLOTS];
}

/* ISR: kayit zincirini t0 ile acar */
static void claim_record(ButtonEvent e)
{
  BtnRecord *r = rec_slot(e.id);
  r->id    = e.id;
  r->scn   = g_scn;
  r->t0    = e.t0_us;
  r->t1    = 0u;
  r->t2    = 0u;
  r->t3    = 0u;
  r->t4    = 0u;
  r->state = REC_PENDING;
}

/* Slot daha yeni bir olay tarafindan alindiysa yazma: o kayit LOST raporlanir */
static void begin_record(ButtonEvent e, uint32_t t1)
{
  BtnRecord *r = rec_slot(e.id);
  if (r->id == e.id)
  {
    r->t1    = t1;
    r->state = REC_ACTIVE;
  }
}

static TxMsg make_button_reply(uint32_t id)
{
  BtnRecord *r = rec_slot(id);
  TxMsg m = {0};
  m.src        = BTN;
  m.scn        = r->scn;
  m.id         = id;
  m.t_event_us = r->t0;
  return m;
}

static void record_t2(uint32_t id, uint32_t t2)
{
  BtnRecord *r = rec_slot(id);
  if (r->id == id)
  {
    r->t2    = t2;
    r->state = REC_QUEUED;
  }
}

static void mark_drop(uint32_t id)
{
  BtnRecord *r = rec_slot(id);
  if (r->id == id)
  {
    r->state = REC_DROPQ;
  }
  count_tx_drop(BTN);
}

/* Kesmeli gonderim. t3: baslatma cagrisindan hemen once; t4: TC callback'inde.
 * Gonderim surerken task BLOKLU bekler (CPU'yu digerlerine birakir). */
static TxResult uart_tx(const char *s, int len, uint32_t *t3, uint32_t *t4)
{
  if ((len <= 0) || (len >= (int)sizeof(txBuf)))
  {
    g_rt.tx_err++;
    return TX_ERR;
  }

  (void)ulTaskNotifyTake(pdTRUE, 0);          /* eski bildirim kalmasin */
  *t3 = timer_us();                           /* t3 */
  if (HAL_UART_Transmit_IT(&huart2, (const uint8_t *)s, (uint16_t)len) != HAL_OK)
  {
    g_rt.tx_err++;
    return TX_ERR;
  }
  /* 1 bayt = 10 bit (8N1): beklenen hat suresi + pay */
  const uint32_t tmo_ms = ((uint32_t)len * 10000u) / huart2.Init.BaudRate + UART_TX_MARGIN_MS;
  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(tmo_ms)) == 0u)
  {
    (void)HAL_UART_AbortTransmit(&huart2);
    g_rt.tx_tmo++;
    return TX_TMO;
  }
  *t4 = uartTxT4;                             /* t4 */
  return TX_OK;
}

static TxResult uart_send(int len)
{
  uint32_t t3, t4;
  return uart_tx(txBuf, len, &t3, &t4);
}

static void send_scn_line(void)
{
  const uint8_t scn = g_scn;
  int len = snprintf(txBuf, sizeof(txBuf), "SCN,%u,%u,%u,%lu,%lu\n",
                     (unsigned)scn, (unsigned)kScenario[scn].period_ms,
                     (unsigned)kScenario[scn].work_ms, (unsigned long)btnNextId,
                     (unsigned long)huart2.Init.BaudRate);
  (void)uart_send(len);
}

static int baud_supported(uint32_t baud)
{
  for (unsigned i = 0; i < sizeof(kBaud) / sizeof(kBaud[0]); i++)
  {
    if (kBaud[i] == baud) return 1;
  }
  return 0;
}

/* USART2'yi yeni hizla yeniden yapilandirir. TX bosken cagrilir (UartTxTask). */
static void uart_set_baud(uint32_t baud)
{
  HAL_NVIC_DisableIRQ(USART2_IRQn);
  huart2.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart2) != HAL_OK)       /* MspInit tekrar cagrilmaz: sadece BRR/CR */
  {
    Error_Handler();
  }
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
  SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);
  HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* Onay satiri ESKI hizda gider, sonra hiz degisir. PC yeni hizda "?" gondermezse
 * baud_watchdog() BAUD_CONFIRM_MS sonra eski hiza doner. */
static void change_baud(uint32_t baud)
{
  const uint32_t old = huart2.Init.BaudRate;
  int len = snprintf(txBuf, sizeof(txBuf), "BAUD,%lu,%lu\n", (unsigned long)baud, (unsigned long)old);
  (void)uart_send(len);                       /* TC'ye kadar bekler: son bit hatta */
  vTaskDelay(pdMS_TO_TICKS(20));              /* ST-LINK'in satiri USB'ye aktarmasi icin */
  if (baud == old)
  {
    return;
  }
  uart_set_baud(baud);
  baudPrev     = old;
  baudPending  = 1u;
  baudDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(BAUD_CONFIRM_MS);
}

static void baud_watchdog(void)
{
  if (baudPending && ((int32_t)(xTaskGetTickCount() - baudDeadline) >= 0))
  {
    baudPending = 0u;
    uart_set_baud(baudPrev);                  /* PC yeni hiza gelemedi: geri don */
  }
}

/* Senaryo/hiz degisimi yalnizca burada (UartTxTask) yapilir. */
static void apply_ctl(uint8_t cmd, uint32_t baud)
{
  if (cmd == CTL_BAUD)
  {
    change_baud(baud);
    return;
  }
  if (cmd == CTL_QUERY)
  {
    baudPending = 0u;                         /* yeni hizda komut geldi: hiz onaylandi */
  }
  if (cmd < SCN_COUNT)
  {
    taskENTER_CRITICAL();
    g_scn = cmd;
    memset((void *)&g_rt, 0, sizeof(g_rt));
    taskEXIT_CRITICAL();
    xTaskNotifyGive(telTaskHandle);           /* S0'da bloklu olabilir: uyandir */
  }
  send_scn_line();
}

static const char *rec_status_str(RecState s)
{
  switch (s)
  {
    case REC_OK:    return "OK";
    case REC_DROPI: return "DROPI";
    case REC_DROPQ: return "DROPQ";
    case REC_TXERR: return "TXERR";
    case REC_TMO:   return "TMO";
    default:        return "LOST";
  }
}

/* Biten buton kayitlarini kimlik sirasiyla REC satiri olarak raporlar.
 * Hala yolda olan bir kayitta durur; boylece her kimlik tam bir kez raporlanir. */
static void report_records(void)
{
  for (;;)
  {
    BtnRecord r;
    uint32_t  issued;

    taskENTER_CRITICAL();
    issued = btnNextId;
    r      = *rec_slot(reportNextId);
    taskEXIT_CRITICAL();

    if (reportNextId == issued)
    {
      return;
    }

    RecState st;
    if (r.id != reportNextId)
    {
      st = REC_PENDING;                       /* slot ustune yazilmis -> LOST */
      r.scn = g_scn;
      r.t0 = r.t1 = r.t2 = r.t3 = r.t4 = 0u;
      g_rt.rec_lost++;
    }
    else if ((r.state == REC_PENDING) || (r.state == REC_ACTIVE) || (r.state == REC_QUEUED))
    {
      return;                                 /* hala yolda */
    }
    else
    {
      st = r.state;
    }

    int len = snprintf(txBuf, sizeof(txBuf), "REC,%u,%lu,%s,%lu,%lu,%lu,%lu,%lu\n",
                       (unsigned)r.scn, (unsigned long)reportNextId, rec_status_str(st),
                       (unsigned long)r.t0, (unsigned long)r.t1, (unsigned long)r.t2,
                       (unsigned long)r.t3, (unsigned long)r.t4);
    (void)uart_send(len);
    reportNextId++;
  }
}

/* Yanit mesaji: olculen gonderim budur (t3..t4) */
static void handle_button_reply(const TxMsg *m)
{
  uint32_t t3 = 0u, t4 = 0u;
  int len = snprintf(txBuf, sizeof(txBuf), "BTN,%u,%lu\n",
                     (unsigned)m->scn, (unsigned long)m->id);
  TxResult res = uart_tx(txBuf, len, &t3, &t4);

  taskENTER_CRITICAL();
  BtnRecord *r = rec_slot(m->id);
  if (r->id == m->id)
  {
    r->t3 = t3;
    if (res == TX_OK)
    {
      r->t4    = t4;
      r->state = REC_OK;
    }
    else
    {
      r->state = (res == TX_TMO) ? REC_TMO : REC_TXERR;
    }
  }
  taskEXIT_CRITICAL();

  if (res == TX_OK)
  {
    uint32_t resp = t4 - m->t_event_us;
    g_rt.btn_ok++;
    if (resp > g_rt.btn_max_us)
    {
      g_rt.btn_max_us = resp;
    }
    if (resp > BTN_DEADLINE_US)
    {
      g_rt.btn_deadline_miss++;
    }
  }
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);   /* TIM2: 1 MHz serbest sayac (zaman damgasi) */
  calibrate_work();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  txQ     = xQueueCreateStatic(TXQ_LEN, sizeof(TxMsg), txQStorage, &txQCb);
  buttonQ = xQueueCreateStatic(BUTTONQ_LEN, sizeof(ButtonEvent), buttonQStorage, &buttonQCb);
  configASSERT(txQ != NULL);
  configASSERT(buttonQ != NULL);
  vQueueAddToRegistry(txQ, "txQ");
  vQueueAddToRegistry(buttonQ, "buttonQ");

  /* ...FromISR API'si cagiran IRQ'larin onceligi sayisal olarak
   * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5) veya daha buyuk olmali. */
  configASSERT(NVIC_GetPriority(BUTTON_EXTI_IRQn) >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  configASSERT(NVIC_GetPriority(USART2_IRQn) >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

  __HAL_GPIO_EXTI_CLEAR_IT(BUTTON_Pin);       /* acilistaki eski kenarlari unut */
  HAL_NVIC_ClearPendingIRQ(BUTTON_EXTI_IRQn);
  HAL_NVIC_EnableIRQ(BUTTON_EXTI_IRQn);

  /* Komut alimi: RXNE kesmesi (uart_rx_irq). TX tarafini HAL IT yonetir. */
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
  SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  telTaskHandle    = xTaskCreateStatic(TelemetriTask, "TelemetriTask", TEL_STACK_WORDS,
                                       NULL, TEL_TASK_PRIO, telStack, &telTcb);
  btnTaskHandle    = xTaskCreateStatic(ButtonTask, "ButtonTask", BTN_STACK_WORDS,
                                       NULL, BTN_TASK_PRIO, btnStack, &btnTcb);
  uartTxTaskHandle = xTaskCreateStatic(UartTxTask, "UartTxTask", UARTTX_STACK_WORDS,
                                       NULL, UARTTX_TASK_PRIO, uartTxStack, &uartTxTcb);
  configASSERT(telTaskHandle != NULL);
  configASSERT(btnTaskHandle != NULL);
  configASSERT(uartTxTaskHandle != NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* buttonQ kurulana kadar buton kesmesi kapali; RTOS_QUEUES'te acilir */
  HAL_NVIC_DisableIRQ(BUTTON_EXTI_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ---------------------- Buton ISR (EXTI13, sadece basis kenari) ---------------------- */
/* Asagidaki statik durumlarin tamami yalnizca button_irq() icinden degisir; bu IRQ
 * kendini kesemedigi icin kilit/kritik bolge gerekmez. ISR icinde bekleme ve UART yok. */

static inline void clear_button_irq_flag(void)
{
  __HAL_GPIO_EXTI_CLEAR_IT(BUTTON_Pin);
}

/* Ilk kenar her zaman kabul edilir (ayri ilk-olay durumu). Sonrasinda son KABUL
 * edilen kenardan itibaren 30 ms icinde gelen kenarlar sayilip atilir. */
static int accept_edge(uint32_t now)
{
  static uint8_t  have_first;       /* 0: henuz hic kenar kabul edilmedi */
  static uint32_t last_accept_us;

  if (have_first && ((uint32_t)(now - last_accept_us) < BTN_REFILTER_US))
  {
    g_rt.btn_bounce++;
    return 0;
  }

  have_first     = 1u;
  last_accept_us = now;
  g_rt.btn_accepted++;
  return 1;
}

static uint32_t next_id(void)
{
  return btnNextId++;
}

static void count_button_drop(uint32_t id)
{
  g_rt.btn_isr_drop++;
  rec_slot(id)->state = REC_DROPI;
}

/* stm32l4xx_it.c -> EXTI15_10_IRQHandler (USER CODE) icinden cagrilir */
void button_irq(void)
{
  const uint32_t now = timer_us();          /* t0: once zaman, sonra her sey */
  clear_button_irq_flag();
  if (!accept_edge(now)) return;

  ButtonEvent e = { next_id(), now };
  claim_record(e);                          /* olcum zincirini ac */
  BaseType_t wake = pdFALSE;
  if (xQueueSendFromISR(buttonQ,
                        &e, &wake) != pdPASS)
    count_button_drop(e.id);
  portYIELD_FROM_ISR(wake);
}

/* ------------------------ USART2: komut alimi ve TX tamamlanmasi ------------------------ */
/* stm32l4xx_it.c -> USART2_IRQHandler (USER CODE, HAL'dan once) icinden cagrilir.
 * Komutlar: "S0".."S5" -> senaryo, "?" -> durum, "B<baud>" -> hiz ('\n' ile biter).
 * Isleme UartTxTask'ta. */
void uart_rx_irq(void)
{
  static char    line[CMD_BUF_LEN];
  static uint8_t n;
  static uint8_t overflow;
  const uint32_t isr = huart2.Instance->ISR;

  if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0u)
  {
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    g_rt.rx_err++;
  }
  if ((isr & USART_ISR_RXNE) == 0u)
  {
    return;
  }

  const char c = (char)(huart2.Instance->RDR & 0xFFu);   /* okumak RXNE'yi temizler */
  if (c == '\r')
  {
    return;
  }
  if (c != '\n')
  {
    if (n < (CMD_BUF_LEN - 1u)) line[n++] = c;
    else                        overflow = 1u;
    return;
  }

  uint8_t  cmd  = 0xFEu;                     /* gecersiz */
  uint32_t baud = 0u;
  if (!overflow)
  {
    if ((n >= 5u) && (line[0] == 'B'))
    {
      for (uint8_t i = 1u; i < n; i++)
      {
        if ((line[i] < '0') || (line[i] > '9')) { baud = 0u; break; }
        baud = baud * 10u + (uint32_t)(line[i] - '0');
      }
      if (baud_supported(baud)) cmd = CTL_BAUD;
    }
    if ((n == 2u) && (line[0] == 'S') && (line[1] >= '0') && (line[1] < (char)('0' + SCN_COUNT)))
    {
      cmd = (uint8_t)(line[1] - '0');
    }
    else if ((n == 1u) && (line[0] == '?'))
    {
      cmd = CTL_QUERY;
    }
  }
  n = 0u;
  overflow = 0u;

  if (cmd == 0xFEu)
  {
    g_rt.cmd_bad++;
    return;
  }

  TxMsg m = {0};
  m.src       = CTL;
  m.u.ctl.cmd  = cmd;
  m.u.ctl.baud = baud;
  BaseType_t wake = pdFALSE;
  (void)xQueueSendToFrontFromISR(txQ, &m, &wake);
  portYIELD_FROM_ISR(wake);
}

/* t4: TC kesmesi isleniyor (son bit hattan ciktiktan sonra) */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }
  uartTxT4 = timer_us();
  BaseType_t wake = pdFALSE;
  vTaskNotifyGiveFromISR(uartTxTaskHandle, &wake);
  portYIELD_FROM_ISR(wake);
}

/* HAL, ORE gibi hatalarda RX kesmesini kapatir; komut alimi icin geri ac */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }
  g_rt.rx_err++;
  SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);
}

/* ------------------------------------- Tasklar ------------------------------------- */

/* Yuksek oncelik: senaryoya gore periyodik veri uretir, txQ'ya birakir. UART kullanmaz.
 * S0: bildirim gelene kadar BLOKLU bekler (bos dongude donmez). */
static void TelemetriTask(void *argument)
{
  (void)argument;
  TickType_t last        = xTaskGetTickCount();
  uint8_t    active      = 0xFFu;
  uint32_t   prevRelease = 0u;
  int        first       = 1;

  for (;;)
  {
    const uint8_t      scn = g_scn;
    const ScenarioCfg *cfg = &kScenario[scn];

    if (cfg->period_ms == 0u)
    {
      active = 0xFFu;
      (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      continue;
    }
    if (scn != active)                  /* yeni senaryo: periyodu simdiden baslat */
    {
      active = scn;
      last   = xTaskGetTickCount();
      first  = 1;
    }

    uint32_t tRelease = timer_us();
    if (!first)
    {
      const uint32_t period_us = (uint32_t)cfg->period_ms * 1000u;
      const uint32_t p   = tRelease - prevRelease;
      const uint32_t jit = (p > period_us) ? (p - period_us) : (period_us - p);
      if (jit > g_rt.tel_max_jitter_us)
      {
        g_rt.tel_max_jitter_us = jit;
      }
    }
    first       = 0;
    prevRelease = tRelease;

    calibrated_work(work_iters(cfg->work_ms));
    TxMsg m = make_telemetry(tRelease);
    if (xQueueSend(txQ, &m, 0) != pdPASS)
      count_tx_drop(TEL);
    xTaskDelayUntil(&last, pdMS_TO_TICKS(cfg->period_ms));
  }
}

/* Orta oncelik: ISR olayini alir, cevabi txQ'ya birakir. UART kullanmaz. */
static void ButtonTask(void *argument)
{
  (void)argument;
  ButtonEvent e;

  for (;;)
  {
    xQueueReceive(buttonQ, &e, portMAX_DELAY);
    begin_record(e, timer_us());              /* t1 */
    TxMsg m = make_button_reply(e.id);
    record_t2(e.id, timer_us());              /* t2 */
    if (xQueueSend(txQ, &m, 0) != pdPASS)
      mark_drop(e.id);
  }
}

/* En dusuk oncelik: txQ'yu FIFO sirasiyla tuketir, UART'in tek sahibi. */
static void UartTxTask(void *argument)
{
  (void)argument;
  TxMsg m;
  int   len;

  len = snprintf(txBuf, sizeof(txBuf), "# Hafta-01 olcum: TIM2=1MHz DL=%luus filtre=%luus\n",
                 (unsigned long)BTN_DEADLINE_US, (unsigned long)BTN_REFILTER_US);
  (void)uart_send(len);
  /* Ek isin gercek suresi (acilista olculdu): 2 ms -> ~2000 us, 5 ms -> ~5000 us olmali */
  len = snprintf(txBuf, sizeof(txBuf), "# CAL iters_per_ms=%lu work2ms=%luus work5ms=%luus\n",
                 (unsigned long)workItersPerMs, (unsigned long)workCheckUs[0],
                 (unsigned long)workCheckUs[1]);
  (void)uart_send(len);
  send_scn_line();

  for (;;)
  {
    if (xQueueReceive(txQ, &m, pdMS_TO_TICKS(UARTTX_IDLE_MS)) == pdPASS)
    {
      switch (m.src)
      {
        case BTN:
          handle_button_reply(&m);
          break;

        case TEL:
          len = snprintf(txBuf, sizeof(txBuf), "TEL,%u,%lu,%d,%u,%lu\n",
                         (unsigned)m.scn, (unsigned long)m.u.tel.sample,
                         (int)m.u.tel.temp_dC, (unsigned)m.u.tel.vbat_mV,
                         (unsigned long)m.u.tel.exec_us);
          if (uart_send(len) == TX_OK)
          {
            g_rt.tel_sent++;
          }
          break;

        case CTL:
          apply_ctl(m.u.ctl.cmd, m.u.ctl.baud);
          break;

        default:
          break;
      }
    }
    report_records();
    baud_watchdog();
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
