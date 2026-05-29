/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
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
#include "platform.h"
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"

/* USER CODE BEGIN Includes */
#include "subghz.h"
#include "adc.h"             /* For ADC_ReadVoltage, adc_voltage */
#include "utilities.h"		/* For randr for jitter */
#include "iwdg.h"
#include "common.h"          /* bmm350_i2c_bus_recover, bmm350_i2c_fail_count */
#include "lora_keys.h"       /* LoRa_InstallPerDeviceKeys */
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
  * LEDs period value of the timer in ms
  */
#define LED_PERIOD_TIME 500

/**
  * Join switch period value of the timer in ms
  */
#define JOIN_TIME 2000

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 128kBytes device
  */
#define LORAWAN_NVM_BASE_ADDRESS                    ((void *)0x0803F000UL)

/* USER CODE BEGIN PD */
//static const char *slotStrings[] = { "1", "2", "C", "C_MC", "P", "P_MC" };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SEND_HB_MIN					5 //90
#define APP_HB_DUTYCYCLE			SEND_HB_MIN * 60000
#define APP_WDG_DUTYCYCLE			15000
#define CHECK_TIME					12 * 60 * 60000
#define CALIBRATION_SAMPLES			1 //5
#define OCC_CONFIRM_READS			2   /* Extra confirmation reads for occupied (total 3 with initial) */
#define UNOCC_CONFIRM_READS			1   /* Extra confirmation reads for empty (total 2 with initial) */
#define CONFIRM_PERIOD_MS			10000 /* ms between confirmation reads — MCU sleeps (STOP2) between */
#define SLOT_EMPTY					(uint8_t)(0x00)
#define SLOT_OCCUPIED				(uint8_t)(0x01)
#define THRESHOLD_STEP_SIZE			0.5f
#define BATTERY_LOW_THRESHOLD		1.4f   /* Volts — send alert when battery drops below this */
#define BATTERY_LOW_STATUS			(uint8_t)(0x09)
#define TEMPERATURE_HIGH_THRESHOLD	60.0f   /* degC — send alert when temperature exceeds this */
#define TEMPERATURE_HIGH_STATUS		(uint8_t)(0x0A)
#define RETRANSMIT_DELAY_MS			15000
#define STATE_CHANGE_TX_REPEATS		2			/* extra copies after the first → 3 total sends per state change (unconfirmed redundancy) */
#define STATE_CHANGE_REPEAT_GAP_MS	4000		/* spacing between redundant state-change copies */
#define JITTER_MS					60000		/* jitter in milliseconds — wider window de-clusters the fleet */
#define JITTER_RETRANSMISSION		2000		/* jitter for retransmission */
#define THREE_HOUR_TIME				3 * 60 * 60000
#define MAX_JOIN_ATTEMPTS			30
#define NVM_STORE_FCNT_INTERVAL		20		/* persist LoRaWAN context to flash at most once per N uplinks (flash-wear protection) */

typedef enum {
  DETECT_IDLE = 0,
  DETECT_CONFIRMING_OCC,
  DETECT_CONFIRMING_UNOCC
} DetectState_t;
/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief callback when LoRaWAN application has sent a frame
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRaWAN application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/**
  * @brief callback when LoRaWAN Beacon status is updated
  * @param params status of Last Beacon
  */
static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params);

/**
  * @brief callback when system time has been updated
  */
static void OnSysTimeUpdate(void);

/**
  * @brief callback when LoRaWAN application Class is changed
  * @param deviceClass new class
  */
static void OnClassChange(DeviceClass_t deviceClass);

/**
  * @brief  LoRa store context in Non Volatile Memory
  */
static void StoreContext(void);

/**
  * @brief  stop current LoRa execution to switch into non default Activation mode
  */
static void StopJoin(void);

/**
  * @brief  Join switch timer callback function
  * @param  context ptr of Join switch context
  */
static void OnStopJoinTimerEvent(void *context);

/**
  * @brief  Notifies the upper layer that the NVM context has changed
  * @param  state Indicates if we are storing (true) or restoring (false) the NVM context
  */
static void OnNvmDataChange(LmHandlerNvmContextStates_t state);

/**
  * @brief  Store the NVM Data context to the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were stored
  */
static void OnStoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * @brief  Restore the NVM Data context from the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were restored
  */
static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * Will be called each time a Radio IRQ is handled by the MAC layer
  *
  */
static void OnMacProcessNotify(void);

/**
  * @brief Change the periodicity of the uplink frames
  * @param periodicity uplink frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnTxPeriodicityChanged(uint32_t periodicity);

/**
  * @brief Change the confirmation control of the uplink frames
  * @param isTxConfirmed Indicates if the uplink requires an acknowledgement
  * @note Compliance test protocol callbacks
  */
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);

/**
  * @brief Change the periodicity of the ping slot frames
  * @param pingSlotPeriodicity ping slot frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/**
  * @brief Will be called to reset the system
  * @note Compliance test protocol callbacks
  */
static void OnSystemReset(void);

/* USER CODE BEGIN PFP */
static void OnConfirmTimerEvent(void *context);
static void OnHeartBeatTimerEvent(void *context);
static void OnBatteryCheckTimerEvent(void *context);
static void OnTemperatureCheckTimerEvent(void *context);
static void OnRetransmitTimerEvent(void *context);
static void JoinTimerEvent(void *context);
static void ConfirmDetection(void);
static void SendStateChange(uint8_t statusByte);
static void SendHeartBeat(void);
static void CheckBatteryVoltage(void);
static void CheckTemperature(void);
static float ComputeMagnitude(BMM350_data_t *reading);
static UTIL_TIMER_Time_t JitterPeriod(UTIL_TIMER_Time_t base, uint32_t jitter);
static void DoRetransmit(void);
static void join_task(void);
static void WDG_Timer_task(void);
static void OnWDGTimerEvent(void *context);
/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief LoRaWAN default activation type
  */
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN force rejoin even if the NVM context is restored
  */
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =              GetBatteryLevel,
  .GetTemperature =               GetTemperatureLevel,
  .GetUniqueId =                  GetUniqueId,
  .GetDevAddr =                   GetDevAddr,
  .OnRestoreContextRequest =      OnRestoreContextRequest,
  .OnStoreContextRequest =        OnStoreContextRequest,
  .OnMacProcess =                 OnMacProcessNotify,
  .OnNvmDataChange =              OnNvmDataChange,
  .OnJoinRequest =                OnJoinRequest,
  .OnTxData =                     OnTxData,
  .OnRxData =                     OnRxData,
  .OnBeaconStatusChange =         OnBeaconStatusChange,
  .OnSysTimeUpdate =              OnSysTimeUpdate,
  .OnClassChange =                OnClassChange,
  .OnTxPeriodicityChanged =       OnTxPeriodicityChanged,
  .OnTxFrameCtrlChanged =         OnTxFrameCtrlChanged,
  .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
  .OnSystemReset =                OnSystemReset,
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .IsTxConfirmed =            LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .TxPower =                  LORAWAN_DEFAULT_TX_POWER,
  .PingSlotPeriodicity =      LORAWAN_DEFAULT_PING_SLOT_PERIODICITY,
  .RxBCTimeout =              LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT
};

/**
  * @brief Type of Event to generate application Tx
  */
static TxEventType_t EventType = TX_ON_TIMER;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/**
  * @brief Tx Timer period
  */
static UTIL_TIMER_Time_t TxPeriodicity = APP_TX_DUTYCYCLE;

/**
  * @brief Join Timer period
  */
static UTIL_TIMER_Object_t StopJoinTimer;

/* USER CODE BEGIN PV */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

static UTIL_TIMER_Time_t HBPeriodicity = APP_HB_DUTYCYCLE;
static UTIL_TIMER_Object_t ConfirmTimer;      /* Timer for confirmation reads (200ms, oneshot) */
static UTIL_TIMER_Object_t SendHeartBeatTimer;
static UTIL_TIMER_Object_t BatteryCheckTimer; /* Timer for periodic battery voltage check */
static UTIL_TIMER_Object_t TemperatureCheckTimer; /* Timer for periodic temperature check */
static UTIL_TIMER_Object_t RetransmitTimer;    /* Oneshot timer for 15 s retransmission after status change */
static UTIL_TIMER_Object_t JoinTimer;
static UTIL_TIMER_Object_t WDGTimer;
static UTIL_TIMER_Time_t WDGPeriodicity = APP_WDG_DUTYCYCLE;

BMM350_data_t BMM350_CALIBRATION_DATA[CALIBRATION_SAMPLES];
BMM350_data_t BMM350_CALIBRATED_BASE;
int calibration_index = 0;
bool is_Calibration_done = false;
bool isJoined = false;
int join_attempts = 0;
float THRESHOLD = 4.5f;
bool parked = false;

/* State machine for confirmation reads */
static DetectState_t detect_state = DETECT_IDLE;
static int confirm_count = 0;

/* Battery monitoring — send alert once when voltage drops to threshold */
static volatile bool battery_low_sent = false;

/* Temperature monitoring — send alert once when temperature exceeds threshold */
static volatile bool temperature_high_sent = false;

static volatile bool retransmit_pending = false;
static uint8_t retransmit_status = 0;
static volatile uint8_t retransmit_count = 0;   /* redundant state-change copies still to send */

/* Uplink counter value at the last NVM context store — used to throttle flash
 * writes to once per NVM_STORE_FCNT_INTERVAL uplinks (see OnTxData). */
static uint32_t last_stored_fcnt = 0;
/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_LV */

  /* USER CODE END LoRaWAN_Init_LV */

  /* USER CODE BEGIN LoRaWAN_Init_1 */
	if (FLASH_IF_Init(NULL) != FLASH_IF_OK)
	{
		Error_Handler();
	}
  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_TIMER_Create(&StopJoinTimer, JOIN_TIME, UTIL_TIMER_ONESHOT, OnStopJoinTimerEvent, NULL);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU, StoreContext);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), UTIL_SEQ_RFU, StopJoin);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaReadBMM350Event), UTIL_SEQ_RFU, ConfirmDetection);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SendHBEvent), UTIL_SEQ_RFU, SendHeartBeat);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_BatteryCheckEvent), UTIL_SEQ_RFU, CheckBatteryVoltage);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_TemperatureCheckEvent), UTIL_SEQ_RFU, CheckTemperature);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_RetransmitEvent), UTIL_SEQ_RFU, DoRetransmit);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_JoinEvent), UTIL_SEQ_RFU, join_task);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_WDGTimerEvent), UTIL_SEQ_RFU, WDG_Timer_task);

  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

  LmHandlerConfigure(&LmHandlerParams);

  /* Override the hardcoded test keys from se-identity.h with per-device keys
   * derived from the MCU UID + PROJECT_SECRET. Must run before LmHandlerJoin(). */
  LoRa_InstallPerDeviceKeys();

  /* USER CODE BEGIN LoRaWAN_Init_2 */
  uint8_t standby_cfg = 0x00; /* STDBY_RC */
  uint8_t hse_in = 0x06;
  uint8_t hse_out = 0x0C;

  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &standby_cfg, 1);
  HAL_SUBGHZ_WriteRegisters(&hsubghz, 0x0911, &hse_in, 1);
  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &standby_cfg, 1);
  HAL_SUBGHZ_WriteRegisters(&hsubghz, 0x0912, &hse_out, 1);
  HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, &standby_cfg, 1);
  /* USER CODE END LoRaWAN_Init_2 */

  /* Per-device boot delay (0..60s) derived from the low 16 bits of MCU UID.
   * Deterministic, spreads 150 nodes across a 60s window so a bulk power-on
   * doesn't produce a synchronised join storm on the gateway. */
  uint32_t boot_join_delay_ms = (HAL_GetUIDw0() & 0xFFFFu) * 60000u / 65536u;
  /* Always at least 100ms so the timer actually arms. */
  if (boot_join_delay_ms < 100u) boot_join_delay_ms = 100u;

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_Create(&ConfirmTimer, CONFIRM_PERIOD_MS, UTIL_TIMER_ONESHOT, OnConfirmTimerEvent, NULL);
    UTIL_TIMER_Create(&SendHeartBeatTimer, HBPeriodicity, UTIL_TIMER_ONESHOT, OnHeartBeatTimerEvent, NULL);
    UTIL_TIMER_Create(&BatteryCheckTimer, CHECK_TIME, UTIL_TIMER_ONESHOT, OnBatteryCheckTimerEvent, NULL);
    UTIL_TIMER_Create(&TemperatureCheckTimer, CHECK_TIME, UTIL_TIMER_ONESHOT, OnTemperatureCheckTimerEvent, NULL);
    UTIL_TIMER_Create(&RetransmitTimer, RETRANSMIT_DELAY_MS, UTIL_TIMER_ONESHOT, OnRetransmitTimerEvent, NULL);
    UTIL_TIMER_Create(&JoinTimer, boot_join_delay_ms, UTIL_TIMER_ONESHOT, JoinTimerEvent, NULL);
    UTIL_TIMER_Create(&WDGTimer, WDGPeriodicity, UTIL_TIMER_PERIODIC, OnWDGTimerEvent, NULL);
    UTIL_TIMER_Start(&WDGTimer);
    /* ConfirmTimer NOT started here — only started on-demand when threshold is crossed */

    /* First join is scheduled via JoinTimer after boot_join_delay_ms, not
     * called inline. join_task() does the actual LmHandlerJoin() and handles
     * retries from there. */
    APP_LOG(TS_ON, VLEVEL_L, "First-join scheduled in %lums\r\n", (unsigned long)boot_join_delay_ms);
    UTIL_TIMER_Start(&JoinTimer);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */

    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */

  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */

#if 0 /* User should remove the #if 0 statement and adapt the below code according with his needs*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case  BUT1_Pin:
      /* Note: when "EventType == TX_ON_TIMER" this GPIO is not initialized */
      if (EventType == TX_ON_EVENT)
      {
        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
      }
      break;
    case  BUT2_Pin:
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
      break;
    case  BUT3_Pin:
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
      break;
    default:
      break;
  }
}
#endif

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */
static float ComputeMagnitude(BMM350_data_t *reading)
{
  float dx = fabs(reading->x - BMM350_CALIBRATED_BASE.x);
  float dy = fabs(reading->y - BMM350_CALIBRATED_BASE.y);
  float dz = fabs(reading->z - BMM350_CALIBRATED_BASE.z);
  return sqrtf((dx * dx) + (dy * dy) + (dz * dz));
}

/* Apply symmetric jitter to a timer period for fleet de-clustering, while
 * guaranteeing the result can never collapse to ~0. The jitter swing is capped
 * at ±50% of the base, so the period always stays within [base/2, 3*base/2].
 * (Previously a ±100% swing on the 60s poll could yield a 0ms period.) */
static UTIL_TIMER_Time_t JitterPeriod(UTIL_TIMER_Time_t base, uint32_t jitter)
{
  if (jitter > base / 2u)
  {
    jitter = base / 2u;
  }
  return base + (UTIL_TIMER_Time_t)randr(-(int32_t)jitter, (int32_t)jitter);
}

/* Threshold of consecutive BMM350 I2C failures before we attempt bus recovery. */
#define BMM350_FAIL_THRESHOLD   3

/* Wrapper around BMM350_Read() that escalates to I2C bus recovery and ultimately
 * a system reset if the sensor has gone unreachable. Returns 0 on success, <0 on failure. */
static int BMM350_Read_Safe(BMM350_data_t *reading)
{
  if (BMM350_Read(reading) == 0 && bmm350_i2c_fail_count == 0)
  {
    return 0;
  }

  /* Read failed (BMM350_Read returned <0) or some inner I2C op silently failed. */
  APP_LOG(TS_ON, VLEVEL_L, "BMM350 read fail (count=%lu)\r\n", (unsigned long)bmm350_i2c_fail_count);

  if (bmm350_i2c_fail_count >= BMM350_FAIL_THRESHOLD)
  {
    APP_LOG(TS_ON, VLEVEL_L, "BMM350: triggering I2C bus recovery\r\n");
    if (bmm350_i2c_bus_recover() != HAL_OK)
    {
      APP_LOG(TS_ON, VLEVEL_L, "BMM350: bus recovery failed, resetting\r\n");
      NVIC_SystemReset();
    }
    /* After recovery, one re-init attempt. */
    BMM350_Init();
    /* Try the read once more — if it works, the caller gets fresh data. */
    if (BMM350_Read(reading) == 0 && bmm350_i2c_fail_count == 0)
    {
      return 0;
    }
  }
  return -1;
}

/**
  * @brief  Confirmation read handler — called by timer after threshold was crossed.
  *         MCU was in STOP2 since the previous read. Reads BMM350, checks threshold,
  *         and either continues confirming, declares state change, or aborts.
  */
static void ConfirmDetection(void)
{
  BMM350_data_t reading;
  if (BMM350_Read_Safe(&reading) != 0)
  {
    /* Sensor unreachable — do NOT update the state machine on garbage data.
     * Abort the confirmation cycle; the next polling tick will retry. */
    APP_LOG(TS_ON, VLEVEL_L, "ConfirmDetection: sensor read failed, aborting cycle\r\n");
    detect_state = DETECT_IDLE;
    return;
  }
  float mag = ComputeMagnitude(&reading);

  if (detect_state == DETECT_CONFIRMING_OCC)
  {
    if (mag >= THRESHOLD)
    {
      confirm_count++;
      if (confirm_count >= OCC_CONFIRM_READS)
      {
        /* All confirmations passed — vehicle is parked */
        parked = true;
        detect_state = DETECT_IDLE;
        APP_LOG(TS_ON, VLEVEL_L, "VEHICLE DETECTED (%d/%d confirmed)\r\n",
                OCC_CONFIRM_READS + 1, OCC_CONFIRM_READS + 1);

        /* Queue redundant unconfirmed copies (1 + STATE_CHANGE_TX_REPEATS = 3 total)
         * so the event survives RF collisions / weak signal on the best-effort link.
         * Every copy is emitted through DoRetransmit(), which handles MAC-busy and
         * duty-cycle internally. */
        SendStateChange(SLOT_OCCUPIED);
      }
      else
      {
        /* Need more confirmations — restart timer, MCU goes back to STOP2 */
        UTIL_TIMER_Start(&ConfirmTimer);
      }
    }
    else
    {
      /* Confirmation failed — false alarm, back to idle */
      detect_state = DETECT_IDLE;
      APP_LOG(TS_ON, VLEVEL_L, "OCC confirmation failed at read %d\r\n", confirm_count + 1);
    }
  }
  else if (detect_state == DETECT_CONFIRMING_UNOCC)
  {
    if (mag < THRESHOLD * 0.9f)
    {
      confirm_count++;
      if (confirm_count >= UNOCC_CONFIRM_READS)
      {
        /* All confirmations passed — vehicle has left */
        parked = false;
        detect_state = DETECT_IDLE;
        APP_LOG(TS_ON, VLEVEL_L, "VEHICLE LEFT (%d/%d confirmed)\r\n",
                UNOCC_CONFIRM_READS + 1, UNOCC_CONFIRM_READS + 1);

        /* Queue redundant unconfirmed copies (3 total) — see SendStateChange(). */
        SendStateChange(SLOT_EMPTY);

        /* Slow baseline drift (EMA) */
        BMM350_CALIBRATED_BASE.x = (BMM350_CALIBRATED_BASE.x * 0.99f) + (reading.x * 0.01f);
        BMM350_CALIBRATED_BASE.y = (BMM350_CALIBRATED_BASE.y * 0.99f) + (reading.y * 0.01f);
        BMM350_CALIBRATED_BASE.z = (BMM350_CALIBRATED_BASE.z * 0.99f) + (reading.z * 0.01f);
        BMM350_CALIBRATED_BASE.temperature = (BMM350_CALIBRATED_BASE.temperature * 0.99f) + (reading.temperature * 0.01f);
      }
      else
      {
        UTIL_TIMER_Start(&ConfirmTimer);
      }
    }
    else
    {
      /* Confirmation failed — false alarm, back to idle */
      detect_state = DETECT_IDLE;
      APP_LOG(TS_ON, VLEVEL_L, "UNOCC confirmation failed at read %d\r\n", confirm_count + 1);
    }
  }
  else
  {
    /* Shouldn't happen — safety reset */
    detect_state = DETECT_IDLE;
  }
}

static void SendHeartBeat(void)
{
	 /* USER CODE BEGIN ReadBMM350 */
	UTIL_TIMER_Time_t nextTxIn = 0;
	LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
	  if (LmHandlerIsBusy() == false)
	  {
		  uint32_t k = 0;
		  AppData.Port = LORAWAN_USER_APP_PORT;
		  AppData.Buffer[k++] = parked;
		  AppData.BufferSize = k;

		  status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
		  if (LORAMAC_HANDLER_SUCCESS == status)
		  {
			  APP_LOG(TS_ON, VLEVEL_L, "SEND HEART BEAT\r\n");
		  }
		  else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
		  {
			  nextTxIn = LmHandlerGetDutyCycleWaitTime();
			  if (EventType == TX_ON_TIMER)
			  {
				  UTIL_TIMER_Stop(&SendHeartBeatTimer);
				  UTIL_TIMER_SetPeriod(&SendHeartBeatTimer, MAX(nextTxIn, HBPeriodicity));
				  UTIL_TIMER_Start(&SendHeartBeatTimer);
			  }
			  if (nextTxIn > 0)
			  {
				APP_LOG(TS_ON, VLEVEL_L, "Next HB in  : ~%d second(s)\r\n", (nextTxIn / 1000));
			  }
		  }
	  }
	 /* USER CODE END SendHeartBeat */
}

/**
  * @brief  Check battery voltage and send low-battery alert (0x09) if below threshold.
  *         Alert is sent only once per low-battery event; resets when voltage recovers.
  */
static void CheckBatteryVoltage(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
  UTIL_TIMER_Time_t nextTxIn = 0;

  float batteryVoltage = ADC_ReadBatteryVoltage();
  APP_LOG(TS_ON, VLEVEL_L, "Battery Voltage: %d.%02d V\r\n",
          (int)batteryVoltage,
          (int)((batteryVoltage - (int)batteryVoltage) * 100));

  if (batteryVoltage < BATTERY_LOW_THRESHOLD)
  {
    if (battery_low_sent == false)
    {
      /* Send battery low alert packet */
      if (LmHandlerIsBusy() == false)
      {
        uint32_t k = 0;
        AppData.Port = LORAWAN_USER_APP_PORT;
        AppData.Buffer[k++] = BATTERY_LOW_STATUS;
        AppData.BufferSize = k;

        status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
        if (LORAMAC_HANDLER_SUCCESS == status)
        {
          APP_LOG(TS_ON, VLEVEL_L, "SEND BATTERY_LOW (0x09)\r\n");
          battery_low_sent = true;
        }
        else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
        {
          nextTxIn = LmHandlerGetDutyCycleWaitTime();
          if (nextTxIn > 0)
          {
            APP_LOG(TS_ON, VLEVEL_L, "Battery low Tx delayed: ~%d second(s)\r\n", (nextTxIn / 1000));
          }
        }
      }
    }
  }
  else
  {
    /* Voltage recovered above threshold — allow re-alerting if it drops again */
    if (battery_low_sent == true)
    {
      battery_low_sent = false;
      APP_LOG(TS_ON, VLEVEL_L, "Battery voltage recovered above threshold\r\n");
    }
  }
}

/**
  * @brief  Check BMM350 temperature and send high-temperature alert (0x0A) if above threshold.
  *         Alert is sent only once per high-temperature event; resets when temperature recovers.
  */
static void CheckTemperature(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
  UTIL_TIMER_Time_t nextTxIn = 0;

  BMM350_data_t reading;
  if (BMM350_Read_Safe(&reading) != 0)
  {
    /* Sensor unreachable — `reading` is uninitialised stack data here, so do NOT
     * evaluate it (a garbage value could trip a false high-temperature alert).
     * Skip this cycle; the next TemperatureCheckTimer tick retries. */
    APP_LOG(TS_ON, VLEVEL_L, "CheckTemperature: sensor read failed, skipping\r\n");
    return;
  }
  float temperature = reading.temperature;
  APP_LOG(TS_ON, VLEVEL_L, "BMM350 Temperature: %d.%02d degC\r\n",
          (int)temperature,
          (int)((temperature - (int)temperature) * 100));

  if (temperature > TEMPERATURE_HIGH_THRESHOLD)
  {
    if (temperature_high_sent == false)
    {
      /* Send temperature high alert packet */
      if (LmHandlerIsBusy() == false)
      {
        uint32_t k = 0;
        AppData.Port = LORAWAN_USER_APP_PORT;
        AppData.Buffer[k++] = TEMPERATURE_HIGH_STATUS;
        AppData.BufferSize = k;

        status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
        if (LORAMAC_HANDLER_SUCCESS == status)
        {
          APP_LOG(TS_ON, VLEVEL_L, "SEND TEMPERATURE_HIGH (0x0A)\r\n");
          temperature_high_sent = true;
        }
        else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
        {
          nextTxIn = LmHandlerGetDutyCycleWaitTime();
          if (nextTxIn > 0)
          {
            APP_LOG(TS_ON, VLEVEL_L, "Temperature high Tx delayed: ~%d second(s)\r\n", (nextTxIn / 1000));
          }
        }
      }
    }
  }
  else
  {
    /* Temperature recovered below threshold — allow re-alerting if it rises again */
    if (temperature_high_sent == true)
    {
      temperature_high_sent = false;
      APP_LOG(TS_ON, VLEVEL_L, "Temperature recovered below threshold\r\n");
    }
  }
}

/* Begin a redundant state-change transmission: the given status byte is sent
 * 1 + STATE_CHANGE_TX_REPEATS times (3 total) on the best-effort unconfirmed link.
 * The first copy is fired immediately via the sequencer; DoRetransmit() emits each
 * copy and schedules the next one STATE_CHANGE_REPEAT_GAP_MS apart. A newer state
 * change simply overwrites the status and resets the counter (latest state wins). */
static void SendStateChange(uint8_t statusByte)
{
  retransmit_status  = statusByte;
  retransmit_count   = STATE_CHANGE_TX_REPEATS;
  retransmit_pending = true;

  UTIL_TIMER_Stop(&RetransmitTimer);
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_RetransmitEvent), CFG_SEQ_Prio_0);
}

static void DoRetransmit(void)
{
  if (!retransmit_pending)
  {
    return;  /* Already cancelled or all copies sent — nothing to do */
  }

  /* If MAC is still busy, reschedule. Do NOT clear retransmit_pending or the
   * copy counter here — that would drop the packet on the floor. */
  if (LmHandlerIsBusy() == true)
  {
    APP_LOG(TS_ON, VLEVEL_L, "RETRANSMIT deferred — MAC busy, retry shortly\r\n");
    UTIL_TIMER_Stop(&RetransmitTimer);
    UTIL_TIMER_SetPeriod(&RetransmitTimer,
        STATE_CHANGE_REPEAT_GAP_MS + randr(-JITTER_RETRANSMISSION, JITTER_RETRANSMISSION));
    UTIL_TIMER_Start(&RetransmitTimer);
    return;
  }

  AppData.Port = LORAWAN_USER_APP_PORT;
  AppData.Buffer[0] = retransmit_status;
  AppData.BufferSize = 1;

  LmHandlerErrorStatus_t status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
  if (LORAMAC_HANDLER_SUCCESS == status)
  {
    /* One copy went out. Schedule the next redundant copy, or finish. */
    if (retransmit_count > 0)
    {
      retransmit_count--;
      APP_LOG(TS_ON, VLEVEL_L, "STATE 0x%02X sent, %d copy(ies) left\r\n", retransmit_status, retransmit_count);
      UTIL_TIMER_Stop(&RetransmitTimer);
      UTIL_TIMER_SetPeriod(&RetransmitTimer,
          STATE_CHANGE_REPEAT_GAP_MS + randr(-JITTER_RETRANSMISSION, JITTER_RETRANSMISSION));
      UTIL_TIMER_Start(&RetransmitTimer);
    }
    else
    {
      retransmit_pending = false;
      APP_LOG(TS_ON, VLEVEL_L, "STATE 0x%02X sent, final copy\r\n", retransmit_status);
    }
  }
  else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
  {
    /* Send did not happen — reschedule after the duty-cycle window, keep the counter. */
    UTIL_TIMER_Time_t nextTxIn = LmHandlerGetDutyCycleWaitTime();
    UTIL_TIMER_Stop(&RetransmitTimer);
    UTIL_TIMER_SetPeriod(&RetransmitTimer,
        MAX(nextTxIn + randr(0, JITTER_RETRANSMISSION), (UTIL_TIMER_Time_t)STATE_CHANGE_REPEAT_GAP_MS));
    UTIL_TIMER_Start(&RetransmitTimer);
    APP_LOG(TS_ON, VLEVEL_L, "RETRANSMIT duty-cycle restricted, retry in ~%ds\r\n", (int)(nextTxIn / 1000));
  }
  else
  {
    /* Other error — send did not happen, retry shortly, keep the counter. */
    UTIL_TIMER_Stop(&RetransmitTimer);
    UTIL_TIMER_SetPeriod(&RetransmitTimer,
        STATE_CHANGE_REPEAT_GAP_MS + randr(-JITTER_RETRANSMISSION, JITTER_RETRANSMISSION));
    UTIL_TIMER_Start(&RetransmitTimer);
    APP_LOG(TS_ON, VLEVEL_L, "RETRANSMIT failed (status %d), rescheduled\r\n", status);
  }
}

static void join_task(void)
{

	if( LmHandlerJoinStatus( ) == LORAMAC_HANDLER_SET )
	{
		isJoined = true;
		join_attempts = 0;
	    UTIL_TIMER_Start(&TxTimer);
	    UTIL_TIMER_Start(&BatteryCheckTimer);
	    UTIL_TIMER_Start(&TemperatureCheckTimer);
		return;
	}

	LmHandlerJoin(ActivationType, ForceRejoin);
	join_attempts++;

    if (join_attempts < MAX_JOIN_ATTEMPTS)
    {
        // FAST retry — jitter ±7s so 150 nodes don't re-join in lockstep on a bulk power-on.
        UTIL_TIMER_SetPeriod(&JoinTimer, RETRANSMIT_DELAY_MS + randr(-7000, 7000));
    }
    else
    {
        // LONG delay — jitter ±5 min so the fleet doesn't return as a herd after the 3h backoff.
        join_attempts = 0;
        UTIL_TIMER_SetPeriod(&JoinTimer, THREE_HOUR_TIME + randr(-300000, 300000));
    }
	UTIL_TIMER_Start(&JoinTimer);
}

static void WDG_Timer_task(void)
{
	HAL_IWDG_Refresh(&hiwdg);
}
/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */

	if (params != NULL)
	{
	    if (params->IsMcpsIndication)
	    {
	      if (appData != NULL)
	      {
	        if (appData->Buffer != NULL)
	        {
	          switch (appData->Port)
	          {
#if defined(ENABLE_CLASS_SWITCH_PORT) && (ENABLE_CLASS_SWITCH_PORT == 1)
	            case LORAWAN_SWITCH_CLASS_PORT:
	              /* Disabled in production: a stray downlink on port 3 could move
	               * the node to Class B/C and burn battery on continuous RX. Set
	               * ENABLE_CLASS_SWITCH_PORT=1 only for bench testing. */
	              if (appData->BufferSize == 1)
	              {
	                switch (appData->Buffer[0])
	                {
	                  case 0:
	                  {
	                    LmHandlerRequestClass(CLASS_A);
	                    break;
	                  }
	                  case 1:
	                  {
	                    LmHandlerRequestClass(CLASS_B);
	                    break;
	                  }
	                  case 2:
	                  {
	                    LmHandlerRequestClass(CLASS_C);
	                    break;
	                  }
	                  default:
	                    break;
	                }
	              }
	              break;
#endif
	            case LORAWAN_USER_APP_PORT:
	              if (appData->BufferSize == 1)
	              {
	            	  if (appData->Buffer[0] == 0xCC)
	            	  {
	            		  is_Calibration_done = false;
	            	  }
	              }
	              else if(appData->BufferSize == 2)
	              {
	            	  if (appData->Buffer[0] == 0xDD)
	            	  {
	            		  THRESHOLD = appData->Buffer[1] * THRESHOLD_STEP_SIZE;
	            	  }
	              }
	              break;

	            default:

	              break;
	          }
	        }
	      }
	    }
	}
  /* USER CODE END OnRxData_1 */
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
  UTIL_TIMER_Time_t nextTxIn = 0;

  if (LmHandlerIsBusy() == false)
  {
    /* Skip if we're in the middle of confirming (timer will handle it) */
    if (detect_state != DETECT_IDLE)
    {
      return;
    }

    /* --- Read BMM350 once --- */
    BMM350_data_t reading;
    if (BMM350_Read_Safe(&reading) != 0)
    {
      /* Sensor unreachable. Skip this cycle and let the next TxTimer try again
       * (with possible bus recovery already attempted inside BMM350_Read_Safe). */
      return;
    }

    if (!is_Calibration_done)
    {
      /* --- Calibration phase --- */
      BMM350_CALIBRATION_DATA[calibration_index] = reading;
      calibration_index++;

      if (calibration_index >= CALIBRATION_SAMPLES)
      {
        BMM350_CALIBRATED_BASE.x = 0;
        BMM350_CALIBRATED_BASE.y = 0;
        BMM350_CALIBRATED_BASE.z = 0;
        BMM350_CALIBRATED_BASE.temperature = 0;

        for (int j = 0; j < CALIBRATION_SAMPLES; j++)
        {
          BMM350_CALIBRATED_BASE.x += BMM350_CALIBRATION_DATA[j].x;
          BMM350_CALIBRATED_BASE.y += BMM350_CALIBRATION_DATA[j].y;
          BMM350_CALIBRATED_BASE.z += BMM350_CALIBRATION_DATA[j].z;
          BMM350_CALIBRATED_BASE.temperature += BMM350_CALIBRATION_DATA[j].temperature;
        }

        BMM350_CALIBRATED_BASE.x /= CALIBRATION_SAMPLES;
        BMM350_CALIBRATED_BASE.y /= CALIBRATION_SAMPLES;
        BMM350_CALIBRATED_BASE.z /= CALIBRATION_SAMPLES;
        BMM350_CALIBRATED_BASE.temperature /= CALIBRATION_SAMPLES;

        is_Calibration_done = true;
        calibration_index = 0;

        /* Send calibration-done uplink */
        if (LmHandlerIsBusy() == false)
        {
          uint32_t k = 0;
          AppData.Port = LORAWAN_USER_APP_PORT;
          AppData.Buffer[k++] = parked;
          AppData.BufferSize = k;
          status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
          if (LORAMAC_HANDLER_SUCCESS == status)
          {
            APP_LOG(TS_ON, VLEVEL_L, "CALIBRATION DONE\r\n");
          }
        }
        UTIL_TIMER_Stop(&SendHeartBeatTimer);
        UTIL_TIMER_Start(&SendHeartBeatTimer);
      }
    }
    else
    {
      /* --- Detection phase --- */
      float mag = ComputeMagnitude(&reading);

      if (parked == false)
      {
        if (mag >= THRESHOLD)
        {
          /* Threshold crossed — start confirmation (MCU will STOP2 between reads) */
          detect_state = DETECT_CONFIRMING_OCC;
          confirm_count = 0;
          UTIL_TIMER_Start(&ConfirmTimer);
          APP_LOG(TS_ON, VLEVEL_L, "OCC threshold crossed, confirming...\r\n");
        }
      }
      else
      {
        if (mag < THRESHOLD * 0.9f)
        {
          /* Threshold crossed — start confirmation */
          detect_state = DETECT_CONFIRMING_UNOCC;
          confirm_count = 0;
          UTIL_TIMER_Start(&ConfirmTimer);
          APP_LOG(TS_ON, VLEVEL_L, "UNOCC threshold crossed, confirming...\r\n");
        }
      }
    }
  }
}
  /* USER CODE END SendTxData_1 */


static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */

  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/

  /* USER CODE BEGIN OnTxTimerEvent_2 */
  UTIL_TIMER_Stop(&TxTimer);
  UTIL_TIMER_SetPeriod(&TxTimer, JitterPeriod(TxPeriodicity, JITTER_MS));
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */
static void OnConfirmTimerEvent(void *context)
{
  /* Trigger confirmation read task — do NOT auto-restart, ConfirmDetection() decides */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaReadBMM350Event), CFG_SEQ_Prio_0);
}


static void OnHeartBeatTimerEvent(void *context)
{
  /* USER CODE BEGIN OnReadBMM350TimerEvent */
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SendHBEvent), CFG_SEQ_Prio_0);
  /* USER CODE END OnReadBMM350TimerEvent */

  /* USER CODE BEGIN OnReadBMM350TimerEvent */
	/*Wait for next tx slot*/
	UTIL_TIMER_Stop(&SendHeartBeatTimer);
	UTIL_TIMER_SetPeriod(&SendHeartBeatTimer, JitterPeriod(HBPeriodicity, JITTER_MS));
	UTIL_TIMER_Start(&SendHeartBeatTimer);
  /* USER CODE END OnReadBMM350TimerEvent */
}

static void OnBatteryCheckTimerEvent(void *context)
{
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_BatteryCheckEvent), CFG_SEQ_Prio_0);
  /* Restart timer for next check */
  UTIL_TIMER_Stop(&BatteryCheckTimer);
  UTIL_TIMER_SetPeriod(&BatteryCheckTimer, JitterPeriod(CHECK_TIME, JITTER_MS));
  UTIL_TIMER_Start(&BatteryCheckTimer);
}

static void OnTemperatureCheckTimerEvent(void *context)
{
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_TemperatureCheckEvent), CFG_SEQ_Prio_0);
  /* Restart timer for next check */
  UTIL_TIMER_Stop(&TemperatureCheckTimer);
  UTIL_TIMER_SetPeriod(&TemperatureCheckTimer, JitterPeriod(CHECK_TIME, JITTER_MS));
  UTIL_TIMER_Start(&TemperatureCheckTimer);
}

static void OnRetransmitTimerEvent(void *context)
{
  /* Oneshot — fire the retransmit task; DoRetransmit() decides whether to
   * reschedule for the next redundant copy and with what period. */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_RetransmitEvent), CFG_SEQ_Prio_0);
}

static void JoinTimerEvent(void *context)
{
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_JoinEvent), CFG_SEQ_Prio_0);
}

static void OnWDGTimerEvent(void *context)
{
  /* USER CODE BEGIN OnWDGTimerEvent_1 */
	//HAL_IWDG_Refresh(&hiwdg);
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_WDGTimerEvent), CFG_SEQ_Prio_0);
  /* USER CODE END OnWDGTimerEvent_1 */
}
/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  if ((params != NULL))
  {
	/* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
	if (params->IsMcpsConfirm != 0)
	{
	  APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
	  APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
			  params->AppData.Port, params->Datarate, params->TxPower);

	  APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
	  if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
	  {
		APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
	  }
	  else
	  {
		APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
	  }

	  /* Throttle NVM persistence on the high-frequency data path. The uplink
	   * frame counter changes on every TX, so storing context on every frame
	   * erases+rewrites a flash page per uplink and wears the page out within
	   * the deployment lifetime. Persist only once every NVM_STORE_FCNT_INTERVAL
	   * uplinks; the worst case on an unexpected reset is losing that many frames
	   * of counter advance (recoverable). Unsigned subtraction is wrap-safe, and
	   * a session reset after join self-corrects with one immediate store. */
	  if ((params->UplinkCounter - last_stored_fcnt) >= NVM_STORE_FCNT_INTERVAL)
	  {
		last_stored_fcnt = params->UplinkCounter;
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
	  }
	}
	else
	{
	  /* Non-MCPS (e.g. MLME join-request) confirm: persist immediately so
	   * DevNonce / session state survive a reset even during a join-failure
	   * storm. These are infrequent and are not the source of flash wear. */
	  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
	}
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
	  if (joinParams != NULL)
	  {
	    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
	    {
	      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
	      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
	      {
	        APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
	      }
	      else
	      {
	        APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
	      }
	      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
	    }
	    else
	    {
	      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
	    }

	    APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:JOIN | DR:%d | PWR:%d\r\n", joinParams->Datarate, joinParams->TxPower);
	  }
  /* USER CODE END OnJoinRequest_1 */
}

static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params)
{
  /* USER CODE BEGIN OnBeaconStatusChange_1 */
  /* USER CODE END OnBeaconStatusChange_1 */
}

static void OnSysTimeUpdate(void)
{
  /* USER CODE BEGIN OnSysTimeUpdate_1 */

  /* USER CODE END OnSysTimeUpdate_1 */
}

static void OnClassChange(DeviceClass_t deviceClass)
{
  /* USER CODE BEGIN OnClassChange_1 */
  /* USER CODE END OnClassChange_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

static void OnTxPeriodicityChanged(uint32_t periodicity)
{
  /* USER CODE BEGIN OnTxPeriodicityChanged_1 */

  /* USER CODE END OnTxPeriodicityChanged_1 */
  TxPeriodicity = periodicity;

  if (TxPeriodicity == 0)
  {
    /* Revert to application default periodicity */
    TxPeriodicity = APP_TX_DUTYCYCLE;
  }

  /* Update timer periodicity */
  UTIL_TIMER_Stop(&TxTimer);
  UTIL_TIMER_SetPeriod(&TxTimer, TxPeriodicity);
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxPeriodicityChanged_2 */

  /* USER CODE END OnTxPeriodicityChanged_2 */
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
  /* USER CODE BEGIN OnTxFrameCtrlChanged_1 */

  /* USER CODE END OnTxFrameCtrlChanged_1 */
  LmHandlerParams.IsTxConfirmed = isTxConfirmed;
  /* USER CODE BEGIN OnTxFrameCtrlChanged_2 */

  /* USER CODE END OnTxFrameCtrlChanged_2 */
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_1 */

  /* USER CODE END OnPingSlotPeriodicityChanged_1 */
  LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_2 */

  /* USER CODE END OnPingSlotPeriodicityChanged_2 */
}

static void OnSystemReset(void)
{
  /* USER CODE BEGIN OnSystemReset_1 */

  /* USER CODE END OnSystemReset_1 */
  if ((LORAMAC_HANDLER_SUCCESS == LmHandlerHalt()) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
  {
    NVIC_SystemReset();
  }
  /* USER CODE BEGIN OnSystemReset_Last */

  /* USER CODE END OnSystemReset_Last */
}

static void StopJoin(void)
{
  /* USER CODE BEGIN StopJoin_1 */

  /* USER CODE END StopJoin_1 */

  UTIL_TIMER_Stop(&TxTimer);

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerStop())
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stop on going ...\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stopped\r\n");
    if (LORAWAN_DEFAULT_ACTIVATION_TYPE == ACTIVATION_TYPE_ABP)
    {
      ActivationType = ACTIVATION_TYPE_OTAA;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to OTAA mode\r\n");
    }
    else
    {
      ActivationType = ACTIVATION_TYPE_ABP;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to ABP mode\r\n");
    }
    LmHandlerConfigure(&LmHandlerParams);
    LmHandlerJoin(ActivationType, true);
    UTIL_TIMER_Start(&TxTimer);
  }
  UTIL_TIMER_Start(&StopJoinTimer);
  /* USER CODE BEGIN StopJoin_Last */

  /* USER CODE END StopJoin_Last */
}

static void OnStopJoinTimerEvent(void *context)
{
  /* USER CODE BEGIN OnStopJoinTimerEvent_1 */

  /* USER CODE END OnStopJoinTimerEvent_1 */
  if (ActivationType == LORAWAN_DEFAULT_ACTIVATION_TYPE)
  {
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
  }
  /* USER CODE BEGIN OnStopJoinTimerEvent_Last */

  /* USER CODE END OnStopJoinTimerEvent_Last */
}

static void StoreContext(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  /* USER CODE BEGIN StoreContext_1 */

  /* USER CODE END StoreContext_1 */
  status = LmHandlerNvmDataStore();

  if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA UP TO DATE\r\n");
  }
  else if (status == LORAMAC_HANDLER_ERROR)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORE FAILED\r\n");
  }
  /* USER CODE BEGIN StoreContext_Last */

  /* USER CODE END StoreContext_Last */
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state)
{
  /* USER CODE BEGIN OnNvmDataChange_1 */

  /* USER CODE END OnNvmDataChange_1 */
  if (state == LORAMAC_HANDLER_NVM_STORE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
  }
  /* USER CODE BEGIN OnNvmDataChange_Last */

  /* USER CODE END OnNvmDataChange_Last */
}

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnStoreContextRequest_1 */

  /* USER CODE END OnStoreContextRequest_1 */
  /* store nvm in flash */
  if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE) == FLASH_IF_OK)
  {
    FLASH_IF_Write(LORAWAN_NVM_BASE_ADDRESS, (const void *)nvm, nvm_size);
  }
  /* USER CODE BEGIN OnStoreContextRequest_Last */

  /* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnRestoreContextRequest_1 */

  /* USER CODE END OnRestoreContextRequest_1 */
  FLASH_IF_Read(nvm, LORAWAN_NVM_BASE_ADDRESS, nvm_size);
  /* USER CODE BEGIN OnRestoreContextRequest_Last */

  /* USER CODE END OnRestoreContextRequest_Last */
}

