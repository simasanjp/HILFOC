#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "FOC_APPL.h"
#include "adc.h"
#include "par_def.h"
#include "mpwm.h"
#include "foc.h"
#include "pse.h"
#include "assert.h"

//#define DEMO_USE_POTENTIOMETER

/*!
 * \brief   Slope of Pot reading vs Speed request
 */
#define APP_POT_SPEED_SLOPE                 ((float)(PAR_MAX_SPEED_RPM - PAR_MIN_SPEED_RPM) / (float)(PAR_ADC_MAX_DIG - PAR_POT_MIN_SPEED_REQ_DIG))

#define MPWM_FAULT_COUNT (23)

/** FOC_APPL states */
typedef enum FOC_APPL_STATE_Ttag
{
  FOC_APPL_IDLE, /**< idle mode, no speed request and speed below sync speed */
  FOC_APPL_RUN, /**< Closed loop motor run state */
  FOC_APPL_STOP, /**< Motor stop state, no speed request speed above sync speed */
  FOC_APPL_ERROR, /**< Error state */
} FOC_APPL_STATE_T;

/*****************************************************************************/
/*! FOC_APPL Data                                                                */
/*****************************************************************************/
typedef struct FOC_APPL_DATA_Ttag
{
  /*!
   * \brief   Indicates if the motor is running
   */
  bool bMotorRunning;
  /*!
   * \brief   Application state definition.
   */
  FOC_APPL_STATE_T EFOC_APPLState;
  /*!
   * \brief   User requested speed in RPM.
   */
  int16_t sUsrReqSpeed;
  /*!
   * \brief   Struct that holds estimated speed values.
   */
  Pse_Speed_T tSpeed;
  /*!
   * \brief   Filtered estimated speed in RPM.
   */
  int16_t sFilteredSpeed;
  /*!
   * \brief   Requested current vector in DQ frame.
   * \details Output of speed controller.
   */
  Math_Vector_DQ_T tReqCurrent;
#if (PAR_USE_OPENLOOP == 1)
  /*!
   * \brief   Requested voltage vector in DQ frame, for open-loop control.
   */
  Math_Vector_DQ_T tReqVoltage;
#endif /*PAR_USE_OPENLOOP*/
  /*!
   * \brief   PI-controller parameters for speed controller.
   */
  Math_PI_Controller_Parameter_T tPIControlPar;
  /*!
   * \brief   PI-controller state parameters for speed controller.
   */
  Math_PI_Controller_State_T tPIControlState;
  /*!
   * \brief   PI-controller parameters for field-weakening controller.
   */
  Math_PI_Controller_Parameter_T tFWPIControlPar;
  /*!
   * \brief   PI-controller state parameters for field-weakening controller.
   */
  Math_PI_Controller_State_T tFWPIControlState;
  /*!
   * \brief   Timer handler for the FOC_APPL main task
   */
  DRV_TIM_HANDLE_T tTimerTask;
} FOC_APPL_DATA_T;

typedef struct MPWM_SPI_FAULT_STATUS_DICTIONARY_Ttag
{
  uint32_t luFaultCode;
  const char *szString;
} MPWM_SPI_FAULT_STATUS_DICTIONARY_T;

/** Fault Strings */
MPWM_SPI_FAULT_STATUS_DICTIONARY_T g_tMPWM_SPI_Fault_Dictionary[] =
{
  {MPWM_SPI_STATUS_NO_FAULT,                    "MPWM_SPI_STATUS_NO_FAULT",              },
  {MPWM_SPI_FAULT_STATUS_FAULT,                 "MPWM_SPI_FAULT_STATUS_FAULT",           },
  {MPWM_SPI_FAULT_STATUS_VDS_OCP,               "MPWM_SPI_FAULT_STATUS_VDS_OCP",         },
  {MPWM_SPI_FAULT_STATUS_GDF,                   "Gate Driver Fault!       ",             },
  {MPWM_SPI_FAULT_STATUS_UVLO,                  "MPWM_SPI_FAULT_STATUS_UVLO",            },
  {MPWM_SPI_FAULT_STATUS_OTSD,                  "MPWM_SPI_FAULT_STATUS_OTSD",            },
  {MPWM_SPI_FAULT_STATUS_VDS_HA,                "MPWM_SPI_FAULT_STATUS_VDS_HA",          },
  {MPWM_SPI_FAULT_STATUS_VDS_LA,                "MPWM_SPI_FAULT_STATUS_VDS_LA",          },
  {MPWM_SPI_FAULT_STATUS_VDS_HB,                "MPWM_SPI_FAULT_STATUS_VDS_HB",          },
  {MPWM_SPI_FAULT_STATUS_VDS_HB,                "MPWM_SPI_FAULT_STATUS_VDS_HB",          },
  {MPWM_SPI_FAULT_STATUS_VDS_LB,                "MPWM_SPI_FAULT_STATUS_VDS_LB",          },
  {MPWM_SPI_FAULT_STATUS_VDS_HC,                "MPWM_SPI_FAULT_STATUS_VDS_HC",          },
  {MPWM_SPI_FAULT_STATUS_VDS_LC,                "MPWM_SPI_FAULT_STATUS_VDS_LC",          },
  {MPWM_SPI_FAULT_STATUS_SA_OC,                 "MPWM_SPI_FAULT_STATUS_SA_OC",           },
  {MPWM_SPI_FAULT_STATUS_SB_OC,                 "MPWM_SPI_FAULT_STATUS_SB_OC",           },
  {MPWM_SPI_FAULT_STATUS_SC_OC,                 "MPWM_SPI_FAULT_STATUS_SC_OC",           },
  {MPWM_SPI_FAULT_STATUS_OTW,                   "MPWM_SPI_FAULT_STATUS_OTW",             },
  {MPWM_SPI_FAULT_STATUS_CPUV,                  "MPWM_SPI_FAULT_STATUS_CPUV",            },
  {MPWM_SPI_FAULT_STATUS_VGS_HA,                "MPWM_SPI_FAULT_STATUS_VGS_HA",          },
  {MPWM_SPI_FAULT_STATUS_VGS_LA,                "MPWM_SPI_FAULT_STATUS_VGS_LA",          },
  {MPWM_SPI_FAULT_STATUS_VGS_HB,                "MPWM_SPI_FAULT_STATUS_VGS_HB",          },
  {MPWM_SPI_FAULT_STATUS_VGS_LB,                "MPWM_SPI_FAULT_STATUS_OTW",             },
  {MPWM_SPI_FAULT_STATUS_VGS_HC,                "MPWM_SPI_FAULT_STATUS_VGS_HC",          },
  {MPWM_SPI_FAULT_STATUS_VGS_LC,                "MPWM_SPI_FAULT_STATUS_VGS_LC",          },
};

static bool gb_helpPrinted;
/*!
 * \brief print Gate Drive Unit errors
 */
void FOC_APPL_Print_GDU_Errors()
{
  uint_fast16_t i = 0;
  uint32_t ulFaultRegisters = 0;
  mpwm_GetFaultErrorRegisters(&ulFaultRegisters);
  if(ulFaultRegisters & MPWM_SPI_FAULT_STATUS_FAULT)
  {
    printf("The following fault flags are set in the GDU:\r\n");
    for(i = 2; i < MPWM_FAULT_COUNT; i++)
    {
      if(g_tMPWM_SPI_Fault_Dictionary[i].luFaultCode & ulFaultRegisters)
      {
        printf("%s\r\n", g_tMPWM_SPI_Fault_Dictionary[i].szString);
      }
    }
  }
}

/*!
 * \brief Sets motor speed in RPM
 */
extern uint16_t FOC_APPL_setSpeedSetpoint(uint16_t uSpeed, FOC_APPL_HANDLE_T tHnd)
{
  FOC_APPL_DATA_T * ptData = (FOC_APPL_DATA_T *) tHnd;
  if(!ptData)
  {
    assert(0);
  }
  ptData->sUsrReqSpeed = uSpeed;
  if(ptData->sUsrReqSpeed)
  {
    ptData->bMotorRunning = true;
  }
  else
  {
    ptData->bMotorRunning = false;
  }
  return ptData->sUsrReqSpeed;
}
/*!
 * \brief Returns current motor speed in RPM
 */
uint16_t FOC_APPL_getSpeed(FOC_APPL_HANDLE_T tHnd)
{
  FOC_APPL_DATA_T * ptData = (FOC_APPL_DATA_T *) tHnd;
  if(!ptData)
  {
    assert(0);
  }

  return ptData->sFilteredSpeed;
}
/**
 * \brief   Application speed controller.
 * \image   html "Field-Weakening Method.svg"
 */
static void FOC_APPL_SpeedController(FOC_APPL_DATA_T * ptData);
/*!
 * \brief Reset speed controller values.
 */
static void FOC_APPL_ResetSpeedController(FOC_APPL_DATA_T * ptData);
/*!
 * \brief Main application loop.
 * \image    html "Speed Request.svg"
 */
static void FOC_APPL_Main(void * pvDriverHandle, void * pvUserHandle);

/**
 * \brief   Application state machine function.
 * \image   html "Main State Machine.svg"
 */
static void FOC_APPL_StateMachine(FOC_APPL_DATA_T * ptData);

/**
 * \MPWM interrupt handler
 */
void MPWM_IRQHandler(void)
{
  mpwm_AcknowledgeIRQ();
  mpwm_Synch();
  adc_Start();
  Foc_StateMachine();
}

/**
 * \brief   Prepare the drivers needed for the PROFIDrive application
 */
static void FOC_APPL_Prepare()
{
  DRV_DIO_Init();
  mpwm_Initialize();
  adc_Initialize();

  mpwm_Start();
  adc_Start();
  Foc_Init();
}

/*!
 * \Check GDU error if GDU enable pin is set
 */
extern DRV_STATUS_E FOC_APPL_checkGDUError()
{
  return (mpwm_app->mpwm_irq_raw_b.eci_val == 0) && (pio_app->pio_oe_b.val & DRV_DIO_MSK_PIO_8);
}

/*!
 * \details   5ms timer object interrupt enabled. Reset speed controller parameters.
 * \param [in]    ptHnd  FOC APPL data struct
 * \return  DRV_OK
 *          DRV_ERROR
 */
DRV_STATUS_E FOC_APPL_init(FOC_APPL_HANDLE_T * ptHnd)
{
  DRV_STATUS_E eRet; /* driver status declaration */
  void * pvUserParam; /*user param for FOC_APPL_Main */
  DRV_TIM_PRELOAD_VALUE_E const eTimerPeriod = (DRV_TIM_PRELOAD_VALUE_E) DRV_TIM_PRELOAD_VALUE_10ms; /* 5ms */
  FOC_APPL_DATA_T* ptData = NULL;

  FOC_APPL_Prepare();

  ptData = malloc(sizeof(FOC_APPL_DATA_T));
  if(!ptData)
  {
    assert(0);
  }

  memset(&(ptData->tTimerTask), 0, sizeof(DRV_TIM_HANDLE_T));
  *ptHnd = ptData;

  ptData->EFOC_APPLState = FOC_APPL_IDLE;
  ptData->sUsrReqSpeed = 0;
  ptData->tSpeed.MechSpeedRPM = 0;
  Pse_Init();

  FOC_APPL_ResetSpeedController(ptData);

  ptData->tReqCurrent.f_d = 0;
  ptData->tReqCurrent.f_q = 0;

  ptData->bMotorRunning = false;

  /** Configure the timer peripheral */
  ptData->tTimerTask.tConfiguration.eDeviceID = DRV_TIM_DEVICE_ID_GPIOCNTR0; /** DRV_TIM_DEVICE_ID_SYSTICK, DRV_TIM_DEVICE_ID_TIMER0/1/2 or DRV_TIM_DEVICE_ID_GPIOCNTR0/1/2 can be chose */
  ptData->tTimerTask.tConfiguration.eDioIdInputReference = DRV_DIO_ID_GPIO_0; /** DIO Input Reference must be set for GPIOCNT timer */
  ptData->tTimerTask.tConfiguration.eOperationMode = DRV_OPERATION_MODE_IRQ; /** interrupt mode */
  ptData->tTimerTask.tConfiguration.tPreloadValue = eTimerPeriod; /** 5ms*/
  ptData->tTimerTask.tConfiguration.eCountingMode = DRV_TIM_COUNTING_MODE_CONTINUOUS; /** continuous mode */

  /** initialize the timer device */
  if(DRV_OK != DRV_TIM_Init(&ptData->tTimerTask))
  {
    return DRV_ERROR;
  }

  pvUserParam = ptData;

  eRet = DRV_TIM_IRQAttach(&ptData->tTimerTask, FOC_APPL_Main, pvUserParam); /* attaches the callback function */

  if(DRV_OK != eRet)
  {
    return DRV_ERROR;
  }

  DRV_NVIC_SetPriority(gpio_app_timer0_IRQn,2,0);
  DRV_NVIC_SetPriority(mpwm_IRQn,1,0);


  eRet = DRV_TIM_Start(&ptData->tTimerTask); /* Starts the timer */
  if(DRV_OK != eRet)
  {
    return DRV_ERROR;
  }

  gb_helpPrinted = false;

  ptData->bMotorRunning = true;

//  ptData->bMotorRunning = true;
  return DRV_OK;
}

/**
 \brief   FOC application Speed Controller
 \details calculate and send a current setpoint to the FOC
 \param [in]    ptData  FOC APPL data struct
 */
static void FOC_APPL_SpeedController(FOC_APPL_DATA_T * ptData)
{
  int16_t ls16_SpeedError;
  float lf_FWError;
  float lf_SquareSum;
  Math_Vector_DQ_T s_VoltageVec = Foc_GetRequestVoltage();

#if (PAR_USE_HALL_SPEED == 0)
  ls16_SpeedError = ptData->sUsrReqSpeed - ptData->tSpeed.MechSpeedRPM;
#else
  /* Speed estimated only by hall sensors requires filtering. */
  ls16_SpeedError = ptData->sUsrReqSpeed - ptData->sFilteredSpeed;
#endif
  /* Control q current */
  ptData->tReqCurrent.f_q = Math_PIController(&ptData->tPIControlState, &ptData->tPIControlPar, (float) ls16_SpeedError);

  /* Control d current, Field weakening  */
  lf_SquareSum = (s_VoltageVec.f_d * s_VoltageVec.f_d) + (s_VoltageVec.f_q * s_VoltageVec.f_q);
  lf_FWError = PAR_FIELDWEAK_START_VOLTAGE_RATIO - Math_Sqrt(lf_SquareSum, 1.0f, 2);
  ptData->tReqCurrent.f_d = Math_PIController(&ptData->tFWPIControlState, &ptData->tFWPIControlPar, lf_FWError);
}

/**
 \brief   FOC application Reset Speed Controller
 \details Reset speed controller parameters
 \param [in,out] ptData  FOC APPL data struct
 */
static void FOC_APPL_ResetSpeedController(FOC_APPL_DATA_T * ptData)
{
  ptData->tPIControlPar.f_KP = PAR_SPEED_CONTROLLER_KP;
  ptData->tPIControlPar.f_KI = PAR_SPEED_CONTROLLER_KI;
  ptData->tPIControlPar.f_UpperLimit = PAR_MAX_PHASE_CURRENT_AMP;
  ptData->tPIControlPar.f_LowerLimit = -PAR_MAX_PHASE_CURRENT_AMP;
  ptData->tPIControlState.f_integral = 0.0f;

  ptData->tFWPIControlPar.f_KP = PAR_FIELDWEAK_CONTROLLER_KP;
  ptData->tFWPIControlPar.f_KI = PAR_FIELDWEAK_CONTROLLER_KI;
  ptData->tFWPIControlPar.f_UpperLimit = 0.0f;
  ptData->tFWPIControlPar.f_LowerLimit = -PAR_MAX_PHASE_CURRENT_AMP;
  ptData->tFWPIControlState.f_integral = 0.0f;

  ptData->tReqCurrent.f_q = 0.0f;
  ptData->tReqCurrent.f_d = 0.0f;
  ptData->tSpeed.MechSpeedRPM = 0;
  ptData->sFilteredSpeed = 0;
}

/**
 \brief   FOC main application
 \details called every 5ms
 \param [in,out] pvUserHandle  FOC APPL data struct
 */
static void FOC_APPL_Main(void * pvDriverHandle, void * pvUserHandle)
{
  FOC_APPL_DATA_T * ptData = (FOC_APPL_DATA_T*) pvUserHandle;
#ifdef DEMO_USE_POTENTIOMETER
  uint16_t lu16_pot;
  lu16_pot = adc_GetPotValue();
  /* Get requested speed from user-interface */
  if(ptData->bMotorRunning)
  {
    if(lu16_pot >= PAR_POT_MIN_SPEED_REQ_DIG)
    {
      ptData->sUsrReqSpeed = (int16_t) (PAR_MIN_SPEED_RPM + (float) (lu16_pot - PAR_POT_MIN_SPEED_REQ_DIG) * APP_POT_SPEED_SLOPE);
    }
    else if(lu16_pot < PAR_POT_HYSTERESIS_DIG)
    {
      ptData->sUsrReqSpeed = 0u;
    }
    else /* Do not change speed*/
    {

    }
  }
#endif /* DEMO_USE_POTENTIOMETER*/

#if (PAR_USE_HALL_SPEED == 0)
  ptData->tSpeed = Pse_EstimateSpeed(); /* Get rotor speed from FOC_APPL/Pse using QEI*/
#else
  ptData->tSpeed = Pse_EstimateSpeedHall(); /* Get rotor speed from FOC_APPL/Pse using Hall*/
#endif /*PAR_USE_HALL_SPEED*/
  ptData->sFilteredSpeed = ptData->sFilteredSpeed + (ptData->tSpeed.MechSpeedRPM - ptData->sFilteredSpeed) / 4;
  /* Check nFault - GDU error signal only if GDU is enabled */
  if((mpwm_app->mpwm_irq_raw_b.eci_val == 0) && (pio_app->pio_oe_b.val & DRV_DIO_MSK_PIO_8))
  {
    ptData->EFOC_APPLState = FOC_APPL_ERROR;
    mpwm_app->mpwm_cfg_b.eci_inv = ~(mpwm_app->mpwm_cfg_b.eci_inv); /* Inversion is needed since signal is flip-flop */
  }
  /* Run App_StateMachine */
  FOC_APPL_StateMachine(ptData);

#ifdef USE_XCP
  /* xcp interface: internal debug use only */
  XcpnetX90SerialTask1ms();
#endif /*USE_XCP*/
  /* Acknowledge IRQ */
  gpio_app->gpio_app_irq_raw = gpio_app->gpio_app_irq_raw;
}

/**
 \brief   FOC application state machine
 \param [in,out] ptData  FOC APPL data struct
 */
static void FOC_APPL_StateMachine(FOC_APPL_DATA_T * ptData)
{
  FOC_APPL_STATE_T ls_State;
  ls_State = ptData->EFOC_APPLState;

  switch (ls_State)
  {
  default:
  case FOC_APPL_IDLE: /*  */
    Foc_TransitState(FOC_IDLE);
    if(ptData->sUsrReqSpeed != 0u)
    {
      ptData->EFOC_APPLState = FOC_APPL_RUN;
    }
    break;
  case FOC_APPL_RUN:/* */
    if(ptData->sUsrReqSpeed == 0u)
    {
      ptData->EFOC_APPLState = FOC_APPL_STOP;
    }
    FOC_APPL_SpeedController(ptData);
    Foc_SetRequestCurrent(ptData->tReqCurrent);
    Foc_TransitState(FOC_CURRENT_CONTROL);
    break;
  case FOC_APPL_STOP:/* */
    FOC_APPL_SpeedController(ptData);
    Foc_SetRequestCurrent(ptData->tReqCurrent);
    if(ptData->sUsrReqSpeed != 0u)
    {
      ptData->EFOC_APPLState = FOC_APPL_RUN;
    }
    else
    {
      if(ptData->tSpeed.MechSpeedRPM <= PSE_SYNC_MECH_SPEED_RPM)
      {
        ptData->EFOC_APPLState = FOC_APPL_IDLE;
        FOC_APPL_ResetSpeedController(ptData);
        Pse_Init();
      }
    }
    break;
  case FOC_APPL_ERROR:/*  */
    if(!gb_helpPrinted)
    {
      FOC_APPL_Print_GDU_Errors();
    }
    gb_helpPrinted = true;
    Foc_TransitState(FOC_FAILED);
    if(mpwm_app->mpwm_irq_raw_b.eci_ks_state == 1U)/* iIf error clears */
    {
      if(ptData->sUsrReqSpeed == 0u)
      {
        gb_helpPrinted = false;
        ptData->EFOC_APPLState = FOC_APPL_STOP;
      }
    }
    break;
  }
}

/**
 \brief The de-initialize function is supposed to reset the handle
 \param [out] tHnd  FOC APPL data struct
 \return DRV_OK
 \       DRV_ERROR
 */
DRV_STATUS_E FOC_APPL_deInit(FOC_APPL_HANDLE_T tHnd)
{
  DRV_STATUS_E eRet; /* driver status declaration */
  FOC_APPL_DATA_T * ptData = (FOC_APPL_DATA_T *) tHnd;
  if(!ptData)
  {
    assert(0);
  }

  ptData->sUsrReqSpeed = 0;
  mpwm_DisableGDU();

  eRet = DRV_TIM_Stop(&ptData->tTimerTask); /* Stops the timer */
  if(DRV_OK != eRet)
  {
    return DRV_ERROR;
  }

  eRet = DRV_TIM_Stop(&ptData->tTimerTask); /* Stops the timer */
  if(DRV_OK != eRet)
  {
    return DRV_ERROR;
  }

  eRet = DRV_TIM_IRQAttach(NULL, NULL, NULL); /* detaches the callback function */
  if(DRV_OK != eRet)
  {
    return DRV_ERROR;
  }

  eRet = DRV_TIM_IRQAttach(NULL, NULL, NULL); /* deinitializes the callback function */
  if(DRV_OK != eRet)
  {
    return DRV_ERROR;
  }

  free(ptData);
  ptData = NULL;

  return DRV_OK;
}
