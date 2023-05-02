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
#include "sens.h"
#include "calibrate.h"

/*!
 * \brief   Slope of Pot reading vs Speed request
 */
#define APP_POT_SPEED_SLOPE                 ((float)(PAR_MAX_SPEED_RPM - PAR_MIN_SPEED_RPM) / (float)(PAR_ADC_MAX_DIG - PAR_POT_MIN_SPEED_REQ_DIG))

/** FOC_APPL states */
typedef enum FOC_APPL_STATE_Ttag
{
  FOC_APPL_IDLE, /**< idle mode, no speed request and speed below sync speed */
  FOC_APPL_RUN, /**< Closed loop motor run state */
  FOC_APPL_STOP, /**< Motor stop state, no speed request speed above sync speed */
  FOC_APPL_CALIBRATE,/**< Hall sensor calibration state*/
  FOC_APPL_MANUAL,
  FOC_APPL_ERROR, /**< Error state */
} FOC_APPL_STATE_T;




typedef struct {
  /*!
 * \brief   Application state definition.
 */
 FOC_APPL_STATE_T               Main_State;

  /*!
   * \brief   Struct that holds estimated speed values.
   */
   Pse_Speed_T                  Speed;

  /*!
   * \brief   Filtered estimated speed in RPM.
   */
   int16_t                      FilteredSpeed;

  /*!
   * \brief   PI-controller state parameters for speed controller.
   */
   Math_PI_Controller_State_T   SpeedPIControlState;

  /*!
   * \brief   Hall sensor calibration state.
   */
   CalibrationStateT            HallSensorCalibrationState;

  /*!
   * \brief   Low pass speed filter state.
   */
   int32_t                      SpeedFilterState;

  /*!
   * \brief   PI-controller state parameters for field-weakening controller.
   */
   Math_PI_Controller_State_T   FWPIControlState;

}ApplicationStateT;


typedef struct {

  /*!
   * \brief   PI-controller parameters for speed controller.
   */
   Math_PI_Controller_Parameter_T SpeedPIControlPar;
  /*!
   * \brief   PI-controller parameters for field-weakening controller.
   */
   Math_PI_Controller_Parameter_T FWPIControlPar;
  /*!
   * \brief   Hall sensor calibration parameters.
   */
   CalibrationParameterT          HallSensorCalibrationPar;

}ApplicationParameterT;


typedef struct{
  /*!
   * \brief   User requested speed in RPM.
  */
  int16_t             UsrReqSpeed;

  /*!
   * \brief   Requested current vector in DQ frame.
   * \details Output of speed controller.
   */
   Math_Vector_DQ_T   ReqCurrent;
  /*!
   * \brief   Requested voltage vector in DQ frame, for open-loop control.
   */
   Math_Vector_DQ_T   ReqVoltage;

}ApplicationControlT;


ApplicationParameterT App_ms_Parameters;
ApplicationStateT     App_ms_State;
ApplicationControlT   App_ms_Control;


/*!
 * \brief   Indicates if the motor is running
 */
bool                  bMotorRunning;
/*!
 * \brief   Timer handler for the FOC_APPL main task
 */
DRV_TIM_HANDLE_T      tTimerTask;

/*!
   * \brief   'Counter to detect motor stall.
*/
uint16_t              App_mu16_StallCounter10ms;
/*!
   * \brief   Switch to manual mode.
*/
bool                  App_mb_ManualMode;

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
    for(i = 2; i < 23; i++)
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
extern uint16_t FOC_APPL_setSpeedSetpoint(uint16_t uSpeed)
{

  App_ms_Control.UsrReqSpeed = uSpeed;
  if(App_ms_Control.UsrReqSpeed)
  {
    bMotorRunning = true;
  }
  else
  {
    bMotorRunning = false;
  }
  return App_ms_Control.UsrReqSpeed;
}
/*!
 * \brief Returns current motor speed in RPM
 */
uint16_t FOC_APPL_getSpeed()
{
  return App_ms_State.FilteredSpeed;
}

/**
 * \brief   Application speed controller.
 * \image   html "Field-Weakening Method.svg"
 */
static void FOC_APPL_SpeedController();
/*!
 * \brief Reset speed controller values.
 */
static void FOC_APPL_ResetSpeedController();

/**
* \brief   Application calibration state.
*/
static void FOC_APPL_Calibration();
/**
* \brief   Stall Detection.
*/
static bool FOC_APPL_StallDetection(int16_t as16_RequestedSpeed, int16_t as16_MeasuredSpeed);
/*!
 * \brief Main application loop.
 * \image    html "Speed Request.svg"
 */
static void FOC_APPL_Main();

/**
 * \brief   Application state machine function.
 * \image   html "Main State Machine.svg"
 */
static void FOC_APPL_StateMachine();

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

  App_ms_Parameters.HallSensorCalibrationPar.s16_SamplingSpeedRPM                = CALIBRATION_SAMPLING_RPM;
  App_ms_Parameters.HallSensorCalibrationPar.u16_WaitTimeBetweenEachSample10Msec = 1;
  App_ms_Parameters.HallSensorCalibrationPar.f_RequestedVoltage                  = 0.12f;
  App_ms_Parameters.HallSensorCalibrationPar.s16_NumberOfTurns                   = 4;
  App_ms_Parameters.HallSensorCalibrationPar.u16_WaitTimeAfterAlignment10Msec    = 50;
  App_ms_Parameters.HallSensorCalibrationPar.u16_MaximumDeviationAngle           = CALIBRATION_MAX_DEVIATION_ANGLE;
  App_mb_ManualMode = false;
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
DRV_STATUS_E FOC_APPL_init()
{
  DRV_STATUS_E eRet; /* driver status declaration */
  void * pvUserParam; /*user param for FOC_APPL_Main */
  DRV_TIM_PRELOAD_VALUE_E const eTimerPeriod = (DRV_TIM_PRELOAD_VALUE_E) DRV_TIM_PRELOAD_VALUE_10ms; /* 10ms */


  FOC_APPL_Prepare();

  memset(&(tTimerTask), 0, sizeof(DRV_TIM_HANDLE_T));

  Pse_Init();

  FOC_APPL_ResetSpeedController();


  bMotorRunning = false;

  /** Configure the timer peripheral */
  tTimerTask.tConfiguration.eDeviceID = DRV_TIM_DEVICE_ID_GPIOCNTR0; /** DRV_TIM_DEVICE_ID_SYSTICK, DRV_TIM_DEVICE_ID_TIMER0/1/2 or DRV_TIM_DEVICE_ID_GPIOCNTR0/1/2 can be chose */
  tTimerTask.tConfiguration.eDioIdInputReference = DRV_DIO_ID_GPIO_0; /** DIO Input Reference must be set for GPIOCNT timer */
  tTimerTask.tConfiguration.eOperationMode = DRV_OPERATION_MODE_IRQ; /** interrupt mode */
  tTimerTask.tConfiguration.tPreloadValue = eTimerPeriod; /** 5ms*/
  tTimerTask.tConfiguration.eCountingMode = DRV_TIM_COUNTING_MODE_CONTINUOUS; /** continuous mode */

  /** initialize the timer device */
  if(DRV_OK != DRV_TIM_Init(&tTimerTask))
  {
    return DRV_ERROR;
  }

  eRet = DRV_TIM_IRQAttach(&tTimerTask, FOC_APPL_Main, pvUserParam); /* attaches the callback function */

  if(DRV_OK != eRet)
  {
    return DRV_ERROR;
  }

  DRV_NVIC_SetPriority(gpio_app_timer0_IRQn,2,0);
  DRV_NVIC_SetPriority(mpwm_IRQn,1,0);


  eRet = DRV_TIM_Start(&tTimerTask); /* Starts the timer */
  if(DRV_OK != eRet)
  {
    return DRV_ERROR;
  }

  gb_helpPrinted = false;

  bMotorRunning = true;

  return DRV_OK;
}

/**
 \brief   FOC application Speed Controller
 \details calculate and send a current setpoint to the FOC

 */

static void FOC_APPL_SpeedController()
{
  float lf_FWError;
  float lf_SpeedError;
  float lf_SquareSum;
  float lf_MaxIqCurrent;
  float lf_MaxIqCurrentSquare;
  Math_Vector_DQ_T ls_VoltageVec = Foc_GetRequestVoltage();

  /* Speed estimated filtering. */
  lf_SpeedError = (float)App_ms_Control.UsrReqSpeed - (float)App_ms_State.FilteredSpeed;

  /* Control d current, Field weakening  */
  lf_SquareSum = (ls_VoltageVec.f_d * ls_VoltageVec.f_d) + (ls_VoltageVec.f_q * ls_VoltageVec.f_q);
  lf_FWError                    = PAR_FIELDWEAK_START_VOLTAGE_RATIO - Math_Sqrt(lf_SquareSum, 1.0f, 2);
  App_ms_Control.ReqCurrent.f_d = Math_PIController(&App_ms_State.FWPIControlState, &App_ms_Parameters.FWPIControlPar, lf_FWError);

  /* Limit Iq current to maintain max phase current */
  lf_MaxIqCurrentSquare = (PAR_MAX_PHASE_CURRENT_AMP * PAR_MAX_PHASE_CURRENT_AMP) - (App_ms_Control.ReqCurrent.f_d*App_ms_Control.ReqCurrent.f_d);

  if (lf_MaxIqCurrentSquare < 0.0f){
    lf_MaxIqCurrentSquare = 0.0f;
  }

  lf_MaxIqCurrent = Math_Sqrt(lf_MaxIqCurrentSquare, 2.5f,2);
  /* Limit q current by setting upper limit */
  App_ms_Parameters.SpeedPIControlPar.f_UpperLimit = lf_MaxIqCurrent;

  /* Control q current */
  App_ms_Control.ReqCurrent.f_q = Math_PIController(&App_ms_State.SpeedPIControlState, &App_ms_Parameters.SpeedPIControlPar, (float) lf_SpeedError);

}

/**
 \brief   FOC application Reset Speed Controller
 \details Reset speed controller parameters
 \param [in,out] ptData  FOC APPL data struct
 */
static void FOC_APPL_ResetSpeedController()
{
  App_ms_Parameters.SpeedPIControlPar.f_KP = PAR_SPEED_CONTROLLER_KP;
  App_ms_Parameters.SpeedPIControlPar.f_KI = PAR_SPEED_CONTROLLER_KI;
  App_ms_Parameters.SpeedPIControlPar.f_UpperLimit = PAR_MAX_PHASE_CURRENT_AMP;
  App_ms_Parameters.SpeedPIControlPar.f_LowerLimit = 0.0f;
  App_ms_State.SpeedPIControlState.f_integral = 0.0f;

  App_ms_Parameters.FWPIControlPar.f_KP = PAR_FIELDWEAK_CONTROLLER_KP;
  App_ms_Parameters.FWPIControlPar.f_KI = PAR_FIELDWEAK_CONTROLLER_KI;
  App_ms_Parameters.FWPIControlPar.f_UpperLimit = 0.0f;
  App_ms_Parameters.FWPIControlPar.f_LowerLimit = -1.0f;
  App_ms_State.FWPIControlState.f_integral = 0.0f;

  App_ms_Control.ReqCurrent.f_q   = 0.0f;
  App_ms_Control.ReqCurrent.f_d   = 0.0f;
  App_ms_State.Speed.MechSpeedRPM = 0;
  App_ms_State.FilteredSpeed = 0;
  MATH_LowPassFilter_Init(&App_ms_State.SpeedFilterState, 0);
}

/**
 \brief   FOC main application
 \details called every 10ms

 */
static void FOC_APPL_Main()
{

#ifdef DEMO_USE_POTENTIOMETER
  uint16_t lu16_pot;
  lu16_pot = adc_GetPotValue();
  /* Get requested speed from user-interface */
  if(bMotorRunning)
  {
    if(lu16_pot >= PAR_POT_MIN_SPEED_REQ_DIG)
    {
      App_ms_Control.UsrReqSpeed = (int16_t) (PAR_MIN_SPEED_RPM + (float) (lu16_pot - PAR_POT_MIN_SPEED_REQ_DIG) * APP_POT_SPEED_SLOPE);
    }
    else if(lu16_pot < PAR_POT_HYSTERESIS_DIG)
    {
      App_ms_Control.UsrReqSpeed = 0u;
    }
    else /* Do not change speed*/
    {

    }
  }
#endif /* DEMO_USE_POTENTIOMETER*/

#if (PAR_USE_HALL_SPEED == 0)
  App_ms_State.Speed = Pse_EstimateSpeed(); /* Get rotor speed from FOC_APPL/Pse using QEI*/
#else
  App_ms_State.Speed = Pse_EstimateSpeedHall(); /* Get rotor speed from FOC_APPL/Pse using Hall*/
#endif /*PAR_USE_HALL_SPEED*/
  App_ms_State.FilteredSpeed = MATH_LowPassFilter(&App_ms_State.SpeedFilterState, App_ms_State.Speed.MechSpeedRPM, 8190);

  if (App_mb_ManualMode){
    App_ms_State.Main_State = FOC_APPL_MANUAL;
  }

  /* Run App_StateMachine */
  FOC_APPL_StateMachine();

  /* Acknowledge IRQ */
  gpio_app->gpio_app_irq_raw = gpio_app->gpio_app_irq_raw;
}

/**
 \brief   FOC application state machine
 */
static void FOC_APPL_StateMachine()
{
  FOC_APPL_STATE_T ls_State;
  ls_State = App_ms_State.Main_State;
  Math_Vector_DQ_T ls_ZeroVector;
  ls_ZeroVector.f_d = 0.0f;
  ls_ZeroVector.f_q = 0.0f;

  switch (ls_State)
  {
  default:
  case FOC_APPL_IDLE: /*  */
    Foc_TransitState(FOC_IDLE);
    if(App_ms_Control.UsrReqSpeed != 0u)
    {
      if (App_ms_State.HallSensorCalibrationState.e_State == CALIBRATION_MODULE_DONE) {
        App_ms_State.Main_State = FOC_APPL_RUN;
      } else{
        App_ms_State.Main_State = FOC_APPL_CALIBRATE;
      }
    }
    break;
  case FOC_APPL_RUN:/* */
    if(App_ms_Control.UsrReqSpeed == 0u)
    {
      App_ms_State.Main_State = FOC_APPL_STOP;
    }
    FOC_APPL_SpeedController();
    Foc_SetRequestCurrent(App_ms_Control.ReqCurrent);
    Foc_TransitState(FOC_CURRENT_CONTROL);
    if(FOC_APPL_StallDetection(App_ms_Control.UsrReqSpeed, App_ms_State.FilteredSpeed))
    {
      App_ms_State.Main_State = FOC_APPL_ERROR;
    }

    break;
  case FOC_APPL_STOP:/* */
    Foc_SetRequestCurrent(ls_ZeroVector);
    if(App_ms_Control.UsrReqSpeed != 0u)
    {
      App_ms_State.Main_State = FOC_APPL_RUN;
    }
    else
    {
      if(App_ms_State.FilteredSpeed <= PSE_SYNC_MECH_SPEED_RPM)
      {
        App_ms_State.Main_State = FOC_APPL_IDLE;
        FOC_APPL_ResetSpeedController();
        Pse_Init();
      }
    }
    break;
  case FOC_APPL_CALIBRATE:
    if (App_ms_State.HallSensorCalibrationState.e_State == CALIBRATION_MODULE_DONE){
      Pse_Init();
      FOC_APPL_ResetSpeedController();
      App_ms_State.Main_State = FOC_APPL_IDLE;
    }
    else if (App_ms_State.HallSensorCalibrationState.e_State == CALIBRATION_MODULE_FAILED){
      App_ms_State.Main_State = FOC_APPL_ERROR;
    }else{
      Foc_TransitState(FOC_VOLTAGE_CONTROL);
      if (Foc_GetFOCState() == FOC_VOLTAGE_CONTROL) {
        FOC_APPL_Calibration();
      }
    }
    break;
  case FOC_APPL_MANUAL:
      Foc_TransitState(FOC_CURRENT_CONTROL);
    break;

  case FOC_APPL_ERROR:/*  */
  //TODO: Not changed but needs to be tested again to make sure it still works.
    Foc_TransitState(FOC_FAILED);
  //  if(!gb_helpPrinted)
    {
      //FOC_APPL_Print_GDU_Errors();
    }
  //  gb_helpPrinted = true;
  //  Foc_TransitState(FOC_FAILED);
  //  if(mpwm_app->mpwm_irq_raw_b.eci_ks_state == 1U)/* iIf error clears */
    {
    //  if(ptData->sUsrReqSpeed == 0u)
      {
     //   gb_helpPrinted = false;
     //   ptData->EFOC_APPLState = FOC_APPL_STOP;
      }
    }
    break;
  }
}
static void FOC_APPL_Calibration(){
  CalibrationControlOutputT ls_Output;
  CalibrationResultT        le_CalibrationResult;
  CalibrationErrorT         ls_CalibrationError;
  uint16_t lu16_Position;

  lu16_Position = Pse_GetPosition().u16_Theta;
  Calibration_Task10ms(&App_ms_State.HallSensorCalibrationState, &App_ms_Parameters.HallSensorCalibrationPar, sens_HallState(), lu16_Position, &ls_Output);

  switch (ls_Output.e_Mode) {
    case CALIBRATION_CONTROL_SET_POSITION:
      Foc_TransitState(FOC_VOLTAGE_CONTROL);
      Pse_SetPositionManually(ls_Output.s16_ElectricalAngle);
      Foc_SetRequestVoltage(ls_Output.s_RequestedVoltage);
      break;
    case CALIBRATION_CONTROL_SET_SPEED:
      Foc_TransitState(FOC_VOLTAGE_CONTROL);
      Pse_SetSpeedManually(ls_Output.s16_SpeedRPM);
      Foc_SetRequestVoltage(ls_Output.s_RequestedVoltage);
      break;
    default:
    case CALIBRATION_CONTROL_OFF:
      Foc_TransitState(FOC_IDLE);
      break;
  }

  le_CalibrationResult = Calibration_GetStatus(&App_ms_State.HallSensorCalibrationState);
  switch (le_CalibrationResult) {
    case CALIBRATION_RESULT_OK:
      /* Copy calibration results */
      Pse_SetHallBoundaries((uint16_t *) &App_ms_State.HallSensorCalibrationState.s_Sector.u16_Calibrated,
                            (uint8_t *) &App_ms_State.HallSensorCalibrationState.u8_HallStates);
      break;

    case CALIBRATION_RESULT_FAILED:
      /* enter safe application state */
      Calibration_GetErrors(&App_ms_State.HallSensorCalibrationState, &ls_CalibrationError);
      break;

    default:
    case CALIBRATION_RESULT_BUSY:
      /* Wait */
      break;
  }

}
static bool FOC_APPL_StallDetection(int16_t as16_RequestedSpeed, int16_t as16_MeasuredSpeed){

  if(Math_s16_Abs(as16_RequestedSpeed -  as16_MeasuredSpeed) > STALL_DETECTION_SPEED_DIFFERENCE){
    /* Increase stall counter */
    App_mu16_StallCounter10ms++;
  }else{
    /* Reset stall counter */
    App_mu16_StallCounter10ms = 0u;
  }

  //if ( App_mu16_StallCounter10ms > STALL_DETECTION_TIME_10MS){
    /* If speed can not be reached within defined time, function is set to stall. */
    //return true;
  //} else
  {
    return false;
  }
}


/**
 \brief The de-initialize function is supposed to reset the handle
 \param [out] tHnd  FOC APPL data struct
 \return DRV_OK
 \       DRV_ERROR
 */
DRV_STATUS_E FOC_APPL_deInit()
{
  DRV_STATUS_E eRet; /* driver status declaration */

  mpwm_DisableGDU();

  eRet = DRV_TIM_Stop(&tTimerTask); /* Stops the timer */
  if(DRV_OK != eRet)
  {
    return DRV_ERROR;
  }

  eRet = DRV_TIM_Stop(&tTimerTask); /* Stops the timer */
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

  return DRV_OK;
}
