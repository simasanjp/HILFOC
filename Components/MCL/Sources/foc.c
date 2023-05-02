/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */

#include "foc.h"
#include "par_def.h"
#include "mpwm.h"
#include "adc.h"
#include "pse.h"
#include "mathematics.h"
#include "gatedriver_drv8323.h"

/**
 * \brief Number of measurements for current offset.
 */
#define FOC_CURRENT_OFFSET_MEASURE_NUMBER       (uint8_t)PAR_NUMBER_OF_CURRENT_OFFSET_MEAS
/**
 * \brief Minimum value for current offset.
 */
#define FOC_CURRENT_OFFSET_MIN                  (uint16_t)(2048U - PAR_CURRENT_OFFSET_THRESHOLD_DIG)
/**
 * \brief Maximum value for current offset.
 */
#define FOC_CURRENT_OFFSET_MAX                  (uint16_t)(2048U + PAR_CURRENT_OFFSET_THRESHOLD_DIG)
/**
 * \brief Minimum value for sum of current offset.
 */
#define FOC_CURRENT_OFFSET_SUM_MIN              (uint32_t)((uint16_t)FOC_CURRENT_OFFSET_MEASURE_NUMBER * FOC_CURRENT_OFFSET_MIN)
/**
 * \brief Maximum value for sum of current offset.
 */
#define FOC_CURRENT_OFFSET_SUM_MAX              (uint32_t)((uint16_t)FOC_CURRENT_OFFSET_MEASURE_NUMBER * FOC_CURRENT_OFFSET_MAX)
/**
 * \brief In case of current offset measurement error wait time for next measurement.
 */
#define FOC_CURRENT_OFFSET_WAIT                 (uint16_t)20000u
/**
 * \brief Space-Vector-Modulation limit.
 */
#define FOC_SVM_LIMIT                           (float)((PAR_MAX_DUTY_CYCLE_PERCENT/100.0f) / MATH_SQRT3_OVER_2)
/**
 * \brief Current PI-controller lower limit.
 */
#define FOC_PICONTROL_LOWER_LIM                 (float)(-FOC_SVM_LIMIT)
/**
 * \brief Current PI-controller upper limit.
 */
#define FOC_PICONTROL_UPPER_LIM                 (float)( FOC_SVM_LIMIT)
/**
 * \brief FOC maximum allowed duty cycle.
 */
#define FOC_MAX_DUTY                            ((MPWM_HALF_PERIOD * PAR_MAX_DUTY_CYCLE_PERCENT) / 50U)
/**
 * \brief FOC half period value.
 */
#define FOC_HALF_PERIOD                         (uint16_t)(MPWM_HALF_PERIOD)

/**
 * \brief FOC state definition.
 */
static FocStateMachine_MainStateT Foc_gs_State;
/**
 * \brief Current offset state definition.
 */
static FocCurrentOffset_StateT Foc_gs_CurrentOffsetState;
/**
 * \brief Current offset parameters.
 */
Foc_CurrOffsetParameterT Foc_gs_CurrentOffsetPar;
/**
 * \brief PI-controller state for D axis.
 */
Math_PI_Controller_State_T Foc_gs_PIStateD;
/**
 * \brief PI-controller state for Q axis.
 */
Math_PI_Controller_State_T Foc_gs_PIStateQ;
/**
 * \brief PI-controller state parameters for D axis.
 */
Math_PI_Controller_Parameter_T Foc_gs_PIParD;
/**
 * \brief PI-controller state parameters for Q axis.
 */
Math_PI_Controller_Parameter_T Foc_gs_PIParQ;
/**
 * \brief Requested current vector in DQ frame.
 */
static Math_Vector_DQ_T Foc_gs_ReqCurrDQ;
/**
 * \brief Requested voltage vector in DQ frame.
 */
static Math_Vector_DQ_T Foc_gs_ReqVoltDQ;
/**
 * \brief DC bus ripple elimination parameters.
 */
static Foc_DCBusRipple_ParameterT Foc_gs_DCBusRipple;

GatedriverDrv8323StateT      GatedriverState;
static uint8_t Foc_gu8_Trials;

/**
 * \brief   Open-loop voltage control in dq frame.
 * \details Requested voltage is set with Foc_SetRequestVoltage \n
 *          Position must be set with Pse_SetPositionManually    \n
 *          Function creates a flux field with given parameters.
 */
void Foc_VoltageControl(Pse_Position_T *as_Position);


/**
 * \brief   Closed - loop current control in dq frame.
 * \details Current is set with Foc_SetRequestCurrent via output of speed controller.     \n
 *          Function reads adc current measurements performs Clarke-Park transformations, \n
 *          applies PI control then performs inverse Clarke-Park transformations.         \n
 *          Finally calls Foc_Modulation to apply PWM output .
 */
void Foc_CurrentControl(Pse_Position_T *as_Position);
/**
 * \brief         PI current regulator function.
 * @param[in]     *as_MeasVector Pointer to measured  vector in DQ frame.
 * @param[in]     *as_ReqVector  Pointer to requested  vector in DQ frame.
 * @return        Control vector in DQ frame.
 */
Math_Vector_DQ_T Foc_PICurrentControl(Math_Vector_DQ_T *as_MeasVector, Math_Vector_DQ_T *as_ReqVector);
/**
 * \brief         Function applies UVW duty cycle by interacting MPWM.
 * @param[in]     *aps_Duty  Pointer to requested duty cycle vector in UVW frame.
 */
void Foc_Modulation(Math_Vector_UVW_T *aps_Duty);
/**
 * \brief State machine for current offset measurement.
 */
void Foc_CurrentOffsetStateMachine();

/**
 * \brief         DC-Bus ripple elimination by feed-forward method.
 * @param[in]     *as_VoltageVector  Pointer to voltage vector to be modified in DQ frame.
 * \image   html "DC-Bus Ripple elimination.svg"
 * @return        Modified voltage vector in DQ frame.
 */
Math_Vector_DQ_T Foc_DCBusRippleElimination(Math_Vector_DQ_T *as_VoltageVector);

void Foc_Init()
{
  Foc_gs_State = FOC_IDLE;
  Foc_gs_CurrentOffsetState = CURRENT_OFFSET_INIT;

  Foc_gs_PIStateD.f_integral = 0.0f;
  Foc_gs_PIStateQ.f_integral = 0.0f;

  Foc_gs_PIParD.f_KP = PAR_CURRENT_CONTROLLER_KP;
  Foc_gs_PIParD.f_KI = PAR_CURRENT_CONTROLLER_KI;
  Foc_gs_PIParD.f_LowerLimit = FOC_PICONTROL_LOWER_LIM;
  Foc_gs_PIParD.f_UpperLimit = FOC_PICONTROL_UPPER_LIM;

  Foc_gs_PIParQ.f_KP = PAR_CURRENT_CONTROLLER_KP;
  Foc_gs_PIParQ.f_KI = PAR_CURRENT_CONTROLLER_KI;
  Foc_gs_PIParQ.f_LowerLimit = FOC_PICONTROL_LOWER_LIM;
  Foc_gs_PIParQ.f_UpperLimit = FOC_PICONTROL_UPPER_LIM;
}

/*!
 * \details  Called from PWM interrupt. Performs configured tasks.
 */
void Foc_StateMachine()
{
  Pse_Position_T ls_EstimatedPosition;
  /* Gate driver state machine */
  GatedriverDrv8323_PwmPeriod(&GatedriverState);

  if (GatedriverState.e_MainState == GATEDRIVER_DRV8323_FAILED){
    /* In case GDU is failed, FOC state is set to FOC_FAILED */
    Foc_gs_State = FOC_FAILED;
    Foc_gu8_Trials++;
  }

  switch (Foc_gs_State)
  {
  default:
  case FOC_IDLE: /* Disable PWM output */
    Foc_gs_ReqCurrDQ.f_d = 0.0f;
    Foc_gs_ReqCurrDQ.f_q = 0.0f;
    Foc_gs_ReqVoltDQ.f_d = 0.0f;
    Foc_gs_ReqVoltDQ.f_q = 0.0f;
    mpwm_DisablePWM();
    break;
  case FOC_CURRENT_OFFSET: /* Enable low side and  measure current offset*/
    Foc_CurrentOffsetStateMachine();
    break;
  case FOC_CURRENT_CONTROL:/* FOC current control */
    ls_EstimatedPosition = Pse_EstimatePosition();
    Foc_CurrentControl(&ls_EstimatedPosition);
    break;
  case FOC_VOLTAGE_CONTROL:/* FOC voltage control for open-loop */
    ls_EstimatedPosition = Pse_GetPosition();
    Foc_VoltageControl(&ls_EstimatedPosition);
    break;
  case FOC_PWM_PATTERN:/* FOC manual pwm pattern */
    break;
  case FOC_FAILED:/* Error state */
      mpwm_DisablePWM();
      /* SW stays at the failed state after 10 trials. */
      if(Foc_gu8_Trials < 10)
      {
        GatedriverDrv8323_Prepare(&GatedriverState, GATEDRIVER_OPERATION_ENABLE);
      }

      break;
    }
  }

  /*!
   * \details  FOC states can not be changed directly.                     \n
   *           Application requests to change the state via this function. \n
   *           This ensures transitioning safely.
   * @param[in]     as_State Requested state to transit.
   */
void Foc_TransitState(FocStateMachine_MainStateT as_State)
{
  FocCurrentOffset_StateT ls_CurrState = Foc_gs_CurrentOffsetState;
  FocStateMachine_MainStateT ls_State = Foc_gs_State;
  if(ls_State != as_State)
  {
    switch (as_State)
    {
    default:
    case FOC_IDLE: /* Transit to idle state */
      /* Do not change state while current measurement is ongoing */
      if((ls_CurrState == CURRENT_OFFSET_INIT) || (ls_CurrState == CURRENT_OFFSET_DONE))
      {
        Foc_gs_State = FOC_IDLE;
      }
      break;
    case FOC_CURRENT_OFFSET:
      /* Check if motor stopped then transit to idle first, handled in Application state machine. */
      break;
    case FOC_CURRENT_CONTROL:/* Switch to FOC current control */
    case FOC_VOLTAGE_CONTROL:/* Switch to FOC voltage control */
      if((ls_CurrState == CURRENT_OFFSET_INIT) && (ls_State == FOC_IDLE))
      {
        Foc_gs_State = FOC_CURRENT_OFFSET;
      }
      else if(ls_CurrState == CURRENT_OFFSET_DONE)
      {
        Foc_gs_State = as_State;
      }
      else if(ls_State == FOC_FAILED)
      {
        //TODO: Things to do before transitioning from error state
        // Read Gdu error
        // Reinit GDU

      }
      else
      {
        /*Do nothing */
      }
      break;
    case FOC_PWM_PATTERN:/* FOC manual pwm pattern */
    case FOC_FAILED:/* Transit to Error state */
      Foc_gs_State = as_State;
      break;
    }
  }
}

/*!
 * \details       Use only in closed - loop mode.
 * @param[in]     as_ReqCurrent Requested current in DQ frame.
 */
void Foc_SetRequestCurrent(Math_Vector_DQ_T as_ReqCurrent)
{
  Foc_gs_ReqCurrDQ = as_ReqCurrent;
}


/*!
 * \details       Use only in open-loop mode.
 * @param[in]     as_ReqVoltage Requested voltage in DQ frame.
 */
void Foc_SetRequestVoltage(Math_Vector_DQ_T as_ReqVoltage)
{
  Foc_gs_ReqVoltDQ = as_ReqVoltage;
}

Math_Vector_DQ_T Foc_GetRequestVoltage()
{
  return Foc_gs_ReqVoltDQ;
}

FocStateMachine_MainStateT Foc_GetFOCState(){
  return Foc_gs_State;
}

void Foc_VoltageControl(Pse_Position_T *as_Position)
{
  Math_Vector_AB_T voltageAB;
  Math_Vector_UVW_T voltageUVW;
  Math_Vector_UVW_T voltageUVW_svm;
  Math_Vector_SC_T ls_SinCos;

  ls_SinCos = Math_SinCos(as_Position->u16_Theta);

  /* Open loop voltage control */
  voltageAB = Math_InversePark(&Foc_gs_ReqVoltDQ, &ls_SinCos);
  voltageUVW = Math_InverseClarke(&voltageAB);
  voltageUVW_svm = Math_SVM(&voltageUVW);

  Foc_Modulation(&voltageUVW_svm);
}

void Foc_CurrentControl(Pse_Position_T *as_Position)
{
  Math_Vector_DQ_T MeasCurrDQ;
  Math_Vector_AB_T MeasCurrAB;
  Math_Vector_UVW_T MeasCurrUVW;

  Math_Vector_AB_T VoltageAB;
  Math_Vector_UVW_T VoltageUVW;
  Math_Vector_UVW_T VoltageUVW_svm;
  Math_Vector_SC_T ls_SinCos;


  ls_SinCos = Math_SinCos(as_Position->u16_Theta);

  MeasCurrUVW = adc_GetMeasuredCurrents();
  MeasCurrAB = Math_Clarke(&MeasCurrUVW);
  MeasCurrDQ = Math_Park(&MeasCurrAB, &ls_SinCos);

  Foc_gs_ReqVoltDQ = Foc_PICurrentControl(&MeasCurrDQ, &Foc_gs_ReqCurrDQ);
  Foc_gs_ReqVoltDQ = Foc_DCBusRippleElimination(&Foc_gs_ReqVoltDQ);


  VoltageAB = Math_InversePark(&Foc_gs_ReqVoltDQ, &ls_SinCos);
  VoltageUVW = Math_InverseClarke(&VoltageAB);
  VoltageUVW_svm = Math_SVM(&VoltageUVW);

  Foc_Modulation(&VoltageUVW_svm);
}

Math_Vector_DQ_T Foc_PICurrentControl(Math_Vector_DQ_T *as_MeasVector, Math_Vector_DQ_T *as_ReqVector)
{
  Math_Vector_DQ_T l_Return;
  Math_Vector_DQ_T l_Errors;

  l_Errors.f_d = as_ReqVector->f_d - as_MeasVector->f_d;
  l_Errors.f_q = as_ReqVector->f_q - as_MeasVector->f_q;

  l_Return.f_d = Math_PIController(&Foc_gs_PIStateD, &Foc_gs_PIParD, l_Errors.f_d);
  l_Return.f_q = Math_PIController(&Foc_gs_PIStateQ, &Foc_gs_PIParQ, l_Errors.f_q);

  return l_Return;
}

void Foc_Modulation(Math_Vector_UVW_T *aps_Duty)
{
  uint16_t lu16_DutyA;
  uint16_t lu16_DutyB;
  uint16_t lu16_DutyC;

  lu16_DutyA = (uint16_t) ((int16_t) (aps_Duty->f_u * FOC_HALF_PERIOD ) + FOC_HALF_PERIOD );
  lu16_DutyB = (uint16_t) ((int16_t) (aps_Duty->f_v * FOC_HALF_PERIOD ) + FOC_HALF_PERIOD );
  lu16_DutyC = (uint16_t) ((int16_t) (aps_Duty->f_w * FOC_HALF_PERIOD ) + FOC_HALF_PERIOD );

  if(lu16_DutyA > FOC_MAX_DUTY)
  {
    lu16_DutyA = FOC_MAX_DUTY;
  }
  if(lu16_DutyB > FOC_MAX_DUTY)
  {
    lu16_DutyB = FOC_MAX_DUTY;
  }
  if(lu16_DutyC > FOC_MAX_DUTY)
  {
    lu16_DutyC = FOC_MAX_DUTY;
  }
  mpwm_SetDutyCycleA(lu16_DutyA);
  mpwm_SetDutyCycleB(lu16_DutyB);
  mpwm_SetDutyCycleC(lu16_DutyC);
  mpwm_EnablePWM();
}

Math_Vector_DQ_T Foc_DCBusRippleElimination(Math_Vector_DQ_T *as_VoltageVector)
{
  Math_Vector_DQ_T s_Return;
  float lf_VoltRatio;

  Foc_gs_DCBusRipple.f_DCBusVoltage = adc_GetVSenVM();
  Foc_gs_DCBusRipple.u8_Index++;

  if(Foc_gs_DCBusRipple.u8_Index >= 4U)
  {
    Foc_gs_DCBusRipple.u8_Index = 0;
  }
  Foc_gs_DCBusRipple.f_DiscardVoltage = Foc_gs_DCBusRipple.f_DCBusArray[Foc_gs_DCBusRipple.u8_Index];

  Foc_gs_DCBusRipple.f_DCBusArray[Foc_gs_DCBusRipple.u8_Index] = Foc_gs_DCBusRipple.f_DCBusVoltage;

  Foc_gs_DCBusRipple.f_Sum += Foc_gs_DCBusRipple.f_DCBusVoltage; /* Add new capture value */
  Foc_gs_DCBusRipple.f_Sum -= Foc_gs_DCBusRipple.f_DiscardVoltage; /* Discard first value */

  lf_VoltRatio = ((Foc_gs_DCBusRipple.f_Sum) / 4U) / Foc_gs_DCBusRipple.f_DCBusVoltage;

  if((lf_VoltRatio < 1.05f) && (lf_VoltRatio > 0.95f))
  {
    s_Return.f_d = as_VoltageVector->f_d * lf_VoltRatio;
    s_Return.f_q = as_VoltageVector->f_q * lf_VoltRatio;
  }
  else
  {
    s_Return.f_d = as_VoltageVector->f_d;
    s_Return.f_q = as_VoltageVector->f_q;
  }
  return s_Return;
}

void Foc_CurrentOffsetStateMachine()
{

  FocCurrentOffset_StateT lp_State = Foc_gs_CurrentOffsetState;
  adcCurrentOffsetT l_CurrentOffset;

  switch (lp_State)
  {
  default:
  case CURRENT_OFFSET_INIT: /* Init gatedriver(write config and check for errors) then enable GDU and set Low side On */
    Foc_gs_CurrentOffsetPar.u8_NumberOfMeasurements = 0u;
    Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_u = 0U;
    Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_v = 0U;
    Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_w = 0U;

    mpwm_EnableLowSide();
    mpwm_EnablePWM();
    Foc_gs_CurrentOffsetState = CURRENT_OFFSET_MEASURE;

    break;
  case CURRENT_OFFSET_MEASURE: /*  */
    if(Foc_gs_CurrentOffsetPar.u8_NumberOfMeasurements <= (FOC_CURRENT_OFFSET_MEASURE_NUMBER ))
    {
      /* Scrap first measurement */
      l_CurrentOffset = adc_MeasureCurrentOffset();
      if(Foc_gs_CurrentOffsetPar.u8_NumberOfMeasurements > 0u)
      {

        Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_u += l_CurrentOffset.u16_CurrentOffset_u;
        Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_v += l_CurrentOffset.u16_CurrentOffset_v;
        Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_w += l_CurrentOffset.u16_CurrentOffset_w;
      }

      Foc_gs_CurrentOffsetPar.u8_NumberOfMeasurements++;
    }
    else
    {

      Foc_gs_CurrentOffsetState = CURRENT_OFFSET_CALC;
    }
    break;
  case CURRENT_OFFSET_CALC: /*  */

    if((Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_u > FOC_CURRENT_OFFSET_SUM_MIN )
      && (Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_u < FOC_CURRENT_OFFSET_SUM_MAX )
      && (Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_v > FOC_CURRENT_OFFSET_SUM_MIN )
      && (Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_v < FOC_CURRENT_OFFSET_SUM_MAX )
      && (Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_w > FOC_CURRENT_OFFSET_SUM_MIN )
      && (Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_w < FOC_CURRENT_OFFSET_SUM_MAX ))
    {
      l_CurrentOffset.u16_CurrentOffset_u = (uint16_t) (Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_u / FOC_CURRENT_OFFSET_MEASURE_NUMBER );
      l_CurrentOffset.u16_CurrentOffset_v = (uint16_t) (Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_v / FOC_CURRENT_OFFSET_MEASURE_NUMBER );
      l_CurrentOffset.u16_CurrentOffset_w = (uint16_t) (Foc_gs_CurrentOffsetPar.u32_SumCurrOffset_w / FOC_CURRENT_OFFSET_MEASURE_NUMBER );
      adc_SetCurrentOffsets(&l_CurrentOffset);
      Foc_gs_CurrentOffsetState = CURRENT_OFFSET_DONE;
    }
    else
    {
      Foc_gs_CurrentOffsetState = CURRENT_OFFSET_FAILED;
    }
    break;
  case CURRENT_OFFSET_DONE:
    break;
  case CURRENT_OFFSET_FAILED: /* If current offset failed wait some time and reinitialize */
    Foc_gs_CurrentOffsetPar.u16_CurrOffsetCounter++;
    mpwm_DisablePWM();
    if(Foc_gs_CurrentOffsetPar.u16_CurrOffsetCounter > FOC_CURRENT_OFFSET_WAIT)
    {
      Foc_gs_CurrentOffsetState = CURRENT_OFFSET_INIT;
      Foc_gs_CurrentOffsetPar.u16_CurrOffsetCounter = 0u;
    }
    break;
  }
}
