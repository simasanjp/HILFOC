/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */

#include "pse.h"
#include "par_def.h"
#include "mpwm.h"
#include "sens.h"
#include "mathematics.h"
/*!
 * \brief    Number of signals per revolution from encoder.
 */
#define PSE_MENC_STEPS_PER_REV              (int32_t)(PAR_QEI_NUMBER_OF_SLOTS * 4U)
/*!
 * \brief     Speed capture frequency.
 */
#define PSE_CAPTURE_FREQ                    PAR_APP_FREQ_HZ
/*!
 * \brief     One minute in seconds.
 */
#define PSE_MIN_TO_SEC                      60.0f
/*!
 * \brief     Used to convert to RPM.
 */
#define PSE_CAPTUREPOS_TO_RPM               (int32_t)(PSE_MIN_TO_SEC * PSE_CAPTURE_FREQ)
/*!
 * \brief    Mechanical to electrical conversation factor.
 */
#define PSE_MECH_TO_ELEC_SPEED              PAR_MOTOR_POLE_PAIRS
/*!
 * \brief     PWM frequency.
 */
#define PSE_PWM_FREQ                        (((PAR_CPU_FREQ_HZ/2U)/MPWM_HALF_PERIOD)/(EVENT_COUNTER_TOP + 1U))
/*!
 * \brief     Electrical in RPM to per PWM conversation factor.
 */
#define PSE_ELECRPM_TO_ELECPERPWM           (float)(PSE_PWM_FREQ * PSE_MIN_TO_SEC)
#/*!
 * \brief    Electrical speed revolution per PWM to Digits per PWM.
 */
#define PSE_ELECPERPWM_TO_ELECPERPWM_DIG    (0xFFFF)
/*!
 * \brief     Capture frequency clock cycle.
 */
#define PSE_CAPTURE_FREQ_CLOCK_CYCLE        (PAR_CPU_FREQ_HZ/PSE_CAPTURE_FREQ)
/*!
 * \brief     Clock cycle to frequency.
 */
#define PSE_CLOCK_CYCLE_TO_FREQ             (PSE_CAPTURE_FREQ_CLOCK_CYCLE / PSE_CAPTURE_FREQ)
/*!
 * \brief     Capture frequency to seconds.
 */
#define PSE_CPUFREQ_TO_SECONDS                  (1.0f/PAR_CPU_FREQ_HZ)


int16_t  Pse_gs16_MechSpeedRPM;
/*!
 * \brief      Estimated speed variables.
 */
static Pse_Speed_T Pse_gs_Speed;
/*!
 * \brief     Estimated position variables.
 */
static Pse_Position_T Pse_gs_Position;
/*!
 * \brief     Calibrated hall state boundaries.
 */
static Pse_HallStateBoundaries_T Pse_gs_HallBoundaries;


/*!
 * \brief     Estimated speed by hall.
 */
static Pse_Speed_T Pse_gs_SpeedHall;
/*!
 * \brief    PI-filter parameters for hall speed estimation.
 */
static MATH_Fixedpt_PI_Controller_Parameter_T Pse_gs_HallPIControlPar;
/*!
 * \brief    PI-filter state for hall speed estimation.
 */
static MATH_Fixedpt_PI_Controller_State_T Pse_gs_HallPIControlState;

/*!
 * \brief     QEI speed estimation parameters.
 */
static Pse_MENC_ParameterT Pse_gs_EncoderPar;
/*!
 * \brief     Speed Filter State.
 */
static int32_t Pse_SpeedFilterState;


/*!
 * \details   Initialize parameters and Sensors (hall and QEI).
 */
void Pse_Init()
{
  /* Initialize variables */
  Pse_gs_Speed.ElecSpeedDIGPERPWM = 0;
  Pse_gs_Speed.ElecSpeedRPM = 0;
  Pse_gs_Speed.MechSpeedRPM = 0;
  Pse_gs_Speed.ElecSpeedPERPWM = 0.0f;


  Pse_gs_SpeedHall.ElecSpeedDIGPERPWM = 0;
  Pse_gs_SpeedHall.ElecSpeedRPM = 0;
  Pse_gs_SpeedHall.MechSpeedRPM = 0;
  Pse_gs_SpeedHall.ElecSpeedPERPWM = 0.0f;

  Pse_gs_HallPIControlPar.s32_KP = 400;
  Pse_gs_HallPIControlPar.s32_KI = 10;
  Pse_gs_HallPIControlPar.s32_LowerLimit = -(32000 << 16);
  Pse_gs_HallPIControlPar.s32_UpperLimit = (32000 << 16);
  Pse_gs_HallPIControlState.s32_i_n = 0;

  Pse_gs_Position.u16_Theta = 0u;
  Pse_gs_Position.s_State = PSE_UNKNOWN;
  Pse_gs_EncoderPar.s32_CapturedPosition = 0;
  Pse_gs_EncoderPar.s32_DiscardPositions = 0;
  Pse_gs_EncoderPar.s32_SumPositions = 0;
  Pse_gs_EncoderPar.u8_Index = 0;

  /* Initialize sensors */
  sens_HallInit();
  sens_QEIInit();
}

/*!
 * \details   By setting position manually, a voltage vector can be created for open loop control.
 * @param[in]     au16_Position Rotor position in uint16 scale.
 */
void Pse_SetPositionManually(uint16_t au16_Position)
{
  Pse_gs_Position.s_State = PSE_MANUAL;
  Pse_gs_Position.u16_Theta = au16_Position;
  Pse_gs_Speed.ElecSpeedDIGPERPWM = 0;
}


Pse_Position_T Pse_GetPosition(void)
{
  Pse_gs_Position.s_State = PSE_MANUAL;
  Pse_gs_Position.u16_Theta += Pse_gs_Speed.ElecSpeedDIGPERPWM;
  return Pse_gs_Position;
}

Pse_Speed_T Pse_GetSpeed(void){
  return Pse_gs_Speed;
}

void Pse_SetSpeedManually(int16_t as16_SpeedRPM){
  Pse_gs_Speed.MechSpeedRPM = as16_SpeedRPM;

  /* Calculate Electrical rpm */
  Pse_gs_Speed.ElecSpeedRPM = Pse_gs_Speed.MechSpeedRPM * PSE_MECH_TO_ELEC_SPEED;

  /* Electrical speed  per mpwm period */
  Pse_gs_Speed.ElecSpeedPERPWM = Pse_gs_Speed.ElecSpeedRPM / PSE_ELECRPM_TO_ELECPERPWM;

  /* Speed DIGITS per mpwm period */
  Pse_gs_Speed.ElecSpeedDIGPERPWM = (int16_t) (PSE_ELECPERPWM_TO_ELECPERPWM_DIG * Pse_gs_Speed.ElecSpeedPERPWM);
}

void Pse_SetHallBoundaries(uint16_t * ap_Values, uint8_t * ap_HallState){
  uint16_t lu16_Mid;
  uint8_t lu8_Prev;
  for(uint8_t i = 0; i < 6; i++) {
    if (i > 0u) {
      lu8_Prev = i - (uint8_t) 1u;
    } else {
      lu8_Prev = 5u;
    }
    /* Overflow is intended */
    lu16_Mid = ((uint16_t)(ap_Values[i] - ap_Values[lu8_Prev])/2u);
    Pse_gs_HallBoundaries.u16_CalibratedMidPoints[i] = ap_Values[lu8_Prev] + lu16_Mid;
    Pse_gs_HallBoundaries.u16_CalibratedBoundaries[i] = ap_Values[i];
    Pse_gs_HallBoundaries.u8_CalibratedHallStates[i] = ap_HallState[i];
    Pse_gs_HallBoundaries.u8_HallStatesToIndex[ap_HallState[i]] = i;
  }
}
/*!
 * \details   Speed is estimated by counting signals from QEI. \n
 *            For better accuracy Ta + Te is subtracted from   \n
 *            sampling period to find effective period.        \n
 *            CCW is positive direction.
 * \image    html "Speed Estimation.svg"
 */
Pse_Speed_T Pse_EstimateSpeed(void)
{
  /* Initialize with 10ms in clock cycles */
  uint32_t lu32_CapturePeriod = (uint32_t) PSE_CAPTURE_FREQ_CLOCK_CYCLE;
  uint32_t lu32_CaptureRPM;


  /* Ta + Te is subtracted to find effective period */
  lu32_CapturePeriod = lu32_CapturePeriod - sens_QEITaTe();

  /* Calculate Factor to convert RPM is calculated */
  lu32_CaptureRPM = (PSE_CAPTUREPOS_TO_RPM * PSE_CAPTURE_FREQ) / (lu32_CapturePeriod / PSE_CLOCK_CYCLE_TO_FREQ);

  /* Get delta position from menc */
  Pse_gs_EncoderPar.s32_CapturedPosition = sens_QEIPosition();

#if (PAR_USE_MAF_CAPTURE == 1U)

  Pse_gs_EncoderPar.u8_Index++;

  if (Pse_gs_EncoderPar.u8_Index >= PSE_CAPTURE_ARRAY_SIZE)
  {
    Pse_gs_EncoderPar.u8_Index = 0;

  }
  Pse_gs_EncoderPar.s32_DiscardPositions = Pse_gs_EncoderPar.s32_PositionArray[Pse_gs_EncoderPar.u8_Index];

  Pse_gs_EncoderPar.s32_PositionArray[Pse_gs_EncoderPar.u8_Index] = Pse_gs_EncoderPar.s32_CapturedPosition;

  Pse_gs_EncoderPar.s32_SumPositions += Pse_gs_EncoderPar.s32_CapturedPosition; /* Add new capture value */
  Pse_gs_EncoderPar.s32_SumPositions -= Pse_gs_EncoderPar.s32_DiscardPositions; /* Discard first value */
  ls16_MechSpeedRPM = (int16_t)(((lu32_CaptureRPM * Pse_gs_EncoderPar.s32_SumPositions) / PSE_CAPTURE_ARRAY_SIZE)/PSE_MENC_STEPS_PER_REV);

#else
  /* Check for overflow */
  if(Pse_gs_EncoderPar.s32_CapturedPosition >= 0) {
    /* Calculate mechanical rpm */
    Pse_gs_Speed.MechSpeedRPM =
        (int16_t) (((lu32_CaptureRPM * Pse_gs_EncoderPar.s32_CapturedPosition)) / PSE_MENC_STEPS_PER_REV);
  }

#endif

  /* Calculate Electrical rpm */
  Pse_gs_Speed.ElecSpeedRPM = Pse_gs_Speed.MechSpeedRPM * PSE_MECH_TO_ELEC_SPEED;

  /* Electrical speed  per mpwm period */
  Pse_gs_Speed.ElecSpeedPERPWM = Pse_gs_Speed.ElecSpeedRPM / PSE_ELECRPM_TO_ELECPERPWM;

  /* Speed DIGITS per mpwm period */
  Pse_gs_Speed.ElecSpeedDIGPERPWM = (int16_t) (PSE_ELECPERPWM_TO_ELECPERPWM_DIG * Pse_gs_Speed.ElecSpeedPERPWM);

  return Pse_gs_Speed;
}

#if (PAR_USE_HALL_SPEED == 1)
/*!
 * \details   Speed is estimated by sampling signals from hall sensor.        \n
 *            Whenever function is called, values are converted.      \n
 *
 * \image    html "Speed Estimation by Hall Sensors.svg"
 */

Pse_Speed_T Pse_EstimateSpeedHall(void)
{
  Pse_gs_Speed.ElecSpeedPERPWM = (float)(Pse_gs_Speed.ElecSpeedDIGPERPWM / 65535.0f);
  /* Calculate other speed values */
  Pse_gs_Speed.ElecSpeedRPM = (int16_t)((float)PSE_PWM_FREQ * (float)(Pse_gs_Speed.ElecSpeedPERPWM * PSE_MIN_TO_SEC));
  Pse_gs_Speed.MechSpeedRPM = Pse_gs_Speed.ElecSpeedRPM / PSE_MECH_TO_ELEC_SPEED;

  if (Pse_gs_Speed.MechSpeedRPM > 10000){
    Pse_gs_Speed.MechSpeedRPM = 10000;
  }

  return Pse_gs_Speed;
}
#endif


/*!
 * \details   Estimation is based on Quadrature Encoder(QE) and Hal sensors                       \n
 *            Hal sensor defines the sector with 60 degrees intervals                             \n
 *            QE tracks the position based on estimated speed. Speed also calculated              \n
 *            based on hall sensors for comparison.                                               \n
 *            If estimated position does not match with Hal sensor measurement,                   \n
 *            Estimated position is moved towards hall sensor measurement with a PI filter.       \n
 * \image   html "Position Estimation.svg"
 */

Pse_Position_T Pse_EstimatePosition(void)
{
  uint16_t  lu16_HallSensorTheta;
  int32_t   ls32_PrevTheta32;
  int32_t   ls32_CurrTheta32;
  int32_t   ls32_Speed32;
  int16_t   ls16_Error;
  int16_t   ls16_H;
  int16_t   ls16_T;
  uint8_t   lu8_currentState;


  ls32_PrevTheta32 = (int16_t)((int32_t)Pse_gs_Position.u16_Theta - (int32_t)32767);
  /* Measure Hal position */
  lu8_currentState = sens_HallState();

  lu16_HallSensorTheta = Pse_gs_HallBoundaries.u16_CalibratedMidPoints[Pse_gs_HallBoundaries.u8_HallStatesToIndex[lu8_currentState]];
  ls16_T = (int16_t)((int32_t)Pse_gs_Position.u16_Theta - (int32_t)32767);
  ls16_H = (int16_t)((int32_t)lu16_HallSensorTheta - (int32_t)32767);
  ls16_Error = ls16_H - ls16_T;


  ls16_T  += MATH_Fixedpt_PI_Controller(&Pse_gs_HallPIControlState, &Pse_gs_HallPIControlPar, ls16_Error);

  Pse_gs_Position.u16_Theta  = (uint16_t)((int32_t)ls16_T + (int32_t)32767);

  ls32_CurrTheta32 = ls16_T;
  ls32_Speed32 = (int16_t)((int16_t)ls32_CurrTheta32 - (int16_t)ls32_PrevTheta32);
  Pse_gs_SpeedHall.ElecSpeedDIGPERPWM = Pse_gs_SpeedHall.ElecSpeedDIGPERPWM + ((int16_t)(ls32_Speed32) - Pse_gs_SpeedHall.ElecSpeedDIGPERPWM)/2;
  Pse_gs_Speed.ElecSpeedDIGPERPWM = MATH_LowPassFilter(&Pse_SpeedFilterState, Pse_gs_SpeedHall.ElecSpeedDIGPERPWM, 32767);

  return Pse_gs_Position;
}
