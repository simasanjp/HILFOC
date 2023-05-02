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
/*!
 * \brief      Estimated speed variables.
 */
static Pse_Speed_T Pse_gs_Speed;
/*!
 * \brief     Estimated position variables.
 */
static Pse_Position_T Pse_gs_Position;
#if (PAR_USE_HALL_SPEED == 1)
/*!
 * \brief     Estimated speed by hall.
 */
static Pse_Speed_T Pse_gs_SpeedHall;
/*!
 * \brief    PI-filter parameters for hall speed estimation.
 */
static Math_PI_Controller_Parameter_T Pse_gs_HallPIControlPar;
/*!
 * \brief    PI-filter state for hall speed estimation.
 */
static Math_PI_Controller_State_T Pse_gs_HallPIControlState;
#endif
/*!
 * \brief     QEI speed estimation parameters.
 */
static Pse_MENC_ParameterT Pse_gs_EncoderPar;
/*!
 * \brief     Holds the state of hall sensors.
 */
static Pse_HallState_T Pse_HallStateHybrid;

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
#if (PAR_USE_HALL_SPEED == 1)
  Pse_gs_SpeedHall.ElecSpeedDIGPERPWM = 0;
  Pse_gs_SpeedHall.ElecSpeedRPM = 0;
  Pse_gs_SpeedHall.MechSpeedRPM = 0;
  Pse_gs_SpeedHall.ElecSpeedPERPWM = 0.0f;

  Pse_gs_HallPIControlPar.f_KP = 0.5f;
  Pse_gs_HallPIControlPar.f_KI = 0.0001f;
  Pse_gs_HallPIControlPar.f_LowerLimit = -(float) ((PAR_MAX_SPEED_RPM * PSE_MECH_TO_ELEC_SPEED) / (PSE_ELECRPM_TO_ELECPERPWM ));
  Pse_gs_HallPIControlPar.f_UpperLimit = (float) (PAR_MAX_SPEED_RPM * PSE_MECH_TO_ELEC_SPEED) / (PSE_ELECRPM_TO_ELECPERPWM );
  Pse_gs_HallPIControlState.f_integral = 0.0f;
#endif
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

#if (PAR_USE_OPENLOOP== 1)


/*!
 * \details   By setting position manually, a voltage vector can be created for open loop control.
 * @param[in]     au16_Position Rotor position in uint16 scale.
 */
void Pse_SetPositionManually(uint16_t au16_Position)
{
  Pse_gs_Position.s_State = PSE_MANUAL;
  Pse_gs_Position.u16_Theta = au16_Position;
}
#endif

Pse_Position_T Pse_GetPosition(void)
{
  return Pse_gs_Position;
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
  /* Initialize with 1ms in clock cycles */
  uint32_t lu32_CapturePeriod = (uint32_t) PSE_CAPTURE_FREQ_CLOCK_CYCLE;
  uint32_t lu32_CaptureRPM;
  int16_t ls16_MechSpeedRPM;

  /* Ta + Te is subtracted to find effective period */
  lu32_CapturePeriod = lu32_CapturePeriod - sens_QEITaTe();

  /* Calculate Factor to convert RPM is calculated */
  lu32_CaptureRPM = (PSE_CAPTUREPOS_TO_RPM * PSE_CAPTURE_FREQ) / (lu32_CapturePeriod / PSE_CLOCK_CYCLE_TO_FREQ);

  /* Get delta position from menc */
  Pse_gs_EncoderPar.s32_CapturedPosition = sens_QEIPosition();

  /* Check for overflow */
  if(Pse_gs_EncoderPar.s32_CapturedPosition < 0)
  {
    Pse_gs_EncoderPar.s32_CapturedPosition = PSE_MENC_STEPS_PER_REV - Pse_gs_EncoderPar.s32_CapturedPosition;
  }

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

  /* Calculate mechanical rpm */
  ls16_MechSpeedRPM = (int16_t) (((lu32_CaptureRPM * Pse_gs_EncoderPar.s32_CapturedPosition)) / PSE_MENC_STEPS_PER_REV );

#endif

  /* Low pass filter */
  Pse_gs_Speed.MechSpeedRPM = Pse_gs_Speed.MechSpeedRPM + (ls16_MechSpeedRPM - Pse_gs_Speed.MechSpeedRPM) / 4;

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
 *            Whenever hall state changes new speed value is calculated.      \n
 *            Calculated speed passed through a PI filter for better accuracy.\n
 *            This function returns the filtered speed value.
 * \image    html "Speed Estimation by Hall Sensors.svg"
 */
Pse_Speed_T Pse_EstimateSpeedHall(void)
{
  /* Apply PI filter */
  Pse_gs_Speed.ElecSpeedPERPWM = Pse_gs_Speed.ElecSpeedPERPWM
    + Math_PIController(&Pse_gs_HallPIControlState, &Pse_gs_HallPIControlPar, (Pse_gs_SpeedHall.ElecSpeedPERPWM - Pse_gs_Speed.ElecSpeedPERPWM));
  /* Calculate other speed values */
  Pse_gs_Speed.ElecSpeedRPM = PSE_PWM_FREQ * Pse_gs_Speed.ElecSpeedPERPWM * PSE_MIN_TO_SEC;
  Pse_gs_Speed.MechSpeedRPM = Pse_gs_Speed.ElecSpeedRPM / PSE_MECH_TO_ELEC_SPEED;
  Pse_gs_Speed.ElecSpeedDIGPERPWM = (int16_t) (PSE_ELECPERPWM_TO_ELECPERPWM_DIG * Pse_gs_Speed.ElecSpeedPERPWM);
  if(Pse_HallStateHybrid.u16_count > 666u)
  {
    Pse_gs_Speed.MechSpeedRPM = 0;
  }
  return Pse_gs_Speed;
}
#endif
#if (PAR_USE_HALL_SPEED == 2)
Pse_Speed_T Pse_EstimateSpeedHall(void)
{
  uint32_t *pu32_HallTimes;
  uint32_t lu32_MedianTime;
  float fl_ElecSpeedPerS;

  pu32_HallTimes = sens_HallTimes(); /* Get delta-time for each hall sensor */

  lu32_MedianTime = Math_MedianFilter(pu32_HallTimes); /* Apply median filter */

  /* Calculate electrical speed */
  fl_ElecSpeedPerS = (float)(lu32_MedianTime * PSE_CPUFREQ_TO_SECONDS);

  Pse_gs_Speed.ElecSpeedRPM = (int16_t)(PSE_MIN_TO_SEC/fl_ElecSpeedPerS);
  Pse_gs_Speed.MechSpeedRPM = Pse_gs_Speed.ElecSpeedRPM / PSE_MECH_TO_ELEC_SPEED;
  /* Electrical speed  per mpwm period */
  Pse_gs_Speed.ElecSpeedPERPWM = Pse_gs_Speed.ElecSpeedRPM / PSE_ELECRPM_TO_ELECPERPWM;

  /* Speed DIGITS per mpwm period */
  Pse_gs_Speed.ElecSpeedDIGPERPWM = (int16_t)(PSE_ELECPERPWM_TO_ELECPERPWM_DIG * Pse_gs_Speed.ElecSpeedPERPWM);

  return Pse_gs_Speed;
}
#endif
/*!
 * \details   Estimation is based on Quadrature Encoder(QE) and Hal sensors                       \n
 *            Hal sensor defines the sector with 60 degrees intervals                             \n
 *            QE tracks the position based on estimated speed. Speed also calculated              \n
 *            based on hall sensors for comparison.                                               \n
 *            If estimated position does not match with Hal sensor measurement,                   \n
 *            Estimated position is moved towards hall sensor measurement with a low pass filter. \n
 *            This way QE position is aligned and position is not changed abruptly.
 * \image   html "Position Estimation.svg"
 */
Pse_Position_T Pse_EstimatePosition(void)
{
  uint16_t lu16_LowerTheta;
  uint16_t lu16_UpperTheta;
  uint8_t lu8_ExpectedHallState;

  /* Store previous state */
  Pse_HallStateHybrid.s_prevState = Pse_HallStateHybrid.s_currState;

  /* Measure Hal position */
  Pse_HallStateHybrid.s_currState = sens_HallState();

  if(Pse_HallStateHybrid.s_prevState == Pse_HallStateHybrid.s_currState)
  {
    /* Rotor at the same sector, increase counter */
    Pse_HallStateHybrid.u16_count++;
  }
  else
  {
    /* First check if the new state is valid depends on CW/CCW */
    switch (Pse_HallStateHybrid.s_prevState)
    {
    case PSE_SECTOR_1:
      lu8_ExpectedHallState = PSE_SECTOR_2;
      break;
    case PSE_SECTOR_2:
      lu8_ExpectedHallState = PSE_SECTOR_3;
      break;
    case PSE_SECTOR_3:
      lu8_ExpectedHallState = PSE_SECTOR_4;
      break;
    case PSE_SECTOR_4:
      lu8_ExpectedHallState = PSE_SECTOR_5;
      break;
    case PSE_SECTOR_5:
      lu8_ExpectedHallState = PSE_SECTOR_6;
      break;
    case PSE_SECTOR_6:
      lu8_ExpectedHallState = PSE_SECTOR_1;
      break;
    default:
    case PSE_INVALID:
      lu8_ExpectedHallState = 0xFF;
      Pse_HallStateHybrid.s_prevState = Pse_HallStateHybrid.s_currState;
      break;
    }
    if((Pse_HallStateHybrid.s_currState == lu8_ExpectedHallState) && (Pse_HallStateHybrid.u16_count > 0))
    {
#if (PAR_USE_HALL_SPEED == 1)
      if(Pse_HallStateHybrid.u16_count < 666u)
      {
        Pse_gs_SpeedHall.ElecSpeedPERPWM = 1.0f / (Pse_HallStateHybrid.u16_count * 6u);
      }
      else
      {
        Pse_gs_SpeedHall.ElecSpeedPERPWM = 0.0f;
      }
      /* Calculate other speed values */
      Pse_gs_SpeedHall.ElecSpeedRPM = (float) PSE_PWM_FREQ * Pse_gs_SpeedHall.ElecSpeedPERPWM * PSE_MIN_TO_SEC;
      Pse_gs_SpeedHall.MechSpeedRPM = Pse_gs_SpeedHall.ElecSpeedRPM / PSE_MECH_TO_ELEC_SPEED;
      Pse_gs_SpeedHall.ElecSpeedDIGPERPWM = (int16_t) (PSE_ELECPERPWM_TO_ELECPERPWM_DIG * Pse_gs_SpeedHall.ElecSpeedPERPWM);
      Pse_HallStateHybrid.u16_count = 0;
#endif
    }
    else
    {
      /* Assume hall sensor input is false */
      Pse_HallStateHybrid.u16_count++;
      Pse_HallStateHybrid.s_currState = Pse_HallStateHybrid.s_prevState;
    }

  }

  /* Update estimated position */
  Pse_gs_Position.u16_Theta += Pse_gs_Speed.ElecSpeedDIGPERPWM;

  /* Check if estimated position matches with Hal sensor measurement */
  switch (Pse_HallStateHybrid.s_currState)
  {
  case PSE_SECTOR_1:
    lu16_LowerTheta = PSE_POSITION_SECTOR_1;
    lu16_UpperTheta = PSE_POSITION_SECTOR_2;
    break;

  case PSE_SECTOR_2:
    lu16_LowerTheta = PSE_POSITION_SECTOR_2;
    lu16_UpperTheta = PSE_POSITION_SECTOR_3;
    break;

  case PSE_SECTOR_3:
    lu16_LowerTheta = PSE_POSITION_SECTOR_3;
    lu16_UpperTheta = PSE_POSITION_SECTOR_4;
    break;

  case PSE_SECTOR_4:
    lu16_LowerTheta = PSE_POSITION_SECTOR_4;
    lu16_UpperTheta = PSE_POSITION_SECTOR_5;
    break;

  case PSE_SECTOR_5:
    lu16_LowerTheta = PSE_POSITION_SECTOR_5;
    lu16_UpperTheta = PSE_POSITION_SECTOR_6;
    break;

  case PSE_SECTOR_6:
    lu16_LowerTheta = PSE_POSITION_SECTOR_6;
    lu16_UpperTheta = 0xFFFF;
    break;
  default:
  case PSE_INVALID:
    lu16_LowerTheta = 0xFFFF;
    lu16_UpperTheta = 0xFFFF;
    break;

  }

  if((Pse_gs_Position.u16_Theta >= lu16_LowerTheta) && (Pse_gs_Position.u16_Theta < lu16_UpperTheta))
  { /* Estimated position matches*/
    Pse_gs_Position.s_State = PSE_KNOWN;

  }
  else
  { /* Use Hal measurement */
    Pse_gs_Position.s_State = PSE_UNKNOWN;
    /* Move estimated position towards hall sensor value */
    Pse_gs_Position.u16_Theta = (uint16_t) (Pse_gs_Position.u16_Theta - ((int16_t) (Pse_gs_Position.u16_Theta - lu16_LowerTheta) / 16));

  }

  return Pse_gs_Position;
}
