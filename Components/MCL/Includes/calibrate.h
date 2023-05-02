/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */

/**
 * \file calibrate.h
 *
 * \brief  Calibrate hall sensor.
 */
#ifndef __CALIBRATE_H_
#define __CALIBRATE_H_
#include <stdint.h>
#include "mathematics.h"
/**
 * \brief the calibration main state machine enumeration.
 */
typedef enum {
  CALIBRATION_MODULE_INIT,                        /**< initial state, prepares the second state */
  CALIBRATION_MODULE_WAIT_FOR_INITIAL_ALIGNMENT,  /**< a fixed current (Id = Requested, Iq = 0) is applied in direction of electrically 0 deg for a given duration */
  CALIBRATION_MODULE_TURN_CW,                   /**< the fixed current is rotated in mathematically positive direction */
  CALIBRATION_MODULE_WAIT_FOR_DIRECTION_CHANGE_ALIGNMENT, /**< the fixed current is set to speed 0 for the rotor to stop */
  CALIBRATION_MODULE_TURN_CCW,                  /**< the fixed current is rotated in mathematically negative direction */
  CALIBRATION_MODULE_EVALUATE,
  CALIBRATION_MODULE_VALIDATE,                  /**< the fixed current is applied in direction of electrically 0 deg, the mechanical position is read back after a time and checked */
  CALIBRATION_MODULE_DONE,                        /**< the calibration terminated successfully, final state */
  CALIBRATION_MODULE_FAILED                       /**< the calibration failed, final state */
} CalibrationMainState_T;

/**
 * \brief the calibration error result
 *
 */
typedef struct CalibrationErrorS {
  uint8_t u1_AlignmentFailed; /**< the validation substates d and q did not reach accurate results */
  uint8_t u1_WrongHallSequence;    /**< hall sequence detected doesn't match CW/CCW */
} CalibrationErrorT;
/**
 * \brief the calibration Sector data
 *
 */
typedef struct CalibrationSectorS {
  uint16_t                          u16_SamplesCW[6];
  uint16_t                          u16_SamplesCCW[6];
  uint16_t                          u16_Calibrated[6];
} CalibrationSectorT;
/**
 *\brief the calibration state
 *
 * This state is maintained by the component itself and should never be written from external functions.
 * The only reason to read values from it is to read the result (s_CalibrationValues).
 *
 */
typedef struct CalibrationStateS {
  CalibrationMainState_T            e_State;                        /**< main calibration state */
  uint8_t                           u8_LastHallState;        /**< the mechanical angle of the last invocation */
  uint16_t                          u16_LastTheta;                  /**< the electrical angle of the last invocation */
  uint16_t                          u16_ToWait10Msec;
  CalibrationSectorT                s_Sector;
  uint8_t                           u8_HallStates[6];
  int8_t                            s8_Index;
  uint8_t                           u8_nthTurn;
  CalibrationErrorT                 s_Error;
  /**< errors when the calibration failed, should not be read directly, see Calibration_GetErrors */
} CalibrationStateT;

/**
 * \brief parameters for the calibration process
 *
 * This structure contains all parameters needed for performing the calibration.
 */
typedef struct CalibrationParameterS {
  float    f_RequestedVoltage;
  /**< the length of the current vector used for calibration in units of the commutation, the direction is always Id */
  int16_t  s16_SamplingSpeedRPM;
  /**< the speed during turn CW and turn CCW operation in units of the commutation */
  uint16_t u16_WaitTimeBetweenEachSample10Msec;
  /**< wait time in 4ms when turing left and right before performing linear regression on another sample of (mechanical angle, electrical angle) */
  uint16_t u16_WaitTimeAfterAlignment10Msec;
  /**< wait time in 4ms during the alignment phases (initial alignment, direction change, validation) */
  int16_t  s16_NumberOfTurns;
  /**< number of turns (mechanical angle, electrical angle) of the linear regression in one direction */
  uint16_t  u16_MaximumDeviationAngle;
  /**< maximum electrical angle deviation when validating the calibration values, unit complies with Angle definition above */
} CalibrationParameterT;

/**
 * \brief electrical angle source requested
 *
 * The calibration component requests the motor commutation to select the appropriate angle source of the
 * electrical field using this enumeration type.
 */
typedef enum {
  CALIBRATION_CONTROL_OFF,            /**< commutation should be switched off, pwm can be disabled, no current should flow through the motor */
  CALIBRATION_CONTROL_SET_POSITION,   /**< the commutation should use the electrical angle theta CalibrationControlOutputT::s16_ElectricalAngle */
  CALIBRATION_CONTROL_SET_SPEED       /**< the commutation should turn the electrical field with the speed CalibrationControlOutputT::s16_Omega, initial position is given by CALIBRATION_CONTROL_SET_POSITION */
} CalibrationControlModeT;

/**
 * \brief commutation operation requested by the calibration
 *
 * This structure contains the mode operation requested by the calibration module.
 * It has to be passed to the motor commutation by the user of this module.
 */
typedef struct CalibrationControlOutputS {
  CalibrationControlModeT e_Mode;              /**< mode of motor operation requested by the calibration module */
  Math_Vector_DQ_T        s_RequestedVoltage;
  /**< requested voltage vector in dq terms, units are given be the commutation */
  int16_t s16_ElectricalAngle;
  /**< electrical angle when in SET_POSITION mode, unit complies with Angle definition */
  int16_t s16_SpeedRPM;
  /**< mechanical speed when in SET_SPEED mode, unit is RPM */
} CalibrationControlOutputT;

/**
 * \brief calibration component state
 *
 * The state machine indicates its state using this enumeration type. It is returned by Calibration_GetStatus
 */
typedef enum {
  CALIBRATION_RESULT_OK,     /**< calibration terminated successfully */
  CALIBRATION_RESULT_FAILED, /**< calibration failed */
  CALIBRATION_RESULT_BUSY    /**< calibration is busy */
} CalibrationResultT;

/**
 * \brief prepare RPS calibration
 *
 * This function initializes the state given to it's initial value and thus to a
 * defined state.
 *
 * \warning This initialization must be called once on the state before calling other functions.
 *
 * Otherwise, their behaviour is undefined. Actually, the functions sets the internal state
 * machine to initial state and resets the error bits.
 *
 * \param[in,out] aps_State pointer to current calibration state
 */
void Calibration_Prepare(CalibrationStateT *aps_State);

/**
 * \brief main worker function 4ms periodically called task for RPS calibration
 *
 * \param[in,out] aps_State pointer to RPS cal state space
 * \param[in]     aps_Parameter pointer to param set
 * \param[in]     as16_MechanicalAngle mechanical angle of rotor - unit must comply with Angle definition
 * \param[in]     as16_ElectricalAngle electrical angle of rotor - unit must comply with Angle definition
 * \param[out]    aps_Output output structure containing the control values for motor commutation */
void Calibration_Task10ms(CalibrationStateT *aps_State,
                          const CalibrationParameterT *aps_Parameter,
                          uint8_t au8_HallState,
                          int16_t as16_ElectricalAngle,
                          CalibrationControlOutputT *aps_Output);

/**
 * \brief returns current calibration state
 * \param aps_State pointer to RPS cal state space
 * \return current state of the calibration
 */
CalibrationResultT Calibration_GetStatus(const CalibrationStateT *aps_State);

/**
 * \brief fills in the current calibration run's error values
 *
 * The function only returns valid values when the calibration is CALIBRATION_RESULT_FAILED.
 *
 * \param[in,out] aps_State pointer to RPS cal state space
 * \param[out] aps_Errors point to error structure to receive the current error bits
 */
void Calibration_GetErrors(const CalibrationStateT *aps_State,
                           CalibrationErrorT *aps_Errors);



#endif /* __CALIBRATE_H_ */
