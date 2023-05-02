/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */

/**
 * \file pse.h
 *
 * \brief  Position and Speed Estimation.
 */
#ifndef __PSE_H_
#define __PSE_H_
#include "netx_drv.h"
/**
 * \brief Minimum speed value for speed estimation.
 */
#define PSE_SYNC_MECH_SPEED_RPM       PAR_MIN_SPEED_RPM
/**
 * \brief Size of moving average filter.
 */
#define PSE_CAPTURE_ARRAY_SIZE        (int32_t)3

/**
 * \brief Estimated speed.
 */
typedef struct EstimatedSpeedValues
{
    int16_t   MechSpeedRPM;             /**< Mechanical speed  in RPM */
    int16_t   ElecSpeedRPM;             /**< Electrical speed  in RPM*/
    int16_t   ElecSpeedDIGPERPWM;       /**< Digits (2^16 is one revolution) per PWM cycle*/
    float     ElecSpeedPERPWM;          /**< Revolution per PWM cycle*/


} Pse_Speed_T;

/**
 * \brief Speed estimation encoder parameters.
 */
typedef struct EncoderParameters
{
    int32_t   s32_CapturedPosition;                       /**< Delta position */
    int32_t   s32_SumPositions;                           /**< Sum to use in moving average filter */
    int32_t   s32_DiscardPositions;                       /**< Position to be discarded used in moving average filter  */
    uint8_t   u8_Index;                                   /**< Index of moving average filter */
    int32_t   s32_PositionArray[PSE_CAPTURE_ARRAY_SIZE];  /**< Holds last captured positions */
} Pse_MENC_ParameterT;

/**
 * \brief Enumeration to indicate if position is known
 *
 */
typedef enum PositionEstimationState
{
    PSE_UNKNOWN,                  /**< Position of the rotor is unknown, Hal sensor measurement is used. */
    PSE_KNOWN,                    /**< Position of the rotor is known. */
    PSE_MANUAL,                   /**< Position of the rotor is manually set. */
} Pse_PositionStateT;

/**
 * \brief Estimated position.
 */
typedef struct Position
{
    uint16_t              u16_Theta;             /**< Rotor electrical position in uint16 scale */
    Pse_PositionStateT    s_State;               /**< State of position estimation. If unknown open-loop.*/

} Pse_Position_T;

/**
 * \brief Enum for Sectors
 *
 */
typedef enum HallSectors
{
    PSE_SECTOR_1 = 2,                 /**< 0   - 59 degrees  HA = 0 , HB = 1 , HC = 0 */
    PSE_SECTOR_2 = 3,                 /**< 60  - 119 degrees HA = 0 , HB = 1 , HC = 1 */
    PSE_SECTOR_3 = 1,                 /**< 120 - 179 degrees HA = 0 , HB = 0 , HC = 1 */
    PSE_SECTOR_4 = 5,                 /**< 180 - 239 degrees HA = 1 , HB = 0 , HC = 1 */
    PSE_SECTOR_5 = 4,                 /**< 240 - 299 degrees HA = 1 , HB = 0 , HC = 0 */
    PSE_SECTOR_6 = 6,                 /**< 300 - 359 degrees HA = 1 , HB = 1 , HC = 0 */
    PSE_INVALID  = 0,                 /**< Invalid sensor input */
} Pse_SectorT;

/**
 * \brief Calibrated hall state boundaries.
 */
typedef struct HallStateBoundaries
{
  uint8_t  u8_HallStatesToIndex[7];
  uint8_t  u8_CalibratedHallStates[6];
  uint16_t u16_CalibratedBoundaries[6];
  uint16_t u16_CalibratedMidPoints[6];
} Pse_HallStateBoundaries_T;


/**
 * \brief Hall state structure.
 */
typedef struct HallState
{
    Pse_SectorT    s_prevState;             /**< Previous state of hall sensor. */
    Pse_SectorT    s_currState;             /**< Current state of hall sensor. */
    uint16_t       u16_count;               /**< Number of samples taken in current state.*/
} Pse_HallState_T;

/*!
 * \brief     Initialize Position&Speed Estimation.
 */
void Pse_Init();

/*!
 * \brief         Set position manually. Used in  open-loop.
 */
void Pse_SetPositionManually(uint16_t au16_Position);
/*!
 * \brief         Set speed manually. Used in  open-loop.
 */
void Pse_SetSpeedManually(int16_t as16_SpeedRPM);
/*!
 * \brief         Set calibrated hall boundaries.
 */
void Pse_SetHallBoundaries(uint16_t * ap_Values, uint8_t * ap_HallState);
/*!
 * \brief     Speed estimation.
 * @return    Estimated speed struct.
 */
Pse_Speed_T Pse_EstimateSpeed(void);

/*!
 * \brief     Speed estimation by hall sensor.
 * @return    Estimated speed struct by hall sensor.
 */
Pse_Speed_T Pse_EstimateSpeedHall(void);


/*!
 * \brief     Hybrid Position estimation.
 * @return    Estimated position struct.
 */
Pse_Position_T Pse_EstimatePosition(void);


/*!
 * \brief     Get position. Used in  open-loop.
 * @return    Estimated position struct.
 */
Pse_Position_T Pse_GetPosition(void);
/*!
 * \brief     Get position. Used in  open-loop.
 * @return    Estimated position struct.
 */
Pse_Speed_T Pse_GetSpeed(void);

#endif /* __PSE_H_ */
