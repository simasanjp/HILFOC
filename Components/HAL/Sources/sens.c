/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */
#include "sens.h"

/*!
 * \brief     Previous QEI position.
 */
static int32_t sens_s32_PosPrev = 0U;
/*!
 * \brief     Current QEI position.
 */
static int32_t sens_s32_PosCurr = 0U;
/*!
 * \brief     Previous Ta value.
 */
static uint32_t sens_u32_TaPrev = 0U;
/*!
 * \brief     Current Ta value.
 */
static uint32_t sens_u32_TaCurr = 0U;

#if (PAR_USE_HALL_SPEED == 2)
/*!
 * \brief     Previous hall times.
 * \details   Order: Hall-A, Hall-B, Hall-C
 */
static uint32_t sens_u32_TimePrev[3]  = {0U, 0U, 0U};
/*!
 * \brief     Current hall times.
 * \details   Order: Hall-A, Hall-B, Hall-C
 */
static uint32_t sens_u32_TimeCurr[3]  = {0U, 0U, 0U};
/*!
 * \brief     Delta hall times.
 * \details   Order: Hall-A, Hall-B, Hall-C
 */
static uint32_t  sens_s16_HallTimes[3];

/*!
 * \brief     GPIO interrupt for hall sensors.
 * \details   Speed is calculated by each hall signal individually.
 *            Calculated speed values are stored in sens_s16_HallSpeed.
 * @param[in]     *pvDriverHandle  Pointer to GPIO driver.
 * @param[in]     *pvUnused  Pointer to unused user variable(required by netX drive library).
 */
void sens_HallInterrupt(void* pvDriverHandle, void* pvUnused);
#endif
void sens_HallInit()
{
    DRV_DIO_ChannelSetMode(DRV_DIO_ID_GPIO_0, DRV_DIO_MODE_READ);
    DRV_DIO_ChannelSetMode(DRV_DIO_ID_GPIO_1, DRV_DIO_MODE_READ);
    DRV_DIO_ChannelSetMode(DRV_DIO_ID_GPIO_2, DRV_DIO_MODE_READ);

#if (PAR_USE_HALL_SPEED == 2)
    uint32_t lu32_Unused = 0U;/* Required by netX drivers */

    DRV_DIO_ChannelSetMode(DRV_DIO_ID_GPIO_0, DRV_DIO_MODE_CAPTURE_CONT_RISING);
    DRV_DIO_ChannelIRQAttach(DRV_DIO_ID_GPIO_0, sens_HallInterrupt, &lu32_Unused);
    DRV_DIO_ChannelIRQUnmask(DRV_DIO_ID_GPIO_0);

    DRV_DIO_ChannelSetMode(DRV_DIO_ID_GPIO_1, DRV_DIO_MODE_CAPTURE_CONT_RISING);
    DRV_DIO_ChannelIRQAttach(DRV_DIO_ID_GPIO_1, sens_HallInterrupt, &lu32_Unused);
    DRV_DIO_ChannelIRQUnmask(DRV_DIO_ID_GPIO_1);

    DRV_DIO_ChannelSetMode(DRV_DIO_ID_GPIO_2, DRV_DIO_MODE_CAPTURE_CONT_RISING);
    DRV_DIO_ChannelIRQAttach(DRV_DIO_ID_GPIO_2, sens_HallInterrupt, &lu32_Unused);
    DRV_DIO_ChannelIRQUnmask(DRV_DIO_ID_GPIO_2);
#endif
}

#if (PAR_USE_HALL_SPEED == 2)
void sens_HallInterrupt(void* pvDriverHandle, void* pvUnused)
{
    uint32_t lu32_deltaT;
    uint8_t  lu8_Index;


    if(((DRV_DIO_ID_T)pvDriverHandle) == DRV_DIO_ID_GPIO_0)
    {
        lu8_Index = 0U;
    }
    else if(((DRV_DIO_ID_T)pvDriverHandle) == DRV_DIO_ID_GPIO_1)
    {
        lu8_Index = 1U;
    }
    else if(((DRV_DIO_ID_T)pvDriverHandle) == DRV_DIO_ID_GPIO_2)
    {
        lu8_Index = 2U;
    }
    else
    {
        lu8_Index = 255U; /* Invalid */
    }

    sens_u32_TimePrev[lu8_Index] = sens_u32_TimeCurr[lu8_Index];
    sens_u32_TimeCurr[lu8_Index] = timer_app->timer_systime_ns_b.val;

    /* Check for overflow */
    if (sens_u32_TimeCurr[lu8_Index] < sens_u32_TimePrev[lu8_Index])
    {
        sens_u32_TimeCurr[lu8_Index] = sens_u32_TimeCurr[lu8_Index] + (0xFFFFFFFF - sens_u32_TimePrev[lu8_Index]);
    }

    /* Calculate delta time */
    lu32_deltaT = sens_u32_TimeCurr[lu8_Index] - sens_u32_TimePrev[lu8_Index];

    sens_s16_HallTimes[lu8_Index] = lu32_deltaT;

}
/*!
 * \details Hall_A - index 0
 *          Hall_B - index 1
 *          Hall_C - index 2
 */
uint32_t * sens_HallTimes()
{
    return sens_s16_HallTimes;
}

#endif
void sens_QEIInit()
{

    menc_app->menc_config_b.enc0_en = 1u;
    menc_app->menc_config_b.enc0_filter_sample_rate = 7u;
    menc_app->menc_capture0_config_b.trigger = 0x1000a;// GPIO_APP_COUNTER0 = 0 //0x10004;
    menc_app->menc_capture0_config_b.src_nr = 0u; //Channel 0
    menc_app->menc_capture0_config_b.src = 2u;
    menc_app->menc_capture1_config_b.trigger = 0x1000a;// GPIO_APP_COUNTER0 = 0 //0x10004;
    menc_app->menc_capture1_config_b.src_nr = 0u; //Channel 0
    menc_app->menc_capture1_config_b.src = 3u;
    menc_app->menc_config_b.enc0_count_dir = 0U; //Set to Invert counting direction
    /*Capture source (what to capture):
    0:  system time ns (independent of src_nr)
    1:  position channel 0/1
    2:  Ta of channel 0/1
    3:  Te of channel 0/1
    4:  Ta+Te of channel 0/1
    5:  period in clock cycles (independent of src_nr)*/
    menc_app->menc_enc0_position_b.val = 0u; //Set position to 0

}
/*!
 * \details   Bit order: HA-HB-HC
 * @return    Value of hall state.
 */
uint8_t sens_HallState()
{
    return (uint8_t)(gpio_app->gpio_app_in_b.val & 7u);
}

/*!
 * \details   Calculates delta position.
 * @return    Delta position.
 */
int32_t sens_QEIPosition()
{
    int32_t ls32_Return;
    sens_s32_PosPrev = sens_s32_PosCurr;
    sens_s32_PosCurr = menc_app->menc_enc0_position_b.val;

    ls32_Return = sens_s32_PosCurr - sens_s32_PosPrev;

    return ls32_Return;
}

/*!
 * \details   Capture0 captures Ta, Capture1 captures Te.
 * @return    Ta + Te.
 */
uint32_t sens_QEITaTe()
{
    uint32_t lu32_Return;
    sens_u32_TaPrev = sens_u32_TaCurr;
    sens_u32_TaCurr = menc_app->menc_capture0_val;
    lu32_Return = menc_app->menc_capture1_val + sens_u32_TaPrev;
    return lu32_Return;
}
