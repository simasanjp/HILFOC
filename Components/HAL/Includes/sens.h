/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */
/**
 * \file sens.h
 *
 * \brief  Sensor abstraction.
 */
#ifndef __SENS_H_
#define __SENS_H_

#include "netx_drv.h"
#include "par_def.h"
/*!
 * \brief     Initialize hall sensor as gpio read.
 */
void sens_HallInit();

/*!
 * \brief     Initialize menc0 unit.
 */
void sens_QEIInit();

/*!
 * \brief     Returns hall sensor state.
 */
uint8_t sens_HallState();

#if (PAR_USE_HALL_SPEED == 2)
/*!
 * \brief   Returns array of hall sensor duration times.
 */
uint32_t * sens_HallTimes();
#endif

/*!
 * \brief     Returns delta position.
 */
int32_t sens_QEIPosition();

/*!
 * \brief     Returns Ta + Te.
 */
uint32_t sens_QEITaTe();



#endif /*_SENS_H_ */
