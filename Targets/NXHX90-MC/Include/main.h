/**************************************************************************//**
 * @file    main.h
 * @brief   Header for main.c module
 * $Revision: 4314 $
 * $Date: 2018-09-06 12:16:33 +0200 (Do, 06 Sep 2018) $
 ******************************************************************************/

/**************************************************************************************
 Exclusion of Liability for this demo software:
 The following software is intended for and must only be used for reference and in an
 evaluation laboratory environment. It is provided without charge and is subject to
 alterations. There is no warranty for the software, to the extent permitted by
 applicable law. Except when otherwise stated in writing the copyright holders and/or
 other parties provide the software "as is" without warranty of any kind, either
 expressed or implied.
 Please refer to the Agreement in README_DISCLAIMER.txt, provided together with this file!
 By installing or otherwise using the software, you accept the terms of this Agreement.
 If you do not agree to the terms of this Agreement, then do not install or use the
 Software!
 **************************************************************************************/

/**************************************************************************************

 Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.


 **************************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include "netx_drv.h"
#include "libc_support.h"
#include "TerminalHandler.h"
#include "FOC_APPL.h"

#define DEMO_MOTOR_START_SPEED 2000
#define DEMO_MOTOR_STOP_SPEED  0

typedef struct APP_DATA_Ttag
{
  uint16_t data;
}APP_DATA_T;

#endif /* __MAIN_H */

