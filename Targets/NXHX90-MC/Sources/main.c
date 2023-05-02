/***************************************************************************************
 * @file    main.c
 * @brief   Main program body
 * $Revision: 4507 $
 * $Date: 2018-11-20 13:25:13 +0100 (Di, 20 Nov 2018) $
 * \copyright Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
 * \note Exclusion of Liability for this demo software:
 * The following software is intended for and must only be used for reference and in an
 * evaluation laboratory environment. It is provided without charge and is subject to
 * alterations. There is no warranty for the software, to the extent permitted by
 * applicable law. Except when otherwise stated in writing the copyright holders and/or
 * other parties provide the software "as is" without warranty of any kind, either
 * expressed or implied.
 * Please refer to the Agreement in README_DISCLAIMER.txt, provided together with this file!
 * By installing or otherwise using the software, you accept the terms of this Agreement.
 * If you do not agree to the terms of this Agreement, then do not install or use the
 * Software!
 ***************************************************************************************/

/***************************************************************************************
 * For User Interface the Motor Control Example uses libc and a simple UART console
 * interface to send commands to the motor and receive feedback.
 * The UART is routed through the USB via the FTDI chip on the board.
 * The default baudrate is 115200. The available commands are:
 * Possible commands:
 * 1. motor_start
 * 2. motor_get_speed
 * 3. motor_stop
 * send the commands with Carriage Return (CR) or Line Feed(LF)
 ***************************************************************************************/

#include "main.h"

/*!
 * \brief The main of the template project.
 *
 * Contains an infinity loop. In case it breaks, the function will return
 * properly.
 * \return Will always return 0.
 */
int main(void)
{
  APP_DATA_T tData;
  libcInit();
  printf("netX90 Field-Oriented Motor Control\r\n");
  printf("%s %s\r\n",__DATE__,__TIME__);

  if(FOC_APPL_init())
  {
    printf("FOC_APPL initialization error\r\n");
  }

  while(1)
  {
    App_TerminalHandler(&tData);
  }
  return 0;
}
