﻿/**********************************************************************************//**
Copyright (c) Hilscher Gesellschaft fuer Systemautomation mbH. All Rights Reserved.
**************************************************************************************//*
$Id: README.txt 7838 2020-06-03 06:30:46Z kai $:
**************************************************************************************//**
<H2>DISCLAIMER</H2>
  Exclusion of Liability for this demo software:
  The following software is intended for and must only be used for reference and in an
  evaluation laboratory environment. It is provided without charge and is subject to
  alterations. There is no warranty for the software, to the extent permitted by
  applicable law. Except when otherwise stated in writing the copyright holders and/or
  other parties provide the software "as is" without warranty of any kind, either
  expressed or implied.
\note
  Please refer to the Agreement in \ref disclaimer "README_DISCLAIMER.txt", provided together with this file!
  By installing or otherwise using the software, you accept the terms of this Agreement.
  If you do not agree to the terms of this Agreement, then do not install or use the
  Software!
**************************************************************************************//**

\mainpage

<H2>Introduction</H2>
This example is simple application driver example without protocols.

FDL templates for different use case are provided under the directory FDL, the device
number, serial number and MAC address must be changed accordingly.

netx90_app_iflash_dummy.nai is a dummy file to be flashed on app iflash, if "netx90_app_sdram.elf" is used to debug the 
application instead of "netx90_app_iflash.nai". Because the firmware on com side will start, only if 
it has detected a valid application code on the app iflash.


<H2>Documentation</H2>
Documentation in HTML-format can be generated by using <B>doxygen</B>.
A respective <EM>Doxyfile</EM> is part of this example. You can run doxygen from command line
or from inside <EM>netX Studio CDT</EM>. The output will be created in folder <EM>Doc</EM>.

<H3>netX Studio CDT</H3>
Just click on @ symbol at menu bar and choose Doxyfile.
First doing this <EM>netX Studio CDT</EM>
will ask for installing Doxygen and Graphviz, which need to be installed.

<H3>Command line</H3>
Just execute <code>doxygen</code> from command line in the root directory of this example.
It is required to have doxygen.exe in the PATH. If the "graphviz" package is installed,
the documentation contains visual dependency diagrams.
Due to a bug in dogygen, it is necessary to specify the path to dot.exe (Graphviz)
in the "Doxyfile" -> DOT_PATH parameter

<H3>Notes</H3>
- If new source code is added to the project, it might be necessary to add new source code directories or files to the INPUT statement in the doxygen configuration file Doxyfile.
- The doxygen documentation requires doxygen version 1.8.0. or higher in order to support Markdown plain text formatting
- It is recommended to edit Doxyfile using doxywizard to make a diff of two Doxyfiles easier.


**************************************************************************************/