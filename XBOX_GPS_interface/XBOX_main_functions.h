//==============================================================================
//
// Title:		XBOX_main_functions.h
// Purpose:		A short description of the interface.
//
// Created on:	25-Jun-16 at 11:40:00 AM by Dan.
// Copyright:	. All Rights Reserved.
//
//==============================================================================

#include <Windows.h>
#include "Xinput.h"


//Check if the controller is disconnected
BOOL IsXBOXControlConnected();
XINPUT_STATE GetState(int number);
