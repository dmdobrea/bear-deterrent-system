#include "XBOX_main_functions.h"

int noExistingXBOXctrl;
int XBOXctrlFound[20];

BOOL IsXBOXControlConnected()
{
	XINPUT_STATE XBOX_CONTROLLER_State;
	int i;
	
   	//Invoke the memset(); function to zero the XBOX_CONTROLLER_State. 
   	memset(&XBOX_CONTROLLER_State, 0, sizeof(XINPUT_STATE)); 

	for (i=0; i< XUSER_MAX_COUNT; i++ )
		{
	   	//We store the XInputGetState value inside result, note that result is a DWORD which is a typedef unsigned long. 
	   	DWORD result = XInputGetState(i, &XBOX_CONTROLLER_State); 

	   	//Check if the controller is disconnected using the Ternary Operator. 
		if (result == ERROR_SUCCESS)
			{
			XBOXctrlFound [noExistingXBOXctrl] = i;
			noExistingXBOXctrl ++;	
			}
		}
		
	if (noExistingXBOXctrl != 0)
		return TRUE;
	else
		return FALSE;
}


XINPUT_STATE GetState(int number)
{
	XINPUT_STATE XBOX_CONTROLLER_State;
	
    memset(&XBOX_CONTROLLER_State, 0, sizeof(XINPUT_STATE)); 
    XInputGetState(number, &XBOX_CONTROLLER_State); 
    return XBOX_CONTROLLER_State; 
}
