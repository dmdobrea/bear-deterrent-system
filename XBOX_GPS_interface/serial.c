/*---------------------------------------------------------------------------*/
/*                                                                           */
/* FILE:    serial.c                                                         */
/*                                                                           */
/* PURPOSE: This example illustrates how to use the RS232 Library to send and*/
/*          receive commands to and from a device via a serial port.  First, */
/*          you must configure the port, then send a command to the device,  */
/*          then receive a command from the device.                          */
/*                                                                           */
/*          It is important to select the correct terminator on sending and  */
/*          reading, which depends upon your particular RS232 device.  This  */
/*          program gives the option of none, line feed or carriage return   */
/*          for terminators.  It is also important to make sure you have the */
/*          correct type of cable.  See the LabWindows/CVI documenation for  */
/*          more details on cabling and handshaking.                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Include files                                                             */
/*---------------------------------------------------------------------------*/
#include <Windows.h>
#include <cvirte.h>
#include <userint.h>
#include <rs232.h>
#include <utility.h>
#include <formatio.h>
#include <string.h>
#include "serial.h"
#include "XBOX_main_functions.h"
#include <ansi_c.h>

/*---------------------------------------------------------------------------*/
/* Defines		                                                             */
/*---------------------------------------------------------------------------*/

#define TEXT_LENGTH 	2000
#define QuitHelp        1
#define InputqHelp      2
#define FloatToInt		23860929.f

/*---------------------------------------------------------------------------*/
/* Module-globals                                                            */
/*---------------------------------------------------------------------------*/
int panel_handle,
    config_handle,
    comport,
    baudrate,
    portindex,
    parity,
    databits,
    stopbits,
    inputq,         /* Sets input queue length in OpenComConfig */
    outputq,        /* Sets output queue length in OpenComConfig */
    xmode,
    ctsmode,
	stringsize,
    bytes_sent,
    bytes_read,
    RS232Error,
    config_flag,
    breakstatus,
    port_open,
    com_status,
    send_mode,
    send_byte,
    send_term_index,
    read_term_index,
    read_term,
    inqlen,         /* Stores result from GetInQLen */
    outqlen,        /* Stores result from GetOutQLen */
	mode = 0,
	canSendSerial = 0,
	send_messages = 0;
short read_cnt;
double timeout;
char devicename[30],
     send_data[TEXT_LENGTH],
     read_data[TEXT_LENGTH],
     tbox_read_data[TEXT_LENGTH],
     com_msg[500],
     msg[100];
float latitudeG = 0.0f;
float longitudeG = 0.0f;




/*---------------------------------------------------------------------------*/
/* Internal function prototypes                                              */
/*---------------------------------------------------------------------------*/
void DisplayRS232Error (void);
void SetConfigParms (void);
void GetConfigParms (void);
void DisplayHelp (int);
void EnablePanelControls (int);
void DisplayComStatus (void);
void ActivateSendControls (int);
void SendAscii (void);
void SendByte (void);


/*---------------------------------------------------------------------------*/
/* This is the application's entry-point.                                    */
/*---------------------------------------------------------------------------*/
int main (int argc, char *argv[])
{
    if (InitCVIRTE (0, argv, 0) == 0)
        return -1;
    if ((panel_handle = LoadPanel (0, "serial.uir", SERIAL)) < 0)
		return -1;
	
	/*Limit the number of characters that can be sent.*/
	SetCtrlAttribute (panel_handle, SERIAL_TBOX_SEND, ATTR_MAX_ENTRY_LENGTH, TEXT_LENGTH-2);
	/*Limit the number of characters that can be read*/
	SetCtrlAttribute(panel_handle,SERIAL_READ_COUNT,ATTR_MAX_VALUE,TEXT_LENGTH-1);
	
    DisplayPanel (panel_handle);
    RunUserInterface ();
    DiscardPanel (panel_handle);
    CloseCVIRTE ();
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Set the port configuration parameters.                                    */
/*---------------------------------------------------------------------------*/
void SetConfigParms (void)
{
    SetCtrlVal (config_handle, CONFIG_COMPORT, comport);
    SetCtrlVal (config_handle, CONFIG_BAUDRATE, baudrate);
    SetCtrlVal (config_handle, CONFIG_PARITY, parity);
    SetCtrlVal (config_handle, CONFIG_DATABITS, databits);
    SetCtrlVal (config_handle, CONFIG_STOPBITS, stopbits);
    SetCtrlVal (config_handle, CONFIG_INPUTQ, inputq);
    SetCtrlVal (config_handle, CONFIG_OUTPUTQ, outputq);
    SetCtrlVal (config_handle, CONFIG_CTSMODE, ctsmode);
    SetCtrlVal (config_handle, CONFIG_XMODE, xmode);
    SetCtrlVal (config_handle, CONFIG_TIMEOUT, timeout);
    SetCtrlIndex (config_handle, CONFIG_COMPORT, portindex);
}

/*---------------------------------------------------------------------------*/
/* Get the port configuration parameters.                                    */
/*---------------------------------------------------------------------------*/
void GetConfigParms (void)
{
	GetCtrlVal (config_handle, CONFIG_COMPORT, &comport);
    GetCtrlVal (config_handle, CONFIG_BAUDRATE, &baudrate);
    GetCtrlVal (config_handle, CONFIG_PARITY, &parity);
    GetCtrlVal (config_handle, CONFIG_DATABITS, &databits);
    GetCtrlVal (config_handle, CONFIG_STOPBITS, &stopbits);
    GetCtrlVal (config_handle, CONFIG_INPUTQ, &inputq);
    GetCtrlVal (config_handle, CONFIG_OUTPUTQ, &outputq);
    GetCtrlVal (config_handle, CONFIG_CTSMODE, &ctsmode);
    GetCtrlVal (config_handle, CONFIG_XMODE, &xmode);
    GetCtrlVal (config_handle, CONFIG_TIMEOUT, &timeout);
    GetCtrlIndex (config_handle, CONFIG_COMPORT, &portindex);
    #ifdef _NI_unix_
        devicename[0]=0;
    #else
        GetLabelFromIndex (config_handle, CONFIG_COMPORT, portindex,
                       devicename);
    #endif                   
}

/*---------------------------------------------------------------------------*/
/* Let the user configure the port.                                          */
/*---------------------------------------------------------------------------*/
int CVICALLBACK ConfigCallBack (int panel, int control, int event,
                                void *callbackData, int eventData1,
                                int eventData2)
{
    switch (event)
        {
        case EVENT_COMMIT:
            config_handle = LoadPanel (panel_handle, "serial.uir", CONFIG);
            InstallPopup (config_handle);

            /*  If user already has done configuration, then
                display those new parameters.  If entering
                configuration for 1st time, set config_flag
                and use default settings.
            */
            if (config_flag)    /* Configuration done at least once.*/
                SetConfigParms ();
            else                /* 1st time.*/
                config_flag = 1;
            break; 
        }
    return(0);
}

/*---------------------------------------------------------------------------*/
/* close the configuration panel.                                            */
/*---------------------------------------------------------------------------*/
int CVICALLBACK CloseConfigCallback (int panel, int control, int event,
                                     void *callbackData, int eventData1,
                                     int eventData2)
{
    switch (event)
        {
        case EVENT_COMMIT :
            port_open = 0;  /* initialize flag to 0 - unopened */
            GetConfigParms ();
            DisableBreakOnLibraryErrors ();
            RS232Error = OpenComConfig (comport, devicename, baudrate, parity,
                                        databits, stopbits, inputq, outputq);
            EnableBreakOnLibraryErrors ();
            if (RS232Error) DisplayRS232Error ();
            if (RS232Error == 0)
                {
                port_open = 1;
                GetCtrlVal (config_handle, CONFIG_XMODE, &xmode);
                SetXMode (comport, xmode);
                GetCtrlVal (config_handle, CONFIG_CTSMODE, &ctsmode);
                SetCTSMode (comport, ctsmode);
                GetCtrlVal (config_handle, CONFIG_TIMEOUT, &timeout);
                SetComTime (comport, timeout);
                EnablePanelControls (0); /* Enable: no errors */
                }
            else
                EnablePanelControls (1); /* Disable: errors found */
            DiscardPanel (config_handle);
            break;
        }
    return(0);
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void EnablePanelControls (int enable)
{
    SetCtrlAttribute (panel_handle, SERIAL_SEND, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_READ, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_READ_COUNT, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_TBOX_READ, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_BYTES, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_ERROR, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_FLUSHINQ, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_FLUSHOUTQ, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_GETINQ, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_GETOUTQ, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_COMSTATUS, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_READTERM, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_SENDMODE, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_RCV_HELP_MSG, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_TRANS_HELP_MSG, ATTR_DIMMED, enable);
    SetCtrlAttribute (panel_handle, SERIAL_CLEARBOX, ATTR_DIMMED, enable);
    ActivateSendControls (enable);
}

/*---------------------------------------------------------------------------*/
/*  Activate or deactivate the Send controls.  For activate, enable = 0,     */
/*  for deactivate, enable = 1, since 0 is not dimmed and 1 is dimmed.       */
/*---------------------------------------------------------------------------*/
void ActivateSendControls (int enable)
{
	if(enable)
	{
		canSendSerial = 0;
	}
	else
	{
		canSendSerial = 1;
	}
	
    GetCtrlVal (panel_handle, SERIAL_SENDMODE, &send_mode);
    if (send_mode)
        {   /* ascii mode */
        SetCtrlAttribute (panel_handle, SERIAL_TBOX_SEND, ATTR_DIMMED,
                          enable);
        SetCtrlAttribute (panel_handle, SERIAL_SENDTERM, ATTR_DIMMED, enable);
        SetCtrlAttribute (panel_handle, SERIAL_TRANS_HELP_MSG, ATTR_DIMMED,
                          enable);
        SetCtrlAttribute (panel_handle, SERIAL_SENDBYTE, ATTR_DIMMED,
                          !enable);
        }
    else
        {             /* byte mode */
        SetCtrlAttribute (panel_handle, SERIAL_SENDBYTE, ATTR_DIMMED, enable);
        SetCtrlAttribute (panel_handle, SERIAL_TBOX_SEND, ATTR_DIMMED,
                          !enable);
        SetCtrlAttribute (panel_handle, SERIAL_SENDTERM, ATTR_DIMMED,
                          !enable);
        SetCtrlAttribute (panel_handle, SERIAL_TRANS_HELP_MSG, ATTR_DIMMED,
                          !enable);
        }
}

/*---------------------------------------------------------------------------*/
/* Clear the character display.                                              */
/*---------------------------------------------------------------------------*/
int CVICALLBACK ClearBoxCallBack (int panel, int control, int event,
                                  void *callbackData, int eventData1,
                                  int eventData2)
{
    if (event == EVENT_COMMIT)
        ResetTextBox (panel_handle, SERIAL_TBOX_READ, "\0");
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Active the send-mode controls.                                            */
/*---------------------------------------------------------------------------*/
int CVICALLBACK SendModeCallBack (int panel, int control, int event,
                                  void *callbackData, int eventData1,
                                  int eventData2)
{
    if (event == EVENT_COMMIT)
        ActivateSendControls (0);
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Flush the input queue.                                                    */
/*---------------------------------------------------------------------------*/
int CVICALLBACK FlushInCallBack (int panel, int control, int event,
                                 void *callbackData, int eventData1,
                                 int eventData2)
{
    if (event == EVENT_COMMIT)
        {
        FlushInQ (comport);
        MessagePopup ("RS232 Message", "Input queue flushed.");
        }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Flush the output queue.                                                   */
/*---------------------------------------------------------------------------*/
int CVICALLBACK FlushOutQCallBack (int panel, int control, int event,
                                   void *callbackData, int eventData1,
                                   int eventData2)
{
    if (event == EVENT_COMMIT)
        {
        FlushOutQ (comport);
        MessagePopup ("RS232 Message", "Output queue flushed.");
        }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Get the number of bytes in the input queue.                               */
/*---------------------------------------------------------------------------*/
int CVICALLBACK GetInQCallBack (int panel, int control, int event,
                                void *callbackData, int eventData1,
                                int eventData2)
{
    if (event == EVENT_COMMIT)
        {
        inqlen = GetInQLen (comport);
        Fmt (msg, "%s<Input queue length = %i", inqlen);
        MessagePopup ("RS232 Message", msg);
        }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Get the number of bytes in the output queue.                              */
/*---------------------------------------------------------------------------*/
int CVICALLBACK GetOutQCallBack (int panel, int control, int event,
                                 void *callbackData, int eventData1,
                                 int eventData2)
{
    if (event == EVENT_COMMIT)
        {
        outqlen = GetOutQLen (comport);
        Fmt (msg, "%s<Output queue length = %i", outqlen);
        MessagePopup ("RS232 Message", msg);
        }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Get the status of the COM port and display it to the user.                */
/*---------------------------------------------------------------------------*/
int CVICALLBACK ComStatusCallBack (int panel, int control, int event,
                                   void *callbackData, int eventData1,
                                   int eventData2)
{
    if (event == EVENT_COMMIT)
        {
        com_status = GetComStat (comport);
        DisplayComStatus ();
        }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Display status information to the user.                                   */
/*---------------------------------------------------------------------------*/
void DisplayComStatus ()
{
    com_msg[0] = '\0';
    if (com_status & 0x0001)
        strcat (com_msg, "Input lost: Input queue"
                " filled and characters were lost.\n");
    if (com_status & 0x0002)
        strcat (com_msg, "Asynch error: Problem "
                "determining number of characters in input queue.\n");
    if (com_status & 0x0010)
        strcat (com_msg, "Parity error.\n");
    if (com_status & 0x0020)
        strcat (com_msg, "Overrun error: Received"
                " characters were lost.\n");
    if (com_status & 0x0040)
        strcat (com_msg, "Framing error: Stop bits were not received"
                " as expected.\n");
    if (com_status & 0x0080)
        strcat (com_msg, "Break: A break signal was detected.\n");
    if (com_status & 0x1000)
        strcat (com_msg, "Remote XOFF: An XOFF character was received."
                "\nIf XON/XOFF was enabled, no characters are removed"
                " from the output queue and sent to another device "
                "until that device sends an XON character.\n");
    if (com_status & 0x2000)
        strcat (com_msg, "Remote XON: An XON character was received."
                "\nTransmisson can resume.\n");
    if (com_status & 0x4000)
        strcat (com_msg, "Local XOFF: An XOFF character was sent to\n"
                " the other device.  If XON/XOFF was enabled, XOFF is\n"
                " transmitted when the input queue is 50%, 75%, and 90%\n"
                " full.\n");
    if (com_status & 0x8000)
        strcat (com_msg, "Local XON: An XON character was sent to\n"
                " the other device.  If XON/XOFF was enabled, XON is\n"
                " transmitted when the input queue empties after XOFF\n"
                " was sent.  XON tells the other device that it can \n"
                " resume sending data.\n");
    if (strlen (com_msg) == 0)
        strcat (com_msg, "No status bits are set.");
    MessagePopup ("RS232 Message", com_msg);
}

/*---------------------------------------------------------------------------*/
/* Send the appropriate data to the port.                                    */
/*---------------------------------------------------------------------------*/
int CVICALLBACK SendCallBack (int panel, int control, int event,
                              void *callbackData, int eventData1,
                              int eventData2)
{
    if (event == EVENT_COMMIT)
        {
        GetCtrlVal (panel_handle, SERIAL_SENDMODE, &send_mode);
        if (send_mode)
            SendAscii ();
        else
            SendByte ();
        RS232Error = ReturnRS232Err ();
        if (RS232Error)
            DisplayRS232Error ();
        SetCtrlAttribute (panel_handle, SERIAL_BYTES, ATTR_DIMMED, 0);
        SetCtrlVal (panel_handle, SERIAL_BYTES, bytes_sent);
        }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Send one byte to the port.                                                */
/*---------------------------------------------------------------------------*/
void SendByte (void)
{
    GetCtrlVal (panel_handle, SERIAL_SENDBYTE, &send_byte);
    bytes_sent = ComWrtByte (comport, send_byte);
}

/*---------------------------------------------------------------------------*/
/* Send ASCII characters to the port.                                        */
/*---------------------------------------------------------------------------*/
void SendAscii (void)
{
    GetCtrlVal (panel_handle, SERIAL_TBOX_SEND, send_data);
    GetCtrlIndex (panel_handle, SERIAL_SENDTERM, &send_term_index);
    switch (send_term_index)
        {
        case 1:
            strcat(send_data, "\r");
            break;
        case 2:
            strcat(send_data, "\n");
            break;
        }
    stringsize = StringLength (send_data);
    bytes_sent = ComWrt (comport, send_data, stringsize);
}

/*---------------------------------------------------------------------------*/
/* Display pertinent error information.                                      */
/*---------------------------------------------------------------------------*/
int CVICALLBACK ErrorCallBack (int panel, int control, int event,
                               void *callbackData, int eventData1,
                               int eventData2)
{
    switch (event)
        {
        case EVENT_COMMIT:
            RS232Error = ReturnRS232Err ();
            DisplayRS232Error ();
            break;
        case EVENT_RIGHT_CLICK :
            break;
        }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Read data from the COM port.                                              */
/*---------------------------------------------------------------------------*/
int CVICALLBACK ReadCallBack (int panel, int control, int event,
                              void *callbackData, int eventData1,
                              int eventData2)
{
    switch (event)
        {
        case EVENT_COMMIT:
            read_data[0] = '\0';
            GetCtrlVal (panel_handle, SERIAL_READ_COUNT, &read_cnt);
            GetCtrlIndex (panel_handle, SERIAL_READTERM, &read_term_index);
            switch (read_term_index)
                {
                case 0:
                    read_term = 0;
                    break;
                case 1:
                    read_term = 13;
                    break;
                case 2:
                    read_term = 10;
                    break;
                }
            if (read_term)
                bytes_read = ComRdTerm (comport, read_data, read_cnt,
                                        read_term);
            else
                bytes_read = ComRd (comport, read_data, read_cnt);
            /*  Copy subset of read_data into tbox string for display.
                ComRdTerm does not automatically put null byte after
                number of bytes read into read_data string. */
            CopyString (tbox_read_data, 0, read_data, 0, bytes_read);
            SetCtrlVal (panel_handle, SERIAL_TBOX_READ, tbox_read_data);
            RS232Error = ReturnRS232Err ();
            if (RS232Error)
                DisplayRS232Error ();
            break;
        case EVENT_RIGHT_CLICK :
            break;
        }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Display Help on the Input Queue control.                                  */
/*---------------------------------------------------------------------------*/
int CVICALLBACK InputQCallBack (int panel, int control, int event,
                                void *callbackData, int eventData1,
                                int eventData2)
{
    if (event == EVENT_RIGHT_CLICK)
        DisplayHelp (InputqHelp);
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Check the status of the queues and then terminate the RunUserInterface    */
/* loop and end the application...or just provide help.                      */
/*---------------------------------------------------------------------------*/
int CVICALLBACK QuitCallBack (int panel, int control, int event,
                              void *callbackData, int eventData1,
                              int eventData2)
{
    switch (event)
        {
        case EVENT_COMMIT :
            if (port_open)
                {
                outqlen = GetOutQLen (comport);
                if (outqlen > 0)
                    {
                    MessagePopup ("RS232 Message", "The output queue has\n"
                                    "data in it. Wait for device to receive\n"
                                    "the data or flush the queue.\n");
                    break;
                    }
                RS232Error = CloseCom (comport);
                if (RS232Error)
                    DisplayRS232Error ();
                }
            QuitUserInterface (0);
            break;
        case EVENT_RIGHT_CLICK :
            DisplayHelp (QuitHelp);
            break;
        }
    return 0;
}

/*---------------------------------------------------------------------------*/
/* Display error information to the user.                                    */
/*---------------------------------------------------------------------------*/
void DisplayRS232Error (void)
{
    char ErrorMessage[200];
    switch (RS232Error)
        {
        default :
            if (RS232Error < 0)
                {  
                Fmt (ErrorMessage, "%s<RS232 error number %i", RS232Error);
                MessagePopup ("RS232 Message", ErrorMessage);
                }
            break;
        case 0  :
            MessagePopup ("RS232 Message", "No errors.");
            break;
        case -2 :
            Fmt (ErrorMessage, "%s", "Invalid port number (must be in the "
                                     "range 1 to 8).");
            MessagePopup ("RS232 Message", ErrorMessage);
            break;
        case -3 :
            Fmt (ErrorMessage, "%s", "No port is open.\n"
                 "Check COM Port setting in Configure.");
            MessagePopup ("RS232 Message", ErrorMessage);
            break;
        case -99 :
            Fmt (ErrorMessage, "%s", "Timeout error.\n\n"
                 "Either increase timeout value,\n"
                 "       check COM Port setting, or\n"
                 "       check device.");
            MessagePopup ("RS232 Message", ErrorMessage);
            break;
        }
}

/*---------------------------------------------------------------------------*/
/* Display help information to the user.                                     */
/*---------------------------------------------------------------------------*/
void DisplayHelp (int HelpId)
{
    switch (HelpId)
        {
        case 1 :
            MessagePopup ("Quit Help",
                          "The Quit button closes the current COM port,\n"
                          "checks and displays any error messages, \n"
                          "and then exits this program.");
            break;
        case 2 :
            MessagePopup ("Input Queue Help",
                          "Specifies the size of the input queue for the "
                          "selected port.\n"
                          "Default Value:  512\n"
                          "Valid Range:    28 to 65516");
            break;
        }
}

int CVICALLBACK fTIMER (int panel, int control, int event,
						void *callbackData, int eventData1, int eventData2)
{
	GetCtrlVal (panel_handle, SERIAL_MODE, &mode);
	if(!send_messages)
		return 0;
	switch (event)
	{
		case EVENT_TIMER_TICK:
			if(IsXBOXControlConnected && canSendSerial)
			{
				XINPUT_STATE controllerState = GetState(0);
				short int rx = controllerState.Gamepad.sThumbRX;
				short int ry = controllerState.Gamepad.sThumbRY;
				short int lx = controllerState.Gamepad.sThumbLX;
				short int ly = controllerState.Gamepad.sThumbLY;
				short int buttonA = (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_A) ? 1 : 0;
				short int buttonB = (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_B) ? 1 : 0;
				short int buttonX = (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_X) ? 1 : 0;
				short int buttonY = (controllerState.Gamepad.wButtons & XINPUT_GAMEPAD_Y) ? 1 : 0;
				
				int latitude;
				int longitude;
				
				char message[13];
				message[0]  = 0x55;
				message[1]  = 0x55;
				
				message[2]  = 0;
				message[2]  = message[2] | (mode << 6);
				message[2]  = message[2] ^ (buttonX << 3);
				message[2]  = message[2] ^ (buttonB << 2);
				message[2]  = message[2] ^ (buttonA << 1);
				message[2]  = message[2] ^  buttonY;
				
				message[11] = 0;
				
				if(mode == 0)
				{
					message[3] = rx >> 8;
					message[4] = rx & 0xFF;
					message[5] = ry >> 8;
					message[6] = ry & 0xFF;
					message[7] = lx >> 8;
					message[8] = lx & 0xFF;
					message[9] = ly >> 8;
					message[10]= ly & 0xFF;
				}
				else if(mode == 1)
				{
					longitude = longitudeG * (FloatToInt / 2.f);
					latitude  = latitudeG  * FloatToInt;
					
					message[3] = (latitude & (0xFF << (8 * 3))) >> 8 * 3;
					message[4] = (latitude & (0xFF << (8 * 2))) >> 8 * 2;
					message[5] = (latitude & (0xFF << (8 * 1))) >> 8 * 1;
					message[6] = (latitude & (0xFF << (8 * 0))) >> 8 * 0;
					
					message[7] = (longitude & (0xFF << (8 * 3))) >> 8 * 3;	
					message[8] = (longitude & (0xFF << (8 * 2))) >> 8 * 2;
					message[9] = (longitude & (0xFF << (8 * 1))) >> 8 * 1;
					message[10]= (longitude & (0xFF << (8 * 0))) >> 8 * 0;
				}
				
				
				for(int i = 0; i < 8; i++)
				{
					if(message[i + 3] == 0x55)
					{
						message[i + 3] = 0x56;
						message[11] = message[11] ^ (1 << (7 - i));
					}
				}

				message[12] = message[2];
							  
				for(i = 3; i < 12; i++)
					message[12] = message[12] ^ message[i];
							  
				for(i = 0; i < 13; i++)
					printf("%d ", (int)message[i]);
				printf("\n");
				
				/*printf("%d\n", rx);
				for(i = 7; i >= 0; i--)
				{
					printf("%d", (message[3] & (1 << i)) >> i);
				}
				for(i = 7; i >= 0; i--)
				{
					printf("%d", (message[4] & (1 << i)) >> i);
				}
				printf("\n");*/
				
				//printf("%d\n%d\n%d\n%d\n\n", rx, ry, lx, ly);
				
				bytes_sent = ComWrt(comport, message, 13);
			}
			break;
	}
	return 0;
}

int CVICALLBACK Set_GPS_callback (int panel, int control, int event,
								  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal (panel_handle, SERIAL_LATITUDE, &latitudeG);
			GetCtrlVal (panel_handle, SERIAL_LONGITUDE, &longitudeG);
			break;
	}
	return 0;
}

int CVICALLBACK Send_messages_callback (int panel, int control, int event,
										void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal (panel_handle, SERIAL_SEND_MESSAGES, &send_messages);
			break;
	}
	return 0;
}
