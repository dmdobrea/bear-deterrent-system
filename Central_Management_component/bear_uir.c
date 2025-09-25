#define UNICODE
#define _UNICODE

#include <windows.h>
#include <utility.h>
#include <mmsystem.h>
#include <winhttp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cvirte.h>		
#include <userint.h>
#include "bear_uir.h"

#pragma comment(lib, "winhttp.lib")

#ifdef _MSC_VER
#define snprintf _snprintf
#endif


// ---- CONFIG ----
#define FIREBASE_HOST L"bear-alert-828c1-default-rtdb.firebaseio.com"
#define DB_PATH       L"/Bear_Alert/GPS_KEY.json"   // node we want
#define AUTH_PARAM    ""                // rules allow all, so leave empty
// ---------------

char alertInfo[4000]; 

int readStringFromFirebase (const wchar_t *path, char *value, int dbg);
int readIntFromFirebase    (const wchar_t *path, int  *value, int dbg);

int writeStringToFirebase  (const wchar_t *path, char *value, int dbg);
int writeIntToFirebase     (const wchar_t *path, int number,  int dbg);


HINTERNET hSession = NULL, hConnect = NULL, hRequest = NULL;
BOOL      bResults = FALSE;
LPSTR     pszOutBuffer = NULL;

DWORD dwSize = 0, dwDownloaded = 0;

static int panelHandle;

int main (int argc, char *argv[])
{
	if (InitCVIRTE (0, argv, 0) == 0)
		return -1;	/* out of memory */
	if ((panelHandle = LoadPanel (0, "bear_uir.uir", PANEL)) < 0)
		return -1;
	DisplayPanel (panelHandle);
	
		/* optional: make console interpret UTF-8 output (Firebase returns UTF-8) */
	    SetConsoleOutputCP(CP_UTF8);
	
	    // Open session
	    hSession = WinHttpOpen(L"FirebaseGPSClient/1.0",
	                           WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
	                           WINHTTP_NO_PROXY_NAME,
	                           WINHTTP_NO_PROXY_BYPASS, 0);
		if (!hSession) 
		{
			SetCtrlVal (panelHandle, PANEL_TEXTBOX, "[Err] : WinHttpOpen failed!");
	        goto cleanup;
	    }
	
	
		 // Connect to Firebase host (HTTPS on port 443)
	    hConnect = WinHttpConnect(hSession, FIREBASE_HOST, INTERNET_DEFAULT_HTTPS_PORT, 0);
	    if (!hConnect) 
			{
			SetCtrlVal (panelHandle, PANEL_TEXTBOX, "[Err] : WinHttpConnect failed!");
	        goto cleanup;
	    	}
		else
			InsertTextBoxLine (panelHandle, PANEL_TEXTBOX, -1, "[Info]  : Successfully connected to Bear Alert database.");  

		SetCtrlVal       (panelHandle, PANEL_TEXTMSG_Number, "No");
		SetCtrlVal       (panelHandle, PANEL_TEXTMSG_Meadle, "bear detected.");
		SetCtrlAttribute (panelHandle, PANEL_TEXTMSG_GPS, ATTR_VISIBLE, 0);
		SetCtrlAttribute (panelHandle, PANEL_COMMANDBUTTON_SendUAV, ATTR_VISIBLE, 0);

	RunUserInterface ();
	
cleanup:	
	DiscardPanel (panelHandle);
	
	if (hRequest) WinHttpCloseHandle(hRequest);
	if (hConnect)  WinHttpCloseHandle(hConnect);
	if (hSession)  WinHttpCloseHandle(hSession);
	
	return 0;
}

int CVICALLBACK fPanel (int panel, int event, void *callbackData,
						int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_GOT_FOCUS:

			break;
		case EVENT_LOST_FOCUS:

			break;
		case EVENT_CLOSE:
			QuitUserInterface (0);
			break;
	}
	return 0;
}


#define FP33gps L"/Bear_Alert/Fixed_point_33/GPS.json"
#define FP33no  L"/Bear_Alert/Fixed_point_33/bearsNo.json" 

#define SP3gps  L"/Bear_Alert/SmartPhone_3/GPS.json" 
#define SP3no   L"/Bear_Alert/SmartPhone_3/bearsNo.json"

char gpsBuff[10000];
int  noBears;
char noBearsStr[200];

int CVICALLBACK fGet_Timer_Data (int panel, int control, int event,
								 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_TIMER_TICK:
			if (readStringFromFirebase(FP33gps, gpsBuff, 0))
				{
				if (strlen (gpsBuff) > 0)
					{
					// Now we have new data, so get the number of brown bears
			
					readIntFromFirebase(FP33no, &noBears, 0); 
					sprintf (noBearsStr, "%d", noBears);
				
					//displat data
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_Number, noBearsStr);
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_Meadle, "bear detected at");
					SetCtrlAttribute (panelHandle, PANEL_TEXTMSG_GPS, ATTR_VISIBLE, 1);  
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_GPS, gpsBuff);
					SetCtrlAttribute (panelHandle, PANEL_COMMANDBUTTON_SendUAV, ATTR_VISIBLE, 1);
					
					SYSTEMTIME dh;
					GetLocalTime(&dh);
					
					if ( noBears == 1)
						sprintf(alertInfo, "[Alert] : In %d/%d/%04d at %d:%02d - %d bear was detecte at %s.", 
								           dh.wMonth, dh.wDay, dh.wYear, dh.wHour, dh.wMinute, 1, gpsBuff);
					else
						sprintf(alertInfo, "[Alert] : In %d/%d/%04d at %d:%02d - %d bears were detecte at the following GPS coordinates: %s.", 
								           dh.wMonth, dh.wDay, dh.wYear, dh.wHour, dh.wMinute, noBears, gpsBuff);
					
					InsertTextBoxLine (panelHandle, PANEL_TEXTBOX, -1, alertInfo);
					
					//clear DB
					gpsBuff[0] = 0;
					writeStringToFirebase  (FP33gps, gpsBuff, 0);
					writeIntToFirebase     (FP33no, 0, 0);
					}
				}

			if (readStringFromFirebase(SP3gps, gpsBuff, 0))
				{
				if (strlen (gpsBuff) > 0)
					{
					// Now we have new data, so get the number of brown bears
			
					readIntFromFirebase(SP3no, &noBears, 0); 
					sprintf (noBearsStr, "%d", noBears);
				
					//displat data
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_Number, noBearsStr);
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_Meadle, "bear detected at");
					SetCtrlAttribute (panelHandle, PANEL_TEXTMSG_GPS, ATTR_VISIBLE, 1);  
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_GPS, gpsBuff);
					SetCtrlAttribute (panelHandle, PANEL_COMMANDBUTTON_SendUAV, ATTR_VISIBLE, 1);
					
					SYSTEMTIME dh;
					GetLocalTime(&dh);
					
					if ( noBears == 1)
						sprintf(alertInfo, "[Alert] : In %d/%d/%04d at %d:%02d - %d bear was announced to be present at %s.", 
								           dh.wMonth, dh.wDay, dh.wYear, dh.wHour, dh.wMinute, 1, gpsBuff);
					else
						sprintf(alertInfo, "[Alert] : In %d/%d/%04d at %d:%02d - %d bears were announced to be present at the following GPS coordinates: %s.", 
								           dh.wMonth, dh.wDay, dh.wYear, dh.wHour, dh.wMinute, noBears, gpsBuff);
					
					InsertTextBoxLine (panelHandle, PANEL_TEXTBOX, -1, alertInfo);
					
					//clear DB
					gpsBuff[0] = 0;
					writeStringToFirebase  (SP3gps, gpsBuff, 0);
					writeIntToFirebase     (SP3no, 0, 0);
					}
				}		
			break;
	}
	return 0;
}


int CVICALLBACK f_getDB (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			if (readStringFromFirebase(FP33gps, gpsBuff, 0))
				{
				if (strlen (gpsBuff) > 0)
					{
					// Now we have new data, so get the number of brown bears
			
					readIntFromFirebase(FP33no, &noBears, 0); 
					sprintf (noBearsStr, "%d", noBears);
				
					//displat data
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_Number, noBearsStr);
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_Meadle, "bear detected at");
					SetCtrlAttribute (panelHandle, PANEL_TEXTMSG_GPS, ATTR_VISIBLE, 1);  
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_GPS, gpsBuff);
					SetCtrlAttribute (panelHandle, PANEL_COMMANDBUTTON_SendUAV, ATTR_VISIBLE, 1);
					
					SYSTEMTIME dh;
					GetLocalTime(&dh);
					
					if ( noBears == 1)
						sprintf(alertInfo, "[Alert] : In %d/%d/%04d at %d:%02d - %d bear was detecte at %s.", 
								           dh.wMonth, dh.wDay, dh.wYear, dh.wHour, dh.wMinute, 1, gpsBuff);
					else
						sprintf(alertInfo, "[Alert] : In %d/%d/%04d at %d:%02d - %d bears were detecte at the following GPS coordinates: %s.", 
								           dh.wMonth, dh.wDay, dh.wYear, dh.wHour, dh.wMinute, noBears, gpsBuff);
					
					InsertTextBoxLine (panelHandle, PANEL_TEXTBOX, -1, alertInfo);
					
					//clear DB
					gpsBuff[0] = 0;
					writeStringToFirebase  (FP33gps, gpsBuff, 0);
					writeIntToFirebase     (FP33no, 0, 0);
					}
				}

			if (readStringFromFirebase(SP3gps, gpsBuff, 0))
				{
				if (strlen (gpsBuff) > 0)
					{
					// Now we have new data, so get the number of brown bears
			
					readIntFromFirebase(SP3no, &noBears, 0); 
					sprintf (noBearsStr, "%d", noBears);
				
					//displat data
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_Number, noBearsStr);
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_Meadle, "bear detected at");
					SetCtrlAttribute (panelHandle, PANEL_TEXTMSG_GPS, ATTR_VISIBLE, 1);  
					SetCtrlVal    (panelHandle, PANEL_TEXTMSG_GPS, gpsBuff);
					SetCtrlAttribute (panelHandle, PANEL_COMMANDBUTTON_SendUAV, ATTR_VISIBLE, 1);
					
					SYSTEMTIME dh;
					GetLocalTime(&dh);
					
					if ( noBears == 1)
						sprintf(alertInfo, "[Alert] : In %d/%d/%04d at %d:%02d - %d bear was announced to be present at %s.", 
								           dh.wMonth, dh.wDay, dh.wYear, dh.wHour, dh.wMinute, 1, gpsBuff);
					else
						sprintf(alertInfo, "[Alert] : In %d/%d/%04d at %d:%02d - %d bears were announced to be present at the following GPS coordinates: %s.", 
								           dh.wMonth, dh.wDay, dh.wYear, dh.wHour, dh.wMinute, noBears, gpsBuff);
					
					InsertTextBoxLine (panelHandle, PANEL_TEXTBOX, -1, alertInfo);
					
					//clear DB
					gpsBuff[0] = 0;
					writeStringToFirebase  (SP3gps, gpsBuff, 0);
					writeIntToFirebase     (SP3no, 0, 0);
					}
				}				


			break;
	}
	return 0;
}


#define caleScr L"/Bear_Alert/MY_STRING.json" 


int CVICALLBACK f_putDB (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			char sirDeScris[2000];
			
			GetCtrlVal (panel, PANEL_STRING_ToWrite, sirDeScris);
			
			writeStringToFirebase(SP3gps, sirDeScris, 0);
			
			
			break;
	}
	return 0;
}


int CVICALLBACK f_put_int_DB (int panel, int control, int event,
							  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			int value;
			
			GetCtrlVal (panel, PANEL_NUMERIC_int_val, &value);
			
			writeIntToFirebase(SP3no, value, 44); 

			
			break;
	}
	return 0;
}




#define NUMBER_DB L"/Bear_Alert/MY_NUMBER.json"     

int CVICALLBACK f_get_int_DB (int panel, int control, int event,
							  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			int outValue;

			readIntFromFirebase(NUMBER_DB, &outValue, 0); 
				
			SetCtrlVal (panel, PANEL_NUMERIC_int_val_read, outValue);

			break;
	}
	return 0;
}

int CVICALLBACK fSenUAV (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			int vrtificationStatus = ConfirmPopup ("Last verification !!!!", "Are you sure you want to send the UAV?");
			
			if (vrtificationStatus == 0)
				{
				SetCtrlVal    (panelHandle, PANEL_TEXTMSG_Number, "No");
				SetCtrlVal    (panelHandle, PANEL_TEXTMSG_Meadle, "bear detected.");
				SetCtrlAttribute (panelHandle, PANEL_TEXTMSG_GPS, ATTR_VISIBLE, 0);  
				//SetCtrlVal    (panelHandle, PANEL_TEXTMSG_GPS, "GPS");
				SetCtrlAttribute (panelHandle, PANEL_COMMANDBUTTON_SendUAV, ATTR_VISIBLE, 0);
				
				SYSTEMTIME dh;  GetLocalTime(&dh);
				sprintf(alertInfo, "[Info]  : In %d/%d/%04d at %d:%02d : a decision was made by the human operator not to send the UAV!", 
								           dh.wMonth, dh.wDay, dh.wYear, dh.wHour, dh.wMinute);
				InsertTextBoxLine (panelHandle, PANEL_TEXTBOX, -1, alertInfo);
				}
			else
				{
				// Send UAV	
				}
			break;
	}
	return 0;
}

//============= MAIN FUNCTIONS ==================================
int readStringFromFirebase(const wchar_t *path, char *value, int dbg)
{
	    // Create GET request
        hRequest = WinHttpOpenRequest(hConnect, L"GET", path,
                                      NULL, WINHTTP_NO_REFERER,
                                      WINHTTP_DEFAULT_ACCEPT_TYPES,
                                      WINHTTP_FLAG_SECURE);

	    if (!hRequest) {
			SetCtrlVal (panelHandle, PANEL_TEXTBOX, "[Err] : WinHttpOpenRequest failed!"); 
			
	        return 0;
	    }

	    /* Send request */
	    bResults = WinHttpSendRequest(hRequest,
	                                  WINHTTP_NO_ADDITIONAL_HEADERS, 0,
	                                  WINHTTP_NO_REQUEST_DATA, 0,
	                                  0, 0);
	    if (!bResults) 
			{
			SetCtrlVal (panelHandle, PANEL_TEXTBOX, "[Err] : WinHttpSendRequest failed!"); 	
			
	        return 0;
	    	}		

	    /* Receive response */
	    bResults = WinHttpReceiveResponse(hRequest, NULL);
	    if (!bResults) 
			{
			SetCtrlVal (panelHandle, PANEL_TEXTBOX, "[Err] : WinHttpReceiveResponse failedd!"); 			
		    
			return 0;
		    }
	

      if (bResults) {
        do {
            dwSize = 0;
            if (!WinHttpQueryDataAvailable(hRequest, &dwSize))
				{
                printf("Error %u in WinHttpQueryDataAvailable.\n", GetLastError());
				
				return 0;
				}

            if (dwSize == 0)
                break;

            pszOutBuffer = (LPSTR)malloc(dwSize + 1);
            if (!pszOutBuffer) 
				{
                printf("Out of memory\n");
				
				return 0;
            	}

            ZeroMemory(pszOutBuffer, dwSize + 1);

            if (!WinHttpReadData(hRequest, (LPVOID)pszOutBuffer, dwSize, &dwDownloaded)) 
				{
                printf("Error %u in WinHttpReadData.\n", GetLastError());
				
				return 0;
				} 
			else 
				{
                pszOutBuffer[dwDownloaded] = '\0';

                // Trim quotes if it's a plain JSON string
                char *json = pszOutBuffer;
                size_t len = strlen(json);
                if (len >= 2 && json[0] == '"' && json[len - 1] == '"') {
                    json[len - 1] = '\0';  // remove trailing quote
                    json++;                // skip leading quote
                }
				
			strcpy(value, json); 
            }

            free(pszOutBuffer);

        } while (dwSize > 0);
    }	

	return 1;
}


int readIntFromFirebase(const wchar_t *path, int *value, int dbg) 
{
	if (hConnect)
        hRequest = WinHttpOpenRequest(hConnect, L"GET", path,
                                      NULL, WINHTTP_NO_REFERER,
                                      WINHTTP_DEFAULT_ACCEPT_TYPES,
                                      WINHTTP_FLAG_SECURE);
	else
		return 0;

    if (hRequest)
        bResults = WinHttpSendRequest(hRequest,
                                      WINHTTP_NO_ADDITIONAL_HEADERS, 0,
                                      WINHTTP_NO_REQUEST_DATA, 0,
                                      0, 0);
	else
		return 0;

    if (bResults)
        bResults = WinHttpReceiveResponse(hRequest, NULL);
	else
		return 0;

    if (bResults) 
	{
        do {
            dwSize = 0;
            if (!WinHttpQueryDataAvailable(hRequest, &dwSize))
                break;

            if (dwSize == 0)
                break;

            pszOutBuffer = (LPSTR)malloc(dwSize + 1);
            if (!pszOutBuffer)
                break;

            ZeroMemory(pszOutBuffer, dwSize + 1);

            if (WinHttpReadData(hRequest, (LPVOID)pszOutBuffer,
                                dwSize, &dwDownloaded)) 
			{
                pszOutBuffer[dwDownloaded] = '\0';

                // Convert response string to int
                *value = atoi(pszOutBuffer);
            }

            free(pszOutBuffer);

        } while (dwSize > 0);
    }
	
	return 1;
}


int writeStringToFirebase(const wchar_t *path, char *value, int dbg)
{
	if (hConnect)
        hRequest = WinHttpOpenRequest(hConnect, L"PUT", path,
                                      NULL, WINHTTP_NO_REFERER,
                                      WINHTTP_DEFAULT_ACCEPT_TYPES,
                                      WINHTTP_FLAG_SECURE);
	else
		{
		SetCtrlVal (panelHandle, PANEL_TEXTBOX, "[Err] : WinHttpOpenRequest failed! Not connected to database.");	
		
		return 0;
		}

    if (hRequest) {
        // Prepare JSON payload: value must be quoted
        char jsonBody[256];
        snprintf(jsonBody, sizeof(jsonBody), "\"%s\"", value);

        bResults = WinHttpSendRequest(hRequest,
                                      L"Content-Type: application/json\r\n",
                                      -1L,
                                      (LPVOID)jsonBody,
                                      (DWORD)strlen(jsonBody),
                                      (DWORD)strlen(jsonBody),
                                      0);
    }

    if (bResults)
        bResults = WinHttpReceiveResponse(hRequest, NULL);
	else
		return 0;

	char errMsg[3000];
    if (!bResults)
		{
        sprintf(errMsg ,"Error %u in writeStringToFirebase.\n", GetLastError());
		SetCtrlVal (panelHandle, PANEL_TEXTBOX, errMsg); 
		
		return 0;
		}
    else
		{
		if (dbg)	
			{
        	sprintf(errMsg, "Successfully wrote %s to Firebase.\n", value);
			InsertTextBoxLine (panelHandle, PANEL_TEXTBOX, -1, errMsg); 
			}
		}
		
	return 1;	
}


int writeIntToFirebase(const wchar_t *path, int number, int dbg)
{
	if (hConnect)
    	hRequest = WinHttpOpenRequest(hConnect, L"PUT", path,
                                      NULL, WINHTTP_NO_REFERER,
                                      WINHTTP_DEFAULT_ACCEPT_TYPES,
                                      WINHTTP_FLAG_SECURE);
	else
		{
		SetCtrlVal (panelHandle, PANEL_TEXTBOX, "[Err] : WinHttpOpenRequest failed! Not connected to database.");	
		
		return 0;
		}

    if (hRequest) {
        // Prepare JSON payload: integer, no quotes
        char jsonBody[64];
        snprintf(jsonBody, sizeof(jsonBody), "%d", number);

        bResults = WinHttpSendRequest(hRequest,
                                      L"Content-Type: application/json\r\n",
                                      -1L,
                                      (LPVOID)jsonBody,
                                      (DWORD)strlen(jsonBody),
                                      (DWORD)strlen(jsonBody),
                                      0);
    }

    if (bResults)
        bResults = WinHttpReceiveResponse(hRequest, NULL);
	
	char errMsg[3000];
    if (!bResults)
		{
        sprintf(errMsg ,"Error %u in write integer to Firebase.\n", GetLastError());
		SetCtrlVal (panelHandle, PANEL_TEXTBOX, errMsg);
		
		return 0;
		}
    else
		{
		if (dbg)	
			{
	        sprintf(errMsg, "Successfully wrote %d to Firebase.\n", number);
			InsertTextBoxLine (panelHandle, PANEL_TEXTBOX, -1, errMsg); 
			}
		}	
		
	return 1;	
}

int CVICALLBACK fStopTimer (int panel, int control, int event,
							void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			int stareCheck;
			GetCtrlVal (panel, PANEL_CHECKBOX_stopTimer, &stareCheck);
			
			if (stareCheck)
				SetCtrlAttribute (panelHandle, PANEL_TIMER, ATTR_ENABLED, 0);
			else
				SetCtrlAttribute (panelHandle, PANEL_TIMER, ATTR_ENABLED, 1);
			break;
	}
	return 0;
}

int CVICALLBACK fTest (int panel, int control, int event,
					   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			char AlarmWavFile[5000];
			
			GetProjectDir (AlarmWavFile); 
			strncat (AlarmWavFile, "\\sunet.wav", 1000);
			PlaySound(AlarmWavFile, NULL, SND_FILENAME | SND_ASYNC);
			
			
			break;
	}
	return 0;
}

