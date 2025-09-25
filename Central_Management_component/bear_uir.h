/**************************************************************************/
/* LabWindows/CVI User Interface Resource (UIR) Include File              */
/*                                                                        */
/* WARNING: Do not add to, delete from, or otherwise modify the contents  */
/*          of this include file.                                         */
/**************************************************************************/

#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

     /* Panels and Controls: */

#define  PANEL                            1       /* callback function: fPanel */
#define  PANEL_TEXTBOX                    2       /* control type: textBox, callback function: (none) */
#define  PANEL_STRING_ToWrite             3       /* control type: string, callback function: (none) */
#define  PANEL_STRING_Result              4       /* control type: string, callback function: (none) */
#define  PANEL_COMMANDBUTTON_PutDB_i      5       /* control type: command, callback function: f_get_int_DB */
#define  PANEL_COMMANDBUTTON_GetDB_3      6       /* control type: command, callback function: f_put_int_DB */
#define  PANEL_COMMANDBUTTON_GetDB_2      7       /* control type: command, callback function: f_putDB */
#define  PANEL_COMMANDBUTTON_GetDB        8       /* control type: command, callback function: f_getDB */
#define  PANEL_NUMERIC_int_val_read       9       /* control type: numeric, callback function: (none) */
#define  PANEL_NUMERIC_int_val            10      /* control type: numeric, callback function: (none) */
#define  PANEL_TEXTMSG_Number             11      /* control type: textMsg, callback function: (none) */
#define  PANEL_TEXTMSG_GPS                12      /* control type: textMsg, callback function: (none) */
#define  PANEL_COMMANDBUTTON_SendUAV      13      /* control type: command, callback function: fSenUAV */
#define  PANEL_TEXTMSG_Meadle             14      /* control type: textMsg, callback function: (none) */
#define  PANEL_CHECKBOX_stopTimer         15      /* control type: radioButton, callback function: fStopTimer */
#define  PANEL_COMMANDBUTTON              16      /* control type: command, callback function: fTest */
#define  PANEL_TIMER                      17      /* control type: timer, callback function: fGet_Timer_Data */


     /* Control Arrays: */

          /* (no control arrays in the resource file) */


     /* Menu Bars, Menus, and Menu Items: */

          /* (no menu bars in the resource file) */


     /* Callback Prototypes: */

int  CVICALLBACK f_get_int_DB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK f_getDB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK f_put_int_DB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK f_putDB(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fGet_Timer_Data(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fPanel(int panel, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fSenUAV(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fStopTimer(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK fTest(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);


#ifdef __cplusplus
    }
#endif
