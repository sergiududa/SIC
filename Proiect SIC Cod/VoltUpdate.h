/**************************************************************************/
/* LabWindows/CVI User Interface Resource (UIR) Include File              */
/* Copyright (c) National Instruments 2018. All Rights Reserved.          */
/*                                                                        */
/* WARNING: Do not add to, delete from, or otherwise modify the contents  */
/*          of this include file.                                         */
/**************************************************************************/

#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

     /* Panels and Controls: */

#define  PANEL                            1       /* callback function: PanelCallback */
#define  PANEL_START                      2       /* callback function: StartCallback */
#define  PANEL_STOP                       3       /* callback function: StopCallback */
#define  PANEL_TD_VALUE                   4
#define  PANEL_TI_VALUE                   5
#define  PANEL_KR_VALUE                   6
#define  PANEL_MOTOR_ONE_SLIDER           7       /* callback function: MotorOneCallback */
#define  PANEL_MOTOR_ZERO_SLIDER          8       /* callback function: MotorZeroCallback */
#define  PANEL_control_type_select        9       /* callback function: tipControl */
#define  PANEL_DECORATION                 10
#define  PANEL_DECORATION_2               11
#define  PANEL_LED_AUTOMATIC              12
#define  PANEL_LED_MANUAL                 13
#define  PANEL_DECORATION_3               14
#define  PANEL_PERIOD                     15
#define  PANEL_SET_POINT_VALUE            16      /* callback function: ReferenceChanged */
#define  PANEL_TIMER                      17      /* callback function: TimerCallback */
#define  PANEL_SAME_COMMAND_CHBX          18      /* callback function: SameCommandCHECKED */
#define  PANEL_STRIPCHART                 19
#define  PANEL_TEXTMSG                    20
#define  PANEL_TIMER_2                    21      /* callback function: UpdateLabelsTimer */
#define  PANEL_REF_LABEL                  22
#define  PANEL_COMMAND_LABEL              23
#define  PANEL_OUTPUT_LABEL               24
#define  PANEL_REF_VAL                    25
#define  PANEL_U_VAL                      26
#define  PANEL_Y_VAL                      27

#define  PANEL_2                          2


     /* Menu Bars, Menus, and Menu Items: */

          /* (no menu bars in the resource file) */


     /* Callback Prototypes: */ 

int  CVICALLBACK MotorOneCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK MotorZeroCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK PanelCallback(int panel, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ReferenceChanged(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK SameCommandCHECKED(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK StartCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK StopCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK TimerCallback(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK tipControl(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK UpdateLabelsTimer(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);


#ifdef __cplusplus
    }
#endif
