
/************* INDICATII/IDEI *************

	- Procesul are histerezis.
	- Mentinem bila intr-un anumit interval pe gradatie, ca sa nu avem probleme cu momentul de intertie initial.
	- Starting point : -0.89
	- Ending point   :  0.00
	- Adaugare legenda
	- Scalare setpoint
	- Acordare PID + pozitia initiala
	
*************************************/ 

#include <ansi_c.h>
#include <cvirte.h>
#include <userint.h>
#include <NIDAQmx.h>
#include <DAQmxIOctrl.h>
#include "VoltUpdate.h"
#include "control.h"

// ***** DEFINE REGION *****
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else
#define MANUAL_CONTROL      0
#define AUTOMATIC_CONTROL   1
#define PROCESS_STOPPED     0
#define PROCESS_STARTED     1
#define NUM_GRAPH_CHANNELS  3
// ***** END REGION *****


// ***** VARIABLE DECLARATIONS REGION *****
static int     panelHandle;
int            control_type = MANUAL_CONTROL; // Control type is manual by default
char           CHAN_0[256] = "Dev1/ao0";      // MOTOR 0
char           CHAN_1[256] = "Dev1/ao1";      // MOTOR 1 
char           CHAN_I[256] = "Dev1/ai0";	  // Analog input channel  
double         setpoint = 0;
double         Kr = 0;
double         Ti = 0;
double         Td = 0;
double         T  = 0.1;                      // The sampling rate
int            process_running = PROCESS_STOPPED;
int32          numRead;
float64        sensor_data = 0; 
// ***** END REGION *****

// ***** PROTOTYPES REGION *****  
int SetMotorCommand(char chan[256], float64 command);
// ***** END REGION ***** 

int main(int argc, char *argv[])
{
	if(InitCVIRTE(0,argv,0)==0 )
		return -1;  /* out of memory */
	if( (panelHandle=LoadPanel(0,"VoltUpdate.uir",PANEL))<0 )
		return -1;
	
	// Configure leds
	SetCtrlAttribute(panelHandle,PANEL_LED_MANUAL,ATTR_ON_COLOR,VAL_GREEN);
	SetCtrlAttribute(panelHandle,PANEL_LED_MANUAL,ATTR_OFF_COLOR,VAL_RED);
	SetCtrlAttribute(panelHandle,PANEL_LED_AUTOMATIC,ATTR_ON_COLOR,VAL_GREEN);
	SetCtrlAttribute(panelHandle,PANEL_LED_AUTOMATIC,ATTR_OFF_COLOR,VAL_RED);
	// Set default led active
	SetCtrlAttribute(panelHandle,PANEL_LED_MANUAL, ATTR_CTRL_VAL, 1);
	
	// Set number of displayed traces
	SetCtrlAttribute(panelHandle,PANEL_STRIPCHART, ATTR_NUM_TRACES, NUM_GRAPH_CHANNELS);
	
	// Disable automatic controls
	SetCtrlAttribute(panelHandle, PANEL_START, ATTR_DIMMED, 1);
	SetCtrlAttribute(panelHandle, PANEL_STOP, ATTR_DIMMED, 1);

	//NIDAQmx_NewPhysChanAOCtrl(panelHandle,PANEL_CHANNEL  ,0);  // Adauga canalele de iesire in dropdown
	//NIDAQmx_NewPhysChanAICtrl(panelHandle,PANEL_CHANNEL_2,1);  // Adauga canalele de intrare in dropdown  
	
	DisplayPanel(panelHandle);
	RunUserInterface();
	DiscardPanel(panelHandle);
	return 0;
}

int CVICALLBACK PanelCallback(int panel, int event, void *callbackData, int eventData1, int eventData2)
{
	if( event==EVENT_CLOSE )
	{
		// Stop both motors when the application is closed
		SetMotorCommand(CHAN_0, 0);
		SetMotorCommand(CHAN_1, 0);
				
		QuitUserInterface(0);
	}
	return 0;
}

/***********************************************
 * This function handles the control type and 
 * updates the interface properly.
 ***********************************************/
int CVICALLBACK tipControl (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{				   
		case EVENT_COMMIT:
			if(control_type == MANUAL_CONTROL)
			{
				SetCtrlAttribute(panelHandle,PANEL_control_type_select,ATTR_LABEL_TEXT,"AUTOMATIC CONTROL");
				control_type = AUTOMATIC_CONTROL;
				
				// Update led indicators 
				SetCtrlAttribute(panelHandle, PANEL_LED_MANUAL, ATTR_CTRL_VAL, 0);
				SetCtrlAttribute(panelHandle, PANEL_LED_AUTOMATIC, ATTR_CTRL_VAL, 1);  
				
				// Disable manual controls
				SetCtrlAttribute(panelHandle, PANEL_MOTOR_ZERO_SLIDER, ATTR_DIMMED, 1);
				SetCtrlAttribute(panelHandle, PANEL_MOTOR_ONE_SLIDER, ATTR_DIMMED, 1);
				SetCtrlAttribute(panelHandle, PANEL_SAME_COMMAND_CHBX, ATTR_DIMMED, 1);
				
				// Configure automatic controls
				SetCtrlAttribute(panelHandle, PANEL_START, ATTR_DIMMED, 0);
				SetCtrlAttribute(panelHandle, PANEL_STOP, ATTR_DIMMED, 1);
				
				// Reset both motors when switching to automatic control
				SetMotorCommand(CHAN_0, 0);
				SetMotorCommand(CHAN_1, 0);
			}
			else
			{
				SetCtrlAttribute(panelHandle,PANEL_control_type_select,ATTR_LABEL_TEXT,"MANUAL CONTROL");
				control_type = MANUAL_CONTROL;	
				
				// Update led indicators  
				SetCtrlAttribute(panelHandle, PANEL_LED_MANUAL, ATTR_CTRL_VAL, 1);
				SetCtrlAttribute(panelHandle, PANEL_LED_AUTOMATIC, ATTR_CTRL_VAL, 0);
				
				// Enable manual controls 
				SetCtrlAttribute(panelHandle, PANEL_MOTOR_ZERO_SLIDER, ATTR_DIMMED, 0);
				SetCtrlAttribute(panelHandle, PANEL_MOTOR_ONE_SLIDER, ATTR_DIMMED, 0);
				SetCtrlAttribute(panelHandle, PANEL_SAME_COMMAND_CHBX, ATTR_DIMMED, 0); 
				
				// Disable automatic controls
				SetCtrlAttribute(panelHandle, PANEL_START, ATTR_DIMMED, 1);
				SetCtrlAttribute(panelHandle, PANEL_STOP, ATTR_DIMMED, 1);
				
				// Reset both motors when switching to manual control
				SetMotorCommand(CHAN_0, 0);
				SetMotorCommand(CHAN_1, 0);
			}
		break;
	}
	return 0;
}

int SetMotorCommand(char chan[256], float64 command)
{
	int         error=0;
	TaskHandle  taskHandle=0;
	double      min = 0, max = 5;
	char        errBuff[2048]={'\0'};
	
	// DAQmx Configure Code
	SetWaitCursor(1);
	DAQmxErrChk (DAQmxCreateTask("",&taskHandle));
	DAQmxErrChk (DAQmxCreateAOVoltageChan(taskHandle,chan,"",min,max,DAQmx_Val_Volts,""));

	// DAQmx Start Code
	DAQmxErrChk (DAQmxStartTask(taskHandle));

	// DAQmx Write Code
	DAQmxErrChk (DAQmxWriteAnalogF64(taskHandle,1,1,10.0,DAQmx_Val_GroupByChannel,&command,NULL,NULL));

Error:
	SetWaitCursor(0);
	if( DAQmxFailed(error) )
		DAQmxGetExtendedErrorInfo(errBuff,2048);
	if( taskHandle!=0 ) {
		// DAQmx Stop Code
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if( DAQmxFailed(error) )
		MessagePopup("DAQmx Error",errBuff);
	
	return 0;
}

/**************************************************
 * This callback handles the speed of motor nr
 * zero. The voltage applied is set whith a slider. 
 **************************************************/
int CVICALLBACK MotorZeroCallback (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	float64 data;
	int     sameCommand = 0;
	
	switch (event)
	{
		case EVENT_COMMIT:
			// Retrieve voltage value from the slider
			GetCtrlVal(panel, PANEL_MOTOR_ZERO_SLIDER, &data);
			GetCtrlVal(panel, PANEL_SAME_COMMAND_CHBX, &sameCommand);
			 
			if(sameCommand == 1)
			{
				SetCtrlAttribute(panelHandle, PANEL_MOTOR_ONE_SLIDER, ATTR_CTRL_VAL, data);
				SetMotorCommand(CHAN_1, data); 	
			}
			
			SetMotorCommand(CHAN_0, data);
		break;
	}
	return 0;
}

/**************************************************
 * This callback handles the speed of motor nr
 * one. The voltage applied is set whith a slider. 
 **************************************************/
int CVICALLBACK MotorOneCallback (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	float64 data;
	int     sameCommand = 0;
	
	switch (event)
	{
		case EVENT_COMMIT:
			// Retrieve voltage value from the slider  
			GetCtrlVal(panel, PANEL_MOTOR_ONE_SLIDER, &data);
			GetCtrlVal(panel, PANEL_SAME_COMMAND_CHBX, &sameCommand); 
			
			if(sameCommand == 1)
			{
				SetCtrlAttribute(panelHandle, PANEL_MOTOR_ZERO_SLIDER, ATTR_CTRL_VAL, data);
				SetMotorCommand(CHAN_0, data); 	
			}
			
			SetMotorCommand(CHAN_1, data);  
		break;
	}
	return 0;
}

/**************************************************
 * This callback STARTS the control process.
 **************************************************/
int CVICALLBACK StartCallback (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			// Set control algorithm parameters
			GetCtrlVal(panel,PANEL_SET_POINT_VALUE, &setpoint);
			GetCtrlVal(panel,PANEL_KR_VALUE, &Kr);
			GetCtrlVal(panel,PANEL_TI_VALUE, &Ti);
			GetCtrlVal(panel,PANEL_TD_VALUE, &Td);
			// Get and set properly the sampling period
			GetCtrlVal(panel,PANEL_PERIOD, &T); 
			SetCtrlAttribute(panelHandle,PANEL_TIMER,ATTR_INTERVAL,T);  
			
			// Configure buttons
			SetCtrlAttribute(panelHandle, PANEL_START, ATTR_DIMMED, 1);
			SetCtrlAttribute(panelHandle, PANEL_STOP, ATTR_DIMMED, 0); 
			
			// Disable control type button
			SetCtrlAttribute(panelHandle, PANEL_control_type_select, ATTR_DIMMED, 1); 
			
			process_running = PROCESS_STARTED;
			
		break;
	}
	return 0;
}

/**************************************************
 * This callback STOPS the control process.
 **************************************************/
int CVICALLBACK StopCallback (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			// Configure buttons
			SetCtrlAttribute(panelHandle, PANEL_START, ATTR_DIMMED, 0);
			SetCtrlAttribute(panelHandle, PANEL_STOP, ATTR_DIMMED, 1); 
			
			// Enable control type button
			SetCtrlAttribute(panelHandle, PANEL_control_type_select, ATTR_DIMMED, 0); 
			
			// Reset both motors when switching to automatic control
			SetMotorCommand(CHAN_0, 0);
			SetMotorCommand(CHAN_1, 0);
		
			process_running = PROCESS_STOPPED; 
		break;
	}
	return 0;
}

/***************************************************
 * This callback serves as an event for sampling the 
 * output. The sampling period should be configured 
 * properly. Default is 0.1 seconds.
 **************************************************/
int CVICALLBACK TimerCallback (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	int32       error=0;
	float64     min = 0, max = 5;
	uInt32      numChannels = 1;   
	TaskHandle  taskHandle=0;  
	uInt32      sampsPerChan = 1;
	char        errBuff[2048]={'\0'};
	float64     plotData[NUM_GRAPH_CHANNELS];
	double      command = 0;

	switch (event)
	{
		case EVENT_TIMER_TICK:

			// Analog voltage input handling
			
			// DAQmx Configure Code
			DAQmxErrChk (DAQmxCreateTask("",&taskHandle));
			DAQmxErrChk (DAQmxCreateAIVoltageChan(taskHandle,CHAN_I,"",DAQmx_Val_Cfg_Default,min,max,DAQmx_Val_Volts,NULL));
			//DAQmxErrChk (DAQmxGetTaskAttribute(taskHandle,DAQmx_Task_NumChans,&numChannels));
		
			// DAQmx Start Code
			DAQmxErrChk (DAQmxStartTask(taskHandle));

			// DAQmx Read Code
			DAQmxErrChk (DAQmxReadAnalogF64(taskHandle,-1,10.0,DAQmx_Val_GroupByChannel,&sensor_data,sampsPerChan*numChannels,&numRead,NULL));
			
			if(process_running)
			{
				// Here goes PID control logic
				
				command = pid_incremental(Kr, Ti, Td, T, setpoint, sensor_data, 1, 1.7); 
				//printf("\nSensor data = %f, command = %f", sensor_data, command); 
				SetMotorCommand(CHAN_0, command);
				SetMotorCommand(CHAN_1, command);
			}
			
			plotData[0] = sensor_data;
		    plotData[1] = command;
		    plotData[2] = setpoint;
			PlotStripChart(panelHandle,PANEL_STRIPCHART,plotData,numRead*NUM_GRAPH_CHANNELS,0,0,VAL_DOUBLE); 
		break;
	}
	

Error:
#if 1
	//SetWaitCursor(0);
	if( DAQmxFailed(error) )
		DAQmxGetExtendedErrorInfo(errBuff,2048);
	if( taskHandle!=0 ) {
		// DAQmx Stop Code
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if( DAQmxFailed(error) )
		MessagePopup("DAQmx Error",errBuff);
#endif
	
	return 0;
}


/*****************************************************
 * This callback handles the same command checkbox.
 * When the chkbox is checked, both motor commands are
 * set to zero.
 ****************************************************/
int CVICALLBACK SameCommandCHECKED (int panel, int control, int event,
		void *callbackData, int eventData1, int eventData2)
{
	int sameCommand = 0;
	
	switch (event)
	{
		case EVENT_COMMIT:
			GetCtrlVal(panel, PANEL_SAME_COMMAND_CHBX, &sameCommand);
			
			// When checkbox is checked, reset all motors to zero and update sliders accordingly
			if(sameCommand == 1)
			{
				SetCtrlAttribute(panelHandle, PANEL_MOTOR_ZERO_SLIDER, ATTR_CTRL_VAL, 0.0);  
				SetCtrlAttribute(panelHandle, PANEL_MOTOR_ONE_SLIDER, ATTR_CTRL_VAL, 0.0);
				SetMotorCommand(CHAN_1, 0);
				SetMotorCommand(CHAN_0, 0);  
			}
		break;
	}
	return 0;
}
