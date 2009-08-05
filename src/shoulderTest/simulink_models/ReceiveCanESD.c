/*
 * File: ReceiveCanESD.c
 *
 *
  *
  *   --- THIS FILE GENERATED BY S-FUNCTION BUILDER: 3.0 ---
  *
  *   This file is an S-function produced by the S-Function
  *   Builder which only recognizes certain fields.  Changes made
  *   outside these fields will be lost the next time the block is
  *   used to load, edit, and resave this file. This file will be overwritten
  *   by the S-function Builder block. If you want to edit this file by hand, 
  *   you must change it only in the area defined as:  
  *
  *        %%%-SFUNWIZ_defines_Changes_BEGIN
  *        #define NAME 'replacement text' 
  *        %%% SFUNWIZ_defines_Changes_END
  *
  *   DO NOT change NAME--Change the 'replacement text' only.
  *
  *   For better compatibility with the Real-Time Workshop, the
  *   "wrapper" S-function technique is used.  This is discussed
  *   in the Real-Time Workshop User's Manual in the Chapter titled,
  *   "Wrapper S-functions".
  *
  *  -------------------------------------------------------------------------
  * | See matlabroot/simulink/src/sfuntmpl_doc.c for a more detailed template |
  *  ------------------------------------------------------------------------- 
 * Created: Wed Mar 15 17:07:55 2006
 * 
 *
 */


#define S_FUNCTION_NAME ReceiveCanESD
#define S_FUNCTION_LEVEL 2
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
/* %%%-SFUNWIZ_defines_Changes_BEGIN --- EDIT HERE TO _END */
#define NUM_INPUTS           0

#define NUM_OUTPUTS          1
/* Output Port  0 */
#define OUT_PORT_0_NAME      y0
#define OUTPUT_0_WIDTH       8
#define OUTPUT_DIMS_0_COL    1
#define OUTPUT_0_DTYPE       real_T
#define OUTPUT_0_COMPLEX     COMPLEX_NO
#define OUT_0_FRAME_BASED    FRAME_NO
#define OUT_0_DIMS           1-D
#define OUT_0_ISSIGNED        1
#define OUT_0_WORDLENGTH      8
#define OUT_0_FIXPOINTSCALING 1
#define OUT_0_FRACTIONLENGTH  3
#define OUT_0_BIAS            0
#define OUT_0_SLOPE           0.125

#define NPARAMS              1
/* Parameter  1 */
#define PARAMETER_0_NAME      msgID
#define PARAMETER_0_DTYPE     real_T
#define PARAMETER_0_COMPLEX   COMPLEX_NO

#define SAMPLE_TIME_0        INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES      0
#define DISC_STATES_IC       [0]
#define NUM_CONT_STATES      0
#define CONT_STATES_IC       [0]

#define SFUNWIZ_GENERATE_TLC 0
#define SOURCEFILES "__SFB__ntcan.lib__SFB__"
#define PANELINDEX           6
#define USE_SIMSTRUCT        0
#define SHOW_COMPILE_STEPS   0                   
#define CREATE_DEBUG_MEXFILE 0
#define SAVE_CODE_ONLY       0
#define SFUNWIZ_REVISION     3.0
/* %%%-SFUNWIZ_defines_Changes_END --- EDIT HERE TO _BEGIN */
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
#include "simstruc.h"
#include "ntcan.h"
#define PARAM_DEF0(S) ssGetSFcnParam(S, 0)

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

 HANDLE h_rec=NULL;
 long m_baudrate =0;
 long len=1;
 CMSG message[1];
 long rec;  
 int x=0;   
/*====================*
 * S-function methods *
 *====================*/
#define MDL_CHECK_PARAMETERS
 #if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
   /* Function: mdlCheckParameters =============================================
     * Abstract:
     *    Validate our parameters to verify they are okay.
     */
    static void mdlCheckParameters(SimStruct *S)
    {
     #define PrmNumPos 46
     int paramIndex = 0;
     bool validParam = false;
     char paramVector[] ={'1'};
     static char parameterErrorMsg[] ="The data type and/or complexity of parameter    does not match the information "
     "specified in the S-function Builder dialog. For non-double parameters you will need to cast them using int8, int16,"
     "int32, uint8, uint16, uint32 or boolean."; 

     /* All parameters must match the S-function Builder Dialog */
     

	 {
	  const mxArray *pVal0 = ssGetSFcnParam(S,0);
	  if (!IS_PARAM_DOUBLE(pVal0)) {
	    validParam = true;
	    paramIndex = 0;
	    goto EXIT_POINT;
	  }
	 }
     EXIT_POINT:
      if (validParam) {
	  parameterErrorMsg[PrmNumPos] = paramVector[paramIndex];
	  ssSetErrorStatus(S,parameterErrorMsg);
      }
	return;
    }
 #endif /* MDL_CHECK_PARAMETERS */
/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
 
//const real_T  *msgID  = mxGetData(PARAM_DEF0(S)); 
static void mdlInitializeSizes(SimStruct *S)
{
    int_T m_net=0;
    long m_txbufsize =1;
    long m_rxbufsize =10;
    long m_txtout    =10;
    long m_rxtout    =10;
    unsigned long m_mode =0;
    int_T i,j=0;
    x=0;

    //DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
      #if defined(MATLAB_MEX_FILE)
	if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
	  mdlCheckParameters(S);
	  if (ssGetErrorStatus(S) != NULL) {
	    return;
	  }
	 } else {
	   return; /* Parameter mismatch will be reported by Simulink */
	 }
      #endif

    ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES);

    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;

    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    ssSetOutputPortWidth(S, 0, OUTPUT_0_WIDTH);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, OUTPUT_0_COMPLEX);
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE ));
    rec = canOpen(m_net, m_mode, m_txbufsize, m_rxbufsize, m_txtout, m_rxtout, &h_rec);
  if(rec == NTCAN_SUCCESS)
    {
    
    }
   else
   {
	 
      
    ssSetErrorStatus(S,"CanOpen failed");

      return;
   }
 
  rec = canSetBaudrate(h_rec,m_baudrate);//canSetBaudrate(m_h0, z-1);
   if(rec == NTCAN_SUCCESS)
    {
  
    }
   else
   {
    	 ssSetErrorStatus(S,"CANSetBaudrate failed");
      return;
   }
  
  for (i =0; i<2047; i++)
  {
   rec = canIdAdd(h_rec, i);
  
  if(rec == NTCAN_SUCCESS)
    {
  }
   else
   {
	 
     ssSetErrorStatus(S,"CANIdAdd failed");
      return;   
   }
  }
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME_0);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
   ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}
/* Function: mdlOutputs =======================================================
 *
*/
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T        *y0  = (real_T *)ssGetOutputPortRealSignal(S,0);
    const int_T   p_width0  = mxGetNumberOfElements(PARAM_DEF0(S));
    const real_T  *msgID  = mxGetData(PARAM_DEF0(S));
     int d;
    
  len=1;
   
   rec =canRead(h_rec, message, &len, NULL);
   
   if(rec == NTCAN_SUCCESS)
   {
       if ((message!=NULL) && (message[0].id== msgID[0]))
       {   
            for (d=0;d<8;d++)
            {     
            y0[d]=(int) message[0].data[d];
            }
       }
   }

  
   
}



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


