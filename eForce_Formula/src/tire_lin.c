/*
******************************************************************************
**  CarMaker - Version 9.1.1
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive Engineering Software + Consulting GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
*
* S-function of a simple linear tire model, used in UserVehicle_RTW.mdl,
* must be compiled before use.
*
* Compile in 32-bit Matlab with one of
*   mex '-IC:/IPG/carmaker/win64-9.1.1/include' '-DLINUX' tire_lin.c
*   mex '-IC:/IPG/carmaker/win64-9.1.1/include' '-DWIN32' tire_lin.c
* Compile in 64-bit Matlab with one of
*   mex '-IC:/IPG/carmaker/win64-9.1.1/include' '-DLINUX' '-DLINUX64' tire_lin.c
*   mex '-IC:/IPG/carmaker/win64-9.1.1/include' '-DWIN32' '-DWIN64'   tire_lin.c
*/

#define S_FUNCTION_NAME  tire_lin
#define S_FUNCTION_LEVEL 2

#include <string.h>
#include <math.h>

#include <Global.h>
#include <MathUtils.h>
#include <Vehicle/Tire.h>


#include "simstruc.h"

#define NUM_PWORK		1
#define  PW_TIRE_PARAM(S)	(ssGetPWork(S)[0])



typedef struct tTire_LinParam {
    tTParamHeader Head;

    double  muRoad;

    struct {
	double FrictionRate;	
	double FrcRate;
	double FrcAmplify;
    } Side;

    struct {
	double FrcRate;
	double FrictionRate;
    } Longi;

    /* tire dynamics */
    double  lLF;	/* entrance length	LongFrc */
    double  lSF;	/* entrance length	SideFrc */
    double  lAT;	/* entrance length	AlignTrq */

    /* Internals */
    double  LFlast;	/* last values */
    double  SFlast;
    double  ATlast;
} tTire_LinParam;


static tTire_LinParam *
Tire_Lin_Alloc (void)
{
    tTire_LinParam tp = {
        {0,		/* TireNo		*/
	 1,		/* Side			*/
	 0,		/* Mount Id             */
	 0.3,		/* KinRollRadius	*/
	 {550000.0,	/* Radial.Stiffness	*/
	  8750.0},	/* Radial.Damping	*/
	{RollResistTrqLoad,	/* RollResist.Kind	*/
	 0.01},			/* RollResist.Value	*/
	 6000.,		/* maximale Radlast	*/
	 0.20,		/* NomWidth	  	*/
	 0.3,		/* NomRadius	  	*/
	 0.20320,	/* RimRadius	  	*/
	},
 
	1.0,		/* muRoad               */

        {0.9,		/* Side.FrictionRate	*/
	6.88,		/* Side.FrcRate		*/
	1.0},		/* Side.FrcAmplify	*/
	{3.0,		/* Longi.FrcRate	*/
	0.9},		/* Longi.FrictionRate	*/

        0.05,
	0.5,
	0.5,
	0,0,0
    };

    tTire_LinParam *p = malloc(sizeof(*p));
    *p = tp;
    return p;
}


static void
mdlInitializeSizes (SimStruct *S)
{
    int i;

    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!ssSetNumInputPorts(S, 7)) return;
    for (i=0; i<ssGetNumInputPorts(S); i++) {
	ssSetInputPortWidth(S, i, 1);
	ssSetInputPortDirectFeedThrough(S, i, 1);
    }

    if (!ssSetNumOutputPorts(S, 11)) return;
    for (i=0; i<ssGetNumOutputPorts(S); i++) {
	ssSetOutputPortWidth(S, i, 1);
    }

    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, NUM_PWORK);

    ssSetOptions(S, 0);
}


static void
mdlInitializeSampleTimes (SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}


#define MDL_START
static void
mdlStart (SimStruct *S)
{
    PW_TIRE_PARAM(S) = Tire_Lin_Alloc();
}


static void
mdlOutputs (SimStruct *S, int_T tid)
{
    /* TireXX.In */
    real_T Load		= **ssGetInputPortRealSignalPtrs(S, 0);
    real_T vx		= **ssGetInputPortRealSignalPtrs(S, 1);
    real_T vy		= **ssGetInputPortRealSignalPtrs(S, 2);
    real_T vrot		= **ssGetInputPortRealSignalPtrs(S, 3);
    /*real_T rim_turnv	= **ssGetInputPortRealSignalPtrs(S, 4);*/
    /*real_T InclAngle	= **ssGetInputPortRealSignalPtrs(S, 5);*/
    real_T mu		= **ssGetInputPortRealSignalPtrs(S, 6);

    /* TireXX.Out */
    real_T *Slp		= ssGetOutputPortRealSignal(S, 0);
    real_T *Alpha	= ssGetOutputPortRealSignal(S, 1);
    real_T *TurnSlp	= ssGetOutputPortRealSignal(S, 2);
    real_T *rBelt_eff	= ssGetOutputPortRealSignal(S, 3);
    real_T *vBelt	= ssGetOutputPortRealSignal(S, 4);
    real_T *LongFrc	= ssGetOutputPortRealSignal(S, 5);
    real_T *SideFrc	= ssGetOutputPortRealSignal(S, 6);
    real_T *LoadFrc	= ssGetOutputPortRealSignal(S, 7);
    real_T *OverturnTrq = ssGetOutputPortRealSignal(S, 8);
    real_T *RollResist	= ssGetOutputPortRealSignal(S, 9);
    real_T *AlignTrq	= ssGetOutputPortRealSignal(S, 10);
 
    tTire_LinParam *tp = (tTire_LinParam *)PW_TIRE_PARAM(S);
    double	absvx = fabs(vx);
    double 	F, d, deltav, vslp;

    mu /= tp->muRoad;

    /* Side Slip Angle */
    *Alpha = (absvx > 0.25 ? atan2(vy, vx) : 0.0);

    *rBelt_eff = tp->Head.KinRollRadius;	/* Effective tire radius */
    *vBelt = *rBelt_eff * vrot;			/* Belt speed */

    /* Slip */
    deltav = *vBelt - vx;
    if (deltav >= 0.0) {
	vslp = fabs(vrot);
    } else {
	vslp = fabs(vx);	
    }
    if (vslp == 0)
	vslp = 0.000001;

    if (vslp != 0)
	*Slp = deltav / vslp;
    else
	*Slp = 0.0;

    /* Side Force */
    F = tp->Side.FrcRate * Load * *Alpha;
    d = tp->Side.FrictionRate * Load;

    if (fabs(F) > d) {
	F = M_COPYSIGN(d, F);
    }
    *SideFrc = F * (-tp->Side.FrcAmplify) * mu;

    /* Longitudinal Force */
    F = tp->Longi.FrcRate * Load * *Slp;
    d = tp->Longi.FrictionRate * Load;
    if (fabs(F) > d) {
	F = M_COPYSIGN(d, F);
    }
    *LongFrc = F * mu;

    *TurnSlp = 0;				/* Turn Slip */

    *rBelt_eff = tp->Head.KinRollRadius;	/* Effective tire radius */

    *vBelt = *rBelt_eff * vrot;			/* Belt speed */

    *LoadFrc = Load;				/* Load force */

    *OverturnTrq = 0;				/* Overturn torque */

    *RollResist = 0;				/* Roll resistance */

    *AlignTrq = 0;				/* Aligning torque */
}


static void
mdlTerminate(SimStruct *S)
{
    if (PW_TIRE_PARAM(S) != NULL)
	free(PW_TIRE_PARAM(S));
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include <simulink.c>      /* MEX-file interface mechanism */
#else
#include <cg_sfun.h>       /* Code generation registration function */
#endif
