/***************************************************** target specific file ***/
/*  Wrapper module for Simulink models                                        */
/*  ------------------------------------------------------------------------  */
/*  (c) IPG Automotive GmbH    www.ipg-automotive.com   Fon: +49.721.98520-0  */
/*  Bannwaldallee 60      D-76185 Karlsruhe   Germany   Fax: +49.721.98520-99 */
/******************************************************************************/

#ifndef IS_CAR
# define IS_CAR
#endif

#include "Global.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <math.h>

#include "simstruc.h"
#include "rt_sim.h"
#include "simstruc_types.h"

#include "InfoUtils.h"
#include "SimCore.h"
#include "Log.h"
#include "MatSupp.h"
#include "ModelManager.h"

#if defined (IS_CAR) || defined (IS_TRUCK)
# include "Car/Susp.h"
#elif defined(IS_MCYCLE)
# include "MCycle/MCycle.h"
#endif

#include "BodyCtrl_RTW.h"
#include "BodyCtrl_RTW_wrap.h"


#define QUOTE1(name)	#name
#define QUOTE(name)	QUOTE1(name)		/* need to expand name */

#ifndef CLASSIC_INTERFACE
# define EXPAND_CONCAT(name1,name2) name1 ## name2
# define CONCAT(name1,name2) EXPAND_CONCAT(name1,name2)
# define MODEL_INITIALIZE CONCAT(MODEL,_initialize)
# define MODEL_STEP       CONCAT(MODEL,_step)
# define MODEL_TERMINATE  CONCAT(MODEL,_terminate)
# define RT_MDL_TYPE      CONCAT(MODEL,_M_TYPE)
#endif

extern const char BodyCtrl_RTW_LibIdent[];
const char BodyCtrl_RTW_LibIdent[] = "(@@)" QUOTE(MODEL) " " ARCH " 1.0 " BUILDDATE;

static const char Modelname[] = QUOTE(MODEL);
static const tModelClass Modelclass = ModelClass_SuspExtFrcs;
static tMatSuppSampling SampleParams;

static void
DoOneStep (rtModel_BodyCtrl_RTW *rtm)
{
#ifdef CLASSIC_INTERFACE
    real_T tnext;

    tnext = rt_SimGetNextSampleHit(rtmGetTimingData(rtm),
				   rtmGetNumSampleTimes(rtm));
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(rtm),tnext);

    rtmiOutputs(rtmGetRTWRTModelMethodsInfo(rtm),0);
    rtmiUpdate(rtmGetRTWRTModelMethodsInfo(rtm),0);
    rt_SimUpdateDiscreteTaskSampleHits(rtmGetNumSampleTimes(rtm),
				       rtmGetTimingData(rtm),
				       rtmGetSampleHitPtr(rtm),
				       rtmGetTPtr(rtm));

    if (rtmGetSampleTime(rtm,0) == CONTINUOUS_SAMPLE_TIME) {
	if (rtmGetNumContStates(rtm) > 0) {
	    rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(rtm));
	} else {
	    rtsiSetT(rtmGetRTWSolverInfo(rtm), rtsiGetSolverStopTime(rtmGetRTWSolverInfo(rtm)));
	}
    }
#else
    MODEL_STEP(rtm);
#endif
}


/*
 * Define dictionary entries made from tunable parameters.
 * The function will actually be called twice, first with tuns==NULL.
 */
static void
DeclParameterQuants (struct tMatSuppTunables *tuns)
{
    /*MatSupp_TunDDictDefScalar(tuns, "MyParam", INFOMAT_DOUBLE, "kappa", "kg/s");*/
}


static void
BodyCtrl_RTW_DeclQuants (void *MP)
{
    rtModel_BodyCtrl_RTW *rtm = (rtModel_BodyCtrl_RTW *)MP;
    int i;

    /*Log("%s_DeclQuants()\n", Modelname);*/

    if (rtm == NULL) {
	/* Remember body frames defined in this model for later registration. */
        MdlBdyFrame_Add(BodyCtrl_RTW_BdyFrameDefines);

	/* Define dict entries for non-dynamically allocated variables. */
	if ((i = MatSupp_DeclQuants(BodyCtrl_RTW_DictDefines)) >= 0) {
	    LogErrF(EC_Init, "Model '%s': Error defining quantity '%s'\n",
		    Modelname, BodyCtrl_RTW_DictDefines[i].Name);
	}

	/* Define dict entries for tunable parameters (with dummy address). */
	DeclParameterQuants(NULL);
    } else {
	/* Define dict entries for dynamically allocated variables. */
    }
}


/*
 * BodyCtrl_RTW_SetParams() will be invoked indirectly by the generated
 * model C code each time BodyCtrl_RTW_New() is called.
 */
void
BodyCtrl_RTW_SetParams (rtModel_BodyCtrl_RTW *rtm, struct tMatSuppTunables *tuns,
		   struct tInfos *Inf)
{
    /*Log("%s_SetParams()\n", Modelname);*/

    /*
     * Parameter tuning - Part 1
     * This is the place to modify parameters of a storage class
     * other than 'SimulinkGlobal'.
     */

    /* We're purposely reading the parameter manually, not using
       CarMaker target's tunable parameter interface in this simple
       case of a single scalar value. */
    BodyCtrl_k = iGetDblOpt(Inf, "BodyCtrl.k", 300);

    if (tuns == NULL)
        return;

    /* Define dict entries for tunable parameters (address update). */
    DeclParameterQuants(tuns);

    /*
     * Parameter tuning - Part 2
     * This is the place to modify parameters of storage class
     * 'SimulinkGlobal', e.g. using the CarMaker target's tunable parameter
     * interface.
     */

    const char *prefix = Model_Class2Str(Modelclass);
    MatSupp_TunReadAllOpt(tuns, Inf, prefix);
    /*MatSupp_TunReadAll(tuns, ...);*/
    /*MatSupp_TunReadDef(tuns, ...);*/
    /*MatSupp_TunRead(tuns, ...);*/
}


static void *
assignCfgIF (struct tSuspExtFrcsCfgIF *CfgIF, void *MP)
{
    rtModel_BodyCtrl_RTW *rtm = (rtModel_BodyCtrl_RTW *)MP;
    ExternalInputs_BodyCtrl_RTW  *in  = (ExternalInputs_BodyCtrl_RTW *) rtmGetU(rtm);

    /* CfgIF Input */
    in->CfgInFromCM.VehicleClassId	= CfgIF->VhclClassId;
    in->CfgInFromCM.nWheels		= CfgIF->nWheels;

    return rtm;
}


static void *
BodyCtrl_RTW_New (struct tInfos *Inf, tSuspExtFrcsCfgIF *CfgIF, const char *KindKey)
{
    rtModel_BodyCtrl_RTW *rtm;
    double rtmSampleTime;
    const char *ModelKind;
    int VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, Modelclass, KindKey,
	 				 0, PARAMID, &VersionId)) == NULL)
	return NULL;

    /*Log("%s_New()\n", Modelname);*/

    MatSupp_ResetQuants(BodyCtrl_RTW_DictDefines);

    rtm = MODEL(Inf);

#ifdef CLASSIC_INTERFACE
    rtmSetT(rtm, 0.0);
    rtmSetTFinal(rtm, -1 /*run forever*/);

    rtmiInitializeSizes(rtmGetRTWRTModelMethodsInfo(rtm));
    rtmiInitializeSampleTimes(rtmGetRTWRTModelMethodsInfo(rtm));
    rt_SimInitTimingEngine(rtmGetNumSampleTimes(rtm),
			   rtmGetStepSize(rtm),
			   rtmGetSampleTimePtr(rtm),
			   rtmGetOffsetTimePtr(rtm),
			   rtmGetSampleHitPtr(rtm),
			   rtmGetSampleTimeTaskIDPtr(rtm),
			   rtmGetTStart(rtm),
			   &rtmGetSimTimeStep(rtm),
			   &rtmGetTimingData(rtm));
    if (rtmGetNumContStates(rtm) > 0) {
	rt_ODECreateIntegrationData(rtmGetRTWSolverInfo(rtm));
    } else {
	rtsiSetSolverName(rtmGetRTWSolverInfo(rtm), "FixedStepDiscrete");
    }
    rtsiSetVariableStepSolver(rtmGetRTWSolverInfo(rtm), 0);
#endif

    rtmSampleTime = (double) rtmGetStepSize(rtm);
    if (MatSupp_Sampling (&SampleParams, SimCore.DeltaT, rtmSampleTime) !=0) {
	LogErrF(EC_Init, "Model '%s': The sample times of the plugin model and the application have to be integer multiples\n",Modelname);
	return NULL;
    }

    /* assign CfgIF struct */
    if ((assignCfgIF(CfgIF, rtm)) == NULL)  {
	LogErrF(EC_Init, "Model '%s': failed to assign CfgIF\n",Modelname);
	return NULL;
    }

#ifdef CLASSIC_INTERFACE
    rtmiStart(rtmGetRTWRTModelMethodsInfo(rtm));
#else
    MODEL_INITIALIZE(rtm);
#endif

    return rtm; /* Will be passed as MP to the other functions. */
}


static void
BodyCtrl_RTW_Delete (void *MP)
{
    rtModel_BodyCtrl_RTW *rtm = (rtModel_BodyCtrl_RTW *)MP;

    /*Log("%s_Delete()\n", Modelname);*/

#ifdef CLASSIC_INTERFACE
    rt_SimDestroyTimingEngine(rtmGetTimingData(rtm));
    if (rtmGetNumContStates(rtm) > 0)
	rt_ODEDestroyIntegrationData(rtmGetRTWSolverInfo(rtm));
    rtmiTerminate(rtmGetRTWRTModelMethodsInfo(rtm));
#else
    MODEL_TERMINATE(rtm);
#endif
}


static int
BodyCtrl_RTW_Calc (void *MP, tSuspExtFrcsIF *IF, double dt)
{
    rtModel_BodyCtrl_RTW *rtm = (rtModel_BodyCtrl_RTW *)MP;
    int osCount=0;
    ExternalInputs_BodyCtrl_RTW  *in  = (ExternalInputs_BodyCtrl_RTW *) rtmGetU(rtm);
    ExternalOutputs_BodyCtrl_RTW *out = (ExternalOutputs_BodyCtrl_RTW *)rtmGetY(rtm);

#if defined (IS_MCYCLE)
    in->FromCM.lSpring.F     = IF->lSpring[0];
    in->FromCM.vSpring.F     = IF->vSpring[0];
    in->FromCM.lDamp.F       = IF->lDamp[0];
    in->FromCM.vDamp.F       = IF->vDamp[0];
    in->FromCM.lBuf.F        = IF->lBuf[0];
    in->FromCM.vBuf.F        = IF->vBuf[0];

    in->FromCM.lSpring.R     = IF->lSpring[1];
    in->FromCM.vSpring.R     = IF->vSpring[1];
    in->FromCM.lDamp.R       = IF->lDamp[1];
    in->FromCM.vDamp.R       = IF->vDamp[1];
    in->FromCM.lBuf.R        = IF->lBuf[1];
    in->FromCM.vBuf.R        = IF->vBuf[1];
#elif defined (IS_CAR) || defined (IS_TRUCK)
    in->FromCM.lSpring.FL     = IF->lSpring[0];
    in->FromCM.vSpring.FL     = IF->vSpring[0];
    in->FromCM.lDamp.FL       = IF->lDamp[0];
    in->FromCM.vDamp.FL       = IF->vDamp[0];
    in->FromCM.lBuf.FL        = IF->lBuf[0];
    in->FromCM.vBuf.FL        = IF->vBuf[0];
    in->FromCM.lStabi.FL      = IF->lStabi[0];
    in->FromCM.vStabi.FL      = IF->vStabi[0];

    in->FromCM.lSpring.FR     = IF->lSpring[1];
    in->FromCM.vSpring.FR     = IF->vSpring[1];
    in->FromCM.lDamp.FR       = IF->lDamp[1];
    in->FromCM.vDamp.FR       = IF->vDamp[1];
    in->FromCM.lBuf.FR        = IF->lBuf[1];
    in->FromCM.vBuf.FR        = IF->vBuf[1];
    in->FromCM.lStabi.FR      = IF->lStabi[1];
    in->FromCM.vStabi.FR      = IF->vStabi[1];

    in->FromCM.lSpring.RL     = IF->lSpring[2];
    in->FromCM.vSpring.RL     = IF->vSpring[2];
    in->FromCM.lDamp.RL       = IF->lDamp[2];
    in->FromCM.vDamp.RL       = IF->vDamp[2];
    in->FromCM.lBuf.RL        = IF->lBuf[2];
    in->FromCM.vBuf.RL        = IF->vBuf[2];
    in->FromCM.lStabi.RL      = IF->lStabi[2];
    in->FromCM.vStabi.RL      = IF->vStabi[2];

    in->FromCM.lSpring.RR     = IF->lSpring[3];
    in->FromCM.vSpring.RR     = IF->vSpring[3];
    in->FromCM.lDamp.RR       = IF->lDamp[3];
    in->FromCM.vDamp.RR       = IF->vDamp[3];
    in->FromCM.lBuf.RR        = IF->lBuf[3];
    in->FromCM.vBuf.RR        = IF->vBuf[3];
    in->FromCM.lStabi.RR      = IF->lStabi[3];
    in->FromCM.vStabi.RR      = IF->vStabi[3];
#endif
#if defined (IS_TRUCK)	
    in->FromCM.lSpring.RL2    = IF->lSpring[4];
    in->FromCM.vSpring.RL2    = IF->vSpring[4];
    in->FromCM.lDamp.RL2      = IF->lDamp[4];
    in->FromCM.vDamp.RL2      = IF->vDamp[4];
    in->FromCM.lBuf.RL2       = IF->lBuf[4];
    in->FromCM.vBuf.RL2       = IF->vBuf[4];
    in->FromCM.lStabi.RL2     = IF->lStabi[4];
    in->FromCM.vStabi.RL2     = IF->vStabi[4];

    in->FromCM.lSpring.RR2    = IF->lSpring[5];
    in->FromCM.vSpring.RR2    = IF->vSpring[5];
    in->FromCM.lDamp.RR2      = IF->lDamp[5];
    in->FromCM.vDamp.RR2      = IF->vDamp[5];
    in->FromCM.lBuf.RR2       = IF->lBuf[5];
    in->FromCM.vBuf.RR2       = IF->vBuf[5];
    in->FromCM.lStabi.RR2     = IF->lStabi[5];
    in->FromCM.vStabi.RR2     = IF->vStabi[5];

    in->FromCM.lSpring.FL2    = IF->lSpring[6];
    in->FromCM.vSpring.FL2    = IF->vSpring[6];
    in->FromCM.lDamp.FL2      = IF->lDamp[6];
    in->FromCM.vDamp.FL2      = IF->vDamp[6];
    in->FromCM.lBuf.FL2       = IF->lBuf[6];
    in->FromCM.vBuf.FL2       = IF->vBuf[6];
    in->FromCM.lStabi.FL2     = IF->lStabi[6];
    in->FromCM.vStabi.FL2     = IF->vStabi[6];

    in->FromCM.lSpring.FR2    = IF->lSpring[7];
    in->FromCM.vSpring.FR2    = IF->vSpring[7];
    in->FromCM.lDamp.FR2      = IF->lDamp[7];
    in->FromCM.vDamp.FR2      = IF->vDamp[7];
    in->FromCM.lBuf.FR2       = IF->lBuf[7];
    in->FromCM.vBuf.FR2       = IF->vBuf[7];
    in->FromCM.lStabi.FR2     = IF->lStabi[7];
    in->FromCM.vStabi.FR2     = IF->vStabi[7];
#endif

    if (SampleParams.UnderSampFac) {	// Undersampling
     	if (++SampleParams.UnderSampCount == SampleParams.UnderSampFac) {
	    SampleParams.UnderSampCount=0;
	    DoOneStep(rtm);
     	}
    } else { 				// Oversampling (1..OverSampFac)
	do {
	    DoOneStep(rtm);
	} while (++osCount < SampleParams.OverSampFac);
    }

#if defined (IS_MCYCLE)
    IF->FrcSpring[0] = out->ToCM.FrcSpring.F;
    IF->FrcDamp[0]   = out->ToCM.FrcDamp.F;
    IF->FrcBuf[0]    = out->ToCM.FrcBuf.F;
    IF->FrcSpring[1] = out->ToCM.FrcSpring.R;
    IF->FrcDamp[1]   = out->ToCM.FrcDamp.R;
    IF->FrcBuf[1]    = out->ToCM.FrcBuf.R;
#elif defined (IS_CAR) || defined (IS_TRUCK)
    IF->FrcSpring[0] = out->ToCM.FrcSpring.FL;
    IF->FrcDamp[0]   = out->ToCM.FrcDamp.FL;
    IF->FrcBuf[0]    = out->ToCM.FrcBuf.FL;
    IF->FrcStabi[0]  = out->ToCM.FrcStabi.FL;
    IF->FrcSpring[1] = out->ToCM.FrcSpring.FR;
    IF->FrcDamp[1]   = out->ToCM.FrcDamp.FR;
    IF->FrcBuf[1]    = out->ToCM.FrcBuf.FR;
    IF->FrcStabi[1]  = out->ToCM.FrcStabi.FR;
    IF->FrcSpring[2] = out->ToCM.FrcSpring.RL;
    IF->FrcDamp[2]   = out->ToCM.FrcDamp.RL;
    IF->FrcBuf[2]    = out->ToCM.FrcBuf.RL;
    IF->FrcStabi[2]  = out->ToCM.FrcStabi.RL;
    IF->FrcSpring[3] = out->ToCM.FrcSpring.RR;
    IF->FrcDamp[3]   = out->ToCM.FrcDamp.RR;
    IF->FrcBuf[3]    = out->ToCM.FrcBuf.RR;
    IF->FrcStabi[3]  = out->ToCM.FrcStabi.RR;
#endif
#if defined (IS_TRUCK)
    IF->FrcSpring[4] = out->ToCM.FrcSpring.RL2;
    IF->FrcDamp[4]   = out->ToCM.FrcDamp.RL2;
    IF->FrcBuf[4]    = out->ToCM.FrcBuf.RL2;
    IF->FrcStabi[4]  = out->ToCM.FrcStabi.RL2;
    IF->FrcSpring[5] = out->ToCM.FrcSpring.RR2;
    IF->FrcDamp[5]   = out->ToCM.FrcDamp.RR2;
    IF->FrcBuf[5]    = out->ToCM.FrcBuf.RR2;
    IF->FrcStabi[5]  = out->ToCM.FrcStabi.RR2;
    IF->FrcSpring[6] = out->ToCM.FrcSpring.FL2;
    IF->FrcDamp[6]   = out->ToCM.FrcDamp.FL2;
    IF->FrcBuf[6]    = out->ToCM.FrcBuf.FL2;
    IF->FrcStabi[6]  = out->ToCM.FrcStabi.FL2;
    IF->FrcSpring[7] = out->ToCM.FrcSpring.FR2;
    IF->FrcDamp[7]   = out->ToCM.FrcDamp.FR2;
    IF->FrcBuf[7]    = out->ToCM.FrcBuf.FR2;
    IF->FrcStabi[7]  = out->ToCM.FrcStabi.FR2;
#endif

    return 0;
}


int
BodyCtrl_RTW_Register (void)
{
    tModelClassDescr m;

    /*Log("%s_Register()\n", Modelname);*/

    memset(&m, 0, sizeof(m));

    /* Parameter file identification number.
       You may change CompatVersionId to the the lowest parameter
       file version understood by your model code. */
    m.SuspExtFrcs.VersionId		= PARAMID;
    m.SuspExtFrcs.CompatVersionId	= m.SuspExtFrcs.VersionId;

    m.SuspExtFrcs.DeclQuants		= BodyCtrl_RTW_DeclQuants;
    m.SuspExtFrcs.New			= BodyCtrl_RTW_New;
    m.SuspExtFrcs.Calc			= BodyCtrl_RTW_Calc;
    m.SuspExtFrcs.Delete		= BodyCtrl_RTW_Delete;
    /* Should only be used if the model doesn't read params from extra files */
    // m.SuspExtFrcs.ParamsChanged	= ParamsChanged_IgnoreCheck;

    return Model_Register(Modelclass, Modelname, &m);
}

