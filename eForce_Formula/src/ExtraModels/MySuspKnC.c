/*
******************************************************************************
**  CarMaker - Version 9.1.1
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Simple external suspension Model
**
** Add the declaration of the register function to one of your header files,
** for example to User.h and call it in User_Register()
**
**    int Susp_KnC_Register_MyModel (void);
**
******************************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "CarMaker.h"
#include "Car/Vehicle_Car.h"
#include "MyModels.h"

static const char ThisModelClass[]  = "SuspKnC";
static const char ThisModelKind[]   = "MyModel_F";
static const char ThisModelKindLR[] = "MyModel_FLR";
static const int  ThisVersionId     = 1;

/* Modellparameter (statische) */
struct tMyModel {
    double	qComp2tz;
    double	qSteer2rz;
};


static void
MyModel_DeclQuants_dyn (struct tMyModel *mp, int park)
{
    static struct tMyModel MyModel_Dummy = {0};
    if (park)
	mp = &MyModel_Dummy;

    /* Define here dict entries for dynamically allocated variables. */
}


static void
MyModel_DeclQuants (void *MP)
{
    struct tMyModel *mp = (struct tMyModel *)MP;

    if (mp == NULL) {
	/* Define here dict entries for non-dynamically allocated (static) variables. */

    } else {
	MyModel_DeclQuants_dyn (mp, 0);
    }
}


static void
MyModel_Delete (void *MP)
{
    struct tMyModel *mp = (struct tMyModel *)MP;

    /* Park the dict entries for dynamically allocated variables before deleting */
    MyModel_DeclQuants_dyn (mp, 1);
    free (mp);
}


static void *
MyModel_New (
    struct tInfos  	*Inf,
    struct tSuspCfgIF	*SuspCfgIF,
    const char     	*KindKey,
    const char     	*Pre)
{
    struct tMyModel *mp = NULL;
    char MsgPre[64];
    const char *ModelKind;
    int VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, ModelClass_SuspKnC, KindKey,
	 				 0, ThisVersionId, &VersionId)) == NULL)
	return NULL;

    sprintf (MsgPre, "%s %s", ThisModelClass, ThisModelKind);

    mp = (struct tMyModel*)calloc(1, sizeof(*mp));

    /* SuspCfgIF */
    SuspCfgIF->Use_qSteer = 1;

    /* Kinematics */
    mp->qComp2tz  = iGetDblOpt (Inf, "MySusp_F.qComp2tz",  1.0);
    mp->qSteer2rz = iGetDblOpt (Inf, "MySusp_F.qSteer2rz", 5.0);

    return mp;
}


static void *
MyModel_New_LR (
    struct tInfos  	*Inf,
    struct tSuspCfgIF	*SuspCfgIF,
    const char     	*KindKey,
    const char     	*Pre)
{
    struct tMyModel *mp = NULL;
    char MsgPre[64];
    const char *ModelKind;
    int VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, ModelClass_SuspKnC, KindKey,
	 				 0, ThisVersionId, &VersionId)) == NULL)
	return NULL;

    sprintf (MsgPre, "%s %s", ThisModelClass, ThisModelKind);

    mp = (struct tMyModel*)calloc(1, sizeof(*mp));

    /* SuspCfgIF */
    SuspCfgIF->Use_qSteer  = 1;
    SuspCfgIF->DoubleSided = 1;

    /* Kinematics */
    mp->qComp2tz  = iGetDblOpt (Inf, "MySusp_FLR.qComp2tz",  1.0);
    mp->qSteer2rz = iGetDblOpt (Inf, "MySusp_FLR.qSteer2rz", 5.0);

    return mp;
}


static int
MyModel_Calc(void *MP, tSuspIF *IFMain, tSuspIF *IFOpp, double dt)
{
    struct tMyModel *mp = (struct tMyModel *)MP;
    int i;

    /* Kinematics on Main Side */
    IFMain->dqComp[ixtz] = 	 mp->qComp2tz;
    IFMain->dqComp[ixSpring] = 	-mp->qComp2tz;
    IFMain->dqComp[ixDamp] = 	-mp->qComp2tz;
    IFMain->dqComp[ixBuf] = 	-mp->qComp2tz;
    IFMain->dqComp[ixStabi] = 	 mp->qComp2tz;
    IFMain->dqSteer[ixrz] =	 mp->qSteer2rz;

    for (i=0; i<ixKinMax; i++) {
	IFMain->Kin[i] = IFMain->dqComp[i]  * IFMain->qComp
		       + IFMain->dqSteer[i] * IFMain->qSteer;
    }

    return 0;
}


static int
MyModel_Calc_LR (void *MP, tSuspIF *IFMain, tSuspIF *IFOpp, double dt)
{
    struct tMyModel *mp = (struct tMyModel *)MP;
    int i;

    /* Kinematics on Left Side */
    IFMain->dqComp[ixtz] = 	 mp->qComp2tz;
    IFMain->dqComp[ixSpring] = 	-mp->qComp2tz;
    IFMain->dqComp[ixDamp] = 	-mp->qComp2tz;
    IFMain->dqComp[ixBuf] = 	-mp->qComp2tz;
    IFMain->dqComp[ixStabi] = 	 mp->qComp2tz;
    IFMain->dqSteer[ixrz] =	 mp->qSteer2rz;

    for (i=0; i<ixKinMax; i++) {
	IFMain->Kin[i] = IFMain->dqComp[i]  * IFMain->qComp
		       + IFMain->dqSteer[i] * IFMain->qSteer;
    }

    /* Kinematics on Right Side */
    IFOpp->dqComp[ixtz] = 	 mp->qComp2tz;
    IFOpp->dqComp[ixSpring] = 	-mp->qComp2tz;
    IFOpp->dqComp[ixDamp] = 	-mp->qComp2tz;
    IFOpp->dqComp[ixBuf] = 	-mp->qComp2tz;
    IFOpp->dqComp[ixStabi] = 	 mp->qComp2tz;
    IFOpp->dqSteer[ixrz] =	 mp->qSteer2rz;

    for (i=0; i<ixKinMax; i++) {
	IFOpp->Kin[i] = IFOpp->dqComp[i]  * IFOpp->qComp
		      + IFOpp->dqSteer[i] * IFOpp->qSteer;
    }

    return 0;
}


int
Susp_KnC_Register_MyModel (void)
{
    tModelClassDescr m;

    memset (&m, 0, sizeof(m));
    m.SuspKnC.VersionId =	ThisVersionId;
    m.SuspKnC.New =		MyModel_New;
    m.SuspKnC.Calc =		MyModel_Calc;
    m.SuspKnC.Delete =		MyModel_Delete;
    m.SuspKnC.DeclQuants =	MyModel_DeclQuants;
    /* Should only be used if the model doesn't read params from extra files */
    m.SuspKnC.ParamsChanged = 	ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_SuspKnC, ThisModelKind, &m);
}

int
Susp_KnC_Register_MyModel_LR (void)
{
    tModelClassDescr m;

    memset (&m, 0, sizeof(m));
    m.SuspKnC.VersionId =	ThisVersionId;
    m.SuspKnC.New =		MyModel_New_LR;
    m.SuspKnC.Calc =		MyModel_Calc_LR;
    m.SuspKnC.Delete =		MyModel_Delete;
    m.SuspKnC.DeclQuants =	MyModel_DeclQuants;
    /* Should only be used if the model doesn't read params from extra files */
    m.SuspKnC.ParamsChanged = 	ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_SuspKnC, ThisModelKindLR, &m);
}
