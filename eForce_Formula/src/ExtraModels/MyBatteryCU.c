/* $Id$ (c) IPG */
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
** Simple battery controll Model
**
** Add the declaration of the register function to one of your header files,
** for example to User.h and call it in User_Register()
**
**    BatteryCU_Register_MyModel ();
**
******************************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "CarMaker.h"
#include "Car/Vehicle_Car.h"
#include "MyModels.h"

static const char ThisModelKind[]  = "MyModel";
static const int  ThisVersionId    = 1;


struct tMyModel {
    double	Capacity_LV;
};


static void
MyModel_Delete (void *MP)
{
    struct tMyModel *mp = (struct tMyModel *)MP;
    free (mp);
}



static void *
MyModel_New (struct tInfos *Inf, struct tPTBatteryCU_CfgIF *CfgIF, const char *KindKey)
{
    struct tMyModel *mp = NULL;
    const char *ModelKind;
    int VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, ModelClass_PTBatteryCU, KindKey,
	 				 0, ThisVersionId, &VersionId)) == NULL)
	return NULL;

    mp = (struct tMyModel*)calloc(1,sizeof(*mp));
    mp->Capacity_LV = CfgIF->BattLV.Capacity;

    return mp;
}


static int
MyModel_Calc (void *MP, struct tPTBatteryCU_IF *IF, double dt)
{
    struct tMyModel *mp = (struct tMyModel *)MP;

    if (!IF->Ignition) {
	IF->SOC_LV = 0.0;
	IF->SOH_LV = 0.0;
	IF->Pwr_HV1toLV_trg  = 0.0;
	return 0;
    }

    /* State of charge */
    IF->SOC_LV = IF->BattLV.AOC / mp->Capacity_LV * 100.0;

    /* State of health */
    IF->SOH_LV = 100.0;

    return 0;
}

int 
BatteryCU_Register_MyModel (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.PTBatteryCU.VersionId =		ThisVersionId;
    m.PTBatteryCU.New =			MyModel_New;
    m.PTBatteryCU.Calc =		MyModel_Calc;
    m.PTBatteryCU.Delete =		MyModel_Delete;
    m.PTBatteryCU.DeclQuants =		NULL;
    m.PTBatteryCU.ModelCheck =		NULL;
    /* Should only be used if the model doesn't read params from extra files */
    m.PTBatteryCU.ParamsChanged = 	ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_PTBatteryCU, ThisModelKind, &m);
}
