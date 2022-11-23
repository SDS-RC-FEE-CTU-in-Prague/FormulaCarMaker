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
*/

#include <stdlib.h>
#include <string.h>

#include "infoc.h"
#include "Log.h"
#include "DataDict.h"
#include "Vehicle.h"
#include "ModelManager.h"

#include "cruise.h"		/* cruise.h, as delivered by AVL. */


/* ModelName has to match the 'Interface Label' of the Cruise interface block. */
//static const char ModelClass[] = "PTCruiseIF";
static const char ModelName[]  = "CM-MyCruiseIf";


static int IfIndex;


enum { MaxChannels = 100 };

static double Inputs[MaxChannels], Outputs[MaxChannels];
static int InDeclared[MaxChannels] = { 0 }, OutDeclared[MaxChannels] = { 0 };
static int nInputs = 0, nOutputs = 0;

static char *
fix_blanks (char *src)
{
    char *s;
    for (s=src; *s!='\0'; s++) {
	if (*s == ' ')
	    *s = '_';
    }
    return src;
}

static void
CountPorts (int *ninputs, int *noutputs)
{
    const portStruct *cruiseif = &Cruise->ixs->port[IfIndex];
    const STRCT_IO *io = &cruiseif->io;
    int nin = 0, nout = 0, i;

    for (i=1; i<=io->inConnL[0]; i++) {
	int portno = io->inConnL[i];	/* 0 <= portno < n */
	const STRCT_IN *port = &io->inport[portno];
	if (port->connected)
	    nin = portno + 1;
    }
    *ninputs = nin;

    for (i=1; i<=io->outConnL[0]; i++) {
	int portno = io->outConnL[i];	/* 0 <= portno < n */
	const STRCT_OUT *port = &io->outport[portno];
	if (port->connected)
	    nout = portno + 1;
    }
    *noutputs = nout;
}


static void *
CruiseIf_New (struct tInfos *inf, struct tPowerTrainCfgIF *IF,
	      const char *kindkey, const char *ifname, int ifindex)
{
    IfIndex = ifindex;

    CountPorts(&nInputs, &nOutputs);

    /* Reset _all_ inputs and outputs. */
    memset(Inputs,  0, sizeof(Inputs));
    memset(Outputs, 0, sizeof(Outputs));

    return (void *)1;
}


static int
CruiseIf_CalcPre (void *MP, struct tPowerTrainIF *IF, double dt, double *channels)
{

    /* Copy CarMaker outputs to Cruise input channels. */
    memcpy(channels, Outputs, nOutputs * sizeof(double));

    return 0;
}


static int
CruiseIf_CalcPost (void *MP, struct tPowerTrainIF *IF, double dt, const double *channels)
{

    /* Copy Cruise output channels to CarMaker inputs. */
    memcpy(Inputs, channels, nInputs * sizeof(double));

    return 0;
}


static void
CruiseIf_Delete (void *MP)
{

}


static void
CruiseIf_DeclQuants (void *MP, const char *ifname, int ifindex)
{
    const STRCT_IO *io = &Cruise->ixs->port[ifindex].io;
    char name[300];
    int i;

    /* Quantities for additional inputs. */
    for (i=1; i<=io->inConnL[0]; i++) {
	int portno = io->inConnL[i];	/* 0 <= portno < n */
	const STRCT_IN *port = &io->inport[portno];
	if (port->connected && !InDeclared[portno]) {
	    sprintf(name, "AVL_Cruise.%s.SI.in%02d.%s", ifname+3, portno, port->name);
	    DDefDouble4(NULL, fix_blanks(name), "", &Inputs[portno], DVA_IO_Out);
	    InDeclared[portno] = 1;
	}
    }

    /* Quantities for additional outputs. */
    for (i=1; i<=io->outConnL[0]; i++) {
	int portno = io->outConnL[i];	/* 0 <= portno < n */
	const STRCT_OUT *port = &io->outport[portno];
	if (port->connected && !OutDeclared[portno]) {
	    sprintf(name, "AVL_Cruise.%s.SI.out%02d.%s", ifname+3, portno, port->name);
	    DDefDouble4(NULL, fix_blanks(name), "", &Outputs[portno], DVA_VC);
	    OutDeclared[portno] = 1;
	}
    }
}


/*
 * In order to make the Cruise interface known to CarMaker,
 * call this function from Vhcl_Register() in file src/CM_Vehicle.c,
 * _before_ PowerTrain_Register_AVL_Cruise() is executed.
 */
int
CruiseIf_MyCruiseIf_Register (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));

    m.PTCruiseIF.VersionId       = 1;
    m.PTCruiseIF.CompatVersionId = 1;

    m.PTCruiseIF.New        = CruiseIf_New;
    m.PTCruiseIF.CalcPre    = CruiseIf_CalcPre;
    m.PTCruiseIF.CalcPost   = CruiseIf_CalcPost;
    m.PTCruiseIF.Delete     = CruiseIf_Delete;
    m.PTCruiseIF.DeclQuants = CruiseIf_DeclQuants;

    return Model_RegisterIPG(ModelClass_PTCruiseIF, ModelName, &m);
}

