/***************************************************** target specific file ***/
/*  Wrapper module for Simulink models                                        */
/*  ------------------------------------------------------------------------  */
/*  (c) IPG Automotive GmbH    www.ipg-automotive.com   Fon: +49.721.98520-0  */
/*  Bannwaldallee 60      D-76185 Karlsruhe   Germany   Fax: +49.721.98520-99 */
/******************************************************************************/

#ifndef __BODYCTRL_RTW_WRAP_H__
#define __BODYCTRL_RTW_WRAP_H__

#ifndef IS_CAR
# define IS_CAR
#endif

#ifdef __cplusplus
extern "C" {
#endif


struct tInfos;
struct tMdlBdyFrame;
struct tMatSuppDictDef;
struct tMatSuppTunables;


#ifdef CLASSIC_INTERFACE
# define rtModel_BodyCtrl_RTW          RT_MODEL_BodyCtrl_RTW_T
#else
# define rtModel_BodyCtrl_RTW          tag_RTM_BodyCtrl_RTW_T
#endif //CLASSIC_INTERFACE

#define ExternalInputs_BodyCtrl_RTW   ExtU_BodyCtrl_RTW_T
#define ExternalOutputs_BodyCtrl_RTW  ExtY_BodyCtrl_RTW_T

#ifndef BodyCtrl_RTW_rtModel
typedef struct rtModel_BodyCtrl_RTW rtModel_BodyCtrl_RTW;
#endif

/* Model registration function */
rtModel_BodyCtrl_RTW *BodyCtrl_RTW (struct tInfos *Inf);

#if defined(CLASSIC_INTERFACE) && !defined(CM4SLDS)
void rt_ODECreateIntegrationData (RTWSolverInfo *si);
void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
void rt_ODEDestroyIntegrationData(RTWSolverInfo *si);
#endif


/* Dictionary variables and other items of the model */
extern struct tMatSuppDictDef *BodyCtrl_RTW_DictDefines;
extern struct tMdlBdyFrame *BodyCtrl_RTW_BdyFrameDefines;


/* Wrapper functions */
void BodyCtrl_RTW_SetParams (rtModel_BodyCtrl_RTW *rtm,
			struct tMatSuppTunables *tuns,
			struct tInfos *Inf);
int BodyCtrl_RTW_Register (void);


#ifdef __cplusplus
}
#endif

#endif /* __BODYCTRL_RTW_WRAP_H__ */

