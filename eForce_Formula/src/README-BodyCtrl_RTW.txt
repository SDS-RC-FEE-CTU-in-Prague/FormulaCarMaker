
				  BodyCtrl_RTW
				  ============

In this example the Simulink model implements some kind of active body control,
that tries to keep the vehicle upright by applying extra suspension forces to
the chassis.


Relevant files and places to look at
------------------------------------
    .
    |-- Data
    |   |-- TestRun
    |   |   `-- Examples
    |   |       `-- BasicFunctions
    |   |           `-- Simulink
    |   |               `-- BodyCtrl_RTW
    |   `-- Vehicle
    |       `-- Examples
    |           `-- DemoCar_BodyCtrl_RTW
    `-- src
	|-- BodyCtrl_RTW.mdl
	|-- BodyCtrl_RTW_bus.m
	|-- BodyCtrl_RTW_CarMaker_rtw
	|   |-- BodyCtrl_RTW_wrap.c
	|   `-- BodyCtrl_RTW_wrap.h
	|-- BodyCtrl_RTW_params.m
	|-- Makefile
	|-- README-BodyCtrl_RTW.txt
	`-- User.c

o src/BodyCtrl_RTW.mdl
  src/BodyCtrl_RTW_params.mdl
  The Simulink model itself and its parameter file.

o src/BodyCtrl_RTW_CarMaker_rtw/BodyCtrl_RTW_wrap.h
  src/BodyCtrl_RTW_CarMaker_rtw/BodyCtrl_RTW_wrap.c
  The wrapper source code.

o src/User.c
  #include of the wrapper header file.
  Registration of the BodyCtrl_RTW model in function User_Register().
  The corresponding statements are added automatically by the RTW build process.

o src/Makefile
  libBodyCtrl_RTW_$(ARCH).a is added to the list of files that make up the
  CarMaker executable.
  The corresponding statements are added automatically by the RTW build process.

o Data/Vehicle/Examples/DemoCar_BodyCtrl_RTW
  Activation of BodyCtrl_RTW's functionality with an SuspExtFrcs.Kind entry
  (see entries at the end of the file).

o Data/TestRun/Examples/BasicFunctions/Simulink/BodyCtrl_RTW
  A long curve testrun which uses the DemoCar_BodyCtrl_RTW vehicle
  parameter file.

Almost all places where modifications were made to existing files can be
found by doing a textual search for "BodyCtrl_RTW".


Implementation details
----------------------

o The model is connected to CarMaker by "registering" it, i.e. adding it to
  the list of available external suspension forces modules. Registering is
  done in the model wrapper's central interface function BodyCtrl_RTW_Register(),
  which is called from User_Register() in src/User.c. Once it is registered,
  the BodyCtrl_RTW module can be activated with a SuspExtFrcs.Kind entry in
  the vehicle's configuration file. CarMaker then automatically calls the
  functions of our module using the function pointers in the
  tModelClassDescr struct.

o The instance pointer of the Simulink model is returned as the function
  value of the UserSteer_New() function. It is automatically passed via the
  MP argument, when any of the other functions are called by CarMaker,
  hence no local instance pointer variable is needed.

o In this example, to keep the model interface minimal, the parameters to the
  BodyCtrl_RTW_Calc() function are not completely mapped to Simulink Inports and
  Outports. First, none of the function's inputs is actually used, input is
  taken from a dictionary variable instead. Second, only 2 of the 12 return
  values, namely the spring forces, are really calculated.

o The parameter of the Gain block, BodyCtrl_RTW_k, has been made tunable by
  setting its storage class to SimulinkGlobal in the Model Parameter
  Configuration dialog. Real-Time Workshop then creates a member C variable
  for it. The parameter's initial value is read from the vehicle parameter
  file once a testrun begins; this in done in the BodyCtrl_RTW_SetParams()
  function in the wrapper.
