
				 UserSteer_RTW
				 =============


In this example a simple steering module was implemented in Simulink.


Relevant files and places to look at
------------------------------------
    .
    |-- Data
    |   |-- TestRun
    |   |   `-- Examples
    |   |       `-- BasicFunctions
    |   |           `-- Simulink
    |   |               `-- Hockenheim_UserSteer_RTW
    |   `-- Vehicle
    |       `-- Examples
    |           `-- DemoCar_UserSteer_RTW
    `-- src
	|-- Makefile
	|-- README-UserSteer_RTW.txt
	|-- User.c
	|-- UserSteer_RTW.mdl
	|-- UserSteer_RTW_bus.m
	`-- UserSteer_RTW_CarMaker_rtw
	    |-- UserSteer_RTW_wrap.c
	    `-- UserSteer_RTW_wrap.h

o src/UserSteer_RTW.mdl
  The Simulink model itself.

o src/UserSteer_RTW_CarMaker_rtw/UserSteer_RTW_wrap.h
  src/UserSteer_RTW_CarMaker_rtw/UserSteer_RTW_wrap.c
  The wrapper source code.

o src/User.c
  #include of the wrapper header file.
  Registration of the UserSteer_RTW model in function User_Register().
  The corresponding statements are added automatically by the RTW build process.

o src/Makefile
  libUserSteer_RTW_$(ARCH).a is added to the list of files that make up the
  CarMaker executable.
  The corresponding statements are added automatically by the RTW build process.

o Data/Vehicle/Examples/DemoCar_UserSteer_RTW
  Activation of the UserSteer steering module with a Steer.Kind entry
  (see entries at the end of the file).

o Data/TestRun/Examples/BasicFunctions/Simulink/Hockenheim_UserSteer_RTW
  A copy of the Hockenheim testrun which uses the DemoCar_UserSteer_RTW
  vehicle configuration file.

Almost all places where modifications were made to existing files can be
found by doing a textual search for "UserSteer_RTW".


Implementation details
----------------------

o The model is connected to CarMaker by "registering" it, i.e. adding it to
  the list of available steering modules. Registering is done in the model
  wrapper's central interface function UserSteer_RTW_Register(), which is called
  from User_Register() in src/User.c. Once it is registered, the UserSteer_RTW
  module can be activated with a Steering.Kind entry in the vehicle's
  configuration file. CarMaker then automatically calls the functions of our
  steering module using the function pointers in the tModelClassDescr struct.

o The instance pointer of the Simulink model is returned as the function
  value of the UserSteer_RTW_New() function. It is automatically passed via the
  MP argument, when any of the other functions are called by CarMaker,
  hence no local instance pointer variable is needed.

o Since CarMaker communicates with our steering module mainly via the module's
  Calc() function and the parameters passed to it, the connection between the
  Simulink model and CarMaker is implemented accordingly. The toplevel diagram
  of the Simulink model contains only Inport and Outport blocks. In the
  generated C code their counterpart can be found in the model's ExternalInputs
  and ExternalOutputs structs. As a consequence, the UserSteer_RTW_Calc()
  function in UserSteer_RTW_wrap.c consists mainly of statements shuffling data
  between the function's IF interface parameter and the model.
