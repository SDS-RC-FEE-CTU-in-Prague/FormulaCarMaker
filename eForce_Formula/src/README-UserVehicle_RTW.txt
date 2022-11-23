
				UserVehicle_RTW
				===============


In this example a simple vehicle model was implemented in Simulink.


Relevant files and places to look at
------------------------------------
    .
    |-- Data
    |   |-- TestRun
    |   |   `-- Examples
    |   |       `-- BasicFunctions
    |   |           `-- Simulink
    |   |               `-- Hockenheim_UserVehicle_RTW
    |   `-- Vehicle
    |       `-- Examples
    |           `-- UserVehicle_RTW
    `-- src
	|-- Makefile
	|-- README-UserVehicle_RTW.txt
	|-- tire_lin.c
	|-- User.c
	|-- UserVehicle_RTW.mdl
	`-- UserVehicle_RTW_CarMaker_rtw
	    |-- UserVehicle_RTW_wrap.c
	    `-- UserVehicle_RTW_wrap.h

o src/UserVehicle_RTW.mdl
  The Simulink model itself.

o src/tire_lin.c
  An S-function of a simple linear tire model used in UserVehicle_RTW.mdl,
  for compilation instructions see the comments at the top of the file.

o src/UserVehicle_RTW_CarMaker_rtw/UserVehicle_RTW_wrap.h
  src/UserVehicle_RTW_CarMaker_rtw/UserVehicle_RTW_wrap.c
  The wrapper source code.

o src/User.c
  #include of the wrapper header file.
  Registration of the UserVehicle_RTW model in function User_Register().
  The corresponding statements are added automatically by the RTW build process.

o src/Makefile
  libUserVehicle_RTW_$(ARCH).a is added to the list of files that make up the
  CarMaker executable.
  The corresponding statements are added automatically by the RTW build process.

o Data/Vehicle/Examples/UserVehicle_RTW
  The vehicle data set for the vehicle model. In the vehicle data editor see
  also the "Additional Parameters" on the "Misc." tab.

o Data/TestRun/Examples/BasicFunctions/Simulink/Hockenheim_UserVehicle_RTW
  A copy of the Hockenheim testrun which uses the UserVehicle_RTW
  vehicle data set.

Almost all places where modifications were made to existing files can be
found by doing a textual search for "UserVehicle_RTW".


Implementation details
----------------------

o The model is connected to CarMaker by "registering" it, i.e. adding it to
  the list of available vehicle models. Registering is done in the model
  wrapper's central interface function UserVehicle_RTW_Register(), which is
  called from User_Register() in src/User.c. Once it is registered, the
  UserVehicle_RTW module can be activated with the right FileIdent entry in
  the vehicle's configuration file. CarMaker then automatically calls the
  functions of our vehicle model using the function pointers in the
  tModelClassDescr struct.

o The instance pointer of the Simulink model is returned as the function
  value of the UserVehicle_RTW_New() function. It is automatically passed via
  the MP argument, when any of the other functions are called by CarMaker,
  hence no local instance pointer variable is needed.

o Since CarMaker communicates with our vehicle model mainly via the module's
  Calc() function and the parameters passed to it, the connection between the
  Simulink model and CarMaker is implemented accordingly. The toplevel diagram
  of the Simulink model contains only Inport and Outport blocks. In the
  generated C code their counterpart can be found in the model's ExternalInputs
  and ExternalOutputs structs. As a consequence, the UserVehicle_RTW_Calc()
  function in UserVehicle_RTW_wrap.c consists mainly of statements shuffling
  data between the function's IF interface parameter and the model.
