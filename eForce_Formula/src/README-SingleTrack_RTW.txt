
				SingleTrack_RTW
				===============


This example is identical to the SingleTrack.mdl example model of the CarMaker
for Simulink Extras, with some additional wiring at the top level.


Relevant files and places to look at
------------------------------------
    .
    |-- Data
    |   |-- TestRun
    |   |   `-- Examples
    |   |       `-- BasicFunctions
    |   |           `-- Simulink
    |   |               `-- Hockenheim_SingleTrack_RTW
    |   `-- Vehicle
    |       `-- Examples
    |           `-- SingleTrack_RTW
    `-- src
	|-- Makefile
	|-- README-SingleTrack_RTW.txt
	|-- User.c
	|-- SingleTrack_RTW.mdl
	`-- SingleTrack_RTW_CarMaker_rtw
	    |-- SingleTrack_RTW_wrap.c
	    `-- SingleTrack_RTW_wrap.h

o src/SingleTrack_RTW.mdl
  The Simulink model itself.

o src/SingleTrack_RTW_CarMaker_rtw/SingleTrack_RTW_wrap.h
  src/SingleTrack_RTW_CarMaker_rtw/SingleTrack_RTW_wrap.c
  The wrapper source code.

o src/User.c
  #include of the wrapper header file.
  Registration of the SingleTrack_RTW model in function User_Register().
  The corresponding statements are added automatically by the RTW build process.

o src/Makefile
  libSingleTrack_RTW_$(ARCH).a is added to the list of files that make up the
  CarMaker executable.
  The corresponding statements are added automatically by the RTW build process.

o Data/Vehicle/Examples/SingleTrack_RTW
  The vehicle data set for the vehicle model. In the vehicle data editor see
  also the "Additional Parameters" on the "Misc." tab.

o Data/TestRun/Examples/BasicFunctions/Simulink/Hockenheim_SingleTrack_RTW
  A copy of the Hockenheim testrun which uses the SingleTrack_RTW
  vehicle data set.

Almost all places where modifications were made to existing files can be
found by doing a textual search for "SingleTrack_RTW".


Implementation details
----------------------

o The model is connected to CarMaker by "registering" it, i.e. adding it to
  the list of available vehicle models. Registering is done in the model
  wrapper's central interface function SingleTrack_RTW_Register(), which is
  called from User_Register() in src/User.c. Once it is registered, the
  SingleTrack_RTW module can be activated with the right FileIdent entry in
  the vehicle's configuration file. CarMaker then automatically calls the
  functions of our vehicle model using the function pointers in the
  tModelClassDescr struct.

o The instance pointer of the Simulink model is returned as the function
  value of the SingleTrack_RTW_New() function. It is automatically passed via
  the MP argument, when any of the other functions are called by CarMaker,
  hence no local instance pointer variable is needed.

o Since CarMaker communicates with our vehicle model mainly via the module's
  Calc() function and the parameters passed to it, the connection between the
  Simulink model and CarMaker is implemented accordingly. The toplevel diagram
  of the Simulink model contains only Inport and Outport blocks. In the
  generated C code their counterpart can be found in the model's ExternalInputs
  and ExternalOutputs structs. As a consequence, the SingleTrack_RTW_Calc()
  function in SingleTrack_RTW_wrap.c consists mainly of statements shuffling
  data between the function's IF interface parameter and the model.
