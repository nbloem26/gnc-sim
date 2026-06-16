#ifndef fmi2Functions_h
#define fmi2Functions_h

/* This header file declares the C API of FMI 2.0 for Co-Simulation and
   Model Exchange.

   Vendored verbatim from the FMI 2.0.4 standard (Modelica Association /
   Functional Mock-up Interface). Header-only; no external dependency.

   With the prefix mechanism below, an FMU's exported symbols are name-mangled
   with the model identifier so several FMUs can be linked statically into one
   executable without clashing. The in-repo master defines FMI2_FUNCTION_PREFIX
   to match modelDescription.xml's modelIdentifier; the shared library does NOT
   (so the .fmu exports the canonical fmi2* names a generic master expects).

   Copyright (C) 2008-2011 MODELISAR consortium,
                 2012-2022 Modelica Association Project "FMI"
                 All rights reserved.

   This file is licensed by the copyright holders under the 2-Clause BSD
   License (https://opensource.org/licenses/BSD-2-Clause). See
   fmi2TypesPlatform.h for the full license text.
*/

#include <stdlib.h>
#include "fmi2FunctionTypes.h"
#include "fmi2TypesPlatform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Export FMI2 API functions on Windows and under GCC.
   If custom linking is desired then the @p FMI2_Export flag may be defined.
*/
#if !defined(FMI2_Export)
#if !defined(FMI2_FUNCTION_PREFIX)
#if defined _WIN32 || defined __CYGWIN__
/* Note: cygwin is also defining __WIN32__, so we cannot use the second
   block here. */
#define FMI2_Export __declspec(dllexport)
#else
#if __GNUC__ >= 4
#define FMI2_Export __attribute__((visibility("default")))
#else
#define FMI2_Export
#endif
#endif
#else
#define FMI2_Export
#endif
#endif

/* Macros to construct the real function name (prepend FMI2_FUNCTION_PREFIX). */
#if defined(FMI2_FUNCTION_PREFIX)
#define fmi2Paste(a, b) a##b
#define fmi2PasteB(a, b) fmi2Paste(a, b)
#define fmi2FullName(name) fmi2PasteB(FMI2_FUNCTION_PREFIX, name)
#else
#define fmi2FullName(name) name
#endif

/***************************************************
Common Functions
****************************************************/
#define fmi2GetTypesPlatform fmi2FullName(fmi2GetTypesPlatform)
#define fmi2GetVersion fmi2FullName(fmi2GetVersion)
#define fmi2SetDebugLogging fmi2FullName(fmi2SetDebugLogging)
#define fmi2Instantiate fmi2FullName(fmi2Instantiate)
#define fmi2FreeInstance fmi2FullName(fmi2FreeInstance)
#define fmi2SetupExperiment fmi2FullName(fmi2SetupExperiment)
#define fmi2EnterInitializationMode fmi2FullName(fmi2EnterInitializationMode)
#define fmi2ExitInitializationMode fmi2FullName(fmi2ExitInitializationMode)
#define fmi2Terminate fmi2FullName(fmi2Terminate)
#define fmi2Reset fmi2FullName(fmi2Reset)
#define fmi2GetReal fmi2FullName(fmi2GetReal)
#define fmi2GetInteger fmi2FullName(fmi2GetInteger)
#define fmi2GetBoolean fmi2FullName(fmi2GetBoolean)
#define fmi2GetString fmi2FullName(fmi2GetString)
#define fmi2SetReal fmi2FullName(fmi2SetReal)
#define fmi2SetInteger fmi2FullName(fmi2SetInteger)
#define fmi2SetBoolean fmi2FullName(fmi2SetBoolean)
#define fmi2SetString fmi2FullName(fmi2SetString)
#define fmi2GetFMUstate fmi2FullName(fmi2GetFMUstate)
#define fmi2SetFMUstate fmi2FullName(fmi2SetFMUstate)
#define fmi2FreeFMUstate fmi2FullName(fmi2FreeFMUstate)
#define fmi2SerializedFMUstateSize fmi2FullName(fmi2SerializedFMUstateSize)
#define fmi2SerializeFMUstate fmi2FullName(fmi2SerializeFMUstate)
#define fmi2DeSerializeFMUstate fmi2FullName(fmi2DeSerializeFMUstate)
#define fmi2GetDirectionalDerivative fmi2FullName(fmi2GetDirectionalDerivative)

/***************************************************
Functions for FMI2 for Model Exchange
****************************************************/
#define fmi2EnterEventMode fmi2FullName(fmi2EnterEventMode)
#define fmi2NewDiscreteStates fmi2FullName(fmi2NewDiscreteStates)
#define fmi2EnterContinuousTimeMode fmi2FullName(fmi2EnterContinuousTimeMode)
#define fmi2CompletedIntegratorStep fmi2FullName(fmi2CompletedIntegratorStep)
#define fmi2SetTime fmi2FullName(fmi2SetTime)
#define fmi2SetContinuousStates fmi2FullName(fmi2SetContinuousStates)
#define fmi2GetDerivatives fmi2FullName(fmi2GetDerivatives)
#define fmi2GetEventIndicators fmi2FullName(fmi2GetEventIndicators)
#define fmi2GetContinuousStates fmi2FullName(fmi2GetContinuousStates)
#define fmi2GetNominalsOfContinuousStates \
  fmi2FullName(fmi2GetNominalsOfContinuousStates)

/***************************************************
Functions for FMI2 for Co-Simulation
****************************************************/
#define fmi2SetRealInputDerivatives fmi2FullName(fmi2SetRealInputDerivatives)
#define fmi2GetRealOutputDerivatives fmi2FullName(fmi2GetRealOutputDerivatives)
#define fmi2DoStep fmi2FullName(fmi2DoStep)
#define fmi2CancelStep fmi2FullName(fmi2CancelStep)
#define fmi2GetStatus fmi2FullName(fmi2GetStatus)
#define fmi2GetRealStatus fmi2FullName(fmi2GetRealStatus)
#define fmi2GetIntegerStatus fmi2FullName(fmi2GetIntegerStatus)
#define fmi2GetBooleanStatus fmi2FullName(fmi2GetBooleanStatus)
#define fmi2GetStringStatus fmi2FullName(fmi2GetStringStatus)

/***************************************************
Common Functions
****************************************************/

/* Inquire version numbers of header files */
FMI2_Export fmi2GetTypesPlatformTYPE fmi2GetTypesPlatform;
FMI2_Export fmi2GetVersionTYPE fmi2GetVersion;
FMI2_Export fmi2SetDebugLoggingTYPE fmi2SetDebugLogging;

/* Creation and destruction of FMU instances */
FMI2_Export fmi2InstantiateTYPE fmi2Instantiate;
FMI2_Export fmi2FreeInstanceTYPE fmi2FreeInstance;

/* Enter and exit initialization mode, terminate and reset */
FMI2_Export fmi2SetupExperimentTYPE fmi2SetupExperiment;
FMI2_Export fmi2EnterInitializationModeTYPE fmi2EnterInitializationMode;
FMI2_Export fmi2ExitInitializationModeTYPE fmi2ExitInitializationMode;
FMI2_Export fmi2TerminateTYPE fmi2Terminate;
FMI2_Export fmi2ResetTYPE fmi2Reset;

/* Getting and setting variables values */
FMI2_Export fmi2GetRealTYPE fmi2GetReal;
FMI2_Export fmi2GetIntegerTYPE fmi2GetInteger;
FMI2_Export fmi2GetBooleanTYPE fmi2GetBoolean;
FMI2_Export fmi2GetStringTYPE fmi2GetString;

FMI2_Export fmi2SetRealTYPE fmi2SetReal;
FMI2_Export fmi2SetIntegerTYPE fmi2SetInteger;
FMI2_Export fmi2SetBooleanTYPE fmi2SetBoolean;
FMI2_Export fmi2SetStringTYPE fmi2SetString;

/* Getting and setting the internal FMU state */
FMI2_Export fmi2GetFMUstateTYPE fmi2GetFMUstate;
FMI2_Export fmi2SetFMUstateTYPE fmi2SetFMUstate;
FMI2_Export fmi2FreeFMUstateTYPE fmi2FreeFMUstate;
FMI2_Export fmi2SerializedFMUstateSizeTYPE fmi2SerializedFMUstateSize;
FMI2_Export fmi2SerializeFMUstateTYPE fmi2SerializeFMUstate;
FMI2_Export fmi2DeSerializeFMUstateTYPE fmi2DeSerializeFMUstate;

/* Getting partial derivatives */
FMI2_Export fmi2GetDirectionalDerivativeTYPE fmi2GetDirectionalDerivative;

/***************************************************
Functions for FMI2 for Model Exchange
****************************************************/

/* Enter and exit the different modes */
FMI2_Export fmi2EnterEventModeTYPE fmi2EnterEventMode;
FMI2_Export fmi2NewDiscreteStatesTYPE fmi2NewDiscreteStates;
FMI2_Export fmi2EnterContinuousTimeModeTYPE fmi2EnterContinuousTimeMode;
FMI2_Export fmi2CompletedIntegratorStepTYPE fmi2CompletedIntegratorStep;

/* Providing independent variables and re-initialization of caching */
FMI2_Export fmi2SetTimeTYPE fmi2SetTime;
FMI2_Export fmi2SetContinuousStatesTYPE fmi2SetContinuousStates;

/* Evaluation of the model equations */
FMI2_Export fmi2GetDerivativesTYPE fmi2GetDerivatives;
FMI2_Export fmi2GetEventIndicatorsTYPE fmi2GetEventIndicators;
FMI2_Export fmi2GetContinuousStatesTYPE fmi2GetContinuousStates;
FMI2_Export fmi2GetNominalsOfContinuousStatesTYPE
    fmi2GetNominalsOfContinuousStates;

/***************************************************
Functions for FMI2 for Co-Simulation
****************************************************/

/* Simulating the slave */
FMI2_Export fmi2SetRealInputDerivativesTYPE fmi2SetRealInputDerivatives;
FMI2_Export fmi2GetRealOutputDerivativesTYPE fmi2GetRealOutputDerivatives;

FMI2_Export fmi2DoStepTYPE fmi2DoStep;
FMI2_Export fmi2CancelStepTYPE fmi2CancelStep;

/* Inquire slave status */
FMI2_Export fmi2GetStatusTYPE fmi2GetStatus;
FMI2_Export fmi2GetRealStatusTYPE fmi2GetRealStatus;
FMI2_Export fmi2GetIntegerStatusTYPE fmi2GetIntegerStatus;
FMI2_Export fmi2GetBooleanStatusTYPE fmi2GetBooleanStatus;
FMI2_Export fmi2GetStringStatusTYPE fmi2GetStringStatus;

#ifdef __cplusplus
} /* end of extern "C" { */
#endif

#endif /* fmi2Functions_h */
