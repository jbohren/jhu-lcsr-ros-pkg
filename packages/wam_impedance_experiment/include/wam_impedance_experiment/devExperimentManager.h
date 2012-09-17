
/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: devExperimentManager.h 1708 2010-08-16 18:15:50Z adeguet1 $

  Author(s):	Gorkem Sevinc, Anton Deguet
  Created on:   2009-09-17

  (C) Copyright 2008 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _devExperimentManager_h
#define _devExperimentManager_h

#include <cisstCommon.h>
#include <cisstParameterTypes.h>
#include <cisstMultiTask/mtsForwardDeclarations.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <map>

// Always include last
#include <cisstDevices/devExport.h>

/*! Class used to create trigger events and commands using a keyboard.
  The user has to bind keys to a given required interface function or
  to a provided interface event.

  This class also maintains an internal flag for a special "Quit" key.
  The user can define the quit key using SetQuitKey and query if the
  key has been pressed using if (keyboard.Done()). */
class CISST_EXPORT devExperimentManager: public mtsTaskContinuous {
  // declare services, requires dynamic creation
  CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);
public:
  devExperimentManager(void);
  ~devExperimentManager(void) {};
  void Configure(const std::string & CMN_UNUSED(filename) = "") {};
  void Startup(void){};
  void Run(void);
  void Cleanup(void) {};

  // Declare experiment states
  enum ExperimentState {
    INIT,
    STANDBY,
    IMPEDANCE
  };

  ExperimentState experiment_state_;

  // Enable functions for controllers
  mtsFunctionWrite enable_gravity_compensation_l_,
                   enable_gravity_compensation_r_,
                   enable_variable_impedance_l_,
                   enable_variable_impedance_r_;
private:
  mtsBool motors_enabled_;

protected:
  CMN_DECLARE_MEMBER_AND_ACCESSORS(bool, Done);
};


CMN_DECLARE_SERVICES_INSTANTIATION(devExperimentManager);


#endif // _devExperimentManager_h
