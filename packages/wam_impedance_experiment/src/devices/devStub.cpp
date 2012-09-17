/*

  Author(s): Simon Leonard
  Created on: Nov 11 2009

  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "wam_impedance_experiment/devices/devStub.h"
#include <cisstCommon/cmnLogger.h>
using namespace std;

const std::string devStub::OutputInterface = "WAMOutputInterface";
const std::string devStub::Output          = "WAMOutput";

const std::string devStub::InputInterface  = "WAMInputInterface";
const std::string devStub::Input           = "WAMInput";

// main constructor
devStub::devStub( const std::string& taskname, 
                  double period, 
                  osaCPUMask mask,
                  const vctDynamicVector<double>& qinit ) :
  devManipulator( taskname, 
                  period, 
                  devManipulator::ENABLED,
                  mask, 
                  devManipulator::FORCETORQUE ),
  input( NULL ),
  output( NULL ),
  qinit( qinit ),
  q( qinit ) {

  // only 7 links are allowed
  if( qinit.size() != 7 ){
  CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
          << ": Expected 7 links. Got " << qinit.size()
          << std::endl;
  exit(-1);
  }


  input = ProvideInputRn( devManipulator::Input,
                          devRobotComponent::POSITION |
                          devRobotComponent::FORCETORQUE,
                          qinit.size() );

  output = ProvideOutputRn( devManipulator::Output,
                            devRobotComponent::POSITION,
                            qinit.size() );

}

devStub::~devStub(){}

void devStub::Read(){
  // Always output zero position
  vctDynamicVector<double> q(qinit.size(),0.0);
  //q[2] = 1.0;
  output->SetPosition(q);
}

void devStub::Write(){
  // Do nothing here, because there's no arm to send torques to.
}
