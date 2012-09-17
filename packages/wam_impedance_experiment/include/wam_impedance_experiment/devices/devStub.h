/*
  Author(s): Simon Leonard
  Created on: Dec 02 2009

  (C) Copyright 2009 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _devStub_h
#define _devStub_h

#include <cisstVector/vctDynamicMatrix.h>

#include <cisstDevices/robotcomponents/manipulators/devManipulator.h>
#include <cisstDevices/devExport.h>

//! A class for a stub.
/**
  Provides fake encoder readings (all zero) when you don't want to connect to the actual
  WAM or to the simulation.
*/
class CISST_EXPORT devStub : public devManipulator {

 private:

  // WAM specific members
  //
  
  RnIO* input;
  RnIO* output;

  const vctDynamicVector<double> qinit;
  vctDynamicVector<double> q;

 public:
  void Read();
  void Write();


  //! Default constructor
  /**
     Initialize the stub.
     \param taskname    The task name
     \param period      The task period
     \param mask        Which CPU to use
     \param qinit       Initial position (default all zeroes)
  */
  devStub( const std::string& taskname, 
           double period, 
           osaCPUMask mask,
           const vctDynamicVector<double>& qinit=vctDynamicVector<double>( 7, 0.0 ) );

  ~devStub();

  static const std::string OutputInterface;
  static const std::string Output;

  static const std::string InputInterface;
  static const std::string Input;

};

#endif

