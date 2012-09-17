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

#include "wam_impedance_experiment/robotcomponents/devWAM_AAB.h"
#include <cisstCommon/cmnLogger.h>
using namespace std;

const std::string devWAM_AAB::OutputInterface = "WAMOutputInterface";
const std::string devWAM_AAB::Output          = "WAMOutput";

const std::string devWAM_AAB::InputInterface  = "WAMInputInterface";
const std::string devWAM_AAB::Input           = "WAMInput";

// main constructor
devWAM_AAB::devWAM_AAB( const std::string& taskname, 
		double period, 
		osaCPUMask mask,
		devCAN* candev, 
		const vctDynamicVector<double>& qinit ) :
  devManipulator( taskname, 
		  period, 
		  devManipulator::ENABLED, mask, 
		  devManipulator::FORCETORQUE ),
  input( NULL ),
  output( NULL ),
  qinit( qinit ),
  q( qinit ){


  // only 4 or 7 pucks are allowed
  if( qinit.size() != 4 && qinit.size() != 7 ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
			     << ": Expected 4 or 7 pucks. Got " << qinit.size()
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

  // Adjust the pucks vector to the number of requested joints
  pucks.resize( qinit.size() );

  // must have a CAN device
  if( candev == NULL ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
		       << ": CAN device missing."
		       << std::endl;
    exit(-1);
  }

  // copy the CAN device
  this->candev = candev;

}

devWAM_AAB::~devWAM_AAB(){}

void devWAM_AAB::Cleanup(){
  SetIdleMode();
  candev->Close(); 
}

void devWAM_AAB::Configure( const std::string& ){

  //
  // open the can device
  //

  CMN_LOG_INIT_VERBOSE << "Opening the CAN device." << std::endl;
  if( candev->Open() != devCAN::ESUCCESS ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
			     << ": Failed to open the CAN device. Exiting."
			     << std::endl;
    exit(-1);
  }
  CMN_LOG_INIT_VERBOSE << "The CAN device is opened." << std::endl;

  // This filter is to process position messages from pucks 1-4
  candev->AddFilter( devCAN::Filter( 0x051F, 0x0403 ) );
  // This filter is to process position messages from pucks 1-4
  //candev->AddFilter( devCAN::Filter( 0x07FF, 0x0443 ) );

  //
  // Set up the safety module
  //

  // Create the safety module
  safetymodule = devSafetyModule( devSafetyModule::SAFETY_MODULE_ID, candev );

  CMN_LOG_INIT_VERBOSE <<"Querying the status of the safety module."<<std::endl;
  devProperty::Value smstatus;
  if( safetymodule.GetProperty( devProperty::STATUS, smstatus ) 
      != devSafetyModule::ESUCCESS ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
		       << ": Failed to query the safety module. Exiting."
		       << std::endl;
    exit(-1);
  }

  // check the safety module is "ready"
  if( smstatus != devSafetyModule::STATUS_READY ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
		       << ": The safety module is offline. Exiting."
		       << std::endl;
    exit(-1);
  }  
  CMN_LOG_INIT_VERBOSE << "The safety module is online." << std::endl;
  
  // Set the velocity warning
  CMN_LOG_INIT_VERBOSE << "Setting velocity warning." << std::endl;
  if( SetVelocityWarning( 5000 ) != devWAM_AAB::ESUCCESS ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
		       << ": velocity warning not set. Exiting."
		       << std::endl;
    exit(-1);
  }
  CMN_LOG_INIT_VERBOSE << "Velocity warning set." << std::endl;

  // Set the velocity fault
  CMN_LOG_INIT_VERBOSE << "Setting velocity fault." << std::endl;
  if( SetVelocityFault( 12000 ) != devWAM_AAB::ESUCCESS ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
		       << ": velocity fault not set. Exiting."
		       << std::endl;
    exit(-1);
  }
  CMN_LOG_INIT_VERBOSE << "Velocity fault set." << std::endl;

  // Set the torque warning
  CMN_LOG_INIT_VERBOSE << "Setting torque warning." << std::endl;
  if( SetTorqueWarning( 4000 ) != devWAM_AAB::ESUCCESS ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
		       << ": torque warning not set. Exiting."
		       << std::endl;
    exit(-1);
  }
  CMN_LOG_INIT_VERBOSE << "Torque warning set." << std::endl;

  // Set the torque fault
  CMN_LOG_INIT_VERBOSE << "Setting torque fault." << std::endl;
  if( SetTorqueFault( 8000 ) != devWAM_AAB::ESUCCESS ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
		       << ": torque fault not set. Exiting."
		       << std::endl;
    exit(-1);
  }
  CMN_LOG_INIT_VERBOSE << "Torque fault set." << std::endl;
  CMN_LOG_INIT_VERBOSE << "The safety module is good to go" << std::endl;

  //
  // Now set up the groups
  //
  groups.clear();

  CMN_LOG_INIT_VERBOSE << "Creating groups." << std::endl;
  // create'n initialize the groups with their ID and the canbus
  // The group ID are available in devGroup.h.
  for( devGroup::ID gid=devGroup::BROADCAST; gid<devGroup::LASTGROUP; gid++ )
    { groups.push_back( devGroup( gid, candev ) ); }
  
  // Add the pucks ID to each group
  // The first loop handles the pucks in the upper arm
  // Pucks ID indices start at 1
  CMN_LOG_INIT_VERBOSE << "Configuring upper arm." << std::endl;
  for( devPuck::ID pid=devPuck::PUCK_ID1; pid<=devPuck::PUCK_ID4; pid++ ){
    groups[devGroup::BROADCAST].AddPuckIDToGroup( pid );
    groups[devGroup::UPPERARM].AddPuckIDToGroup( pid );
    groups[devGroup::UPPERARM_POSITION].AddPuckIDToGroup( pid );
  }
  
  // The second loop handles the pucks in the forearm (if any)
  if( pucks.size() == 7 ){
    CMN_LOG_INIT_VERBOSE << "Configuring forearm." << std::endl;
    for(devPuck::ID pid=devPuck::PUCK_ID5; pid<=devPuck::PUCK_ID7; pid++){
      groups[devGroup::BROADCAST].AddPuckIDToGroup( pid );
      groups[devGroup::FOREARM].AddPuckIDToGroup( pid );
      groups[devGroup::FOREARM_POSITION].AddPuckIDToGroup( pid );
    }
  }
  else
    { CMN_LOG_INIT_VERBOSE << "Skipping forearm." << std::endl; }
  
  groups[devGroup::HAND_POSITION].AddPuckIDToGroup( devPuck::PUCK_IDF1 );
  groups[devGroup::HAND_POSITION].AddPuckIDToGroup( devPuck::PUCK_IDF2 );
  groups[devGroup::HAND_POSITION].AddPuckIDToGroup( devPuck::PUCK_IDF3 );
  groups[devGroup::HAND_POSITION].AddPuckIDToGroup( devPuck::PUCK_IDF4 );
  
  //
  // Now set up the pucks
  //

  // create'n initialize the pucks with their ID and the CAN device. Note that
  // the pucks IDs are 1-index and the vector of pucks is 0-index.
  CMN_LOG_INIT_VERBOSE << "Creating pucks." << std::endl;

  devPuck::ID pid = devPuck::PUCK_ID1;
  for( size_t i=0; i<pucks.size(); i++ )
    { pucks[i] = devPuck( pid, candev ); pid++; }
  
  // wake up all the pucks. This broadcast the command that every puck should
  // change its status to "ready" 
  CMN_LOG_INIT_VERBOSE << "Waking up the pucks" << std::endl;
  if( SetPucksStatus( devPuck::STATUS_READY ) != devWAM_AAB::ESUCCESS ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
		       << ": Failed to wake up the pucks. Exiting." 
		       << std::endl;
    exit(-1);
  }

  // Wait a bit to let the pucks to boot
  usleep(300000);

  

  // count number of pucks that are "ready"
  CMN_LOG_INIT_VERBOSE << "Querying the status of the pucks" << std::endl;
  size_t npucksready = 0;
  for(size_t i=0; i<pucks.size(); i++){

    devProperty::Value pstatus;
    if(pucks[i].GetProperty(devProperty::STATUS, pstatus) != devPuck::ESUCCESS){
      CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
			 << ": Failed to query the status of puck "
			 << (int)pucks[i].GetID() << ". Exiting."
			 << std::endl;
      exit(-1);
    }

    if( pstatus == devPuck::STATUS_READY){
      CMN_LOG_INIT_VERBOSE << "Puck " << (int)pucks[i].GetID() << " is online." 
			   << std::endl;
      npucksready++;
    }
    else{
      CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
			 << ": Puck " << (int)pucks[i].GetID() << " is offline."
			 << std::endl;
    }
  }

  
  // make sure that all the pucks are ready
  if( npucksready != pucks.size() ){
    CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
		       << ": Found " << npucksready << ". "
		       << "Expected " << pucks.size() << ". Exiting." 
		       << std::endl;
    exit(-1);
  }  
  CMN_LOG_INIT_VERBOSE << "All the pucks on online." << std::endl;
  
  // configure the pucks
  for(size_t i=0; i<pucks.size(); i++){
    if( pucks[i].Configure() != devPuck::ESUCCESS ){
      CMN_LOG_INIT_ERROR << CMN_LOG_DETAILS
			 << ": Failed to configure puck "<<(int)pucks[i].GetID()
			 << std::endl;
      exit(-1);
    }
  }

  //
  // Set up the convertion matrices
  //

  // This sets the matrices for converting motor spaces to joint spaces
  if( pucks.size() == 7 ){
    mpos2jpos.SetSize( 7, 7, VCT_ROW_MAJOR );
    jpos2mpos.SetSize( 7, 7, VCT_ROW_MAJOR );
    jtrq2mtrq.SetSize( 7, 7, VCT_ROW_MAJOR );
  }
  else if( pucks.size() == 4 ){
    mpos2jpos.SetSize( 4, 4, VCT_ROW_MAJOR );
    jpos2mpos.SetSize( 4, 4, VCT_ROW_MAJOR );
    jtrq2mtrq.SetSize( 4, 4, VCT_ROW_MAJOR );
  }

  mpos2jpos.SetAll( 0.0 );
  jpos2mpos.SetAll( 0.0 );
  jtrq2mtrq.SetAll( 0.0 );

  mpos2jpos[0][0] = -0.0238095;
  mpos2jpos[1][1] =  0.0176991;   mpos2jpos[1][2] = -0.0176991;
  mpos2jpos[2][1] = -0.0297345;   mpos2jpos[2][2] = -0.0297345;
  mpos2jpos[3][3] = -0.0555556;

  jpos2mpos[0][0] = -42.0;
  jpos2mpos[1][1] =  28.25;       jpos2mpos[1][2] = -16.8155;
  jpos2mpos[2][1] = -28.25;       jpos2mpos[2][2] = -16.8155;
  jpos2mpos[3][3] = -18.0;

  jtrq2mtrq[0][0] = -0.0238095;
  jtrq2mtrq[1][1] =  0.0176991;   jtrq2mtrq[1][2] = -0.0297345;
  jtrq2mtrq[2][1] = -0.0176991;   jtrq2mtrq[2][2] = -0.0297345;
  jtrq2mtrq[3][3] = -0.0555556;

  if( pucks.size() == 7 ){

    mpos2jpos[4][4] =  0.0527426; mpos2jpos[4][5] = 0.0527426;
    mpos2jpos[5][4] = -0.0527426; mpos2jpos[5][5] = 0.0527426;
    mpos2jpos[6][6] = -0.0669792;

    jpos2mpos[4][4] =   9.48;     jpos2mpos[4][5] = -9.48;
    jpos2mpos[5][4] =   9.48;     jpos2mpos[5][5] =  9.48;
    jpos2mpos[6][6] = -14.93;

    jtrq2mtrq[4][4] =  0.0527426; jtrq2mtrq[4][5] = -0.0527426;
    jtrq2mtrq[5][4] =  0.0527426; jtrq2mtrq[5][5] =  0.0527426;
    jtrq2mtrq[6][6] = -0.0669792;

  }

  //
  // Initialize the position of the WAM
  //
  SendPositions( qinit );
  input->SetPosition( qinit );

}


void devWAM_AAB::SetForceTorqueMode(){
  //devManipulator::SetForceTorqueMode();
  if( SetPucksMode( devPuck::MODE_TORQUE ) != devWAM_AAB::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << " Failed to set torque mode."
		      << std::endl;
  }
}

void devWAM_AAB::SetPositionMode(){
  //devManipulator::SetPositionMode();
  if( SetPucksMode( devPuck::MODE_POSITION ) != devWAM_AAB::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << " Failed to set position mode."
		      << std::endl;
  }
}


void devWAM_AAB::SetIdleMode(){
  //devManipulator::SetIdleMode();
  if( SetPucksMode( devPuck::MODE_IDLE ) != devWAM_AAB::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << " Failed to set idle mode." 
		      << std::endl;
  }
}


devWAM_AAB::Errno devWAM_AAB::SetVelocityWarning( devProperty::Value vw ){
  if( safetymodule.SetProperty( devProperty::VELWARNING, vw, true ) 
      != devSafetyModule::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Unable to set the velocity warning." 
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }
  return devWAM_AAB::ESUCCESS;
}

devWAM_AAB::Errno devWAM_AAB::SetVelocityFault( devProperty::Value vf ){
  if( safetymodule.SetProperty( devProperty::VELFAULT, vf, true ) 
      != devSafetyModule::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Unable to set the velocity fault." 
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }
  return devWAM_AAB::ESUCCESS;
}

devWAM_AAB::Errno devWAM_AAB::SetTorqueWarning( devProperty::Value tw ){
  if( safetymodule.SetProperty( devProperty::TRQWARNING, tw, true ) 
      != devSafetyModule::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Unable to set the torques warning." 
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }
  return devWAM_AAB::ESUCCESS;
}

devWAM_AAB::Errno devWAM_AAB::SetTorqueFault( devProperty::Value tf ){
  // Do not query the SM since it will answer with a reply to group 3!
  if( safetymodule.SetProperty( devProperty::TRQFAULT, tf, false ) 
      != devSafetyModule::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Unable to set the torques fault." 
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }
  return devWAM_AAB::ESUCCESS;
}

devWAM_AAB::Errno devWAM_AAB::SetPucksMode( devProperty::Value mode ){
  if( groups[devGroup::BROADCAST].SetProperty( devProperty::MODE, mode, false ) 
      != devGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Failed to wake up the pucks." 
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }
  return devWAM_AAB::ESUCCESS;
}

devWAM_AAB::Errno devWAM_AAB::SetPucksStatus( devProperty::Value ps ){
  if( groups[devGroup::BROADCAST].SetProperty( devProperty::STATUS, ps, false ) 
      != devGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Failed to wake up the pucks." 
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }
  return devWAM_AAB::ESUCCESS;
}

// set the motor positions 
devWAM_AAB::Errno devWAM_AAB::SendPositions( const vctDynamicVector<double>& jq ){

  if( jq.size() != pucks.size() ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Expected " << pucks.size() << " joint angles. "
		      << "Got " << jq.size()
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }

  // let the safety module ignore a few faults
  // this is necessary because otherwise the safety module will monitor a large
  // change of joint position in a short amount of time and trigger a velocity 
  // fault.
  
  if( safetymodule.SetProperty( devProperty::IGNOREFAULT, 8, true ) 
      != devSafetyModule::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Failed to configure the safety module" 
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }
  

  // convert the joints positions to motor positions
  vctDynamicVector<double> mq = JointsPos2MotorsPos( jq );
  
  // for each puck, send a position 
  for(size_t i=0; i<pucks.size(); i++){
    devProperty::Value position;  // this is the position in encoder ticks
    
    // convert the motor positions to encoder ticks
    position = (devProperty::Value)floor((mq[i]*pucks[i].CountsPerRevolution())/
					 (2.0*M_PI) );
    
    // Set the motor position. Don't double check the position because the 
    // noise might change the encoder.
    if( pucks[i].SetProperty( devProperty::POS, position, false ) 
	!= devPuck::ESUCCESS ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< ": Failed to set pos of puck#: " 
			<< (int)pucks[i].GetID()
			<< std::endl;
    }
    //usleep(1000); // don't know if this is necessary anymore
  }

  // reset the safety module
  
  if( safetymodule.SetProperty( devProperty::IGNOREFAULT, 1, true ) 
      != devSafetyModule::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Failed to configure the safety module>" 
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }
  

  return devWAM_AAB::ESUCCESS;
}

void devWAM_AAB::Read(){
  vctDynamicVector<double> q;
  QueryPositions();
  RecvPositions( q );
  output->SetPosition(q);
}

devWAM_AAB::Errno devWAM_AAB::QueryPositions(){

  // Query all the motor (broadcast group)
  if( groups[ devGroup::UPPERARM_POSITION ].GetProperty( devProperty::POS ) 
      != devGroup::ESUCCESS ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Failed to query the pucks."
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }
  

  if( pucks.size() == 7 ){
    if( groups[ devGroup::FOREARM_POSITION ].GetProperty( devProperty::POS ) 
	!= devGroup::ESUCCESS ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< ": Failed to query the pucks."
			<< std::endl;
      return devWAM_AAB::EFAILURE;
    }
  }

  return devWAM_AAB::ESUCCESS;
}

// query the joint positions
devWAM_AAB::Errno devWAM_AAB::RecvPositions( vctDynamicVector<double>& jq ){

  // this will hold the motor positions
  vctDynamicVector<double> mq( pucks.size(), 0.0 );

  // wait for the pucks to respond
  for(size_t i=0; i<pucks.size(); i++){
    // receive a response from a puck
    devCAN::Frame canframe;
    if( candev->Recv( canframe ) != devCAN::ESUCCESS ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< ": Failed to receive a CAN frame."
			<< std::endl;
    }

    else{

      // determine where the canframe originated (-1 is for zero indexing)
      devPuck::ID pid = devPuck::OriginID( canframe );

      devProperty::ID propid;
      devProperty::Value position;

      // Unpack the can frame
      if( pucks[pid-1].UnpackCANFrame( canframe, propid, position ) 
	  != devPuck::ESUCCESS ){
	CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			  << ": Failed to unpack a CAN frame."
			  << std::endl;
      }
      
      // Ensure that the can frame provides a motor position
      if( propid != devProperty::POS){
	CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			  << ": Oops! Didn't receive a position."
			  << std::endl;
      }

      // convert the position from encoder ticks to radians
      mq[pid-1] = ( ((double)position) * 2.0 * M_PI  /
		    ((double)pucks[pid-1].CountsPerRevolution() ) );
    }
  }
  // convert the motor positions to joints positions
  jq = MotorsPos2JointsPos( mq );

  return devWAM_AAB::ESUCCESS;
}

void devWAM_AAB::Write(){

  switch( GetInputMode() ){
   
  case devManipulator::FORCETORQUE:
    {
      double t;
      vctDynamicVector<double> jt;
      input->GetForceTorque( jt, t );
      SendTorques( jt );
    }
    break;
    
  case devManipulator::POSITION:
    {
      double t;
      vctDynamicVector<double> q;
      input->GetPosition( q, t );
      SendPositions( q );
    }
    break;

  case devManipulator::VELOCITY:
  case devManipulator::IDLE:
    {
      vctDynamicVector<double> jt( pucks.size(), 0.0 );
      SendTorques( jt );
    }
    break;
  }

}

// Send joint torques
devWAM_AAB::Errno devWAM_AAB::SendTorques( const vctDynamicVector<double>& jt ){

  if( jt.size() != pucks.size() ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Expected " << pucks.size() << " joint torques. "
		      << "Got " << jt.size()
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }

  // convert the joint torques to motor torques
  vctDynamicVector<double> mt = JointsTrq2MotorsTrq( jt );

  // Is the upperarm group empty?
  if( !groups[devGroup::UPPERARM].Empty() ){
    // TODO: this block should be in devGroup

    // copy the first 4 torques in a 4 array
    double currents[4] = {0.0, 0.0, 0.0, 0.0};
    for( size_t i=0; i<4; i++)
      { currents[i] = mt[i] * pucks[i].IpNm(); }
    
    // pack the torques in a can frames
    devCAN::Frame canframe;
    if( PackCurrents( canframe, devGroup::UPPERARM, currents ) 
	!= devWAM_AAB::ESUCCESS ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< ": Failed to pack the torques."
			<< std::endl;
      return devWAM_AAB::EFAILURE;
    }

    // send the canframe (blocking)
    if( candev->Send( canframe ) != devCAN::ESUCCESS ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< ": Failed to send upper arm torques." 
			<< std::endl;
      return devWAM_AAB::EFAILURE;
    }

  }
  else{
    CMN_LOG_RUN_WARNING << CMN_LOG_DETAILS
			<< ": The upper arm group is empty!!!"
			<< std::endl;
    return devWAM_AAB::EFAILURE;
  }

  // Is the forearm group empty?
  if( !groups[devGroup::FOREARM].Empty() ){

    // if using the forearm, make sure that 7 torques are available
    if( mt.size() != 7 ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< ": Expected 7 torques. Got " << mt.size()
			<< std::endl;
      return devWAM_AAB::EFAILURE;
    }
    
    // TODO: this block should be in devGroup

    // copy the last 3 torques in a 4 array
    double currents[4] = {0.0, 0.0, 0.0, 0.0};
    for( size_t i=0; i<3; i++)
      { currents[i] = mt[i+4] * pucks[i+4].IpNm(); }

    // pack the torques in a CAN frame
    devCAN::Frame canframe;
    if( PackCurrents( canframe, devGroup::FOREARM, currents ) 
	!= devWAM_AAB::ESUCCESS ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< ": Failed to pack the torques."
			<< std::endl;
      return devWAM_AAB::EFAILURE;
    }

    // Send the CAN frame
    if( candev->Send( canframe ) != devCAN::ESUCCESS ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< ": Failed to send forearm torques." 
			<< std::endl;
      return devWAM_AAB::EFAILURE;
    }
  }

  return devWAM_AAB::ESUCCESS;
}

// pack motor torques in a CAN frame
// this should go into devGroup
devWAM_AAB::Errno devWAM_AAB::PackCurrents( devCAN::Frame& canframe, 
				    devGroup::ID gid, 
				    const double I[4]) {
  
  // we can only pack torques for the upper arm and forearm groups
  if(gid == devGroup::UPPERARM || gid == devGroup::FOREARM){

    // copy each motor torques in this array with the correct index
    devProperty::Value currents[4] = {0, 0, 0, 0};

    // for each puck in the group
    for(devPuck::ID pid=groups[gid].FirstMember();
	            pid<=groups[gid].LastMember();
	            pid++){
      
      // get the index of the puck within its group [0,1,2,3]
      int idx =  pucks[pid-1].GroupIndex()-1;    // -1 because of zero index
      if( idx < 0 || 3 < idx ){                  // sanity check
	CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			  << ": Illegal index." 
			  << std::endl;
	return devWAM_AAB::EFAILURE;
      }
      currents[ idx ] = (devProperty::Value)I[idx];   // cast the torque      
    }

    // pack the torques in a 8 bytes message (see the documentation)
    unsigned char msg[8];
    msg[0]= devProperty::TRQ | 0x80;
    msg[1]=(unsigned char)(( currents[0]>>6)&0x00FF);
    msg[2]=(unsigned char)(((currents[0]<<2)&0x00FC)|((currents[1]>>12)&0x0003));
    msg[3]=(unsigned char)(( currents[1]>>4)&0x00FF);
    msg[4]=(unsigned char)(((currents[1]<<4)&0x00F0)|((currents[2]>>10)&0x000F));
    msg[5]=(unsigned char)(( currents[2]>>2)&0x00FF);
    msg[6]=(unsigned char)(((currents[2]<<6)&0x00C0)|((currents[3]>>8) &0x003F));
    msg[7]=(unsigned char)(  currents[3]    &0x00FF);
    
    // build a can frame addressed to the group ID 
    canframe = devCAN::Frame( devGroup::CANID( gid ), msg, 8 );
    return devWAM_AAB::ESUCCESS;
  }
  else{
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << ": Invalid group ID." 
		      << std::endl;
    return devWAM_AAB::EFAILURE;
  }
}

vctDynamicVector<double> 
devWAM_AAB::MotorsPos2JointsPos( const vctDynamicVector<double>& mq )
{  return mpos2jpos*mq;  }

vctDynamicVector<double> 
devWAM_AAB::JointsPos2MotorsPos( const vctDynamicVector<double>& jq )
{  return jpos2mpos*jq;  }

vctDynamicVector<double> 
devWAM_AAB::JointsTrq2MotorsTrq( const vctDynamicVector<double>& jt )
{  return jtrq2mtrq*jt;  }

