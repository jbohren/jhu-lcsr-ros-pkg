#include "wam_impedance_experiment/devices/devDAS6014.h"
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstOSAbstraction/osaSleep.h>

const std::string devDAS6014::Output   = "Output";
const std::string devDAS6014::ReadVolt = "ReadVolt";
const std::string devDAS6014::GetZero  = "GetZero";


devDAS6014::devDAS6014( const std::string& devicename,
			unsigned int subdevice,
			const vctDynamicVector<unsigned int>& channels ) :
  mtsComponent( devicename ),
  devicename( devicename ),
  subdevice( subdevice ),
  channels( channels ){

  // Open the device
  device = comedi_open( devicename.data() );
  if( device == NULL ){
    CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
		      << "Failed to open device: " << devicename
		      << std::endl;
  }

  zeroforces.SetSize( channels.size() );

  for( size_t i=0; i<channels.size(); i++ ){
    
    CMN_LOG_INIT_VERBOSE << "Initializing channel: " << channels[i] <<std::endl;

    maxdata.push_back( comedi_get_maxdata( device, subdevice, channels[i] ) );
    if( maxdata[i] == 0 ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< "Failed to get maximum data: " << devicename 
			<< std::endl;
    }
    
    ranges.push_back( comedi_get_range( device, subdevice, channels[i], 0 ) );
    if( ranges[i] == NULL ){
      CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
			<< "Failed to get range: " << devicename
			<< std::endl;
    }

    Calibrate( i );

  }    

  mtsInterfaceProvided* output = AddInterfaceProvided( devDAS6014::Output );
  if( output != NULL ){
    output->AddCommandQualifiedRead( &devDAS6014::Read, this, ReadVolt );
    output->AddCommandQualifiedRead( &devDAS6014::Zero, this, GetZero );
  }    

}

void devDAS6014::Read( const mtsInt& i, mtsDouble& V ) const { 

  lsampl_t raw;
  int errval;

  errval = comedi_data_read_delayed( device, 
				     subdevice, 
				     channels[ (int)i ], 
				     0, 
				     AREF_DIFF, 
				     &raw, 
				     3000 );

  V = comedi_to_phys( raw, ranges[ (int)i ], maxdata[ (int)i ] );

}

void devDAS6014::Zero( const mtsInt& i, mtsDouble& z ) const 
{ z = zeroforces[i]; }

void devDAS6014::Calibrate( int channel ){

  double z = 0.0;

  for( size_t i=0; i<500; i++ ){
    mtsDouble volts;
    Read( mtsInt( channel ), volts );
    z = z*((double)i/((double)i+1.0)) + volts/((double)i+1.0);
    osaSleep( 0.01 );
  }
  
  zeroforces[ channel ] = z;
  std::cout << "% zeroforce " << channel << ": " << z << std::endl;
}
