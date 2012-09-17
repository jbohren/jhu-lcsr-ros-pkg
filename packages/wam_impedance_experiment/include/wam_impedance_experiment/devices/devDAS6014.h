
#ifndef _devDAS6014_h
#define _devDAS6014_h

#include <comedilib.h>

#include <cisstVector/vctDynamicVector.h>
#include <cisstMultiTask/mtsComponent.h>

class devDAS6014 : public mtsComponent{

private:

  std::string devicename;
  
  comedi_t* device;
  unsigned int subdevice;

  vctDynamicVector<unsigned int> channels;
  
  std::vector<lsampl_t> maxdata;
  std::vector<comedi_range*> ranges;

  vctDynamicVector<double> zeroforces;

  void Calibrate( int channel );
  void Read( const mtsInt& channel, mtsDouble& V ) const;
  void Zero( const mtsInt& channel, mtsDouble& zero ) const;


public:

  devDAS6014( const std::string& devicename,
	      unsigned int subdevice,
	      const vctDynamicVector<unsigned int>& channels );
  
  void Configure(){}

  static const std::string Output;
  static const std::string ReadVolt;
  static const std::string GetZero;

};

#endif
