/*
program for force sensor calibration
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <comedilib.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <time.h>
#include <sys/time.h>

using namespace std;

struct timespec t;
double t_start = 0;

int subdev = 0; // Specifies the input device (subdevice 0 is analog input)
int chan = 0; // We are reading from channel 2
int range = 0;
int aref = AREF_DIFF;

double volts;
int msg = 0;

comedi_t *cf;
int maxdata;
comedi_range *rangedata;

bool flag = true;
bool paused = true;

int nReadings = 500;
double pauseLength = 1; // pause length in ms, or <= 0 for none
double data[500][65535];
double weight[500];
int count = 0;
char c[10];

int main(int argc, char **argv)
{
	// Set up Comedi
	cf=comedi_open("/dev/comedi0");
	maxdata = comedi_get_maxdata(cf,subdev,chan);
	rangedata = comedi_get_range(cf,subdev,chan,range);
	//comedi_dio_config(cf,5,0,COMEDI_OUTPUT);

	FILE *output_file;
	char fname[60]; // file name
	sprintf(fname,"test.m");
	output_file = fopen(fname,"w");

	while(flag) {
		// get user input of force in kg
		cout << endl << "enter weight (kg) or -1 to end or -2 to redo previous: ";
		cin >> weight[count];

		if (weight[count] == -1) {
			break;
		} else if (weight[count] == -2) {
			// write over previous count
			count--;
		} else {
			// Get a bunch of input force readings
			for (int i=0; i<nReadings; i++) {
				lsampl_t raw;
				msg = comedi_data_read_delayed(cf,subdev,chan,range,aref,&raw,3000);
				data[count][i] = comedi_to_phys(raw,rangedata,maxdata);

				// pause between readings
				if (pauseLength > 0) {
					clock_gettime(CLOCK_REALTIME, &t);
					t_start = (double)t.tv_sec + ((double)t.tv_nsec/(double)1000000000);
					paused = true;
					while (paused) {
						clock_gettime(CLOCK_REALTIME, &t);
						if ((double)t.tv_sec + ((double)t.tv_nsec/(double)1000000000) - t_start >= 0.001*pauseLength) {
							paused = false;
						}
					}
				}
			}
			count++;
		}

		if (count >= nReadings) {
			flag = false;
			count--;
		}
	}

	// write data to file
	fprintf(output_file, "weight = [");
	for (int i=0; i<count; i++) {
		fprintf(output_file, "%f; ", weight[i]);
	}
	fprintf(output_file, "];\n\n");

	fprintf(output_file, "data = [");
	for (int i=0; i<count; i++) {
		for (int j=0; j<nReadings-1; j++) {
			fprintf(output_file, "%f, ", data[i][j]);
		}
		fprintf(output_file, "%f; ", data[i][nReadings-1]);
	}
	fprintf(output_file, "];\n");
	
	fclose(output_file);

	return 0;
}
