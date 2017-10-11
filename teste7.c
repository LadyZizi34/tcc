#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> //?
#include "teste_bib.h"

#define DEVICE "/dev/ttyACM0"
#define BAUDRATE 115200 

int main() {

	int fd = -1; //init file desc

	sensor_data s_data;

    /*
     * 	start communication with the myAHRS+.
     */	

	if(!(serial_open(fd, DEVICE, BAUDRATE))) {
		fprintf(stderr, "Could not open device\n");
		exit(1);//shell code for general errors
	}

    /*
     *  set ascii output format
     *   - select euler angle
     */
    

	return 0; //shell code for success
}
