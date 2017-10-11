#include <fcntl.h>  // for open
#include <unistd.h> // for close, read and write
#include <termios.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "imu_ops.h"
#include "serial_myahrs_ops.h"

#define BUFFERSIZE 100
#define BAUDRATE 9600            
#define DEVICE "/dev/ttyACM1" 

int main() {
   
    int fd, i;
    int bytes_read;
    char buf[BUFFERSIZE];//[BUFFERSIZE] output is around 84 chars, max 91
    char imu_buf[BUFFERSIZE];
    //char buf_imu[12*sizeof(float)];
    char brate[6];

    struct imu_data imu;
    struct timeval t_begin, t_end;

    fd = serial_open(DEVICE, BAUDRATE);
    if(fd < 0) 
        exit(1);//shell code for general errors
    
    sprintf(brate, "@baudrate,%d", BAUDRATE);     
    if(send_command(fd, brate) < 0)
        exit(1);
    wait_response(fd, "~baudrate,OK", 20);

    if(send_command(fd, "@asc_out,IMU") < 0)
        exit(1);
    wait_response(fd, "~asc_out,OK", 20);
    
    // flush I/O buffers
    tcflush(fd, TCIOFLUSH); //a resposta do dispositivo demora e é jogada para a serial
   

    //imu_reset(imu);
    //float arr[10] = {0,0,0,0,0,0,0,0,0,0};
    //imu_set(imu, arr);    

    //Read OK
    while(1) { // reads until Ctrl+C signal
        //memset(buf, 0, sizeof(buf)); //read exact number of characters
        //bytes_read = serial_read(fd, buf, BUFFERSIZE, 0); // MUST CHECK IF FIRST TOKEN IS 'IMU'       
                                                // bc its sometimes repeating characters
                                                // from end of previous line
                                                // update! this is because of mismatching buf length
                                               // will have to discard first entries
        gettimeofday(&t_begin, NULL);
        bytes_read = read(fd,buf,BUFFERSIZE);
        buf[bytes_read]=0;        
        //if(bytes_read > 85) {
            //strcpy(imu_buf,buf);
            printf("%s", buf);
            imu_set_str(&imu,buf,",");
            gettimeofday(&t_end, NULL);
            printf("usec: %06u\n", /*(t_begin.tv_sec - t_end.tv_sec),*/ (t_begin.tv_usec - t_end.tv_usec)); //whole seconds of elapsed time,
                                                    //rest of the elapsed time (a fraction of a second)
            //imu_to_string(imu, buf_imu);
            printf("Struct: %.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f\n\n", 
                    imu.ax, imu.ay, imu.az,
                    imu.gx, imu.gy, imu.gz,
                    imu.mx, imu.my, imu.mz,
                    imu.temperature);
            //fflush(stdout); //precisa?
        //}
        /*else*/ if(bytes_read < 0) // failsafe to avoid running forever?
            break;        
    }
    
    serial_close(fd);

    return 0;
}
