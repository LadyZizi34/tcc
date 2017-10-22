#include <fcntl.h>  // for open
#include <unistd.h> // for close, read and write
#include <termios.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <math.h> //?

#include "imu_ops.h"
#include "serial_myahrs_ops.h"
#include "serial_gps_ops.h"
#include "coord_convertion.h"

#define BUFFERSIZE 100
#define BAUDRATE 9600            
#define DEVICE_AHRS "/dev/ttyACM0" 
#define DEVICE_GPS "/dev/ttyACM1" 
#define INIT_REFS 3 //deve ser uns 20?

int main() {
   
    int fd_ahrs, fd_gps, i;
    int bytes_read;
    char buf[BUFFERSIZE];//[BUFFERSIZE] output is around 84 chars, max 91
    char imu_buf[BUFFERSIZE];
    //char buf_imu[12*sizeof(float)];
    char brate[6];

    double ref_lat=0, ref_lon=0, ref_alt=0;
    double gps_coordinates[3];

    struct imu_data imu;
    struct gps_data gps;
    struct timeval t_begin, t_end;

    fd_ahrs = serial_open(DEVICE_AHRS, BAUDRATE);
    if(fd_ahrs < 0) 
        exit(1);//shell code for general errors

    fd_gps = serial_open(DEVICE_GPS, BAUDRATE);
    if(fd_gps < 0) 
        exit(1);//shell code for general errors    
    
    sprintf(brate, "@baudrate,%d", BAUDRATE);     
    if(send_command(fd_ahrs, brate) < 0)
        exit(1);
    wait_response(fd_ahrs, "~baudrate,OK", 20);

    if(send_command(fd_ahrs, "@asc_out,IMU") < 0)
        exit(1);
    wait_response(fd_ahrs, "~asc_out,OK", 20);
    
    // flush I/O buffers
    tcflush(fd_ahrs, TCIOFLUSH); //a resposta do dispositivo demora e é jogada para a serial
   // tcflush(fd_gps, TCIOFLUSH);

    //imu_reset(imu);
    //float arr[10] = {0,0,0,0,0,0,0,0,0,0};
    //imu_set(imu, arr);    

    for(i=0;i<INIT_REFS;i++) {
        if((bytes_read = read_protocol(fd_gps, "GPGGA", 20, buf, (BUFFERSIZE-25))) > 0) 
            gps_set_str(&gps,buf,","); 
        if(bytes_read < 0) // failsafe to avoid running forever? senao nem precisa do bytes_read la em cima
            break;   
        ref_lat += gps.lat;
        ref_lon += gps.lon;   
        ref_alt += gps.alt;   
        //printf("Initial References: %.5f, %.5f, %.1f\n", gps.lat, gps.lon, gps.alt);
    }
    ref_lat = ref_lat/INIT_REFS; //pega a média das init_refs leituras
    ref_lon = ref_lon/INIT_REFS;
    ref_alt = ref_alt/INIT_REFS;
    //gps_reference = {ref_lon,ref_lat,ref_alt};
    //printf("Initial References: %.5f, %.5f, %.1f\n\n", ref_lat, ref_lon, ref_alt);
    printf("Initial References: %.5f, %.5f, %.1f\n\n", ref_lat, ref_lon, ref_alt);
//tcflush(fd_ahrs, TCIOFLUSH);
    //Read OK
    while(1) { // reads until Ctrl+C signal
        //memset(buf, 0, sizeof(buf)); //read exact number of characters
        //bytes_read = serial_read(fd_ahrs, buf, BUFFERSIZE, 0); // MUST CHECK IF FIRST TOKEN IS 'IMU'       
                                                // bc its sometimes repeating characters
                                                // from end of previous line
                                                // update! this is because of mismatching buf length
                                               // will have to discard first entries

        //gettimeofday(&t_begin, NULL);
        bytes_read = read(fd_ahrs,buf,BUFFERSIZE);
        buf[bytes_read]=0;        
        //if(bytes_read > 85) {
            //strcpy(imu_buf,buf);
            //printf("%s", buf);
            imu_set_str(&imu,buf,",");
            //gettimeofday(&t_end, NULL);
            //printf("usec: %06u\n", (t_begin.tv_usec - t_end.tv_usec)); //whole seconds of elapsed time,
                                                    //rest of the elapsed time (a fraction of a second)
            //imu_to_string(imu, buf_imu); (OK)
            printf("IMU Struct: %.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f\n\n", 
                    imu.ax, imu.ay, imu.az,
                    imu.gx, imu.gy, imu.gz,
                    imu.mx, imu.my, imu.mz,
                    imu.temperature);
            //fflush(stdout); //precisa?


//memset(buf, 0, sizeof(buf));                                              // tava só >
        if((bytes_read = read_protocol(fd_gps, "GPGGA", 20, buf, (BUFFERSIZE-25))) >= 0) {
            //printf("%s\n", buf);  
            gps_set_str(&gps,buf,",");  
            printf("GPS Struct:          %.5f,%c, %.5f,%c,          %.1f\n", 
                    gps.lat, gps.lat_dir, gps.lon,
                    gps.lon_dir, gps.alt);       
        }
        //printf("Refs: %.5f, %.5f\n", ref_lat, ref_lon);
        //}
        /*else*/ if(bytes_read < 0) // failsafe to avoid running forever?
            break;    

        gps_coordinates[0] = gps.lat;
        gps_coordinates[1] = gps.lon;
        gps_coordinates[2] = gps.alt;

        // printf("Coord:           %.5f,   %.5f,            %.1f\n", 
        //         gps_coordinates[0], gps_coordinates[1], gps_coordinates[2]);
        // printf("Refer:           %.5f,   %.5f,            %.5f\n", 
        //         ref_lat, ref_lon, ref_alt);        
        geodetic2ned(gps_coordinates, ref_lat, ref_lon, ref_alt);//devo mandar as refs certas, 000 pra testar
        printf("NED Coord:           %.5f,   %.5f,            %.1f\n\n", 
                gps_coordinates[0], gps_coordinates[1], gps_coordinates[2]);        
    }
    
    serial_close(fd_ahrs);
    serial_close(fd_gps);

    return 0;
}
