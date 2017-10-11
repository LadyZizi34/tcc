#include "myahrs_plus_c.hpp"

static const int BAUDRATE = 115200;
//- output rate(Hz) = max_rate/divider
static const char* DIVIDER = "1";  // 100 Hz
static const char* DEVICE = "/dev/ttyACM0";  

void handle_error(const char* error_msg)
{
    fprintf(stderr, "ERROR: %s\n", error_msg);
    exit(1);
}



int main() {

/***** Configuration of sensor communication *****/
	
	// Initialize sensor
    MyAhrsPlus sensor;
    SensorData sensor_data;    

    // Start communication with myAHRS+
    if(sensor.start(DEVICE, BAUDRATE) == false) 
        handle_error("start() returns false");

    // Set ascii output parameters: euler + IMU
    if(sensor.cmd_ascii_data_format("RPYIMU") == false) 
        handle_error("cmd_ascii_data_format() returns false");   

    // Set divider of output rate 
    if(sensor.cmd_divider(DIVIDER) ==false) 
        handle_error("cmd_divider() returns false");    

    // Set transfer mode (ASCII Continuous)
    if(sensor.cmd_mode("AC") ==false) 
        handle_error("cmd_mode() returns false");    

    return 0;

}