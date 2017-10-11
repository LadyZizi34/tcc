#include <fcntl.h>  // for open
#include <unistd.h> // for close, read and write
#include <termios.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> // for quaternion normalize sqrt, usar -lm !!
          

//typedef enum { false, true } bool;


/**************************************************************************
 *
 * ATTITUDE REPRESENTATIONS
 *
 **************************************************************************/

struct euler_angle
{
    double roll, pitch, yaw; // iniciar tudo com 0
    char *str_rpy;
    char *delimiter; // iniciar com ' '
};

void euler_set_rpy (struct euler_angle ea, double r, double p, double y)
{
	ea.roll = r;
	ea.pitch = p;
	ea.yaw = y;
}

void euler_reset (struct euler_angle ea)
{
	euler_set_rpy(ea, 0, 0, 0);
}

void euler_set_tokens(struct euler_angle ea, char *str_array[3], int size)
{
	if (size != 3) {
  		perror("EulerAngle: size error"); exit(-1);
  	}
    ea.roll  = atof(str_array[0]); //returns string as a double
    ea.pitch = atof(str_array[1]);
    ea.yaw   = atof(str_array[2]);  	
}

void euler_set_str (struct euler_angle ea, char* str_rpy, char* delimiter)
{
	int i=0;
	char *t = strtok(str_rpy, delimiter);
	char *tokens[3];

	while (t != NULL)
    {
        tokens[i] = t;
        t = strtok (NULL, delimiter); //continues scanning from end of previous successful call
        i++;
    }

  	if(i == 3)
  		euler_set_tokens(ea, tokens, i);
  	else
  	{
  		perror("EulerAngle: Invalid String"); exit(-1);
  	}
}

void euler_to_string(struct euler_angle ea, char* str) //colocar tamanho exato?
{
	sprintf(str, "%.2f,%.2f,%.2f", ea.roll,ea.pitch, ea.yaw);
}

struct quaternion
{
    double x, y, z, w; // comeca tudo com 0, exceto w=1
    char *str_xyzw;
    char *delimiter; // ' '
};

void quat_set_xyzw(struct quaternion q, double x, double y, double z, double w)
{
	q.x = x;
	q.y = y;
	q.z = z;
	q.w = w;
}

void quat_reset(struct quaternion q)
{
	quat_set_xyzw(q, 0, 0, 0, 1);
}

void quat_set_tokens(struct quaternion q, char *str_array[4], int size)
{
	if (size != 4) {
  		perror("Quaternion: size error"); exit(-1);
  	}
    q.x = atof(str_array[0]);
    q.y = atof(str_array[1]);
    q.z = atof(str_array[2]);
    q.w = atof(str_array[3]); 	
}

void quat_set_str(struct quaternion q, char* str_xyzw, char* delimiter)
{
	int i=0;
	char *t = strtok(str_xyzw, delimiter);
	char *tokens[4];

	while (t != NULL)
    {
        tokens[i] = t;
        t = strtok (NULL, delimiter); 
        i++;
    }

  	if(i == 4)
  		quat_set_tokens(q, tokens, i);
  	else
  	{
  		perror("Quaternion: Invalid String"); exit(-1);
  	}	
}

void quat_to_string(struct quaternion q, char* str) //[45]?
{
	sprintf(str, "%.4f,%.4f,%.4f,%.4f", q.x, q.y, q.z, q.w);
}

void normalize(struct quaternion q)
{
	double norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
	q.x = q.x/norm;
	q.y = q.y/norm;
	q.z = q.z/norm;
	q.w = q.w/norm;
}

struct quaternion quat_conj(struct quaternion q)
{
    struct quaternion new_q;
    new_q.x = -q.x;
    new_q.y = -q.y;
    new_q.z = -q.z;
    new_q.w = q.w;
    return new_q;	
}

struct quaternion product(struct quaternion q, struct quaternion r)
{
	struct quaternion qxr;
    qxr.w = r.w*q.w - r.x*q.x - r.y*q.y - r.z*q.z;
    qxr.x = r.w*q.x + r.x*q.w - r.y*q.z + r.z*q.y;
    qxr.y = r.w*q.y + r.x*q.z + r.y*q.w - r.z*q.x;
    qxr.z = r.w*q.z - r.x*q.y + r.y*q.x + r.z*q.w;

    return qxr;	
}
    // http://www.mathworks.co.kr/kr/help/aerotbx/ug/quatmultiply.html
    // qxr = q*r
    //static Quaternion product(Quaternion& q, Quaternion& r) {

struct euler_angle to_euler_angle(struct quaternion q)
{
    double xx = q.x*q.x;
    double xy = q.x*q.y;
    double xz = q.x*q.z;
    double xw = q.x*q.w;
    double yy = q.y*q.y;
    double yz = q.y*q.z;
    double yw = q.y*q.w;
    double zz = q.z*q.z;
    double zw = q.z*q.w;
    double ww = q.w*q.w;

    double RAD2DEG = 180/M_PI;
    struct euler_angle e;
    e.roll  = atan2(2.0*(yz + xw), -xx - yy + zz + ww)*RAD2DEG;
    e.pitch = -asin(2.0*(xz - yw))*RAD2DEG;
    e.yaw   = atan2(2.0*(xy + zw), xx - yy - zz + ww)*RAD2DEG;
    return e;	
}

/**************************************************************************
 *
 * IMU
 *
 **************************************************************************/

struct imu_data
{
	float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;
};

void imu_set(struct imu_data imu, float data[10])  ///////// float ou double???
{
	int i=0;

	imu.ax = data[i++];
	imu.ay = data[i++];
	imu.az = data[i++];

	imu.gx = data[i++];
	imu.gy = data[i++];
	imu.gz = data[i++];

	imu.mx = data[i++];
	imu.my = data[i++];
	imu.mz = data[i++];

	imu.temperature = data[i++];
}

void imu_reset(struct imu_data imu)
{
	float d[10];
	memset(d, 0, sizeof(d)); // preenche os bytes com 0
	imu_set(imu, d);
}

void imu_set_tokens(struct imu_data imu, char *str_array[10], int size)
{
	int i;
	if (size != 10) {
  		perror("imu: size error"); exit(-1);
  	}
    float data[10];
    for (i=0; i<10; i++) 
    	data[i] = (float)atof(str_array[i]);
    
    imu_set(imu, data);		
}

void imu_set_str(struct imu_data imu, char* str_mat, char* delimiter)
{
	int i=0;
	char *t = strtok(str_mat, delimiter);
	char *tokens[10];

	while (t != NULL)
    {
        tokens[i] = t;
        t = strtok (NULL, delimiter); 
        i++;
    }

  	if(i == 10)
  		imu_set_tokens(imu, tokens, i);
  	else
  	{
  		perror("imu: Invalid String"); exit(-1);
  	}	
}

void imu_to_string(struct imu_data imu, char* str) //[30~35]?
{
	sprintf(str, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f", 
			imu.ax, imu.ay, imu.az,
			imu.gx, imu.gy, imu.gz,
			imu.mx, imu.my, imu.mz,
			imu.temperature);
}

/**************************************************************************
 *
 * AHRS
 *
 **************************************************************************/


enum Attitude {
    NOT_DEF_ATTITUDE,
    QUATERNION,
    EULER_ANGLE,
};

enum Imu {
    NOT_DEF_IMU,
    COMPENSATED,
    //RAW,
};

struct sensor_data
{
	int            		sequence_number;

	enum Attitude	    attitude_type;
	struct euler_angle  euler_angle; // degree
	struct quaternion   quaternion;

	enum Imu       		imu_type;
	//imu_data<int>   	imu_rawdata; - nao vou usar
	struct imu_data 		 	imu; //float
};

void sensor_reset(struct sensor_data s)
{
	s.sequence_number = -1;
	euler_reset(s.euler_angle);
	quat_reset(s.quaternion);
	//imu_rawdata.reset();
	imu_reset(s.imu);
	s.attitude_type = NOT_DEF_ATTITUDE;
	s.imu_type = NOT_DEF_IMU;
}

void sensor_set(struct sensor_data s)  /// precisa??????
{
	sensor_reset(s);
}

void sensor_update_attitude_euler(struct sensor_data s, struct euler_angle e)
{
	s.euler_angle = e;
	s.attitude_type = EULER_ANGLE;
}

void sensor_update_attitude_quat(struct sensor_data s, struct quaternion q)
{
	s.quaternion = q;
	s.attitude_type = QUATERNION;
}

void sensor_update_imu(struct sensor_data s, struct imu_data imu)
{
	s.imu = imu;
	s.imu_type = COMPENSATED; // compensated with calibration parameter
}

void sensor_to_string(struct sensor_data s, char* str)
{
	//char* str_array[5]; //3 nao e? ------------------
	char temp[40]; //32?
	sprintf(temp, "sequence = %d\n", s.sequence_number);
    strcat(str, temp);

    switch(s.attitude_type) {
    case EULER_ANGLE:
        euler_to_string(s.euler_angle, temp);
        strcat(str, "euler_angle = ");
        strcat(str, temp);
        strcat(str, "\n");
        break;
    case QUATERNION:
        quat_to_string(s.quaternion, temp);
        strcat(str, "quaternion = ");
        strcat(str, temp);
        strcat(str, "\n");
        break;
    default:
        strcat(str, "no attitude\n");
        break;
    }

    switch(s.imu_type) {
    //case RAW:
    //    strcat(str, "imu_raw = " + imu_rawdata.to_string());
    //    break;
    case COMPENSATED:
        imu_to_string(s.imu, temp);        
        strcat(str, "imu_comp = ");
        strcat(str, temp);
        break;
    default:
        strcat(str, "no imu ");
        break;
    }
}


bool send_command(int fd, char* command_string, int timeout_msec) {
    //LockGuard _l(mutex_communication);

    //DBG_PRINTF(debug, "### SEND : %s\n", command_string.c_str());

    unsigned char crc = 0;
    char crc_string[16];
    char command_with_crc[30];

    for(size_t i=0; i<(int)strlen(command_string); i++) {
        crc ^= command_string[i];
    }
    sprintf(crc_string, "*%02X\r\n", crc);
    strcpy(command_with_crc, command_string);
    strcat(command_with_crc, crc_string);
    //command_with_crc = command_string + crc_string;

    /*
     * send command
     */
    //response_message_queue.clear();     
    if(serial_write(fd,(unsigned char*)command_with_crc, (int)strlen(command_with_crc)) <= 0) {
        fprintf(stderr, "serial.Write() return false\n");
        return false;
    }

    if(timeout_msec < 0) {
        // no wait
        return true;
    }

    char *tokens[10];
    char *cmd_tokens[30];

    do {
        /*
         * wait for response
         */
        //DBG_PRINTF(debug, "### WAITING RESPONSE\n");
        //if(response_message_queue.wait(timeout_msec) == false) {
        //    DBG_PRINTF(debug, "TIMEOUT!!(%d)\n", timeout_msec);
        //    return false;
        //}

        /*
         *  check response
         */
        //tokens.clear();
        /* //memset(tokens, 0, sizeof(tokens));
        if(response_message_queue.pop(tokens) == false) {
            if(debug) {
                Platform::msleep(10);
                DBG_PRINTF(debug, "ERROR??? : response_queue.pop() returns false (response_queue.size() %lu, cmd=%s)\n", response_message_queue.size(), command_string.c_str());
            }
            return false;
        }*/

        //cmd_tokens.clear();
        memset(cmd_tokens, 0, sizeof(cmd_tokens));
        int i=0;
        char *t = strtok(command_string, ',');
        while (t != NULL)
        {
            cmd_tokens[i] = t;
            t = strtok (NULL, ','); 
            i++;
        }
        //StringUtil::split(cmd_tokens, command_string.c_str(), ',');
        //StringUtil::replace(cmd_tokens[0], "@", "~");
        for(i=0; i<(int)strlen(cmd_tokens[0], i++)
        {
            if(cmd_tokens[0][i] == '@')
                cmd_tokens[0][i] = '~';
        }

        if(tokens[0] == cmd_tokens[0]) {
            printf("### RCV OK(%s)\n", cmd_tokens[0]);
            break;
        }
        else {
            printf("ERROR: invalid response. command %s, response %s)\n", cmd_tokens[0], tokens[0]);
            continue;
        }
    }while(1);


    /*
     *  handle error response
     */     
    /*if(tokens[1] != std::string("OK")) {
        // print error message
        DBG_PRINTF(debug, "ERROR: status is not OK. command %s)\n", cmd_tokens[0].c_str());
        return false;
    }*/

    /*
     *  run response handler
     */
    /*std::map<std::string, AscRspHandler>::iterator it = ascii_handler_rsp_map.find(tokens[0]);
    if(it != ascii_handler_rsp_map.end()) {
        std::map<std::string, std::string> attributes;
        bool res = true;
        if(StringUtil::extract_attributes(attributes, tokens) > 0) {
            LockGuard _l(mutex_attribute);
            res = (this->*(it->second))(attributes);
            if(res == false) {
                DBG_PRINTF(debug, "ERROR: message hander returns false. command %s)\n", cmd_tokens[0].c_str());
            }
        }

        if(res) {
            DBG_PRINTF(debug, "### OK : message hander returns true. command %s, rcvq_sz %d)\n", cmd_tokens[0].c_str(), response_message_queue.size());
        }

        return res;
    }
    else {
        return false;
    }*/
    return true;
}


bool cmd_ascii_data_format(int fd, char* asc_output) { //aqui é só pra converter sua msg no
                                                // formato certo. tá mandando 'IMU'
    //parameters char* asc_output=0, int timeout_msec=500
    int timeout_msec = 500;
    char command[30];
    if(asc_output == 0) {
        return send_command(fd, "@asc_out", timeout_msec);
    }
    else {
        strcpy(command,"@asc_out,");
        strcat(command, asc_output);
        return send_command(fd, command.c_str(), timeout_msec);
    }
}

/**************************************************************************
 *
 * Serial Communications
 *
 **************************************************************************/

bool serial_open(int port_fd, char* dev, int baudrate)
{
    int fd = 0;
    struct termios options;

    if(port_fd > 0) {
        return true;
    }

    fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd < 0) {
        return false;
    }

    fcntl(fd, F_SETFL, 0);   // clear all flags on descriptor, enable direct I/O
    tcgetattr(fd, &options); // read and save current serial port options

    cfsetspeed(&options, baudrate);
    cfmakeraw(&options);

    options.c_cflag |= CREAD | CLOCAL; // turn on READ
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    options.c_cc[VINTR] = 0;  // Ctrl-c 

    // set the new port options
    tcsetattr(fd, TCSANOW, &options);

    // flush I/O buffers
    tcflush(fd, TCIOFLUSH);

    port_fd = fd;

    return (fd > 0);	
}

void serial_close(int port_fd)
{
   if(port_fd > 0) {
        close(port_fd);
        port_fd = -1;
    }
}

int serial_read(int port_fd, unsigned char* buf, unsigned int buf_len)
{
	if(port_fd < 0) {
	    return -1;
	}

	int n = read(port_fd, buf, buf_len-1); //number of characters read
	if(n > 0) {
	    buf[n] = 0; /* set end of string, so we can printf */
	}

	return n;	
}

int serial_write(int port_fd, unsigned char* data, unsigned int data_len) {
    if(port_fd < 0) {
        return -1;
    }

    return write(port_fd, data, data_len);
}

int serial_flush(int port_fd) {
    if(port_fd < 0) {
        return -1;
    }

    // flush I/O buffers
    return tcflush(port_fd, TCIOFLUSH);
}




