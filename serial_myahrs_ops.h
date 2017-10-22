int serial_open(char* dev, int baudrate)
{
    int fd;
    struct termios options;

    fd = open(dev, O_RDWR | O_NOCTTY /*| O_NDELAY*/);
    if(fd < 0) {
        printf("Could not open device\n");
        return -1;
    }

    fcntl(fd, F_SETFL, 0);   // clear all flags on descriptor, enable direct I/O
    tcgetattr(fd, &options); // read and save current serial port options

    cfsetspeed(&options, baudrate);
    cfmakeraw(&options);

    options.c_cflag |= CREAD | CLOCAL; // turn on READ
    options.c_cflag |= CS8;
    options.c_cflag |= baudrate | CRTSCTS; //new
    options.c_cc[VMIN] = 1; // min chars read
    //options.c_cc[VMIN] = 0; // teste para saber serial.available -- fayou
    options.c_cc[VTIME] = 0;
    //options.c_iflag = IGNPAR | ICRNL;
    //options.c_oflag = 0;
    options.c_lflag = ICANON;

    options.c_cc[VINTR] = 0;  // Ctrl-c 

    // set the new port options
    tcsetattr(fd, TCSANOW, &options);

    // flush I/O buffers
    tcflush(fd, TCIOFLUSH);

    return (fd);	
}

void serial_close(int port_fd)
{
   if(port_fd > 0) {
        close(port_fd);
        port_fd = -1;
    }
}

int serial_read(int port_fd, unsigned char* buf, unsigned int buf_len, int min_accepted)
{
    int n;
    //if(port_fd < 0) {
    //    return -1;
    //}
    //int n = read(port_fd, buf, buf_len-1); // everything in // is old version,xcept buf[n]=0
    do {
        n = read(port_fd, buf, buf_len-1); // number of characters read
    } while(n < min_accepted);
    //if(n > 0) {
        buf[n] = 0; /* set end of string, so we can printf */
    //}

    return n;   
}

int send_command(int fd, char* command_string) 
{
    unsigned char crc = 0;
    char crc_string[16];
    char command_with_crc[25];
    char tmp[25];

    for(size_t i=0; i<(int)strlen(command_string); i++) {
        crc ^= command_string[i];
    }
    sprintf(crc_string, "*%02X\r\n", crc);
    strcpy(command_with_crc, command_string);
    strcat(command_with_crc, crc_string);
    
    if(write(fd,(unsigned char*)command_with_crc, (int)strlen(command_with_crc)) <= 0) {
        fprintf(stderr, "Failed to write on serial device.\n");
        return -1; //false
    }    

    return 1; //true
}

void wait_response(int fd, char* str, int tries)
{
    char needle[15];
    char haystack[20];
    char *substr;

    strcpy(needle, str);
    while(tries > 0) {
        //memset(buf, 0, sizeof(buf));
        //serial_read(fd, haystack, 20, 8);
        read(fd, haystack, 20);
        substr = strstr(haystack, needle);
        //printf("%s\n", haystack);
        if(substr != NULL) {
            //printf("Response found!\n");
            break;
        }
        else tries--;        
    }
    if(tries == 0)
        printf("AHRS did not respond to command.\n");
}

int read_protocol(int fd, char* ptcl, int tries, char* reading_buf, int buffersize)
{
    char ptcl_name[15];    
    char *substr;
    int bytes_read;
    strcpy(ptcl_name, ptcl);

    while(tries > 0) { 
        //memset(reading_buf, 0, sizeof(reading_buf));
        bytes_read = read(fd, reading_buf, buffersize);
        substr = strstr(reading_buf, ptcl_name);
        if(substr != NULL) {            
            reading_buf[bytes_read] = 0;                           
            return bytes_read;            
        }
        else tries--;
    }
    if(tries == 0){
        printf("GPS did not send specified protocol.\n");
        return -1;
    }
}