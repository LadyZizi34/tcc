struct gps_data
{  
  double lat, lon, alt; //2,4,9
  char lat_dir, lon_dir; //3,5
};
//hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx
//GPGGA,183138.00,2200.44399,S,04753.69157,W,1,05,1.92,842.6,M,-6.1,M,,*44
//  0        1        2      3      4      5 6  7   8    9  10  11 12 13 14
void gps_set(struct gps_data* gps, double data[3], char dir_data[2])  ///////// double ou double???
{
  gps->lat = data[0]/100;
  gps->lon = data[1]/100;
  gps->alt = data[2];

  gps->lat_dir = dir_data[0];
  gps->lon_dir = dir_data[1];

  if(gps->lat_dir == 'S')
    gps->lat *= -1;
  if(gps->lon_dir == 'W')
    gps->lon *= -1;  
}

void gps_set_tokens(struct gps_data* gps, char *str_array[14], int size) //there are two
                                    // header tokens
{
  int i;
  if (size != 14) {
    perror("gps: size error"); exit(-1); // should just skip if its one time
  }
  double data[3];
  char dir_data[2];
  data[0] = (double)atof(str_array[2]);  //returns string as a double  
  data[1] = (double)atof(str_array[4]);
  data[2] = (double)atof(str_array[9]);
  //printf("%.5f %.5f %.1f\n", data[0],data[1],data[2]);
  dir_data[0] = str_array[3][0];
  dir_data[1] = str_array[5][0];

  gps_set(gps, data, dir_data);   
}


void gps_set_str(struct gps_data* gps, char* orig_str, char* delimiter)
{
	int i=0;
	char *t = strtok(orig_str, delimiter);
	char *tokens[14];

	while (t != NULL)
    {
        tokens[i] = t;
        t = strtok (NULL, delimiter); 
        i++;
    }

  	if(i == 14)
  		gps_set_tokens(gps, tokens, i);
  	else
  	{	printf("(i = %d)\n",i );
  		printf("gps: Invalid String\n"); //exit(-1); ----------------ta caindo aki
  	}	
}