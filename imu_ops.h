struct imu_data
{
	float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;
};

void imu_set(struct imu_data* imu, float data[10])  ///////// float ou double???
{
	int i=0;

	imu->ax = data[i++];
	imu->ay = data[i++];
	imu->az = data[i++];

	imu->gx = data[i++];
	imu->gy = data[i++];
	imu->gz = data[i++];

	imu->mx = data[i++];
	imu->my = data[i++];
	imu->mz = data[i++];

	imu->temperature = data[i++];
}

void imu_reset(struct imu_data* imu)
{
	//float d[10];
	//memset(d, 0, sizeof(d)+1); // preenche os bytes com 0
	float d[10] = {0,0,0,0,0,0,0,0,0,0};
	imu_set(imu, d);
}

void imu_set_tokens(struct imu_data* imu, char *str_array[12], int size) //there are two
																		// header tokens
{
	int i;
	if (size != 12) {
  		perror("imu: size error"); exit(-1); // should just skip if its one time
  	}
    float data[10];
    for (i=0; i<10; i++) 
    	data[i] = (float)atof(str_array[i+2]);  //returns string as a double
    
    imu_set(imu, data);		
}

void imu_set_str(struct imu_data* imu, char* str_mat, char* delimiter)
{
	int i=0;
	char *t = strtok(str_mat, delimiter);
	char *tokens[12];

	while (t != NULL)
    {
        tokens[i] = t;
        t = strtok (NULL, delimiter); 
        i++;
    }

  	if(i == 12)
  		imu_set_tokens(imu, tokens, i);
  	else
  	{	printf("(i = %d)\n",i );
  		printf("imu: Invalid String\n"); //exit(-1);
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