#include <math.h>
#define M_PI 3.14159265358979323846

//void geodetic2ecef(double*phi, double* lambda, double* h)
void geodetic2ecef(double coord[3])
{
//radians = degrees × π / 180°
	double x,y,z,e2,NE, lat, lon, alt;
	//lat = floor(coord[0])+(coord[0]-floor(coord[0])*100/60);  //convert to degrees
	//lon = floor(coord[1])+(coord[1]-floor(coord[1])*100/60);
	lat = coord[0];
	lon = coord[1];
	lat = lat*M_PI/180; //to rad
	lon = lon*M_PI/180;
	alt = coord[2]; //altitude
	//e = 0.08181919; too little precision maybe?
	e2 = 0.00669437999; //first excentricity squared(from Chapter 3.pdf)
	NE =  6378137/sqrt(1 - e2*sin(lat)*sin(lat));
	     //^semi major axis

	coord[0] = (NE + alt) * cos(lat) * cos(lon); //x (m)
	coord[1] = (NE + alt) * cos(lat) * sin(lon); //y (m)
	coord[2] = (NE * (1 - e2) + alt) * sin(lat); //z (m)
} //xyz em metros

void ecef2ned(double coord[3], double ref[3], double phi_r, double lambda_r)
{
	//double xyz?
	int i,j,k;
	double sum, Psub[3];
	double Rne[3][3] = {  
	   {-sin(phi_r)*cos(lambda_r), 	-sin(phi_r)*sin(lambda_r), 	cos(phi_r)	},
	   {-sin(lambda_r),				cos(lambda_r), 				0 			},
	   {-cos(phi_r)*cos(lambda_r), 	-cos(phi_r)*sin(lambda_r), 	-sin(phi_r)	}
	};	

//Pn = Rne * (Pe − Pe,ref)

    for(i=0; i< 3;i++) 
        Psub[i]=coord[i]-ref[i]; 

	for(i=0; i<3; i++)
	{
		sum=0;
		for(k=0; k<3; k++)
		{
			sum = sum + Rne[i][k] * Psub[k];
		}
		coord[i] = sum;
	}    
}

void geodetic2ned(double coord[3], double lat_r, double lon_r, double h_r)
{
	double ref[3] = {lat_r, lon_r, h_r};
        // printf("Refteste:           %.5f,  %.5f,  %.5f\n", 
        //         ref[0], ref[1], ref[2]); 	
	geodetic2ecef(coord);
	geodetic2ecef(ref);
	ecef2ned(coord,ref, lat_r, lon_r);

	// coord[0] = 1;
	// coord[1] = 2;
	// coord[2] = 3;
}