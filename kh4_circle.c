/* \file kh4_small_ex.c 

 *
 * \brief 
 *         This is the small application example for the Khepera4	 
 *         
 *        
 * \author   Julien Tharin (K-Team SA)                               
 *
 * \note     Copyright (C) 2013 K-TEAM SA
 * \bug      none discovered.                                         
 * \todo     nothing.

 * compile with command (don't forget to source the env.sh of your development folder!):
 		arm-angstrom-linux-gnueabi-gcc kh4_small_ex.c -o kh4_small_ex -I $INCPATH -L $LIBPATH -lkhepera 


*/
#include <khepera/khepera.h>
#include <signal.h>


static knet_dev_t * dsPic; // robot pic microcontroller access



static int quitReq = 0; // quit variable for loop

/*--------------------------------------------------------------------*/
/*!
 * Make sure the program terminate properly on a ctrl-c
 */
static void ctrlc_handler( int sig ) 
{
  quitReq = 1;
  
  kh4_set_speed(0 ,0 ,dsPic); // stop robot
  kh4_SetMode( kh4RegIdle,dsPic );
  
  kh4_SetRGBLeds(0,0,0,0,0,0,0,0,0,dsPic); // clear rgb leds because consumes energy
  
  kb_change_term_mode(0); // revert to original terminal if called
  
  exit(0);
}


/*--------------------------------------------------------------------*/
/*!
 * Main
 */
int main() 
{ 

  double fpos,dval;
  int lpos,rpos;
  char Buffer[100],bar[12][64],revision,version;
  int i,n,val,type_of_test=0,sl,sr,pl,pr,mean;
  short index, value,sensors[12],usvalues[5];
  char c;
  long motspeed;
  char line[80],l[9];
  int kp,ki,kd;
  int pmarg,maxsp,accinc,accdiv,minspacc, minspdec; // SetSpeedProfile
  float x_y_p[3] = {0,0,0};
  long l_mot, r_mot;
  float v,k,w,x,y,theta;
  float ln =0.1054;
  int num=0;
  
  
  printf("\nKhepera 4 small example program\n");
  
  // initiate libkhepera and robot access
  if ( kh4_init(0,NULL)!=0)
  {
  	printf("\nERROR: could not initiate the libkhepera!\n\n");
  	return -1;
  }	

  /* open robot socket and store the handle in its pointer */
  dsPic  = knet_open( "Khepera4:dsPic" , KNET_BUS_I2C , 0 , NULL );

	if ( dsPic==NULL)
  {
  	printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
  	return -2;
  }	

  /* ------  initialize the motors controlers --------------------------------*/
   
  /* tuned parameters */
  pmarg=20;
  kh4_SetPositionMargin(pmarg,dsPic ); 				// position control margin
  kp=10;
  ki=5;
  kd=1;
  kh4_ConfigurePID( kp , ki , kd,dsPic  ); 		// configure P,I,D
  
  accinc=3;
  accdiv=0;
  minspacc=20;
  minspdec=1;
  maxsp=800;
  kh4_SetSpeedProfile(accinc,accdiv,minspacc, minspdec,maxsp,dsPic ); // Acceleration increment ,  Acceleration divider, Minimum speed acc, Minimum speed dec, maximum speed
  
	kh4_SetMode( kh4RegIdle,dsPic );  				// Put in idle mode (no control)


  // get revision
  if(kh4_revision(Buffer, dsPic)==0){
   	version=(Buffer[0]>>4) +'A';
  	revision=Buffer[0] & 0x0F; 
    printf("\r\nVersion = %c, Revision = %u\r\n",version,revision);        
  }
  

  signal( SIGINT , ctrlc_handler ); // set signal for catching ctrl-c
  
  
  //  ------  battery example --------------------------------------------------
	kh4_battery_status(Buffer,dsPic);
	printf("\nBattery:\n  status (DS2781)   :  0x%x\n",Buffer[0]);
	printf("  remaining capacity:  %4.0f mAh\n",(Buffer[1] | Buffer[2]<<8)*1.6);
	printf("  remaining capacity:   %3d %%\n",Buffer[3]);
	printf("  current           : %5.0f mA\n",(short)(Buffer[4] | Buffer[5]<<8)*0.07813);
	printf("  average current   : %5.0f mA\n",(short)(Buffer[6] | Buffer[7]<<8)*0.07813);
	printf("  temperature       :  %3.1f C \n",(short)(Buffer[8] | Buffer[9]<<8)*0.003906);
	printf("  voltage           :  %4.0f mV \n",(Buffer[10] | Buffer[11]<<8)*9.76);
	printf("  charger           :  %s\n",kh4_battery_charge(dsPic)?"plugged":"unplugged");
  
		

	// Tell to the motor controller to move the Khepera 4 backward, in speed profile control
	
	/*motspeed= (long)(-25.0/KH4_SPEED_TO_MM_S); // convert speed from mm/s to pulse/10ms
	kh4_SetMode( kh4RegSpeedProfile,dsPic );
	kh4_set_speed(motspeed ,motspeed ,dsPic);
	

	printf("\nMoving backward %.1f cm at %.1f mm/s (pulse speed %ld) with speed profile control\n",10.0,-25.0,motspeed);*/
	kh4_SetMode( kh4RegSpeedProfile,dsPic );
	float secs = 20;
	float duration = 0;
	float prev_d = 0;

	clock_t start, start_t, end;
	start_t = clock();
	v 	= 0.5;	//linear velocity in m/s
	k	= 1;	// control gain k
	while(duration < secs)
	{	/*struct timespec start, end;
		clock_gettime(CLOCK_MONOTONIC_RAW, &start);
		clock_gettime(CLOCK_MONOTONIC_RAW, &end);

		uint64_t delta_us = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
		*/


		w	= (v+k*((x_y_p[0]-1)*cos(x_y_p[2])+(x_y_p[1]-1)*sin(x_y_p[2])))/0.5;	//angular velocity in rad/s

		l_mot 	= (float)(1000*(v - (w*ln)/2)/KH4_SPEED_TO_MM_S);
		r_mot 	= (float)(1000*(v + (w*ln)/2)/KH4_SPEED_TO_MM_S);
		kh4_set_speed(l_mot ,r_mot ,dsPic);


		end = clock();

		duration = (end - start_t) / 10000.0f;
		float dt = (duration - prev_d);

		x_y_p[0]	= x_y_p[0] + v*cos(x_y_p[2])*dt;
		x_y_p[1]	= x_y_p[1] + v*sin(x_y_p[2])*dt;
		x_y_p[2]	= x_y_p[2] + w*dt;

		printf("Time elapsed: %0.6f \n",duration  );
		prev_d = duration;
		kh4_set_speed(0 ,0,dsPic);

	}

	// Tell to the motor controller to stop the Khepera 4
	kh4_set_speed(0 ,0,dsPic);

	
	sleep(1);	 // Wait 1 seconds
	kh4_get_position(&lpos,&rpos,dsPic);
	printf("\n encoders: left %ld | right %ld\n",lpos,rpos);

	

  return 0;
}
