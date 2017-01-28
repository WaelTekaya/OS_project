#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include <math.h>
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif
const char const *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))

  float error=0,correction=0;
	int i;
	//uint8_t sn;
	FLAGS_T state;
	uint8_t sn_touch;
	uint8_t sn_color;
	uint8_t sn_compass;
	uint8_t sn_sonar;
	uint8_t sn_gyr;
	char s[ 256 ];
	int val;
	float value;
	uint32_t n, ii;
	uint8_t sn[2],snmedium;
	float result;
	int maxmedium;
	float initial_ang;

void Gostraight(int time,int max_speed, int a){
	multi_set_tacho_stop_action_inx( sn, TACHO_COAST );
		multi_set_tacho_speed_sp( sn, a * max_speed * 1 / 4 );
		multi_set_tacho_time_sp( sn, time );
		multi_set_tacho_ramp_up_sp( sn, 0 );
		multi_set_tacho_ramp_down_sp( sn, 0 );
		multi_set_tacho_command_inx( sn, TACHO_RUN_TIMED );
		/* Wait tacho stop */
		Sleep( 300 );
		do {
			get_tacho_state_flags( sn[0], &state );
		} while ( state );  


}

void forwarduntil(int max_speed,float goal){
	value=2000.0;
		multi_set_tacho_command_inx( sn, TACHO_RESET );
  		get_tacho_max_speed( sn[0], &max_speed );
	    	multi_set_tacho_stop_action_inx( sn, TACHO_COAST );
		multi_set_tacho_speed_sp( sn, max_speed * 1 / 5 );
		//multi_set_tacho_time_sp( sn, 500 );
		multi_set_tacho_ramp_up_sp( sn, 0 );
		multi_set_tacho_ramp_down_sp( sn, 0 );
		multi_set_tacho_command_inx( sn, TACHO_RUN_FOREVER );
		
	//	Sleep( 300 );
		
	   // sleep(200);
		while(value>goal){	    
		if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0)){
			//printf("SONAR found, reading sonar...\n");
			if ( !get_sensor_value0(sn_sonar, &value )) {
				value = 0;
			}
			//printf( "\r(%f) \n", value);
			fflush( stdout );
	    	}
   		 }
		multi_set_tacho_command_inx( sn, TACHO_STOP );


}
void search(uint8_t ss, int max_speed,float angle){

		float res,dist;		
		if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
				get_sensor_value0(sn_gyr,&res);
		}
		if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0)){
			printf("SONAR found, reading sonar...\n");
			if ( !get_sensor_value0(sn_sonar, &dist )) {
				dist = 0;
			}
			printf( "\r(%f) \n", dist);
			fflush( stdout );
	    	}
		else {
			printf("SONAR not found\n");
		}
		result=res;
		set_tacho_command_inx( ss, TACHO_RESET );
		set_tacho_stop_action_inx( ss, TACHO_COAST );
		set_tacho_speed_sp( ss, max_speed * 1 / 12 );
		//set_tacho_time_sp( s2, 1400 );
		set_tacho_ramp_up_sp( ss, 0 );
		set_tacho_ramp_down_sp( ss, 0 );
		set_tacho_command_inx( ss, TACHO_RUN_FOREVER );
		Sleep(100);
		/*do {
			get_tacho_state_flags( s2, &state );
		} while ( state );
		*/
      int b;
		  if(sn[0]==ss) b=-1;
      else b=1;
			float min=1000,min_ang=0;
			while(b*result < b*(res+b*(angle-error))){
				if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
				get_sensor_value0(sn_gyr,&result);
				printf( "  angle main turn search value %f\n",result);
				}
				fflush( stdout );
				if ( !get_sensor_value0(sn_sonar, &dist )) {
				dist = 0;
				}
				printf( "distance seen : \r(%f) \n", dist);
				if(dist!=0 && min>dist) {
					min=dist;
					min_ang=result;
					
					
				}
							
			}
		set_tacho_command_inx( ss, TACHO_STOP);
		printf("\n");
		printf( "************\r(%f) *********\n", dist);
		Sleep(1000);
		set_tacho_speed_sp( ss, -1*max_speed * 1 / 10 );
		set_tacho_command_inx( ss, TACHO_RUN_FOREVER );
		get_sensor_value0(sn_gyr,&res);
			while(b*result >b*(min_ang)){
				if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
				get_sensor_value0(sn_gyr,&result);
				//printf( "  value %f\n",result);
				}
				fflush( stdout );
				if ( !get_sensor_value0(sn_sonar, &dist )) {
				dist = 0;
				}
				//printf( "\r(%f) \n", dist);
			}
		
		set_tacho_command_inx( ss, TACHO_STOP);
		
		Sleep(1000);
		
		                 while(b*result < b*(min_ang-correction)){
                                if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );

                                set_tacho_command_inx( ss, TACHO_RESET );
                                 set_tacho_stop_action_inx( ss, TACHO_COAST );
                                set_tacho_speed_sp( ss, max_speed * 1 / 10 );
                                set_tacho_time_sp(ss,100);
                                set_tacho_ramp_up_sp( ss, 0 );
                                set_tacho_ramp_down_sp( ss, 0 );
                                set_tacho_command_inx( ss, TACHO_RUN_TIMED );
                                Sleep(1000);
                              if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }

                        }

                        while(b*result > b*(min_ang-correction)){
                                if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );

                                set_tacho_command_inx( ss, TACHO_RESET );
                                 set_tacho_stop_action_inx( ss, TACHO_COAST );
                                set_tacho_speed_sp( ss, -1*max_speed * 1 / 10 );
                                set_tacho_time_sp(ss,100);
                                set_tacho_ramp_up_sp( ss, 0 );
                                set_tacho_ramp_down_sp( ss, 0 );
                                set_tacho_command_inx( ss, TACHO_RUN_TIMED );
                                Sleep(1000);
                              if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }

                        }
 
	


}

void turn_relative(uint8_t ss, int max_speed, int a,float angle)
{
		float res;	
		if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
				get_sensor_value0(sn_gyr,&res);
		}
		result=res;
		set_tacho_command_inx( ss, TACHO_RESET );
		set_tacho_stop_action_inx( ss, TACHO_COAST );
		set_tacho_speed_sp( ss, a*max_speed * 1 / 10 );
		
		set_tacho_ramp_up_sp( ss, 0 );
		set_tacho_ramp_down_sp( ss, 0 );
		set_tacho_command_inx( ss, TACHO_RUN_FOREVER );
		Sleep(100);
		
		int b;
		if(sn[0]==ss) b=-1;
		else b=1;
		if(a==1){
			while(b*result < b*(res+b*(angle-error))){
				if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
				get_sensor_value0(sn_gyr,&result);
				//printf( "  value %f\n",result);
				}
				fflush( stdout );
		
		
			}
		}else{
			while(b*result > b*(res-b*(angle-error))){
				if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
				get_sensor_value0(sn_gyr,&result);
				//printf( "  value %f\n",result);
				}
				fflush( stdout );

			}
		}
		set_tacho_command_inx( ss, TACHO_STOP);
		Sleep(1000);

		if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );
		

		if(a==1){
                        while(b*result < b*(res+b*(angle-correction))){
                                if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );
				
				set_tacho_command_inx( ss, TACHO_RESET );
				 set_tacho_stop_action_inx( ss, TACHO_COAST );
    	           		set_tacho_speed_sp( ss, a*max_speed * 1 / 10 );
				set_tacho_time_sp(ss,100);
               		 	set_tacho_ramp_up_sp( ss, 0 );
               		 	set_tacho_ramp_down_sp( ss, 0 );
               		 	set_tacho_command_inx( ss, TACHO_RUN_TIMED );
        	        	Sleep(1000);
		
if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                        }

			while(b*result > b*(res+b*(angle-correction))){
                                if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );

                                set_tacho_command_inx( ss, TACHO_RESET );
                                 set_tacho_stop_action_inx( ss, TACHO_COAST );
                                set_tacho_speed_sp( ss, -1*a*max_speed * 1 / 10 );
                                set_tacho_time_sp(ss,100);
                                set_tacho_ramp_up_sp( ss, 0 );
                                set_tacho_ramp_down_sp( ss, 0 );
                                set_tacho_command_inx( ss, TACHO_RUN_TIMED );
                                Sleep(1000);

                          if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                        }

                }else{
                        while(b*result > b*(res-b*(angle-correction))){
                                if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );
				set_tacho_command_inx( ss, TACHO_RESET );
				 set_tacho_stop_action_inx( ss, TACHO_COAST );
    	           		set_tacho_speed_sp( ss, a*max_speed * 1 / 10 );
				set_tacho_time_sp(ss,100);
               		 	set_tacho_ramp_up_sp( ss, 0 );
               		 	set_tacho_ramp_down_sp( ss, 0 );
               		 	set_tacho_command_inx( ss, TACHO_RUN_TIMED );
        	        	Sleep(1000);
          if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                        }
			while(b*result < b*(res-b*(angle-correction))){
                                if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );
				set_tacho_command_inx( ss, TACHO_RESET );
				 set_tacho_stop_action_inx( ss, TACHO_COAST );
    	           		set_tacho_speed_sp( ss, -1*a*max_speed * 1 / 10 );
				set_tacho_time_sp(ss,100);
               		 	set_tacho_ramp_up_sp( ss, 0 );
               		 	set_tacho_ramp_down_sp( ss, 0 );
               		 	set_tacho_command_inx( ss, TACHO_RUN_TIMED );
        	        	Sleep(1000);
        if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                        }
                }
}

void turn_absolute(uint8_t ss, int max_speed, int a,float angle)
{
				
		if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
				get_sensor_value0(sn_gyr,&result);
		}
		
		set_tacho_command_inx( ss, TACHO_RESET );
		set_tacho_stop_action_inx( ss, TACHO_COAST );
		set_tacho_speed_sp( ss, a*max_speed * 1 / 10 );
		
		set_tacho_ramp_up_sp( ss, 0 );
		set_tacho_ramp_down_sp( ss, 0 );
		set_tacho_command_inx( ss, TACHO_RUN_FOREVER );
		Sleep(100);
		
		int b;
		if(sn[0]==ss) b=-1;
		else b=1;
		if(a==1){
			while(b*result < b*(initial_ang+b*(angle-5))){
				if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
				get_sensor_value0(sn_gyr,&result);
				//printf( "  value %f\n",result);
				}
				fflush( stdout );
		
		
			}
		}else{
			while(b*result > b*(initial_ang-b*(angle-5))){
				if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
				get_sensor_value0(sn_gyr,&result);
				//printf( "  value %f\n",result);
				}
				fflush( stdout );

			}
		}
		set_tacho_command_inx( ss, TACHO_STOP);
		Sleep(1000);

		if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );
		

		if(a==1){
                        while(b*result < b*(initial_ang+b*(angle-3))){
                                if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );
				
				set_tacho_command_inx( ss, TACHO_RESET );
				 set_tacho_stop_action_inx( ss, TACHO_COAST );
    	           		set_tacho_speed_sp( ss, a*max_speed * 1 / 10 );
				set_tacho_time_sp(ss,100);
               		 	set_tacho_ramp_up_sp( ss, 0 );
               		 	set_tacho_ramp_down_sp( ss, 0 );
               		 	set_tacho_command_inx( ss, TACHO_RUN_TIMED );
        	        	Sleep(1000);
		

                        }

			while(b*result > b*(initial_ang+b*(angle-3))){
                                if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );

                                set_tacho_command_inx( ss, TACHO_RESET );
                                 set_tacho_stop_action_inx( ss, TACHO_COAST );
                                set_tacho_speed_sp( ss, -1*a*max_speed * 1 / 10 );
                                set_tacho_time_sp(ss,100);
                                set_tacho_ramp_up_sp( ss, 0 );
                                set_tacho_ramp_down_sp( ss, 0 );
                                set_tacho_command_inx( ss, TACHO_RUN_TIMED );
                                Sleep(500);


                        }

                }else{
                        while(b*result > b*(initial_ang-b*(angle-3))){
                                if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );
				set_tacho_command_inx( ss, TACHO_RESET );
				 set_tacho_stop_action_inx( ss, TACHO_COAST );
    	           		set_tacho_speed_sp( ss, a*max_speed * 1 / 10 );
				set_tacho_time_sp(ss,100);
               		 	set_tacho_ramp_up_sp( ss, 0 );
               		 	set_tacho_ramp_down_sp( ss, 0 );
               		 	set_tacho_command_inx( ss, TACHO_RUN_TIMED );
        	        	Sleep(1000);

                        }
			while(b*result < b*(initial_ang-b*(angle-3))){
                                if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&result);
                                //printf( "  value %f\n",result);
                                }
                                fflush( stdout );
				set_tacho_command_inx( ss, TACHO_RESET );
				 set_tacho_stop_action_inx( ss, TACHO_COAST );
    	           		set_tacho_speed_sp( ss, -1*a*max_speed * 1 / 10 );
				set_tacho_time_sp(ss,100);
               		 	set_tacho_ramp_up_sp( ss, 0 );
               		 	set_tacho_ramp_down_sp( ss, 0 );
               		 	set_tacho_command_inx( ss, TACHO_RUN_TIMED );
        	        	Sleep(1000);

                        }
                }
}
//a=1 release the ball; else grab;
void control_ball(uint8_t s3,int a){

		set_tacho_stop_action_inx( s3, TACHO_COAST );
		set_tacho_speed_sp( s3, a*maxmedium * 1 / 5 );
		set_tacho_time_sp( s3, 600 );
		set_tacho_ramp_up_sp( s3, 0 );
		set_tacho_ramp_down_sp( s3, 0 );
		set_tacho_command_inx( s3, TACHO_RUN_TIMED );
		Sleep(300);
		do {
			get_tacho_state_flags( s3, &state );
		} while ( state );


}
int main( void )
{
#ifndef __ARM_ARCH_4T__
	/* Disable auto-detection of the brick (you have to set the correct address below) */
	ev3_brick_addr = "192.168.0.204";

#endif
	if ( ev3_init() == -1 ) return ( 1 );

#ifndef __ARM_ARCH_4T__
	printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );

#else
	printf( "Waiting tacho is plugged...\n" );

#endif
	while ( ev3_tacho_init() < 1 ) Sleep( 1000 );
	ev3_sensor_init();
	printf( "*** ( EV3 ) Hello! ***\n" );

	printf( "Found tacho motors:\n" );
	for ( i = 0; i < DESC_LIMIT; i++ ) {
		if ( ev3_tacho[ i ].type_inx != TACHO_TYPE__NONE_ ) {
			printf( "  type = %s\n", ev3_tacho_type( ev3_tacho[ i ].type_inx ));
			printf( "  port = %s\n", ev3_tacho_port_name( i, s ));
			printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
		}
	}
	//Run motors in order from port A to D
	int port1=65,port2=66,port3=68;
	uint8_t s1,s2,s3;
	
	if ( ev3_search_tacho_plugged_in(port1,0, &s1, 0 ) && ev3_search_tacho_plugged_in(port2,0, &s2, 0 ) && ev3_search_tacho_plugged_in(port3,0, &s3, 0 )) {
		int max_speed;
		sn[0]=s1;sn[1]=s2;
		if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0)) {
                                get_sensor_value0(sn_gyr,&initial_ang);
                                printf( "  value initial angle: %f\n",initial_ang);
                                }
                                fflush( stdout );
		multi_set_tacho_command_inx( sn, TACHO_RESET );
		printf( "2 LEGO_EV3_M_MOTORs  are found, run for 5 sec...\n" );
		get_tacho_max_speed( sn[0], &max_speed );
		printf("  max speed = %d\n", max_speed );
    get_tacho_max_speed( s3, &maxmedium );
    printf("  max medium = %d\n", maxmedium );
    //********go near first obstacle****************
		forwarduntil(max_speed,100);
    Sleep(1000);
    //********first turn to the 7bal****************
    turn_relative(s2,max_speed,1,90);
    Sleep(1000);
    //********go to the 7Bal****************
    Gostraight(3114,max_speed,1);
    Sleep(1000);
    //********turn to the second(2) obstacle****************
    turn_relative(s1,max_speed,1,90);
		Sleep(1000);
    //********go near second(2) obstacle****************
    forwarduntil(max_speed,100);
    Sleep(1000);
    //********direction to release the ball****************
    turn_relative(s1,max_speed,-1,90);
    Sleep(1000);
    //********go marche arrière ****************
    Gostraight(2800,max_speed,-1);
		Sleep(2000);
    //********release the ball ****************
    control_ball(s3,1);
		Sleep(3000);
    //********go marche arrière ****************
    Gostraight(692,max_speed,-1);
		Sleep(1000);
    //********direction to final destination****************
    turn_relative(s2,max_speed,-1,89);
    Sleep(1000);
    //********release the ball ****************
    control_ball(s3,-1);
    Sleep(1000);
    //********go forward until destination ****************
    Gostraight(11487,max_speed,1);
		Sleep(1000);
	} else {
		printf( "LEGO_EV3_M_MOTOR 3 is NOT found\n" );
	} 
		
		
	
	

	ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );

	return ( 0 );
}
