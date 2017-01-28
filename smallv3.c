#include <stdio.h>
#include <stdlib.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

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
#define SERV_ADDR   "38:B1:DB:25:21:18"     /* Whatever the address of the server is */
#define TEAM_ID     13                       /* Your team ID */

#define MSG_ACK     0
#define MSG_NEXT    1
#define MSG_START   2
#define MSG_STOP    3
#define MSG_CUSTOM  4
#define MSG_KICK    5
#define MSG_POSITION 6
#define MSG_BALL 	7

float error=0,correction=0;
int i;
//uint8_t sn;
FLAGS_T state;
uint8_t sn_touch;
uint8_t sn_color;
uint8_t sn_compass;
uint8_t sn_sonar;
uint8_t sn_gyr;
char sss[ 256 ];
int val;
float value;
uint32_t n, ii;
uint8_t sn[2],snmedium;
float result;
int maxmedium;
float initial_ang,min_dist;
bool flag=true;
int first=1;
void debug (const char *fmt, ...)
{
    va_list argp;

    va_start (argp, fmt);

    vprintf (fmt, argp);

    va_end (argp);
}

unsigned char rank = 0;
unsigned char length = 0;
unsigned char previous = 0xFF;
unsigned char next = 0xFF;

int s;

uint16_t msgId = 0;

int read_from_server (int sock, char *buffer, size_t maxSize)
{
    int bytes_read = read (sock, buffer, maxSize);

    if (bytes_read <= 0)
    {
        fprintf (stderr, "Server unexpectedly closed connection...\n");
        close (s);
        exit (EXIT_FAILURE);
    }

    printf ("[DEBUG] received %d bytes\n", bytes_read);

    return bytes_read;
}

void beginner ()
{
    char string[58];
	char type;
    printf ("I'm the beginner...\n");

    /* Send 3 POSITION messages, then a NEXT message */
    int i;
    for (i=0; i<3; i++)
    {
        *((uint16_t *) string) = msgId++;
        string[2] = TEAM_ID;
        string[3] = 0xFF;
        string[4] = MSG_POSITION;
        string[5] = i;          /* x */
        string[6] = 0x00;
        string[7] = i;		/* y */
        string[8]= 0x00;
        write(s, string, 9);
    }
    //BALL drop message
    *((uint16_t *) string) = msgId++;
    string[2] = TEAM_ID;
    string[3] = next;
    string[4] = MSG_BALL;
    string[5] = 0x0;
    string[6] = 0x4;          /* x */
    string[7] = 0x00;
    string[8] = 0x4;		/* y */
    string[9]= 0x00;
    write(s, string, 10);

    //last position message
    *((uint16_t *) string) = msgId++;
    string[2] = TEAM_ID;
    string[3] = 0xFF;
    string[4] = MSG_POSITION;
    string[5] = 0x5;          /* x */
    string[6] = 0x00;
    string[7] = 0x5;		/* y */
    string[8]= 0x00;
    write(s, string, 9);


    
    
 while(1)
    {
        //Wait for other robot to finish
	read_from_server (s, string, 58);
	type = string[4];

        switch (type)
        {
        case MSG_STOP:
	    
            return;
        case MSG_NEXT:
            //send a position
            
            *((uint16_t *) string) = msgId++;
            string[2] = TEAM_ID;
            string[3] = 0xFF;
            string[4] = MSG_POSITION;
            string[5] = 0x00;          /* x */
            string[6]= 0x00;
            string[7] = 0x00;	    /* y */
            string[8] = 0x00;
            write(s, string, 9);
	    rank++;
	    flag=false;
            return;
        default:
            printf ("Ignoring message %d\n", type);
	}
    }
}

void finisher ()
{
    char string[58];
    char type;

    printf ("I'm the finisher...\n");

    /* Get message */
    while (1)
    {
        read_from_server (s, string, 58);
        type = string[4];

        switch (type)
        {
        case MSG_STOP:
	    
            return;
        case MSG_NEXT:
            //send a position
            flag=false;
            *((uint16_t *) string) = msgId++;
            string[2] = TEAM_ID;
            string[3] = 0xFF;
            string[4] = MSG_POSITION;
            string[5] = 0x00;          /* x */
            string[6]= 0x00;
            string[7] = 0x00;	    /* y */
            string[8] = 0x00;
            write(s, string, 9);
	   
    	rank++;
            return;
        default:
            printf ("Ignoring message %d\n", type);
        }
    }
}


void* main_thread(void* unused)
{
    struct sockaddr_rc addr = { 0 };
    int status;

    /* allocate a socket */
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    /* set the connection parameters (who to connect to) */
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba (SERV_ADDR, &addr.rc_bdaddr);

    /* connect to server */
     status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

    /* if connected */
    if( status == 0 )
    {
        char string[58];

        /* Wait for START message */
        read_from_server (s, string, 9);
        if (string[4] == MSG_START)
        {
            printf ("Received start message!\n");
            rank = (unsigned char) string[5];
            next = (unsigned char) string[7];
        }
        flag=false;
	while(1){
        if (rank == 0)
            beginner ();

        else
            finisher ();
	//while(flag);	
	}

        close (s);

        sleep (5);

    }
    else
    {
        fprintf (stderr, "Failed to connect to server...\n");
        sleep (2);
        exit (EXIT_FAILURE);
    }

    close(s);

}


void Gostraight(int time,int max_speed, int a)
{
    multi_set_tacho_stop_action_inx( sn, TACHO_COAST );
    multi_set_tacho_speed_sp( sn, a * max_speed * 1 / 4 );
    multi_set_tacho_time_sp( sn, time );
    multi_set_tacho_ramp_up_sp( sn, 0 );
    multi_set_tacho_ramp_down_sp( sn, 0 );
    multi_set_tacho_command_inx( sn, TACHO_RUN_TIMED );
    /* Wait tacho stop */
    Sleep( 300 );
    do
    {
        get_tacho_state_flags( sn[0], &state );
    }
    while ( state );


}

void forwarduntil(int max_speed,float goal)
{
    value=2000.0;
    if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0))
    {
        //printf("SONAR found, reading sonar...\n");
        if ( !get_sensor_value0(sn_sonar, &value ))
        {
            value = 0;
        }
        //printf( "\r(%f) \n", value);
        fflush( stdout );
    }
    if(value<=goal) return;
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
    while(value>goal)
    {
        if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0))
        {
            //printf("SONAR found, reading sonar...\n");
            if ( !get_sensor_value0(sn_sonar, &value ))
            {
                value = 0;
            }
            //printf( "\r(%f) \n", value);
            fflush( stdout );
        }
    }
    multi_set_tacho_command_inx( sn, TACHO_STOP );


}
void search(uint8_t ss, int max_speed,float angle)
{
    printf("----------------------Search begin----------------------\n");
    float res,dist;
    if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
    {
        get_sensor_value0(sn_gyr,&res);
    }
    if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0))
    {
        printf("SONAR found, reading sonar...\n");
        if ( !get_sensor_value0(sn_sonar, &dist ))
        {
            dist = 0;
        }
        printf( "\r(%f) \n", dist);
        fflush( stdout );
    }
    else
    {
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
    float min=1000,min_ang=res;
    while(b*result < b*(res+b*(angle-error)))
    {
        if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
        {
            get_sensor_value0(sn_gyr,&result);
            printf( "  angle main turn search value %f\n",result);
        }
        fflush( stdout );
        if ( !get_sensor_value0(sn_sonar, &dist ))
        {
            dist = 0;
        }
        printf( "distance seen : (%f) \n", dist);
        if(dist!=0 && min>dist)
        {
            min=dist;
            min_ang=result;


        }

    }
    min_dist=min;
    set_tacho_command_inx( ss, TACHO_STOP);
    printf("\n");
    printf( "************(%f) *********\n", dist);
    Sleep(1000);
    set_tacho_speed_sp( ss, -1*max_speed * 1 / 12 );
    set_tacho_command_inx( ss, TACHO_RUN_FOREVER );
    get_sensor_value0(sn_gyr,&res);
    while(b*result >b*(min_ang))
    {
        if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
        {
            get_sensor_value0(sn_gyr,&result);
            //printf( "  value %f\n",result);
        }
        fflush( stdout );
        if ( !get_sensor_value0(sn_sonar, &dist ))
        {
            dist = 0;
        }
        //printf( "\r(%f) \n", dist);
    }

    set_tacho_command_inx( ss, TACHO_STOP);

    Sleep(1000);

    while(b*result < b*(min_ang-correction))
    {
        if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
        {
            get_sensor_value0(sn_gyr,&result);
            //printf( "  value %f\n",result);
        }
        fflush( stdout );

        set_tacho_command_inx( ss, TACHO_RESET );
        set_tacho_stop_action_inx( ss, TACHO_COAST );
        set_tacho_speed_sp( ss, max_speed * 1 / 12 );
        set_tacho_time_sp(ss,100);
        set_tacho_ramp_up_sp( ss, 0 );
        set_tacho_ramp_down_sp( ss, 0 );
        set_tacho_command_inx( ss, TACHO_RUN_TIMED );
        Sleep(1000);
        if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
        {
            get_sensor_value0(sn_gyr,&result);
            //printf( "  value %f\n",result);
        }

    }

    while(b*result > b*(min_ang-correction))
    {
        if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
        {
            get_sensor_value0(sn_gyr,&result);
            //printf( "  value %f\n",result);
        }
        fflush( stdout );

        set_tacho_command_inx( ss, TACHO_RESET );
        set_tacho_stop_action_inx( ss, TACHO_COAST );
        set_tacho_speed_sp( ss, -1*max_speed * 1 / 12 );
        set_tacho_time_sp(ss,100);
        set_tacho_ramp_up_sp( ss, 0 );
        set_tacho_ramp_down_sp( ss, 0 );
        set_tacho_command_inx( ss, TACHO_RUN_TIMED );
        Sleep(1000);
        if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
        {
            get_sensor_value0(sn_gyr,&result);
            //printf( "  value %f\n",result);
        }

    }


    printf("--------------------------------------------search END----------------------------\n");

}

void turn_relative(uint8_t ss, int max_speed, int a,float angle)
{
    float res;
    if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
    {
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
    if(a==1)
    {
        while(b*result < b*(res+b*(angle-error)))
        {
            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
                get_sensor_value0(sn_gyr,&result);
                //printf( "  value %f\n",result);
            }
            fflush( stdout );


        }
    }
    else
    {
        while(b*result > b*(res-b*(angle-error)))
        {
            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
                get_sensor_value0(sn_gyr,&result);
                //printf( "  value %f\n",result);
            }
            fflush( stdout );

        }
    }
    set_tacho_command_inx( ss, TACHO_STOP);
    Sleep(1000);

    if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
    {
        get_sensor_value0(sn_gyr,&result);
        //printf( "  value %f\n",result);
    }
    fflush( stdout );


    if(a==1)
    {
        while(b*result < b*(res+b*(angle-correction)))
        {
            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
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

            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
                get_sensor_value0(sn_gyr,&result);
                //printf( "  value %f\n",result);
            }
        }

        while(b*result > b*(res+b*(angle-correction)))
        {
            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
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

            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
                get_sensor_value0(sn_gyr,&result);
                //printf( "  value %f\n",result);
            }
        }

    }
    else
    {
        while(b*result > b*(res-b*(angle-correction)))
        {
            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
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
            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
                get_sensor_value0(sn_gyr,&result);
                //printf( "  value %f\n",result);
            }
        }
        while(b*result < b*(res-b*(angle-correction)))
        {
            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
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
            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
                get_sensor_value0(sn_gyr,&result);
                //printf( "  value %f\n",result);
            }
        }
    }
}


//a=1 release the ball; else grab;
void control_ball(uint8_t s3,int a)
{

    set_tacho_stop_action_inx( s3, TACHO_COAST );
    set_tacho_speed_sp( s3, a*maxmedium * 1 / 5 );
    set_tacho_time_sp( s3, 600 );
    set_tacho_ramp_up_sp( s3, 0 );
    set_tacho_ramp_down_sp( s3, 0 );
    set_tacho_command_inx( s3, TACHO_RUN_TIMED );
    Sleep(300);
    do
    {
        get_tacho_state_flags( s3, &state );
    }
    while ( state );


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
    for ( i = 0; i < DESC_LIMIT; i++ )
    {
        if ( ev3_tacho[ i ].type_inx != TACHO_TYPE__NONE_ )
        {
            printf( "  type = %s\n", ev3_tacho_type( ev3_tacho[ i ].type_inx ));
            printf( "  port = %s\n", ev3_tacho_port_name( i, sss ));
            printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
        }
    }
    //Run motors in order from port A to D
    int port1=65,port2=66,port3=68;
    uint8_t s1,s2,s3;
    pthread_t main_thr;
    pthread_create(&main_thr, NULL, main_thread, NULL);
    //pthread_join(main_thr,NULL);
    if ( ev3_search_tacho_plugged_in(port1,0, &s1, 0 ) && ev3_search_tacho_plugged_in(port2,0, &s2, 0 ) && ev3_search_tacho_plugged_in(port3,0, &s3, 0 ))
    {
	char string[58];
        int max_speed;
        sn[0]=s1;
        sn[1]=s2;
	uint8_t inter;
        if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
        {
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
	int i=0;
	while(1){
        while(flag) printf("waiting for value of flag changee.....\n");
	if(i==0 && rank==0) first=0;
	if(i==0 && rank) {inter=s1;s1=s2;s2=inter;}
	
        if(rank%2 == 0)
        {
            Gostraight(5000,max_speed,1);
            Sleep(1000);

            turn_relative(s1,max_speed,1,90.0);
            Sleep(500);
            //GO forward
            Gostraight(700,max_speed,1);
            Sleep(3000);
            control_ball(s3,1);
            //GO BACK
            Gostraight(700,max_speed,-1);

            //turn back left wheel
            Sleep(1000);
            turn_relative(s1,max_speed,-1,90.0);
            Sleep(1000);

            control_ball(s3,-1);
            Sleep(1000);
            forwarduntil(max_speed,100);
            Sleep(1000);
            turn_relative(s1,max_speed,1,90.0);
            Sleep(2000);
            turn_relative(s2,max_speed,-1,90.0);
            Sleep(2000);
		//Next message

	*((uint16_t *) string) = msgId++;
    	string[2] = TEAM_ID;
    	string[3] = next;
    	string[4] = MSG_NEXT;
    	write(s, string, 5);

	flag=true;
	first+=2;

        }
        else
        {
            while(first==1 || flag) printf("waiting for finisher code to be executed.........\n");
            Gostraight(2000,max_speed,1);
            Sleep(1000);
            float before_search,after_search;
            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
                get_sensor_value0(sn_gyr,&before_search);
                printf( "  value before_search %f\n",before_search);
            }
            fflush( stdout );
            Sleep(500);
            search(s2,max_speed,60);
            Sleep(500);
            while(min_dist>250)
            {

                Gostraight(692,max_speed,1);
                Sleep(1000);
                search(s1,max_speed,5);
                Sleep(1000);
                search(s2,max_speed,5);
                Sleep(2000);
            }
            Sleep(1000);
            forwarduntil(max_speed,100);
            Sleep(2000);

            get_tacho_max_speed( s3, &maxmedium );
            set_tacho_stop_action_inx( s3, TACHO_COAST );
            set_tacho_speed_sp( s3, maxmedium * 1 / 5 );
            set_tacho_time_sp( s3, 800);
            set_tacho_ramp_up_sp( s3, 0 );
            set_tacho_ramp_down_sp( s3, 0 );
            set_tacho_command_inx( s3, TACHO_RUN_TIMED );
            Sleep(300);
            do
            {
                get_tacho_state_flags( s3, &state );
            }
            while ( state );
            Sleep( 3000 );
            Gostraight(1100,max_speed,1);


            set_tacho_stop_action_inx( s3, TACHO_COAST );
            set_tacho_speed_sp( s3, -1*maxmedium * 1 / 5 );
            set_tacho_time_sp( s3, 300);
            set_tacho_ramp_up_sp( s3, 0 );
            set_tacho_ramp_down_sp( s3, 0 );
            set_tacho_command_inx( s3, TACHO_RUN_TIMED );
            Sleep(300);
            do
            {
                get_tacho_state_flags( s3, &state );
            }
            while ( state );
            Sleep( 3000 );
            if ( ev3_search_sensor( LEGO_EV3_GYRO, &sn_gyr ,0))
            {
                get_sensor_value0(sn_gyr,&after_search);
                printf( "  value after_search %f\n",after_search);
            }
            fflush( stdout );
            turn_relative(s1,max_speed,1,fabs(before_search-after_search));
            Sleep(1000);
            //turn_relative(s1,max_speed,1,90);
            //search for direction
            search(s1,max_speed,90);
            Sleep(1000);
            if(min_dist>200.0)
            {

                Gostraight(692,max_speed,1);
                Sleep(1000);
                search(s1,max_speed,20);
                Sleep(1000);
                search(s2,max_speed,20);
                Sleep(1000);
            }

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
            while(value>200)
            {
                if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0))
                {
                    //printf("SONAR found, reading sonar...\n");
                    if ( !get_sensor_value0(sn_sonar, &value ))
                    {
                        value = 0;
                    }
                    //printf( "\r(%f) \n", value);
                    fflush( stdout );
                }
            }
            multi_set_tacho_command_inx( sn, TACHO_STOP );
            Sleep(2000);

	    //turn to final destination
            turn_relative(s2,max_speed,1,90);
            Sleep(1000);
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
            while(value>100)
            {
                if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0))
                {
                    //printf("SONAR found, reading sonar...\n");
                    if ( !get_sensor_value0(sn_sonar, &value ))
                    {
                        value = 0;
                    }
                    //printf( "\r(%f) \n", value);
                    fflush( stdout );
                }
            }
            multi_set_tacho_command_inx( sn, TACHO_STOP );
            Sleep(1000);
	    
            turn_relative(s2,max_speed,1,90.0);
            Sleep(2000);
            turn_relative(s1,max_speed,-1,90.0);
	    
            Sleep(2000);
	    //beginner state turn
	    Gostraight(5000,max_speed,1);
            Sleep(1000);

            turn_relative(s1,max_speed,1,90.0);
            Sleep(500);
            //GO forward
            Gostraight(700,max_speed,1);
            Sleep(3000);
            control_ball(s3,1);
            //GO BACK
            Gostraight(700,max_speed,-1);

            //turn back left wheel
            Sleep(1000);
            turn_relative(s1,max_speed,-1,90.0);
            Sleep(1000);

            control_ball(s3,-1);
            Sleep(1000);
            forwarduntil(max_speed,100);
            Sleep(1000);
            turn_relative(s1,max_speed,1,90.0);
            Sleep(2000);
            turn_relative(s2,max_speed,-1,90.0);
            Sleep(2000);
	 //Next message

	*((uint16_t *) string) = msgId++;
    	string[2] = TEAM_ID;
    	string[3] = next;
    	string[4] = MSG_NEXT;
    	write(s, string, 5);
	flag=true;
        }
        Sleep(1000);
	i++;
	}
    }
    else
    {
        printf( "LEGO_EV3_M_MOTOR 3 is NOT found\n" );
    }





    ev3_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );

    return ( 0 );
}

