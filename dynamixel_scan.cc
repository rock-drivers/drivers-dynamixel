#include "dynamixel.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

using namespace std;

int main (int argc, const char** argv){
    printf("Dynamixel test\n");

    if (argc<4){
        printf( "Usage: dynamixel_test <device> <servo_id> <pos> [<end> <step> <delay>]\n");
        return 0;
    }

    DXServo servo;
    
    if (!servo.open(argv[1]))
    {
        cerr << "cannot open device." << endl;
        perror("errno is");
        return 1;
    }

    int id = atoi(argv[2]);
    int pos = atoi(argv[3]);
    int end = pos, step = 1, delay = 200;
    
    if( argc == 7 ) {
        end = atoi(argv[4]);
        step = atoi(argv[5]);
        delay = atoi(argv[6]);
    }

    while( pos <= end ) {
        DxMovement move = { pos, 100, 1000 }; 

        servo.setMovement( id, move );

        
        DxPresentValues values;

        bool hold_time = true;
        int last_pos = 0;
        do {
            usleep( delay * 1000 );

            servo.getPresentValues( id, &values );

            timeval tv;
            gettimeofday(&tv, NULL);
            cout << static_cast<uint64_t>(tv.tv_sec) * 1000 + static_cast<uint64_t>(tv.tv_usec) / 1000 << " ";

            cout << "pos:" << values.presentPosition 
                << " speed:" << values.presentSpeed
                << " load:" << values.presentLoad
                << " voltage:" << (short)values.presentVoltage
                << " temp:" << (short)values.presentTemperature
                << endl;
            hold_time = last_pos != values.presentPosition;
            last_pos = values.presentPosition;
        } while( hold_time );

        pos += step;

    }

    servo.close();
}

