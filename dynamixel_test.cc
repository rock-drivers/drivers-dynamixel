#include "dynamixel.hh"
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>

using namespace std;

int main (int argc, const char** argv){
    printf("Dynamixel test\n");

    if (argc<4){
        printf( "Usage: dynamixel_test <device> <servo_id> <pos>\n");
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

 //   DxInitial init = { 10, 10, 10, 10, 0xff, 0xff, 0xff, 0xff };

  //  servo.setInitialData( id, init );

   DxMovement move = { pos, 200, 200 }; 

    servo.setMovement( id, move );

    servo.close();
}

