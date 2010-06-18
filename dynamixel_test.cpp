#include <inttypes.h>
#include <stdlib.h>

#include <iostream>

#include "dynamixel.h"

/**
 * Requests and prints the control table and (optional) moves the servo to the passed position.\n
 * Usage: dynamixel_test <device> <servo_id> (optional: pos).\n
 * \return 0 if success, 1 if the servo could not be initialized, 2 if the control table \n
 * could not be read and 3 if the goal position could not be set.
 */
int main (int argc, const char** argv){
    std::cout << "Dynamixel test" << std::endl;

    if (argc<3 || argc>5){
        std::cout << "Usage: dynamixel_test <device> <servo_id> <baudrate> (optional: pos_degree)" << std::endl;
        return 0;
    }

    int id = atoi(argv[2]);
    int baud = atoi(argv[3]);
    std::cout << "Testing servo with id " << id << " on device " << argv[1] << " and baudrate " << baud << std::endl;

    Dynamixel dynamixel_;
    dynamixel_.addServo(id);
    dynamixel_.setServoActive(id);

    struct Dynamixel::Configuration dynamixel_config;
    dynamixel_config.mFilename= (argv[1]);
    dynamixel_config.mBaudrate = baud;
    dynamixel_.setTimeout(100);

    if(!dynamixel_.init(&dynamixel_config))
    {
        std::cerr << "cannot open device." << std::endl;
        perror("errno is");
        return 1;
    }

    if(!dynamixel_.readControlTable())
    {
        std::cerr << "readControlTable" << std::endl;
        perror("errno is");
        return 2;
    } else {
        std::cout << dynamixel_.getControlTableString() << std::endl;
    }

	uint16_t retVal;
	dynamixel_.getControlTableEntry("Status Return Level", &retVal);
	if( retVal != 2 ) {
		std::cout << "I have to set Status Return Level to \"always respond\", so I can work with this servo like i'm supposed to do" << std::endl;
		dynamixel_.setControlTableEntry("Status Return Level", 2);
	}

    if(argc == 5)
    {
        uint16_t pos_ = (uint16_t)atoi(argv[4]);
        std::cout << pos_ << std::endl;
        if(!dynamixel_.setGoalPosition(pos_))
        {
            std::cerr << "setGoalPosition" << std::endl;
            perror("errno is");
            return 3;
        } else {
            uint16_t present_pos_ = 0;
            do
            {
                dynamixel_.getPresentPosition(&present_pos_);
                std::cout << "present position is " <<  present_pos_ << std::endl;
            } while(present_pos_ < pos_-3 || present_pos_ > pos_+3);
        }
    }
    return 0;
}
