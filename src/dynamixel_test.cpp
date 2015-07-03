#include <inttypes.h>
#include <stdlib.h>

#include <iostream>
#include <getopt.h>

#include "dynamixel.h"

#define DYNAMIXEL_SUCCESS 0
#define DYNAMIXEL_NO_INIT 1
#define DYNAMIXEL_NO_TABLE 2
#define DYNAMIXEL_NO_WRITE 3

/**
 * Requests and prints the control table and (optional) moves the servo to the passed position.\n
 * Usage: ./dynamixel_test -port /dev/ttyUSB0 -id 1 -baud 115200 -connect -p 200 -w 100 -p 850 \n
 * \return 0 if success, 1 if the servo could not be initialized, 2 if the control table \n
 * could not be read and 3 some ControlTable Vlaues could not be written
 */
int main (int argc, char** argv){
    std::cout << "Dynamixel test" << std::endl;

	// sensible default-values
    int id = 1;
    int baud = 57600;
    std::string portname("/dev/ttyUSB0");
    Dynamixel dynamixel_;
    struct Dynamixel::Configuration dynamixel_config;
	int idx = 0;
	uint16_t retVal;
	bool connected = false;

//-------------------
// parsing of cmdline-options
//-------------------
       int c;
       while (1)
         {
           static struct option long_options[] =
             {
               /* These options set a flag. */
               {"help",      no_argument, 0, '?'},
               {"id",        required_argument, 0, 'i'},
               {"baud",      required_argument, 0, 'b'},
               {"port",      required_argument, 0, 'o'},
               {"connect",   no_argument,       0, 'c'},
               {"CW_angle",  required_argument, 0, 'a'},
               {"CCW_angle", required_argument, 0, 'A'},
               {"CW_margin", required_argument, 0, 'm'},
               {"CCW_margin",required_argument, 0, 'M'},
               {"CW_slope",  required_argument, 0, 's'},
               {"CCW_slope", required_argument, 0, 'S'},
               {"punch",     required_argument, 0, 'u'},
               {"max_speed", required_argument, 0, 'e'},
               {"max_torque",required_argument, 0, 't'},
               {"position",  required_argument, 0, 'p'},
               {"enable",    no_argument, 0, 'E'},
               {"disable",   no_argument, 0, 'D'},
               {"wait",      required_argument, 0, 'w'},
               {0, 0, 0, 0}
             };
           /* getopt_long stores the option index here. */
           int option_index = 0;

           c = getopt_long_only (argc, argv, "i:b:o:ca:A:m:M:s:S:u:e:t:p:EDw:", long_options, &option_index);

           /* Detect the end of the options. */
           if (c == -1)
             break;

           switch (c)
             {
             case 0:
               /* If this option set a flag, do nothing else now. */
               if (long_options[option_index].flag != 0)
                 break;
               printf ("option %s", long_options[option_index].name);
               if (optarg)
                 printf (" with arg %s", optarg);
               printf ("\n");
               break;

             case 'i':
				id = atoi(optarg);
				std::cout << "will talk to servo with id " << id << std::endl;
               break;

             case 'b':
				baud = atoi(optarg);
				std::cout << "will use baudrate of " << baud << std::endl;
               break;
             case 'o':
				portname = optarg;
				std::cout << "will use serial port " << portname << std::endl;
               break;
             case 'c':
				std::cout << "Connecting to servo with id " << id << " on device " << portname << " and baudrate " << baud << std::endl;

				dynamixel_.addServo(id);
				dynamixel_.setServoActive(id);

				dynamixel_config.mFilename = portname.c_str();
				dynamixel_config.mBaudrate = baud;
				dynamixel_.setTimeout(100);

				if(!dynamixel_.init(&dynamixel_config))
				{
					std::cerr << "cannot open device." << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_INIT;
				}
/*
				if(!dynamixel_.readControlTable())
				{
					std::cerr << "readControlTable" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_INIT;
				} else {
					std::cout << dynamixel_.getControlTableString() << std::endl;
				}

				dynamixel_.getControlTableEntry("Status Return Level", &retVal);
				if( retVal != 2 ) {
					std::cout << "I have to set Status Return Level to \"always respond\", so I can work with this servo like i'm supposed to do" << std::endl;
					dynamixel_.setControlTableEntry("Status Return Level", 2);
				}
*/
				connected = true;

               break;

             case 'p':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if(!dynamixel_.setGoalPosition((uint16_t)atoi(optarg)))
				{
					std::cerr << "setGoalPosition" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					uint16_t present_pos_ = 0;
					do
					{
						dynamixel_.getPresentPosition(&present_pos_);
						std::cout << "present position is " <<  present_pos_ << std::endl;
					} while(present_pos_ < (uint16_t)atoi(optarg)-3 || present_pos_ > (uint16_t)atoi(optarg)+3);
				}
               break;

             case 'a':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if (!dynamixel_.getControlTableEntry("CCW Angle Limit",&retVal))
				{
					std::cerr << "set CW_angle" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				}
				if ((uint16_t)atoi(optarg) > retVal)
					std::cerr << "invalid angle, new left margin is greater than right one..." << std::endl;

				if(!dynamixel_.setControlTableEntry("CW Angle Limit", (uint16_t)atoi(optarg)))
				{
					std::cerr << "set CW_angle" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "set CW_angle Limit to " << (uint16_t)atoi(optarg) << std::endl;
				}
               break;

             case 'A':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if (!dynamixel_.getControlTableEntry("CW Angle Limit",&retVal))
				{
					std::cerr << "set CCW_angle" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				}
				if ((uint16_t)atoi(optarg) < retVal)
					std::cerr << "invalid angle, new right margin is smaller than left one..." << std::endl;

				if(!dynamixel_.setControlTableEntry("CCW Angle Limit", (uint16_t)atoi(optarg)))
				{
					std::cerr << "set CCW_angle" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "set CCW_angle Limit to " << (uint16_t)atoi(optarg) << std::endl;
				}
               break;

             case 'm':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if(!dynamixel_.setControlTableEntry("CW Compliance Margin", (uint16_t)atoi(optarg)))
				{
					std::cerr << "CW Compliance Margin" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "set CCW Compliance Margin to " << (uint16_t)atoi(optarg) << std::endl;
				}
               break;

             case 'M':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if(!dynamixel_.setControlTableEntry("CW Compliance Margin", (uint16_t)atoi(optarg)))
				{
					std::cerr << "set CCW Compliance Margin" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "set CCW Compliance Margin to " << (uint16_t)atoi(optarg) << std::endl;
				}
               break;

             case 's':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if(!dynamixel_.setControlTableEntry("CW Compliance Slope", (uint16_t)atoi(optarg)))
				{
					std::cerr << "CW Compliance Slope" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "set CCW Compliance Slope to " << (uint16_t)atoi(optarg) << std::endl;
				}
               break;

             case 'S':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if(!dynamixel_.setControlTableEntry("CW Compliance Slope", (uint16_t)atoi(optarg)))
				{
					std::cerr << "set CCW Compliance Slope" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "set CCW Compliance Slope to " << (uint16_t)atoi(optarg) << std::endl;
				}
               break;

             case 'u':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if(!dynamixel_.setControlTableEntry("Punch", (uint16_t)atoi(optarg)))
				{
					std::cerr << "set punch" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "set punch to " <<  (uint16_t)atoi(optarg) << std::endl;
				}
               break;

             case 'e':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if(!dynamixel_.setControlTableEntry("Moving Speed", (uint16_t)atoi(optarg)))
				{
					std::cerr << "set Moving Speed" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "set Moving Speed to " <<  (uint16_t)atoi(optarg) << std::endl;
				}
               break;

             case 't':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if(!dynamixel_.setControlTableEntry("Torque Limit", (uint16_t)atoi(optarg)))
				{
					std::cerr << "set Torque Limit" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "set Torque Limit to " <<  (uint16_t)atoi(optarg) << std::endl;
				}
               break;

             case 'E':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if(!dynamixel_.setControlTableEntry("Torque Enable", 1))
				{
					std::cerr << "enable torque" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "enabled torque" << std::endl;
				}
               break;

             case 'D':
				if (!connected) {
					std::cerr << "please connect first, using \"-connect\" in commandline" << std::endl;
					return DYNAMIXEL_NO_INIT;
				}

				if(!dynamixel_.setControlTableEntry("Torque Enable", 0))
				{
					std::cerr << "disable torque" << std::endl;
					perror("errno is");
					return DYNAMIXEL_NO_WRITE;
				} else {
					std::cout << "disabled torque" << std::endl;
				}
               break;

             case 'w':
				std::cout << "waiting for " << atoi(optarg) << " ms..." << std::endl;
				usleep( 1000*atoi(optarg) );
				std::cout << " ... done" << std::endl;
               break;

             case '?':

				std::cout << "connect has to be done before any move-commands are executed!" << std::endl;
				std::cout << "valid options:" << std::endl;
				while (long_options[idx].name != 0){
					std::cout << "\t"<< long_options[idx++].name << std::endl;
				}

               break;

             default:
               abort ();
             }
         }

       if (optind < argc)
         {
           std::cout << "non-option ARGV-elements: " << std::endl;
           while (optind < argc)
             std::cout << argv[optind++] <<std::endl;
         }

    return DYNAMIXEL_SUCCESS;
}
