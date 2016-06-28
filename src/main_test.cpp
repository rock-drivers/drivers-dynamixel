#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <limits>
#include <ios>

#include "dynamixel.h"

enum DynamixelMenu {MAIN, CONFIGURATION, POSITION};

void printMainMenu() {
    std::cout << "Menu Main" << std::endl;
    std::cout << "1. Configuration" << std::endl;
    std::cout << "2. Position" << std::endl;
    std::cout << "3. Exit" << std::endl;
}

void printConfigurationMenu() {
    std::cout << "Menu v4l2" << std::endl;
    std::cout << "1. Read control table" << std::endl;
    std::cout << "2. Print control table" << std::endl;
    std::cout << "3. Set control table entry" << std::endl;
    std::cout << "4. Back" << std::endl;
}

void printPositionMenu() {
    std::cout << "Menu GStreamer" << std::endl;
    std::cout << "1. Get position (steps)" << std::endl;
    std::cout << "2. Set position (steps)" << std::endl;
    std::cout << "3. Switch servo off and print positions" << std::endl;
    std::cout << "4. Back" << std::endl;
}

int getRequest(int start, int stop) {
    int value = -1;
    int ret = 0;
    while(value < start || value > stop) {
        std::cout << "Choose (" << start << " - " << stop << "): ";
        ret = scanf("%d", &value);
        if(ret == 0) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    return value;
}

float getRequestf(float start, float stop) {
    float value = -1;
    int ret = 0;
    while(value < start || value > stop) {
        std::cout << "Choose (" << start << " - " << stop << "): ";
        ret = scanf("%f", &value);
        if(ret == 0) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    return value;
}

int main(int argc, char* argv[]) 
{
    int min_pos_step = 0;
    int max_pos_step = 0xfff;    

    if((argc == 2 && strcmp(argv[1], "-h") == 0) || argc < 1 || argc > 3) {
        std::cout << "Dynamixel test program to read and write control values and servo positions" << std::endl;
        std::cout << "dynamixel_control_bin <id> <uri>" << std::endl;
        std::cout << "default: dynamixel_control_bin 1 serial:///dev/tty/USB0:57600" << std::endl;
        return 0;
    }

    int id = 1;
    std::string device = "serial:///dev/ttyUSB0:57600";

    if(argc == 3) {
        id = atoi(argv[1]);
        device = argv[2];
    }

    int ret = 0;

    DynamixelMenu dynamixel_menu = MAIN;

    Dynamixel dynamixel;

    if(!dynamixel.addServo(id)) {
        std::cout << "Servo " << id << "could not be added" << std::endl;
    }
    if(!dynamixel.init(device)) {
        std::cout << "Connection (uri " << device << ") could not be opened" << std::endl;
    }

    dynamixel.setNumberRetries(1);

    while(true) {
        switch(dynamixel_menu) {
            case MAIN: {
                printMainMenu();
                ret = getRequest(1, 3);
                switch(ret) {
                    case 1: {
                        dynamixel_menu = CONFIGURATION;
                        break;
                    }
                    case 2: {
                        dynamixel_menu = POSITION;
                        break;
                    }
                    case 3: {
                        return 0;
                    }
                }
                break;
            }
            case CONFIGURATION: {
                printConfigurationMenu();
                ret = getRequest(1, 4);
                switch(ret) {
                    case 1: {
                        if(!dynamixel.readControlTable()) {
                            std::cout << "Control table could not be read" << std::endl;
                        }
                        break;
                    }
                    case 2: {
                        std::cout << dynamixel.getControlTableString() << std::endl;
                        break;
                    }
                    case 3: {
                        std::cout << "Which control value should be set?" << std::endl;
                        // dont know why, but the buffer has to be cleared.
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        char name[256];
                        std::cin.getline(name, 256);
                        std::string name_str(name);

                        Dynamixel::ControlTableEntry entry;
                        if(!dynamixel.getControlTableEntry(name_str, entry)) {
                            std::cout << "Control name unknown:" << name_str << std::endl;
                            break;
                        }
                        std::cout << "Define new value (" << entry.mBytes << " byte(s))" << std::endl;
                        uint16_t value = 0;
                        int ret = scanf("%hu",&value);
                        if(ret == 0) {
                            std::cin.clear();
                            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                            break;
                        }
                        std::cout << "Set the control " << entry.mName << " to " << value << 
                                "(0x" << std::hex << value << ")? (yes/no)" <<std::endl;
                        std::string answer;
                        std::cin >> answer;
                        if(answer == "yes") {
                            if(dynamixel.setControlTableEntry(name, value)) {
                                std::cout << "Control value has been changed" << std::endl;
                                break;
                            }
                        }   
                        break;
                    }
                    case 4: {
                        dynamixel_menu = MAIN;
                        break;
                    }
                }
                break;
            }
            case POSITION: {
                printPositionMenu();
                ret = getRequest(1, 4);
                switch(ret) {
                    case 1: {
                        uint16_t pos = 0;
                        if(!dynamixel.getPresentPosition(&pos)) {
                            std::cout << "Position (step) could not be requested" << std::endl;
                        } else {
                            std::cout << "Position (step) is " << pos << std::endl;
                        }
                        break;
                    }
                    case 2: {
                        std::cout << "Define new position (step)" << std::endl;
                        uint16_t pos = getRequest(min_pos_step, max_pos_step);
                        dynamixel.setGoalPosition(pos);
                        break;
                    }
                    case 3: {
                        std::cout << "Interupt with Ctrl+C" << std::endl;
                        dynamixel.setControlTableEntry("Torque Enable", 0);
                        uint16_t pos = 0;
                        while(true) {
                            if(!dynamixel.getPresentPosition(&pos)) {
                                std::cout << "Position could not be requested" << std::endl;
                            }
                            std::cout << "Position is " << pos << " (steps)" << std::endl;
                            sleep(1);
                        }
                        break;
                    }
                    case 4: {
                        dynamixel_menu = MAIN;
                        break;
                    }
                }
                break;
            }
            default: {
                std::cout << "Unknown answer" << std::endl; 
            }
        }
    }
}

