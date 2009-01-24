#include "dynamixel.hh"

#include <map>
#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <sstream>

#include <termio.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

using namespace std;

    DXServo::DXServo()
    : IODriver(MAX_PACKET_SIZE)
    , m_baudrate(57600)
      , m_timeout(1000)
{
}

DXServo::~DXServo()
{
    if (isValid())
        close();
}

bool DXServo::close() {
    IODriver::close();
    return true;
}

    bool DXServo::open(std::string const& filename) {
        if (! IODriver::openSerial(filename, 57600))
            return false;

        return true;
    }


bool DXServo::setMovement(unsigned char servo_id, const DxMovement& movement) {
    unsigned char cmd_buf[CMD_BUF_SIZE];
    unsigned char cmd_buf_size;

    dxGetWriteMovementCommand(cmd_buf, &cmd_buf_size, servo_id, &movement);

    if( cmd_buf_size > 0 ) { 
        writePacket(cmd_buf, cmd_buf_size, m_timeout);
    }

    //cout << m_timeout;
    int result_size = readPacket(cmd_buf, CMD_BUF_SIZE, m_timeout);

    return result_size > 0;
}

bool DXServo::setInitialData(unsigned char servo_id, const DxInitial& initial) {
    unsigned char cmd_buf[CMD_BUF_SIZE];
    unsigned char cmd_buf_size;

    dxGetWriteCommand(cmd_buf, &cmd_buf_size, servo_id, 26, (unsigned char*)&initial, sizeof(DxInitial) );

    if( cmd_buf_size > 0 ) { 
        writePacket(cmd_buf, cmd_buf_size, m_timeout);
    }

    int result_size = readPacket(cmd_buf, CMD_BUF_SIZE, m_timeout);

    return result_size > 0;
}


bool DXServo::getPresentValues(unsigned char servo_id, DxPresentValues *values) {
    unsigned char cmd_buf[CMD_BUF_SIZE];
    unsigned char cmd_buf_size;

    dxGetReadPresentCommand(cmd_buf, &cmd_buf_size, servo_id);

    if( cmd_buf_size > 0 ) { 
        writePacket(cmd_buf, cmd_buf_size, m_timeout); 
    }

    int result_size = readPacket(cmd_buf, CMD_BUF_SIZE, m_timeout);

    dxGetPresent(cmd_buf, values);

    return result_size > 0;
}


int DXServo::extractPacket(uint8_t const* buffer, size_t buffer_size) const {
    // find packet marker first
    // minimum packet size is 6 bytes            
    if( buffer_size >= 6 ) {
        int length = dxGetStatusLength(buffer, buffer_size);
//        cerr << buffer_size << " " << length << std::endl;

        if( length != 0 ) {
            return length; 
        }
    }

    // find first 0xff 0xff marker
    unsigned int i=0;
    while(i<(buffer_size-1) && buffer[i] != 0xff && buffer[i+1] != 0xff ) i++;
    return -i;
}
