/// \file dynamixel_iodriver.cpp

#include "dynamixel_iodriver.h"

/////////////////////////////// PUBLIC ///////////////////////////////////////
DynamixelIODriver::DynamixelIODriver() : IODriver(cMaxPacketSize)
{
    mTimeout = cDefaultTimeout;
    mBaudrate = -1;
}

DynamixelIODriver::~DynamixelIODriver()
{
    if (isValid())
    {
        close();
    }
}

bool DynamixelIODriver::close()
{
    IODriver::close();
    return true;
}

bool DynamixelIODriver::open(std::string const& filename_, int baudrate_) {
    if (!IODriver::openSerial(filename_, baudrate_))
    {
        return false;
    }
    mBaudrate = baudrate_;
    return true;
}

/////////////////////////////// PROTECTED ////////////////////////////////////
/*
 * There is four possible cases:
 * - there is no packet in the buffer. In that case, return -buffer_size to
 *   discard all the data that has been gathered until now.
 * - there is the beginning of a packet but it is not starting at the first
 *   byte of \c buffer. In that case, return -position_packet_start, where
 *   position_packet_start is the position of the packet in \c buffer.
 * - a packet begins at the first byte of \c buffer, but the end of the
 *   packet is not in \c buffer yet. Return 0.
 * - there is a full packet in \c buffer, starting at the first buffer byte.
 *   Return the packet size. That data will be copied back to the buffer
 *   given to readPacket.
 */
int DynamixelIODriver::extractPacket(uint8_t const* buffer, size_t buffer_size) const {
    // find packet marker first
    // minimum packet size is 6 bytes            
    if(buffer_size >= 6) 
    {
        int length = dxGetStatusLength(buffer, buffer_size);
        // 0: packet doesnt start at the start of the buffer or packet incomplete
        //<0: invalid checksum
        //>0: length of the packet
        if(length != 0) 
        {
            if(length < 0)
                printf("Dynamixel: Invalid checksum detected!\n");
            return length; //packet in the buffer, returns the packet size
        }
    }

    // find first 0xff 0xff marker
    unsigned int i=0;
    while(i<(buffer_size-1) && 
                buffer[i] != 0xff && 
                buffer[i+1] != 0xff )
    {
        i++;
    }
    return -i;
}
