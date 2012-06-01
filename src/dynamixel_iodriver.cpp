/// \file dynamixel_iodriver.cpp

#include "dynamixel_iodriver.h"

#include "base/logging.h"

/////////////////////////////// PUBLIC ///////////////////////////////////////
DynamixelIODriver::DynamixelIODriver() : iodrivers_base::Driver(cMaxPacketSize)
{
    mTimeout = cDefaultTimeout_ms;
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
    iodrivers_base::Driver::close();
    return true;
}

bool DynamixelIODriver::open(std::string const& filename_, int baudrate_) {
    try {
        iodrivers_base::Driver::openSerial(filename_, baudrate_);
    } catch (std::runtime_error& e) {
	    LOG_ERROR("Could not connect to port %s with baudrate %d",
                filename_.c_str(), baudrate_);
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

    for(unsigned int i=0; i<buffer_size; i++) {
        LOG_DEBUG("Read 0x%x(%d)", buffer[i], buffer[i]);
    }

    // get at least '0xFF 0xFF ID LENGTH'          
    if(buffer_size >= 6) 
    {
        int length = dxGetStatusLength(buffer, buffer_size);
        LOG_DEBUG("status packet length 0x%x(%d)", length, length);

        // 0: packet doesnt start at the start of the buffer or packet incomplete
        //<0: invalid checksum
        //>0: length of the packet
        if(length != 0) 
        {
            if(length < 0) {
                LOG_ERROR("invalid checksum detected, packet will be discarded");
                return length; // Discard complete packet.
            } else {
                // complete packet available?
                if(buffer_size >= (unsigned int)length) {
                    LOG_DEBUG("complete packet available");
                    return length; //packet in the buffer, returns the packet size
                } else {
                    LOG_DEBUG("packet incomplete, need more data");
                    return 0; // need more data
                }
            }
        }
    }

    LOG_DEBUG("buffer size %d", buffer_size);

    // find first 0xff marker
    // The previous implementation got problems with 0 0xff
    unsigned int i=0;
    while(i<buffer_size && buffer[i] != 0xff)
    {
        LOG_DEBUG("Ignore byte 0x%x (%d)", buffer[i], buffer[i]);
        i++;
    }
    // Remove 0xff as well if the next byte is available and if it is not 0xff.
    if(i < (buffer_size - 1) && buffer[i+1] != 0xff) {
        LOG_DEBUG("Ignore byte 0x%x (%d)", buffer[i], buffer[i]);
        i++;
    } 
    LOG_DEBUG("return byte 0x%x (%d)", -i, -i);
    int ret = (int)i;
    return -ret;
}
