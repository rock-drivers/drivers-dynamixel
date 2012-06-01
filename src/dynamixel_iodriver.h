/**
 * \file dynamixel_iodriver.h
 *  
 * \brief   Inherits from iodrivers_base::Driver and specialized the serial communication
 *          for the dynamixel servos.
 *
 * \details Implements the virtual function extractPacket(), see for details.          
 *      
 *          German Research Center for Artificial Intelligence\n
 *          Project: AG Framework, Spaceclimber
 *
 * \date    15.12.2009
 *
 * \author  Stefan.Haase@dfki.de
 */

#ifndef DYNAMIXEL_IODRIVER_H_
#define DYNAMIXEL_IODRIVER_H_

#include <iodrivers_base/Driver.hpp>

extern "C" {
#include <dxseries.h>
}

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)


/**
 * \class DynamixelIODriver
 * See file description for details.
 */
class DynamixelIODriver : public iodrivers_base::Driver
{
 public:
    DynamixelIODriver();
    /**
     * Closes the serial communication.
     */
    ~DynamixelIODriver();        
    /**
     * Invokes the close functions of IODriver and closes the serial connection.
     */        
    bool close();
    /**
     * Returns the variable \a mTimeout, which represents the time in ms to wait 
     * for a serial answer.
     */
    inline int getTimeout() const
    {
        return mTimeout;
    }
    /**
     * Invokes the open functions of IODrivers.
     * \param filename_ Serial device like \a /dev/ttyUSB0.
     * \param baudrate_ Baudrate, default is 57600.
     */
    bool open(std::string const& filename_, int baudrate_ = cDefaultBaudRate);
    /**
     * Invokes the function readPacket of IODriver with the timeout \a mTimeout.
     */
    inline int readPacket(uint8_t* buffer_, int buffer_size)
    {
        return iodrivers_base::Driver::readPacket(buffer_, buffer_size, mTimeout);
    }
    /**
     * Sets the timeout which represents the time in ms to wait for a serial answer.
     */
    inline void setTimeout(int const timeout_)
    {
        mTimeout = timeout_;
    }
    /**
     * Invokes the function writePacket of IODriver with the timeout \a mTimeout.
     */
    inline bool writePacket(uint8_t const* buffer_, int buffer_size)
    {
        return iodrivers_base::Driver::writePacket(buffer_, buffer_size, mTimeout);
    }
 protected:
    /**
     * Main function which extracts a packet out of the serial data stream.
     * \param buffer Serial data buffer which contains the bytes sent by the serial device.
     * \param buffer_size Number of the bytes in the \a buffer.
     * \return See the documentation in IODriver for details or take a look at dynamixel_iodriver.cpp.
     */
    int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

 private:
    static const int cMaxPacketSize = 56; ///maximal size of a packet
    static const int cDefaultBaudRate = 57600; ///default baud rate
    static const int cDefaultTimeout_ms = 2000; ///default timeout to wait for an answer

    int mBaudrate; ///current baudrate, sets in open()
    int mTimeout; ///current timeout

    DISALLOW_COPY_AND_ASSIGN(DynamixelIODriver);
};

#endif

