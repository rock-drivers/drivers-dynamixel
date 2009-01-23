#ifndef DXServo_H
#define DXServo_H

#include <map>
#include <string>
#include <iosfwd>
#include <sys/types.h>
#include <iodrivers_base.hh>
extern "C" {
#include <dxlib/dxseries.h>
}

class DXServo : public IODriver {
    public:

    private:
        static const int MAX_PACKET_SIZE = 256;
        static const int CMD_BUF_SIZE = 512;

        /** The baudrate */
        int m_baudrate;
        int m_timeout;

    protected:
        int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

    public:
        DXServo();
        ~DXServo();

        bool open(std::string const& filename);
        bool fullReset();
        /** Closes the device */
        bool close();

        bool setMovement(unsigned char servo_id, const DxMovement& movement);
        bool setInitialData(unsigned char servo_id, const DxInitial& initial);
        
        bool getPresentValues(unsigned char servo_id, DxPresentValues *values); 
};

#endif

