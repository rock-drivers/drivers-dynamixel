/**
 * \file dynamixel.h
 *  
 * \brief   Allows a simple communication with a dynamixel servo.
 *
 * \details Uses the imoby-iodriver_base for serial communication and the dxseries.h as the\n
 *          communication protocol implementation. You can simply set the control register\n
 *          by using its name (see <a href="http://www.megarobot.net/cj/manualy/robotis/cycloid/DX_series_aj.pdf">Dynamixel Manual</a>).\n
 *          The goal position can be set and the present position can be read directly.           
 *      
 *          German Research Center for Artificial Intelligence\n
 *          Project: AG Framework, Spaceclimber
 *
 * \date    15.12.2009
 *
 * \author  Stefan.Haase@dfki.de
 */
#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include <inttypes.h>

#include <map>
#include <string>
#include <vector>

#include "dynamixel_iodriver.h"
#include "dynamixel_types.hpp"

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  void operator=(const TypeName&)

/**
 * \class Dynamixel
 * See the file description for details.
 */
class Dynamixel 
{
 public:
    static DX_UINT8 const cCommandBufferSize = 255;
    static int const cBufferSize = 512;
    static int const cControlTableEntriesNumber = 34;

    /**
     * \struct ControlTableEntry
     * Every control table entry of the dynamixel is represented by a object of this struct.
     * It contains the address, the name, the number of bytes (one or two) and the current value
     * (if already set or requested, otherwise -1).
     */
    struct ControlTableEntry
    {
        ControlTableEntry()
        {
            mAddress = -1;
            mName = "";
            mBytes = -1;
            mNumber = -1;
        }
        ControlTableEntry(int adress_, std::string name_, int bytes_, int number_)
        {
            mAddress = adress_;
            mName = name_;
            mBytes = bytes_;
            mNumber = number_;
        }
        int mAddress;
        std::string mName;
        int mBytes;
        int mNumber;
    };

    /**
     * Represents a single servo.
     */
    struct Servo
    {
        Servo(DX_UINT8 id_)
        {
            mID = id_;
            for(int i=0; i<cControlTableEntriesNumber; i++)
            {
                mControlTableValues[i]=0;
            }
        }   
        DX_UINT8 mID;
        servo_dynamixel::ErrorStatus status;
        uint16_t mControlTableValues[cControlTableEntriesNumber];
    };
    
    Dynamixel();

    /**
     * Close the serial connection and deletes the control table objects.
     */
    ~Dynamixel();

    bool addServo(DX_UINT8 id_);

    /**
     * Reads the control table entry with the name \a item_name and writes it to \a value_.
     * The names of the control table entries can be found in the manual of the dynamixel (see file description).
     * \param item_name 
     * \param value_ 
     * Fills \a value_ with the value of the passed control table entry (\a item_name).
     * Updates \a mControlTableEntries.
     */
    bool getControlTableEntry(std::string const item_name, uint16_t * const value_);

    /**
     * Builds and returns a string of all control table names and values.
     * Use readControlTable() first.
     */
    std::string getControlTableString();

    /**
     * Fills \a pos_ with the current servo position.
     * Faster than getControlTableEntry("Present Position", &value).
     */
    bool getPresentPosition(uint16_t * const pos_);

    /**
     * Initialise the Dynamixel object.
     */
    bool init(std::string const & uri);

    /**
     * Reads the complete control table and updates the array \a mControlTableEntries.
     * Use \a getControlTableString() to generate a string of the struct.
     */
    bool readControlTable();

    /**
     * Sets the control table entry with the name \a item_name to \a value_.
     * If successfull \a mControlTableEntries is updated.
     * @warning only works with little endian architectures!         
     */
    bool setControlTableEntry(std::string item_name, uint16_t const value_);

    /**
     * Moves the servo to \a pos_. \n
     * Faster than setControlTableEntry("Goal Position" value).
     * @warning only works with little endian architectures!
     */
    bool setGoalPosition(uint16_t const pos_);
    
    /** 
     * @brief return true if the error status of the dynamixel is ok
     */
    bool isErrorStatusOk();

    /**
     * @return the error status of the active servo
     */
    servo_dynamixel::ErrorStatus getErrorStatus();

    void clear()
    {
        return mpDynamixelIODriver->clear();
    }
    int getFileDescriptor() const
    {
        return mpDynamixelIODriver->getFileDescriptor();
    }

    /**
     * Serial read and write timeout, default 1000.
     */
    inline void setTimeout(int const timeout_)
    {
        mpDynamixelIODriver->setTimeout(timeout_);
    }

    Servo* setServoActive(unsigned char id_);

    inline unsigned char getActiveServo() {
        return mActiveServoID;
    }
    
    inline void setNumberRetries(unsigned int num){
        mNumberRetries = num;
    }

    /**
     * Allows to request a copy of the control table entry.
     * \param name Name of the control table entry.
     * \param entry Will be set to the requested control table structure.
     * \return false if the control name is unknown.
     */
    bool getControlTableEntry(std::string const name, struct ControlTableEntry& entry);

    /**
     * Returns a copy of the list of all added servos.
     */
    std::vector<struct Servo> getServoListCopy();

 private:
    //MEMBER VARIABLES
    DX_UINT8 mCommandBuffer[cCommandBufferSize];
    DX_UINT8 mBuffer[cBufferSize];

    DynamixelIODriver* mpDynamixelIODriver; ///serial communication

    std::vector<struct Servo*> mServoList;

    DX_UINT8 mActiveServoID;
    Servo* mpActiveServo;

    struct ControlTableEntry mControlTableEntries[cControlTableEntriesNumber]; 
    /** Used in setControlTableEntry and getControlTableEntry. */
    std::map<std::string, struct ControlTableEntry*> mMapStringCTEntry;
   
   /** 
    * Can be used to resend a command if an error occurred.
    * This is required in our current configuration,  because sometimes
    * one of the start bytes get lost.
    */
   unsigned int mNumberRetries;
    
    //FUNCTIONS    
    void buildControlTable();

    /**
     * First write the command to the \a mCommandBuffer.
     * \param command_length_bytes Length of the command.
     * \return True if the command could be sent and the received
     *         status packet is error free.
     */
    bool writeCommandReadAnswer(int command_length_bytes, servo_dynamixel::ErrorStatus &status);

    DISALLOW_COPY_AND_ASSIGN(Dynamixel);
};

#endif

