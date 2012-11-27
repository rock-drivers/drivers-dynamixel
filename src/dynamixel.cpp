/// \file dynamixel.cpp

#include "dynamixel.h"

#include <iostream>
#include <sstream>

#include "base/logging.h"

/////////////////////////////// PUBLIC ///////////////////////////////////////
Dynamixel::Dynamixel()
{
    mActiveServoID = 0;
    mpActiveServo = NULL;
    mNumberRetries = 0;
    mpDynamixelIODriver = new DynamixelIODriver();
    buildControlTable();
}

Dynamixel::~Dynamixel()
{
    mpDynamixelIODriver->close();
    delete mpDynamixelIODriver;
    mpDynamixelIODriver = NULL;
    for(unsigned int i=0; i<mServoList.size(); i++)
    {
        delete mServoList[i];
    }
    mServoList.clear();
}

bool Dynamixel::addServo(DX_UINT8 id_)
{
    //id already added?
    for(unsigned int i=0; i<mServoList.size(); i++)
    {
        if(mServoList[i]->mID == id_)
        {
            LOG_WARN("Servo ID %d already added", (int)id_);
            return false;
        }
    }
    mServoList.push_back(new Servo(id_));
    LOG_INFO("Servo ID %d added", (int)id_);

    if(mServoList.size() == 1) {
        setServoActive(id_);
    }
    return true;
}

bool Dynamixel::getControlTableEntry(std::string const item_name, uint16_t * const value_)
{
    if(mpActiveServo == NULL)
    {
        LOG_WARN("No active servo available, use setServoActive() first");
        return false;
    }

     struct ControlTableEntry* entry_ = mMapStringCTEntry[item_name];
     if(entry_ == NULL)
     {
         LOG_WARN("Control table entry name %s is unknown", item_name.c_str());
         return false;
     }

     DX_UINT8 command_length_bytes;
     dxGetReadCommand(mCommandBuffer,
         &command_length_bytes,
         mActiveServoID,
         entry_->mAddress,
         entry_->mBytes);
     if(writeCommandReadAnswer(command_length_bytes))
     {
        uint16_t value_temp = mBuffer[5];
        if(entry_->mBytes == 2)
        {
            int byte_high = mBuffer[6];
            value_temp = (value_temp | (byte_high << 8));
        }
        *value_ = value_temp;
        //update the control table entry of the servo with the ID id_,
        //the array position is listed in the entry object
        mpActiveServo->mControlTableValues[entry_->mNumber] = value_temp;
        LOG_INFO("Control table entry %s has been changed to %d", item_name.c_str(), value_temp);
        return true;
     }
     return false;
}

std::string Dynamixel::getControlTableString()
{
    if(mpActiveServo == NULL)
    {
        LOG_WARN("No active servo available, use setServoActive() first");
        return "";
    }
    std::stringstream stream;
    const int NAME_LENGTH = 30;
    for(int i=0; i<cControlTableEntriesNumber; i++)
    {
        if (i<10)
			stream << " ";
        stream << i;
        stream << "  " << mControlTableEntries[i].mName;
        //every name string should have a length of NAME_LENGTH
        for(int j=(int)mControlTableEntries[i].mName.length(); j<NAME_LENGTH; j++)
        {
            stream << ' ';
        }
        stream << mpActiveServo->mControlTableValues[i];
        //additional output: step to degree conversion
        if(mControlTableEntries[i].mName == "CW Angle Limit" ||
             mControlTableEntries[i].mName == "CCW Angle Limit")
        {
             stream << " (" << step2deg(mpActiveServo->mControlTableValues[i]) << " deg)";
        }
        stream << '\n';
    }
    std::string s;
    s = stream.str();
    return s;
}

bool Dynamixel::getPresentPosition(uint16_t * const pos_)
{
    if(mpActiveServo == NULL)
    {
        LOG_WARN("No active servo available, use setServoActive() first");
        return false;
    }

    DX_UINT8 command_length_bytes;
    dxGetReadCommand(mCommandBuffer, &command_length_bytes, mActiveServoID, 36, 2);
    if(writeCommandReadAnswer(command_length_bytes))
    {
        int byte_low = mBuffer[5];
        int byte_high = mBuffer[6];
        *pos_ = (byte_low | (byte_high << 8));
        mpActiveServo->mControlTableValues[25] = *pos_;
        LOG_DEBUG("Current position is %d (steps)", *pos_);
        return true;
    }
    return false;
}

bool Dynamixel::getPresentPositionDegree(float * const pos_deg)
{
    uint16_t pos_step = 0;
    if(getPresentPosition(&pos_step))
    {
        *pos_deg = step2deg(pos_step);
        LOG_DEBUG("Current position is %f (degree)", *pos_deg);
        return true;
    } else {
        return false;
    }
}

bool Dynamixel::init(struct Configuration * const config)
{
    return mpDynamixelIODriver->open(config->mFilename, config->mBaudrate);
}

bool Dynamixel::readControlTable()
{
    if(mpActiveServo == NULL)
    {
        LOG_WARN("No active servo available, use setServoActive() first");
        return false;
    }

    DX_UINT8 command_length_bytes;
    dxGetReadCompleteCommand(mCommandBuffer, &command_length_bytes, mActiveServoID);
    if( (writeCommandReadAnswer(command_length_bytes)) && (mActiveServoID != DX_BROADCAST) ) //fills the mBuffer, if talking to one specific servo
    {
        struct DxComplete_type mCompleteControlTable;
        dxGetComplete(mBuffer, &mCompleteControlTable);
        //fill the control table items
        DX_UINT8* p_ct = (DX_UINT8*)&mCompleteControlTable;
        for(int i=0; i<cControlTableEntriesNumber; i++)
        {
            mpActiveServo->mControlTableValues[i] = *p_ct;
            ++p_ct;
            //build value if it consists of two bytes
            if(mControlTableEntries[i].mBytes == 2)
            {
                mpActiveServo->mControlTableValues[i] = (mpActiveServo->mControlTableValues[i] | (*p_ct << 8));
                ++p_ct;
            }
        }
        LOG_DEBUG("All controls have been read")
        return true;
    }
    LOG_ERROR("Controls could not be read");
    return false;
}

bool Dynamixel::setControlTableEntry(std::string item_name, uint16_t const value_)
{
    if(mpActiveServo == NULL)
    {
        LOG_WARN("No active servo available, use setServoActive() first");
        return "";
    }

    struct ControlTableEntry* entry = mMapStringCTEntry[item_name];
    if(entry == NULL)
    {
        LOG_WARN("Control table entry name %s is unknown", item_name.c_str());
        return false;
    }

    DX_UINT8 command_length_bytes;
    dxGetWriteCommand(mCommandBuffer,
        &command_length_bytes,
        mActiveServoID,
        entry->mAddress,
        (DX_UINT8*)&value_,
        entry->mBytes);
    if(writeCommandReadAnswer(command_length_bytes))
    {
        mpActiveServo->mControlTableValues[entry->mNumber] = value_;
        LOG_INFO("Control table entry %s has been set to %hu", item_name.c_str(), value_);
        return true;
    }
    LOG_ERROR("Control table entry %s could not be changed to %hu", value_);
    return false;
}

bool Dynamixel::setGoalPosition(uint16_t const pos_)
{
    if(mpActiveServo == NULL)
    {
        LOG_WARN("No active servo available, use setServoActive() first");
        return false;
    }

    DX_UINT8 command_length_bytes;
    dxGetWriteCommand(mCommandBuffer, &command_length_bytes, mActiveServoID, 30, (DX_UINT8*)&pos_, 2);
    if(writeCommandReadAnswer(command_length_bytes))
    {
        mpActiveServo->mControlTableValues[22] = pos_;
        LOG_DEBUG("Active servo %d set to %hu (steps)", mActiveServoID, pos_);
        return true;
    }
    LOG_ERROR("Active servo %d position could not be changed", mActiveServoID);
    return false;
}

bool Dynamixel::setGoalPositionDegree(float const pos_deg)
{
    uint16_t pos_step = deg2step(pos_deg);
    LOG_DEBUG("Active servo %d set to %f (degree)", mActiveServoID, pos_deg);
    return setGoalPosition(pos_step);
}

Dynamixel::Servo* Dynamixel::setServoActive(DX_UINT8 id_)
{
    for(unsigned int i=0; i<mServoList.size(); i++)
    {
        if(mServoList[i]->mID == id_)
        {
            mActiveServoID = id_;
            mpActiveServo = mServoList[i];
            LOG_INFO("Servo ID %d activated", id_);
            return mServoList[i];
        }
    }
    LOG_WARN("Servo ID %d is not available and could not be activated", id_);
    return NULL;
}

bool Dynamixel::getControlTableEntry(std::string const name, struct ControlTableEntry& entry) {
    std::map<std::string, struct ControlTableEntry*>::iterator it;
    it = mMapStringCTEntry.find(name);
    if(it != mMapStringCTEntry.end()) {
        entry = *(it->second);
        return true;
    }
    LOG_WARN("Control table entry %s is unknown", name.c_str());
    return false;
}

std::vector<Dynamixel::Servo> Dynamixel::getServoListCopy() {
    std::vector<struct Servo> servo_list_copy;
    for(unsigned int i=0; i < mServoList.size(); ++i) {
        servo_list_copy.push_back(*(mServoList.at(i)));
    }
    return servo_list_copy;
}

/////////////////////////////// PRIVATE //////////////////////////////////////
void Dynamixel::buildControlTable()
{
    mControlTableEntries[ 0] = ControlTableEntry( 0, "Model Number", 2, 0);
    mControlTableEntries[ 1] = ControlTableEntry( 2, "Version of Firmware", 1, 1);
    mControlTableEntries[ 2] = ControlTableEntry( 3, "ID", 1, 2);
    mControlTableEntries[ 3] = ControlTableEntry( 4, "Baud Rate", 1, 3);
    mControlTableEntries[ 4] = ControlTableEntry( 5, "Return Delay Time", 1, 4);
    mControlTableEntries[ 5] = ControlTableEntry( 6, "CW Angle Limit", 2, 5);
    mControlTableEntries[ 6] = ControlTableEntry( 8, "CCW Angle Limit", 2, 6);
    mControlTableEntries[ 7] = ControlTableEntry(11, "the Highest Limit Temperature", 1, 7);
    mControlTableEntries[ 8] = ControlTableEntry(12, "the Lowest Limit Voltage", 1, 8);
    mControlTableEntries[ 9] = ControlTableEntry(13, "the Highest Limit Voltage", 1, 9);
    mControlTableEntries[10] = ControlTableEntry(14, "Max Torque", 2, 10);
    mControlTableEntries[11] = ControlTableEntry(16, "Status Return Level", 1, 11);
    mControlTableEntries[12] = ControlTableEntry(17, "Alarm LED", 1, 12);
    mControlTableEntries[13] = ControlTableEntry(18, "Alarm Shutdown", 1, 13);
    mControlTableEntries[14] = ControlTableEntry(20, "Down Calibration", 2, 14);
    mControlTableEntries[15] = ControlTableEntry(22, "Up Calibration", 2, 15);
    mControlTableEntries[16] = ControlTableEntry(24, "Torque Enable", 1, 16);
    mControlTableEntries[17] = ControlTableEntry(25, "LED", 1, 17);
    mControlTableEntries[18] = ControlTableEntry(26, "CW Compliance Margin", 1, 18);
    mControlTableEntries[19] = ControlTableEntry(27, "CCW Compliance Margin", 1, 19);
    mControlTableEntries[20] = ControlTableEntry(28, "CW Compliance Slope", 1, 20);
    mControlTableEntries[21] = ControlTableEntry(29, "CCW Compliance Slope", 1, 21);
    mControlTableEntries[22] = ControlTableEntry(30, "Goal Position", 2, 22);
    mControlTableEntries[23] = ControlTableEntry(32, "Moving Speed", 2, 23);
    mControlTableEntries[24] = ControlTableEntry(34, "Torque Limit", 2, 24);
    mControlTableEntries[25] = ControlTableEntry(36, "Present Position", 2, 25);
    mControlTableEntries[26] = ControlTableEntry(38, "Present Speed", 2, 26);
    mControlTableEntries[27] = ControlTableEntry(40, "Present Load", 2, 27);
    mControlTableEntries[28] = ControlTableEntry(42, "Present Voltage", 1, 28);
    mControlTableEntries[29] = ControlTableEntry(43, "Present Temperature", 1, 29);
    mControlTableEntries[30] = ControlTableEntry(44, "Registered Instruction", 1, 30);
    mControlTableEntries[31] = ControlTableEntry(46, "Moving", 1, 31);
    mControlTableEntries[32] = ControlTableEntry(47, "Lock", 1, 32);
    mControlTableEntries[33] = ControlTableEntry(48, "Punch", 2, 33);

    //fill the lookup (hash?) map
    for(int i=0; i<cControlTableEntriesNumber; i++)
    {
        mMapStringCTEntry[mControlTableEntries[i].mName] = &(mControlTableEntries[i]);
    }
}

bool Dynamixel::writeCommandReadAnswer(int command_length_bytes)
{
    for(unsigned int i = 0; i <= mNumberRetries; ++i) {  
    try {
        DX_UINT8 packet_size = 0;
        if(!mpDynamixelIODriver->writePacket(mCommandBuffer, command_length_bytes))
        {
            LOG_ERROR("Packet could not be written");
            continue;
        } else {
            for(int i=0; i<command_length_bytes; i++) {
                LOG_DEBUG("Write 0x%x(%d)", mCommandBuffer[i], mCommandBuffer[i]);
            }
        }

        //will we get a status packet? broadcast means no
        if(mActiveServoID == DX_BROADCAST)
        {
            LOG_DEBUG("Broadcasting ID 0x%x is used, no status packet will be received", DX_BROADCAST);
            return true;
        }

        if((packet_size = mpDynamixelIODriver->readPacket(mBuffer, cBufferSize)) <= 0)
        {
            LOG_ERROR("Packet could not be read, %d has been returned", packet_size);
            continue;
        }
        //check status packet
        DX_UINT8 error_flags = dxGetStatusErrorFlags(mBuffer);
        if(error_flags != 0) //error
        {
            LOG_WARN("Status packet error returned (0x%x):", error_flags);
            if(dxInputVoltageErrorOccurred(mBuffer)){LOG_WARN("    Input Voltage Error");}
            if(dxAngleLimitErrorOccurred(mBuffer)){LOG_WARN("    Angle Limit Error");}
            if(dxOverheatingErrorOccurred(mBuffer)){LOG_WARN("    Overheating Error");}
            if(dxRangeErrorOccurred(mBuffer)){LOG_WARN("    Range Error");}
            if(dxChecksumErrorOccurred(mBuffer)){LOG_WARN("    Checksum Error");}
            if(dxOverloadErrorOccurred(mBuffer)){LOG_WARN("    Overload Error");}
            if(dxInstructionErrorOccurred(mBuffer)){LOG_WARN("    Instruction Error");}
            continue;
        }
        if(!dxIsStatusValid(mBuffer, packet_size))
        {
            LOG_WARN("Invalid checksum reported");
            continue;
        }
        return true;
    } catch(iodrivers_base::UnixError& e) {
        LOG_ERROR("UnixError catched: %s", e.what());
    } catch(iodrivers_base::TimeoutError& e) {
        LOG_ERROR("TimeoutError catched: %s", e.what());
    }
    } // for loop
    return false;
}

