/*
 *  Robotis DX series Control Library
 *
 *  by DFKI Bremen, Jochen Kerdels
 *
 *  jochen.kerdels@dfki.de
 *
 *
 *  The library generates the appropriate command strings which then
 *  should be sent by the accordant usart commands of the actually used
 *  system.
 *
 */


#include "dxseries.h"

//------------------------------------------------------------------------------------------------------
// No action. Used to obtain a Dynamixel Status Packet.
void dxGetPingCommand(DX_UINT8 *command,    // buffer to put the resulting command string
                      DX_UINT8 *size,       // length of the resulting command string
                      DX_UINT8  id)         // Servo which should be adressed
{
  command[0] = 255;                    // start of packet
  command[1] = 255;                    // start of packet
  command[2] = id;                     // id of recipient
  command[3] = 2;                      // length of packet (parametercount + 2)
  command[4] = DX_PING;                // instruction
  command[5] = ~(command[2] +
                 command[3] +
                 command[4]);          // checksum
  *size = 6;
}


 
//------------------------------------------------------------------------------------------------------
// Read the values in the Control table.
void dxGetReadCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                      DX_UINT8 *size,           // length of the resulting command string
                      DX_UINT8  id,             // Servo which should be adressed
                      DX_UINT8  startingAdress, // starting adress of read
                      DX_UINT8  dataLength)     // length of data to read
{
  command[0] = 255;                    // start of packet
  command[1] = 255;                    // start of packet
  command[2] = id;                     // id of recipient
  command[3] = 4;                      // length of packet (parametercount + 2)
  command[4] = DX_READ;                // instruction
  command[5] = startingAdress;         // parameter 1
  command[6] = dataLength;             // parameter 2
  command[7] = ~(command[2] +
                 command[3] +
                 command[4] +
                 command[5] +
                 command[6]); // checksum
  *size = 8;
}



//------------------------------------------------------------------------------------------------------
// Write the values to the Control Table.
void dxGetWriteCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                       DX_UINT8 *size,           // length of the resulting command string
                       DX_UINT8  id,             // Servo which should be adressed
                       DX_UINT8  startingAdress, // starting adress of write
                       DX_UINT8 *data,           // data to write
                       DX_UINT8  dataSize)       // length of data to write
{
  command[0] = 255;                        // start of packet
  command[1] = 255;                        // start of packet
  command[2] = id;                         // id of recipient
  command[3] = dataSize + 3;               // length of packet (parametercount + 2)
  command[4] = DX_WRITE;                   // instruction
  command[5] = startingAdress;             // parameter 1
  DX_UINT8 i;
  for (i = 0; i < dataSize; i++)
    command[6+i] = data[i];                // parameter 2..dataSize+1
  DX_UINT8 check = command[2];
  for (i = 3; i < 6 + dataSize; i++)
    check += command[i];
  command[6 + dataSize] = ~check; // checksum
  *size = 7 + dataSize;
}

 
//------------------------------------------------------------------------------------------------------
// Similar to WRITE DATA, but stay in standby mode until write upon the action instruction.
void dxGetRegWriteCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                          DX_UINT8 *size,           // length of the resulting command string
                          DX_UINT8  id,             // Servo which should be adressed
                          DX_UINT8  startingAdress, // starting adress of write
                          const DX_UINT8 *data,           // data to write
                          DX_UINT8  dataSize)       // length of data to write
{
  command[0] = 255;                        // start of packet
  command[1] = 255;                        // start of packet
  command[2] = id;                         // id of recipient
  command[3] = dataSize + 3;               // length of packet (parametercount + 2)
  command[4] = DX_REGWRITE;                // instruction
  command[5] = startingAdress;             // parameter 1
  DX_UINT8 i;
  for (i = 0; i < dataSize; i++)
    command[6+i] = data[i];                // parameter 2..dataSize+1
  DX_UINT8 check = command[2];
  for (i = 3; i < 6 + dataSize; i++)
    check += command[i];
  command[6 + dataSize] = ~check; // checksum
  *size = 7 + dataSize;
}




//------------------------------------------------------------------------------------------------------
// Start the action registered by REG WRITE.
void dxGetActionCommand(DX_UINT8 *command,    // buffer to put the resulting command string
                        DX_UINT8 *size,       // length of the resulting command string
                        DX_UINT8  id)         // Servo which should be adressed
{
  command[0] = 255;                    // start of packet
  command[1] = 255;                    // start of packet
  command[2] = id;                     // id of recipient
  command[3] = 2;                      // length of packet (parametercount + 2)
  command[4] = DX_ACTION;              // instruction
  command[5] = ~(command[2] +
                 command[3] +
                 command[4]);          // checksum
  *size = 6;
}



 
//------------------------------------------------------------------------------------------------------
// Change the values of the Dynamixel in the control table back to the Factory Default Values
void dxGetResetCommand(DX_UINT8 *command,    // buffer to put the resulting command string
                       DX_UINT8 *size,       // length of the resulting command string
                       DX_UINT8  id)         // Servo which should be adressed
{
  command[0] = 255;                    // start of packet
  command[1] = 255;                    // start of packet
  command[2] = id;                     // id of recipient
  command[3] = 2;                      // length of packet (parametercount + 2)
  command[4] = DX_RESET;               // instruction
  command[5] = ~(command[2] +
                 command[3] +
                 command[4]);          // checksum
  *size = 6;
}




//------------------------------------------------------------------------------------------------------
// returns true if checksum is ok
DX_BOOL dxIsStatusValid(const DX_UINT8 *status)       // buffer containing the status packet
{
  return dxGetStatusLength(status)>0;
}

DX_UINT8 dxGetStatusLength(const DX_UINT8 *status)       // buffer containing the status packet
{
  DX_UINT8 check = status[2];
  DX_UINT8 high  = status[3] + 3;
  DX_UINT8 i = 3;
  while (i < high)
    check += status[i++];
  return !(check & status[i])?high+1:0;
}

//------------------------------------------------------------------------------------------------------
// returns the sender id
DX_UINT8 dxGetStatusID(DX_UINT8 *status)         // buffer containing the status packet
{
  return status[2];
}



//------------------------------------------------------------------------------------------------------
// returns the error flags containing error bits (s. defines)
DX_UINT8 dxGetStatusErrorFlags(DX_UINT8 *status) // buffer containing the status packet
{
  return status[4];
}



//------------------------------------------------------------------------------------------------------
// returns true if the voltage is out of the operative range set in the control table.
DX_BOOL dxInputVoltageErrorOccurred(DX_UINT8 *status) // buffer containing the status packet
{
  return ((DX_INPUT_VOLTAGE_ERROR & status[4]) != 0);
}



//------------------------------------------------------------------------------------------------------
// returns true if the goal position is set outside of the range between CW Angle Limit and CCW Angle Limit.
DX_BOOL dxAngleLimitErrorOccurred(DX_UINT8 *status) // buffer containing the status packet
{
  return ((DX_ANGLE_LIMIT_ERROR & status[4]) != 0);
}



//------------------------------------------------------------------------------------------------------
// returns true if the internal temperature of Dynamixel is out of the operative range as set in the control table.
DX_BOOL dxOverheatingErrorOccurred(DX_UINT8 *status) // buffer containing the status packet
{
  return ((DX_OVERHEATING_ERROR & status[4]) != 0);
}



//------------------------------------------------------------------------------------------------------
// returns true if the instruction is out of the usage range.
DX_BOOL dxRangeErrorOccurred(DX_UINT8 *status) // buffer containing the status packet
{
  return ((DX_RANGE_ERROR & status[4]) != 0);
}



//------------------------------------------------------------------------------------------------------
// returns true if the checksum of the intruction packet is incorrect
DX_BOOL dxChecksumErrorOccurred(DX_UINT8 *status) // buffer containing the status packet
{
  return ((DX_CHECKSUM_ERROR & status[4]) != 0);
}



//------------------------------------------------------------------------------------------------------
// returns true if the specified torque can't control the load.
DX_BOOL dxOverloadErrorOccurred(DX_UINT8 *status) // buffer containing the status packet
{
  return ((DX_OVERLOAD_ERROR & status[4]) != 0);
}



//------------------------------------------------------------------------------------------------------
// returns true if an undefined instruction is given without the reg_write instruction.
DX_BOOL dxInstructionErrorOccurred(DX_UINT8 *status) // buffer containing the status packet
{
  return ((DX_INSTRUCTION_ERROR & status[4]) != 0);
}



//------------------------------------------------------------------------------------------------------
// returns the 8 or 16 bit value contained in the status packed
DX_UINT16 dxGetStatusReturnValue(DX_UINT8 *status)         // buffer containing the status packet
{
  if (status[3]-2 > 1)
    return (status[5] | (status[6] << 8));
  else if (status[3]-2 > 0)
    return status[5];
  else
    return 0;
}



//------------------------------------------------------------------------------------------------------
// Read all values in the Control table.
void dxGetReadCompleteCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                              DX_UINT8 *size,           // length of the resulting command string
                              DX_UINT8  id)             // Servo which should be adressed
{
  dxGetReadCommand(command,size,id,0,50);
}



//------------------------------------------------------------------------------------------------------
// Parses the answer of dxGetReadCompleteCommand into a DxComplete struct
void dxGetComplete(DX_UINT8   *status,  // buffer containing the status packet
                   DxComplete *dxComp)  // struct which should receive the data
{
  DX_UINT8 base = 5;

  dxComp->modelNumber             = (status[base   ] | (status[base+ 1] << 8));
  dxComp->firmwareVersion         =  status[base+ 2];
  dxComp->servoID                 =  status[base+ 3];
  dxComp->baudrate                =  status[base+ 4];    
  dxComp->returnDelayTime         =  status[base+ 5];    
  dxComp->cwAngleLimit            = (status[base+ 6] | (status[base+ 7] << 8));
  dxComp->ccwAngleLimit           = (status[base+ 8] | (status[base+ 9] << 8)); // byte 10 reserved
  dxComp->highestLimitTemperature =  status[base+11];
  dxComp->lowestLimitVoltage      =  status[base+12];    
  dxComp->highestLimitVoltage     =  status[base+13];
  dxComp->maxTorque               = (status[base+14] | (status[base+15] << 8));
  dxComp->statusReturnLevel       =  status[base+16];    
  dxComp->alarmLED                =  status[base+17];
  dxComp->alarmShutdown           =  status[base+18]; // byte 19 reserved
  dxComp->downCalibration         = (status[base+20] | (status[base+21] << 8));
  dxComp->upCalibration           = (status[base+22] | (status[base+23] << 8));    
  dxComp->torqueEnable            =  status[base+24];
  dxComp->LED                     =  status[base+25];
  dxComp->cwComplianceMargin      =  status[base+26];
  dxComp->ccwComplianceMargin     =  status[base+27];
  dxComp->cwComplianceSlope       =  status[base+28];
  dxComp->ccwComplianceSlope      =  status[base+29];
  dxComp->goalPosition            = (status[base+30] | (status[base+31] << 8));    
  dxComp->movingSpeed             = (status[base+32] | (status[base+33] << 8));    
  dxComp->torqueLimit             = (status[base+34] | (status[base+35] << 8));    
  dxComp->presentPosition         = (status[base+36] | (status[base+37] << 8));
  dxComp->presentSpeed            = (status[base+38] | (status[base+39] << 8));
  dxComp->presentLoad             = (status[base+40] | (status[base+41] << 8));
  dxComp->presentVoltage          =  status[base+42];
  dxComp->presentTemperature      =  status[base+43];
  dxComp->registeredInstruction   =  status[base+44]; // byte 45 reserved
  dxComp->moving                  =  status[base+46];
  dxComp->lock                    =  status[base+47];
  dxComp->punch                   = (status[base+48] | (status[base+49] << 8));
}



//------------------------------------------------------------------------------------------------------
// Read one item from the control table.
void dxGetReadItemCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                          DX_UINT8 *size,           // length of the resulting command string
                          DX_UINT8  id,             // Servo which should be adressed
                          DX_UINT16 item)           // item to read , see memory defines above
{
  dxGetReadCommand(command,size,id,(item & 0xff),(item & 0xff00) >> 8);
}



//------------------------------------------------------------------------------------------------------
// Write a value to one item in the control Table.
void dxGetWriteItemCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                           DX_UINT8 *size,           // length of the resulting command string
                           DX_UINT8  id,             // Servo which should be adressed
                           DX_UINT16 item,           // item to write , see memory defines above
                           DX_UINT16 value)          // value of item
{
  dxGetWriteCommand(command,size,id,(item & 0xff),(DX_UINT8*)(&value),(item & 0xff00) >> 8);
}




//------------------------------------------------------------------------------------------------------
// Similar to  write Item, but stay in standby mode until write upon the action instruction.
void dxGetRegWriteItemCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                              DX_UINT8 *size,           // length of the resulting command string
                              DX_UINT8  id,             // Servo which should be adressed
                              DX_UINT16 item,           // item to write , see memory defines above
                              DX_UINT16 value)          // value of item
{
  dxGetWriteCommand(command,size,id,(item & 0xff),(DX_UINT8*)(&value),(item & 0xff00) >> 8);
}



//------------------------------------------------------------------------------------------------------
// Read all present values in the Control table.
void dxGetReadPresentCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                             DX_UINT8 *size,           // length of the resulting command string
                             DX_UINT8  id)             // Servo which should be adressed
{
  dxGetReadCommand(command,size,id,36,8);
}



//------------------------------------------------------------------------------------------------------
// Parses the answer of dxGetReadPresentCommand into a DxPresentValues struct
void dxGetPresent(DX_UINT8        *status,     // buffer containing the status packet
                  DxPresentValues *dxPresent)  // struct which should receive the data
{
  DX_UINT8 base = 5;

  dxPresent->presentPosition     = (status[base   ] | (status[base+ 1] << 8));
  dxPresent->presentSpeed        = (status[base+ 2] | (status[base+ 3] << 8));
  dxPresent->presentLoad         = (status[base+ 4] | (status[base+ 5] << 8));
  dxPresent->presentVoltage      =  status[base+ 6];
  dxPresent->presentTemperature  =  status[base+ 7];
}



//------------------------------------------------------------------------------------------------------
// Read all movement values in the Control table.
void dxGetReadMovementCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                              DX_UINT8 *size,           // length of the resulting command string
                              DX_UINT8  id)             // Servo which should be adressed
{
  dxGetReadCommand(command,size,id,30,6);
}



//------------------------------------------------------------------------------------------------------
// Parses the answer of dxGetReadMovementCommand into a DxMovement struct
void dxGetMovement(DX_UINT8   *status,      // buffer containing the status packet
                   DxMovement *dxMovement)  // struct which should receive the data
{
  DX_UINT8 base = 5;

  dxMovement->goalPosition     = (status[base   ] | (status[base+ 1] << 8));
  dxMovement->movingSpeed      = (status[base+ 2] | (status[base+ 3] << 8));
  dxMovement->torqueLimit      = (status[base+ 4] | (status[base+ 5] << 8));
}



//------------------------------------------------------------------------------------------------------
// Write all movement values in the Control table.
void dxGetWriteMovementCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                               DX_UINT8 *size,           // length of the resulting command string
                               DX_UINT8  id,             // Servo which should be adressed
                               const DxMovement *dxMovement)   // struct which contains data to write
{
  dxGetWriteCommand(command,size,id,30,(DX_UINT8*)(dxMovement),6);
}



//------------------------------------------------------------------------------------------------------
// RegWrite all movement values in the Control table.
void dxGetRegWriteMovementCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                                  DX_UINT8 *size,           // length of the resulting command string
                                  DX_UINT8  id,             // Servo which should be adressed
                                  const DxMovement *dxMovement)   // struct which contains data to write
{
  dxGetRegWriteCommand(command,size,id,30,(DX_UINT8*)(dxMovement),6);
}




