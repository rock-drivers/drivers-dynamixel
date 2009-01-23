#ifndef DXSERIES_H
#define DXSERIES_H
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

//------------------------------------------------------------------------------------------------------
// some defines

// types
#define DX_BOOL   short
#define DX_UINT8  unsigned char
#define DX_UINT16 unsigned short

// broadcast adress
#define DX_BROADCAST 254

// available commands
#define DX_PING     0x01
#define DX_READ     0x02
#define DX_WRITE    0x03
#define DX_REGWRITE 0x04
#define DX_ACTION   0x05
#define DX_RESET    0x06

// error bits
#define DX_INPUT_VOLTAGE_ERROR 0x01
#define DX_ANGLE_LIMIT_ERROR   0x02
#define DX_OVERHEATING_ERROR   0x04
#define DX_RANGE_ERROR         0x08
#define DX_CHECKSUM_ERROR      0x10
#define DX_OVERLOAD_ERROR      0x20
#define DX_INSTRUCTION_ERROR   0x40

// status return level
#define DX_NO_RESPONE    0x00
#define DX_READ_ONLY     0x01
#define DX_FULL_RESPONSE 0x02

// dxseries memory (low byte = adress, high byte = size)
#define DX_MODEL_NUMBER          0x0200
#define DX_FIRMWARE_VERSION      0x0102
#define DX_SERVO_ID              0x0103
#define DX_BAUDRATE              0x0104
#define DX_RETURN_DELAY_TIME     0x0105
#define DX_CW_ANGLE_LIMIT        0x0206
#define DX_CCW_ANGLE_LIMIT       0x0208
#define DX_HIGHEST_LIMIT_TEMP    0x010B
#define DX_LOWEST_LIMIT_VOLTAGE  0x010C
#define DX_HIGHEST_LIMIT_VOLTAGE 0x010D
#define DX_MAX_TORQUE            0x020E
#define DX_STATUS_RETURN_LEVEL   0x0110
#define DX_ALARM_LED             0x0111
#define DX_ALARM_SHUTDOWN        0x0112
#define DX_DOWN_CALIBRATION      0x0214
#define DX_UP_CALIBRATION        0x0216
#define DX_TORQUE_ENABLE         0x0118
#define DX_LED                   0x0119
#define DX_CW_COMPLIANCE_MARGIN  0x011A
#define DX_CCW_COMPLIANCE_MARGIN 0x011B
#define DX_CW_COMPLIANCE_SLOPE   0x011C
#define DX_CCW_COMPLIANCE_SLOPE  0x011D
#define DX_GOAL_POSITION         0x021E
#define DX_MOVING_SPEED          0x0220
#define DX_TORQUE_LIMIT          0x0222
#define DX_PRESENT_POSITION      0x0224
#define DX_PRESENT_SPEED         0x0226
#define DX_PRESENT_LOAD          0x0228
#define DX_PRESENT_VOLTAGE       0x012A
#define DX_PRESENT_TEMP          0x012B
#define DX_REG_INSTRUCTION       0x012C
#define DX_MOVING                0x012E
#define DX_LOCK                  0x012F
#define DX_PUNCH                 0x0230

//------------------------------------------------------------------------------------------------------
// some structs

typedef struct DxComplete_type {
  DX_UINT16 modelNumber;                 //-- In the case of the DX-116, the value is 0X0074(116).
  DX_UINT8  firmwareVersion;             //-- exactly that
  DX_UINT8  servoID;                     //-- Unique ID number to identify the Dynamixel. Different ID’s
                                         //   are required to be assigned to “linked” Dynamixels.
  DX_UINT8  baudrate;                    //-- Determines the Communication Speed. The Calculation method
                                         //   is: Speed(BPS) = 2000000/(baudrate+1)
  DX_UINT8  returnDelayTime;             //-- The time taken after sending the Instruction Packet, to receive
                                         //   the requested Status Packet. The delay time is given by
                                         //   2uSec * returnDelayTime .
  DX_UINT16 cwAngleLimit;                //-- Set the operating angle to restrict the Dynamixel’s angular
  DX_UINT16 ccwAngleLimit;               //   range. The Goal Position needs to be within the range of:
                                         //   CW Angle Limit <= Goal Position <= CCW Angle Limit
                                         //   An Angle Limit Error will occur if this relationship is not
                                         //   satisfied.
  DX_UINT8  highestLimitTemperature;     //-- The upper limit of the Dynamixel’s operative temperature.
                                         //   If the Dynamixel’s internal temperature is higher than this
                                         //   value, an Over Heating Error Bit (Bit 2 of the Status Packet)
                                         //   will be set. An alarm will be set in alarmLED & alarmShutdown.
                                         //   The values are in Degrees Celsius.
  DX_UINT8  lowestLimitVoltage;          //-- Setting the operative upper and lower limits of the Dynamixel’s
  DX_UINT8  highestLimitVoltage;         //   voltages. If the presentVoltage is out of the specified range,
                                         //   a Voltage Range Error bit will be set in the Status Packet and
                                         //   an alarm executed will be set in alarmLED & alarmShutdown. The
                                         //   values are 10 times the actual voltages. For example, if
                                         //   lowestLimitVoltage value is 80, then the lower voltage limit is
                                         //   set to 8V.
  DX_UINT16 maxTorque;                   //-- The max torque output for the Dynamixel. When it is set to ‘0’,
                                         //   the Dynamixel enters a Torque Free Run condition. The Max Torque
                                         //   is assigned to EEPROM (maxTorque) and RAM (torqueLimit). A power
                                         //   on condition will copy EEPROM values to RAM. The torque of a
                                         //   Dynamixel is limited by torqueLimit.
  DX_UINT8  statusReturnLevel;           //-- To determine whether the Dynamixel will return the Status Packet
                                         //   after the transmission of an Instruction Packet. Possible options
                                         //   are: DX_NO_RESPONE, DX_READ_ONLY and DX_FULL_RESPONSE.
                                         //   In the case of an instruction which uses the Broadcast ID (0XFE),
                                         //   regardless of the statusReturnLevel value, the Status Packet will
                                         //   not be returned.
  DX_UINT8  alarmLED;                    //-- When an Error occurs, if the corresponding Bit is set to 1, then
                                         //   the LED blinks. For error bits see defines above. This function
                                         //   operates as the logical “OR”ing of all set bits. For example, when
                                         //   the register is set to 0X05, the LED will blink when a Voltage Error
                                         //   occurs or when an Overheating Error occurs. Upon returning to a
                                         //   normal condition from an error state, the LED stops blinking after
                                         //   2 seconds.
  DX_UINT8  alarmShutdown;               //-- When an Error occurs, if the corresponding Bit is set to a 1, then
                                         //   the Dynamixel will shut down (Torque off). This function operates
                                         //   as the logical “OR”ing of all set bits. However, unlike the Alarm
                                         //   LED, after returning to a normal condition, it maintains a torque
                                         //   off status. To remove this restriction, torqueEnable is required
                                         //   to be set to 1.
  DX_UINT16 downCalibration;             //-- Data used for compensating for the differences between Robotis
  DX_UINT16 upCalibration;               //   products. Users cannot change this area.
  DX_UINT8  torqueEnable;                //-- When power is first applied the Dynamixel enters the Torque Free
                                         //   Run condition. To allow torque to be applied torqueEnable must be
                                         //   set to 1. (Torque Enabled Condition)
  DX_UINT8  LED;                         //-- LED is on when set to 1 and LED is off if set to 0.
  DX_UINT8  cwComplianceMargin;          //-- The Dynamixel controls Compliance by setting the Margin and Slope.
  DX_UINT8  ccwComplianceMargin;         //   If used well Compliance will absorb the shocks. The following graph
  DX_UINT8  cwComplianceSlope;           //   demonstrates the use of Compliance values (length of A,B,C & D)
  DX_UINT8  ccwComplianceSlope;          //   relative to Position Error and applied torque.
                                         //
                                         //   y-axis : output torque
                                         //   x-axis : position error
                                         //
                                         //     CW /|\                                                  |
                                         //         |  _____         goal position                      |
                                         //         |       \        |                                  |
                                         //         |        \       |                                  |
                                         //         |         \      |                           _ _    |
                                         //         |          |    \|/                           | E   |
                                         //  CCW /__|__________|__________________________\ CW   _|_    |
                                         //      \  |                      |              /       |     |
                                         //         |                      |                     _|_E   |
                                         //         |                       \                           |
                                         //         |                        \                          |
                                         //         |                         \_________                |
                                         //         |                                                   |
                                         //    CCW \|/     |---|-----|-----|---|
                                         //                  A    B     C    D
                                         //
                                         //   A = ccwComplianceSlope
                                         //   B = ccwComplianceMargin
                                         //   C = cwComplianceMargin
                                         //   D = cwComplianceSlope
                                         //   E = punch
                                         //
  DX_UINT16 goalPosition;                //-- Requested Angular Position for the Dynamixel to move to. If this
                                         //   is set to 0x3ff, then the goal position will be 300°.
  DX_UINT16 movingSpeed;                 //-- The angular speed to move to the Goal Position. If set to the
                                         //   maximum value of 0x3ff, it moves at 70RPM.
  DX_UINT16 torqueLimit;                 //-- Current torque limit of the Dynamixel (s. maxTorque).
  DX_UINT16 presentPosition;             //-- Current position of the Dynamixel.
  DX_UINT16 presentSpeed;                //-- Current Speed of the Dynamixel.
  DX_UINT16 presentLoad;                 //-- Load size on the Dynamixel in action. Bit 10 is the direction of
                                         //   the load:
                                         //   Load Direction = 0 : CCW Load, Load Direction = 1: CW Load
  DX_UINT8  presentVoltage;              //-- The voltage applied to the Dynamixel. The value is 10 times the
                                         //   actual voltage. For example, 10V is read as 100(0x64).
  DX_UINT8  presentTemperature;          //-- Current internal Dynamixel temperature (Degrees Celsius).
  DX_UINT8  registeredInstruction;       //-- Set to 1 when a REG_WRITE instruction is made. After an Action
                                         //   instruction and an action it is reset to 0.
  DX_UINT8  moving;                      //-- Set to 1 when the Dynamixel moves by its own power.
  DX_UINT8  lock;                        //-- If set to 1, only alarmShutdown can be written to. Other areas
                                         //   are not permitted. Once locked, it can only be unlocked by
                                         //   powering down.
  DX_UINT16 punch;                        //-- Minimum current being supplied to the motor during an action.
                                         //   The minimum value is 0x20 and the maximum value as 0x3ff.
} DxComplete;


typedef struct DxInitial_type {

  DX_UINT8  cwComplianceMargin;          //-- The Dynamixel controls Compliance by setting the Margin and Slope.
  DX_UINT8  ccwComplianceMargin;         //   If used well Compliance will absorb the shocks. The following graph
  DX_UINT8  cwComplianceSlope;           //   demonstrates the use of Compliance values (length of A,B,C & D)
  DX_UINT8  ccwComplianceSlope;          //   relative to Position Error and applied torque.
                                         //
                                         //   y-axis : output torque
                                         //   x-axis : position error
                                         //
                                         //     CW /|\                                                  |
                                         //         |  _____         goal position                      |
                                         //         |       \        |                                  |
                                         //         |        \       |                                  |
                                         //         |         \      |                           _ _    |
                                         //         |          |    \|/                           | E   |
                                         //  CCW /__|__________|__________________________\ CW   _|_    |
                                         //      \  |                      |              /       |     |
                                         //         |                      |                     _|_E   |
                                         //         |                       \                           |
                                         //         |                        \                          |
                                         //         |                         \_________                |
                                         //         |                                                   |
                                         //    CCW \|/     |---|-----|-----|---|
                                         //                  A    B     C    D
                                         //
                                         //   A = ccwComplianceSlope
                                         //   B = ccwComplianceMargin
                                         //   C = cwComplianceMargin
                                         //   D = cwComplianceSlope
                                         //   E = punch
                                         //
  DX_UINT16 punch;                       //-- Minimum current being supplied to the motor during an action.
                                         //   The minimum value is 0x20 and the maximum value as 0x3ff.
  DX_UINT16 goalPosition;                //-- Requested Angular Position for the Dynamixel to move to. If this
                                         //   is set to 0x3ff, then the goal position will be 300°.
  DX_UINT16 movingSpeed;                 //-- The angular speed to move to the Goal Position. If set to the
                                         //   maximum value of 0x3ff, it moves at 70RPM.
  DX_UINT16 torqueLimit;                 //-- Current torque limit of the Dynamixel (s. maxTorque).

} DxInitial;


typedef struct DxPresentValues_type {
  DX_UINT16 presentPosition;             //-- Current position of the Dynamixel.
  DX_UINT16 presentSpeed;                //-- Current Speed of the Dynamixel.
  DX_UINT16 presentLoad;                 //-- Load size on the Dynamixel in action. Bit 10 is the direction of
                                         //   the load:
                                         //   Load Direction = 0 : CCW Load, Load Direction = 1: CW Load
  DX_UINT8  presentVoltage;              //-- The voltage applied to the Dynamixel. The value is 10 times the
                                         //   actual voltage. For example, 10V is read as 100(0x64).
  DX_UINT8  presentTemperature;          //-- Current internal Dynamixel temperature (Degrees Celsius).
} DxPresentValues;


typedef struct DxMovement_type {
  DX_UINT16 goalPosition;                //-- Requested Angular Position for the Dynamixel to move to. If this
                                         //   is set to 0x3ff, then the goal position will be 300°.
  DX_UINT16 movingSpeed;                 //-- The angular speed to move to the Goal Position. If set to the
                                         //   maximum value of 0x3ff, it moves at 70RPM.
  DX_UINT16 torqueLimit;                 //-- Current torque limit of the Dynamixel (s. maxTorque).
} DxMovement;


//------------------------------------------------------------------------------------------------------
// basic library functions

// No action. Used to obtain a Dynamixel Status Packet.
void dxGetPingCommand(DX_UINT8 *command,    // buffer to put the resulting command string
                      DX_UINT8 *size,       // length of the resulting command string
                      DX_UINT8  id);        // Servo which should be adressed

// Read the values in the Control table.
void dxGetReadCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                      DX_UINT8 *size,           // length of the resulting command string
                      DX_UINT8  id,             // Servo which should be adressed
                      DX_UINT8  startingAdress, // starting adress of read
                      DX_UINT8  dataLength);    // length of data to read

// Write the values to the Control Table.
void dxGetWriteCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                       DX_UINT8 *size,           // length of the resulting command string
                       DX_UINT8  id,             // Servo which should be adressed
                       DX_UINT8  startingAdress, // starting adress of write
                       DX_UINT8 *data,           // data to write
                       DX_UINT8  dataSize);      // length of data to write

// Similar to WRITE DATA, but stay in standby mode until write upon the action instruction.
void dxGetRegWriteCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                          DX_UINT8 *size,           // length of the resulting command string
                          DX_UINT8  id,             // Servo which should be adressed
                          DX_UINT8  startingAdress, // starting adress of write
                          const DX_UINT8 *data,           // data to write
                          DX_UINT8  dataSize);      // length of data to write

// Start the action registered by REG WRITE.
void dxGetActionCommand(DX_UINT8 *command,    // buffer to put the resulting command string
                        DX_UINT8 *size,       // length of the resulting command string
                        DX_UINT8  id);        // Servo which should be adressed

// Change the values of the Dynamixel in the control table back to the Factory Default Values
void dxGetResetCommand(DX_UINT8 *command,    // buffer to put the resulting command string
                       DX_UINT8 *size,       // length of the resulting command string
                       DX_UINT8  id);        // Servo which should be adressed


//------------------------------------------------------------------------------------------------------
// status packet parsing

// returns the length of the status package
DX_UINT8 dxGetStatusLength(const DX_UINT8 *status);

// returns true if checksum is ok
DX_BOOL dxIsStatusValid(const DX_UINT8 *status);       // buffer containing the status packet

// returns the sender id
DX_UINT8 dxGetStatusID(DX_UINT8 *status);         // buffer containing the status packet

// returns the error flags containing error bits (s. defines)
DX_UINT8 dxGetStatusErrorFlags(DX_UINT8 *status); // buffer containing the status packet

// returns true if the voltage is out of the operative range set in the control table.
DX_BOOL dxInputVoltageErrorOccurred(DX_UINT8 *status); // buffer containing the status packet

// returns true if the goal position is set outside of the range between CW Angle Limit and CCW Angle Limit.
DX_BOOL dxAngleLimitErrorOccurred(DX_UINT8 *status); // buffer containing the status packet

// returns true if the internal temperature of Dynamixel is out of the operative range as set in the control table.
DX_BOOL dxOverheatingErrorOccurred(DX_UINT8 *status); // buffer containing the status packet

// returns true if the instruction is out of the usage range.
DX_BOOL dxRangeErrorOccurred(DX_UINT8 *status); // buffer containing the status packet

// returns true if the checksum of the intruction packet is incorrect
DX_BOOL dxChecksumErrorOccurred(DX_UINT8 *status); // buffer containing the status packet

// returns true if the specified torque can't control the load.
DX_BOOL dxOverloadErrorOccurred(DX_UINT8 *status); // buffer containing the status packet

// returns true if an undefined instruction is given without the reg_write instruction.
DX_BOOL dxInstructionErrorOccurred(DX_UINT8 *status); // buffer containing the status packet

// returns the 8 or 16 bit value contained in the status packed
DX_UINT16 dxGetStatusReturnValue(DX_UINT8 *status);         // buffer containing the status packet


//------------------------------------------------------------------------------------------------------
// extended functions

// Read all values in the Control table.
void dxGetReadCompleteCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                              DX_UINT8 *size,           // length of the resulting command string
                              DX_UINT8  id);            // Servo which should be adressed

// Parses the answer of dxGetReadCompleteCommand into a DxComplete struct
void dxGetComplete(DX_UINT8   *status,  // buffer containing the status packet
                   DxComplete *dxComp); // struct which should receive the data

// Read one item from the control table.
void dxGetReadItemCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                          DX_UINT8 *size,           // length of the resulting command string
                          DX_UINT8  id,             // Servo which should be adressed
                          DX_UINT16 item);          // item to read , see memory defines above

// Write a value to one item in the control Table.
void dxGetWriteItemCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                           DX_UINT8 *size,           // length of the resulting command string
                           DX_UINT8  id,             // Servo which should be adressed
                           DX_UINT16 item,           // item to write , see memory defines above
                           DX_UINT16 value);         // value of item

// Similar to  write Item, but stay in standby mode until write upon the action instruction.
void dxGetRegWriteItemCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                              DX_UINT8 *size,           // length of the resulting command string
                              DX_UINT8  id,             // Servo which should be adressed
                              DX_UINT16 item,           // item to write , see memory defines above
                              DX_UINT16 value);         // value of item

// Read all present values in the Control table.
void dxGetReadPresentCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                             DX_UINT8 *size,           // length of the resulting command string
                             DX_UINT8  id);            // Servo which should be adressed

// Parses the answer of dxGetReadPresentCommand into a DxPresentValues struct
void dxGetPresent(DX_UINT8        *status,     // buffer containing the status packet
                  DxPresentValues *dxPresent); // struct which should receive the data

// Read all movement values in the Control table.
void dxGetReadMovementCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                              DX_UINT8 *size,           // length of the resulting command string
                              DX_UINT8  id);            // Servo which should be adressed

// Parses the answer of dxGetReadMovementCommand into a DxMovement struct
void dxGetMovement(DX_UINT8   *status,      // buffer containing the status packet
                   DxMovement *dxMovement); // struct which should receive the data

// Write all movement values in the Control table.
void dxGetWriteMovementCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                               DX_UINT8 *size,           // length of the resulting command string
                               DX_UINT8  id,             // Servo which should be adressed
                               const DxMovement *dxMovement);  // struct which contains data to write

// RegWrite all movement values in the Control table.
void dxGetRegWriteMovementCommand(DX_UINT8 *command,        // buffer to put the resulting command string
                                  DX_UINT8 *size,           // length of the resulting command string
                                  DX_UINT8  id,             // Servo which should be adressed
                                  const DxMovement *dxMovement);  // struct which contains data to write

 #endif

