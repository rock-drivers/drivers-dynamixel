#ifndef DYNAMIXEL_TYPES_HPP__
#define DYNAMIXEL_TYPES_HPP__

#include <string>
#include <inttypes.h>
#include <stdexcept>

namespace servo_dynamixel {

    
#define PI 3.141592653589793
    
/** 
 * @brief error status of the dynamixel servo
 *
 * Note, that multiple errors can be active at the same time
 */
struct ErrorStatus
{
    bool inputVoltageError;
    bool angleLimitError;
    bool overheatingError;
    bool rangeError;
    bool checksumError;
    bool overloadError;
    bool instructionError;

    void clear()
    {
	inputVoltageError = false;
	angleLimitError = false;
	overheatingError = false;
	rangeError = false;
	checksumError = false;
	overloadError = false;
	instructionError = false;
    }

    bool hasError()
    {
	return 
	    inputVoltageError ||
	    angleLimitError ||
	    overheatingError ||
	    rangeError ||
	    checksumError ||
	    overloadError ||
	    instructionError;
    }
};

enum DYNAMIXEL_TYPE {
    DYN_DX_116,
    DYN_MX_28,
};

/**
 * This structure is not used by this driver but by the Rock module servo_dynamixel.
 * It contains examples (see setScales()) how the scales have to be defined.
 */
struct ServoConfiguration
{
    ServoConfiguration() : 
            name("Undefined"), 
            id(1), 
            cwComplianceMargin(0),
            ccwComplianceMargin(0),
            cwComplianceSlope(0x20),
            ccwComplianceSlope(0x00),
            punch(0),
            positionScale(0.0),
            positionOffset(0.0),
            positionRange(0.0),
            speedScale(0.0), 
            effortScale(0.0), 
            reverse(false) {
     }
    
    /**
     * \param name Name of the servo.
     * \param id ID of the servo, by default 1.
     * \param type Used to set correct values for range, offset and scales.
     */
    ServoConfiguration(std::string name, int id, enum DYNAMIXEL_TYPE type) : 
            name(name), 
            id(id), 
            cwComplianceMargin(0), 
            ccwComplianceMargin(0), 
            cwComplianceSlope(0x20), 
            ccwComplianceSlope(0x00), 
            punch(0), 
            positionScale(0.0),
            positionOffset(0.0), 
            positionRange(0.0),
            speedScale(0.0), 
            effortScale(0.0),
            reverse(false) {
        setScales(type);
     }
    
    /** name of the servo, used to identify with the actuators name */
    std::string name;

    /** servo id on the chain */
    int id;

    //-- The Dynamixel controls Compliance by setting the Margin and Slope.
    //   If used well Compliance will absorb the shocks. The following graph
    //   demonstrates the use of Compliance values (length of A,B,C & D)
    //   relative to Position Error and applied torque.
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

    /** clockwise compliance margin, D Gain */
    uint8_t cwComplianceMargin;    
    /** counterclockwise compliance margin, I Gain */
    uint8_t ccwComplianceMargin;
    /** clockwise compliance slope, P Gain Byte 1*/
    uint8_t cwComplianceSlope;  
    /** counterclockwise compliance slope, P Gain Byte 2 */
    uint8_t ccwComplianceSlope; 

    /** minimum current being supplied to the motor during action */
    uint16_t punch;

    /** 
     * Scale factor, which converts radians to dynamixel ticks.
     * pos_ticks = (pos_rad + positionOffset) * positionScale
     */  
    float positionScale;
    /** 
     * Offset which is used for interpreting the input position angle values
     * pos_ticks = (pos_rad + positionOffset) * positionScale
     * Defines the rotation center in rad.
     */  
    float positionOffset;
    /** 
     * Defines the step resolution of the servo (used to truncate the set positions).
     * The available steps of the series are: DX = 1024, MX = 4096
     */  
    float positionRange;
    
    /**
     * Speed factor, converting rad/s to dynamixel tick values
     * speed_ticks = speed * speedScale
     */
    float speedScale;
    /** 
     * Effort factor, converting Nm to load ticks.
     * load_ticks = effort * effortScale 
     */
    float effortScale;
    
    /**
     * Inverts the direction of rotation. Instead of step s 
     * positionRange - s is used. This can be used e.g. if two
     * servos looking in opposite directions should move sth. together.
     */
    bool reverse;

    void checkValid() const
    {
#define in_range( field, min, max ) if( field < min || field > max ) \
	throw std::runtime_error( #field " range is " #min "-" #max )

	in_range( cwComplianceMargin, 0, 254 );
	in_range( ccwComplianceMargin, 0, 254 );
	in_range( cwComplianceSlope, 0, 254 );
	in_range( ccwComplianceSlope, 0, 254 );
	in_range( punch, 0, 1023 );

#undef in_range
    }
    
    /**
     * Contains the predefined scales and position ranges
     * for some dynamixels.
     */
    void setScales(enum DYNAMIXEL_TYPE type) {
        switch(type) {
            case DYN_DX_116: {
                // Sets the rotation center in rad.
                positionOffset = (300/180)*PI/2.0;
                // 0° to 300° with 1024 steps.
                positionRange = 1023;
                // Calculates steps per rad.
                positionScale = positionRange/((300/180)*PI);
                // Converts rad/sec to DX speed value: 0~1023 (0X3FF). 1023 is 70 rpm.
                speedScale = ((60/(2*PI)) / 70) * 1023;
                // 2.5Nm with 12V
                effortScale = 1023/2.5;
                break;
            }
            case DYN_MX_28: {
                // Sets the rotation center in rad.
                positionOffset = ((360/180)*PI)/2.0;
                // 0° to 360° with 4096 steps.
                positionRange = 4095;
                // Calculates steps per rad.
                positionScale = positionRange/((360/180)*PI);
                // Converts rad/sec to MX speed value: 0~1023 (0X3FF) with 0.114rpm per step. 
                // 0 is maximum. 1023 is 117.07 rpm.
                speedScale = ((60/(2*PI)) / 117.07) * 1023;
                // 2.06Nm with 12V
                effortScale = 1023/2.06;
                break;
            }
        }
    }
};

}


#endif
