#ifndef DYNAMIXEL_TYPES_HPP__
#define DYNAMIXEL_TYPES_HPP__

#include <string>
#include <inttypes.h>
#include <stdexcept>

namespace servo_dynamixel {

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

struct ServoConfiguration
{
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

    /** clockwise compliance margin */
    uint8_t cwComplianceMargin;    
    /** counterclockwise compliance margin */
    uint8_t ccwComplianceMargin;
    /** clockwise compliance slope */
    uint8_t cwComplianceSlope;  
    /** counterclockwise compliance slope */
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
     */  
    float positionOffset;
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

    void checkValid() const
    {
#define in_range( field, min, max ) if( field < min || field > max ) \
	throw std::runtime_error( #field " range is " #min "-" #max )

	in_range( cwComplianceMargin, 0, 254 );
	in_range( ccwComplianceMargin, 0, 254 );
	in_range( cwComplianceSlope, 0, 254 );
	in_range( ccwComplianceSlope, 0, 254 );
	in_range( punch, 32, 1023 );

#undef in_range
    }
};

}


#endif
