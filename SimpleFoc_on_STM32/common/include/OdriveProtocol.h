#ifndef ODRIVE_PROTOCOL_H
#define ODRIVE_PROTOCOL_H

#define ESC_TYPE_STM32_ESC1 extern

//COMMAND ID
#define ODRIVE_HEARTBEAT_MESSAGE		0x001
#define SET_AXIS_NODE_ID				0x006
#define SET_AXIS_REQUESTED_STATE 		0x007
#define ENCODER_ESTIMATES				0x009
#define GET_ENCODER_COUNT				0x00A

#if defined(ESC_TYPE_ODRIVE)
	#define SET_INPUT_POS					0x00C
	#define SET_INPUT_VEL					0x00D
	#define SET_INPUT_TORQUE				0x00E
#endif

#if defined(ESC_TYPE_STM32_ESC1)
	#define SET_INPUT_VALUE					0x00C
	#define SET_INPUT_POS					SET_INPUT_VALUE
	#define SET_INPUT_VEL					SET_INPUT_VALUE
	#define SET_INPUT_TORQUE				SET_INPUT_VALUE
	// Depends on the motor controller mode, this could be position, velocity, or torque
#endif

#define SET_CONTROLLER_MODES			0x00B

#define SET_LIMITS						0x00F
#define GET_IQ							0x014
#define REBOOT							0x016
#define GET_BUS_VOLTAGE_CURRENT			0x017
#define CLEAR_ERRORS					0x018
#define SET_POSITION_GAIN				0x01A
#define SET_VEL_GAINS					0x01B


//Axis States
typedef enum {
	AXIS_STATE_UNDEFINED = 0x0,
	AXIS_STATE_IDLE = 0x1,
	AXIS_STATE_STARTUP_SEQUENCE = 0x2,
	AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 0x3,
	AXIS_STATE_MOTOR_CALIBRATION = 0x4,
	AXIS_STATE_ENCODER_INDEX_SEARCH = 0x6,
	AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 0x7,
	AXIS_STATE_CLOSED_LOOP_CONTROL = 0x8,
	AXIS_STATE_LOCKIN_SPIN = 0x9,
	AXIS_STATE_ENCODER_DIR_FIND = 0xA,
	AXIS_STATE_HOMING = 0xB,
	AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 0xC,
	AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 0xD
} Axis_State_t;

//Control Modes
typedef enum {
    #if defined(ESC_TYPE_ODRIVE)
        CONTROL_MODE_VOLTAGE_CONTROL = 0x0,
        CONTROL_MODE_TORQUE_CONTROL = 0x1,
        CONTROL_MODE_VELOCITY_CONTROL = 0x2,
        CONTROL_MODE_POSITION_CONTROL = 0x3
    #endif
    #if defined(ESC_TYPE_STM32_ESC1)
        CONTROL_MODE_VELOCITY = 0x00,
        CONTROL_MODE_ANGLE = 0x01,
        CONTROL_MODE_VELOCITY_OPENLOOP = 0x02,
        CONTROL_MODE_ANGLE_OPENLOOP = 0x03,
        CONTROL_MODE_TORQUE_VOLTAGE = 0x04,     //!< Torque control using voltage
        CONTROL_MODE_TORQUE_DC_CURRENT = 0x05,  //!< Torque control using DC current (one current magnitude)
        CONTROL_MODE_TORQUE_FOC_CURRENT = 0x06, //!< torque control using dq currents

    #endif
} Control_Mode_t;

//Input Modes
typedef enum {
	INPUT_MODE_INACTIVE = 0x0,
	INPUT_MODE_PASSTHROUGH = 0x1,
	INPUT_MODE_VEL_RAMP = 0x2,
	INPUT_MODE_POS_FILTER = 0x3,
	INPUT_MODE_MIX_CHANNELS = 0x4,
	INPUT_MODE_TRAP_TRAJ = 0x5,
	INPUT_MODE_TORQUE_RAMP = 0x6,
	INPUT_MODE_MIRROR = 0x7,
	INPUT_MODE_TUNING = 0x8
} Input_Mode_t;

// ESC1 TYPES
#if defined(ESC_TYPE_STM32_ESC1)

    typedef enum{  
        FOC_SINE_PWM           = 0x00,     //!< Sinusoidal PWM modulation
        FOC_SVPWM              = 0x01,     //!< Space vector modulation method
        FOC_TRAPEZOID_120      = 0x02,
        FOC_TRAPEZOID_150      = 0x03,
    }FOCModulationType_t;

#endif

#endif // ODRIVE_PROTOCOL_H