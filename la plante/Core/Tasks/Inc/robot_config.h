

#ifndef TASKS_INC_ROBOT_CONFIG_H_
#define TASKS_INC_ROBOT_CONFIG_H_

#include <robot_config_DS.h>
#include "typedefs.h"

// COMMON CONFIGURATION

#define SPINSPIN_RANDOM_DELAY 50
#define MOTOR_ONLINE_CHECK 	1
//1 for annoying beep sound, 0 for some error beeps every 3s, -1 for absolute peace and tranquility (with pancik
#define ARM_SWITCH 			0			//set to 1 to enable remote up and down arming switch
#define FIRING_DISABLE		0			//set to 1 to stop firing
#define LAUNCHER_SAFETY		1			//set to 1 to enable launcher arm safety
//(between mode switch, remote off, power on, left switch MUST be down)


//if no overrides in the respective configs
#ifndef CONTROL_DEFAULT
//#define CONTROL_DEFAULT 		KEYBOARD_CTRL_MODE
#define CONTROL_DEFAULT			REMOTE_CTRL_MODE
//#define CONTROL_DEFAULT			SBC_CTRL_MODE
#endif

#ifndef DAMAGE_TIMEOUT
#define DAMAGE_TIMEOUT 5000
#endif

#ifndef CHECK_AMMO
//#define CHECK_AMMO
#endif

#ifndef PITCH_SURVEILLANCE
#define PITCH_SURVEILLANCE -0.1
#endif
/*********************** OTHERS ***********************/
#define MOTOR_TIMEOUT_MAX	1000

#define REMOTE_TIMEOUT 		200
#define MAX_RC_VALUE 		1320 //660 * 2
#define RC_LIMITS			660



#define KEY_OFFSET_W        ((uint16_t)0x01<<0)
#define KEY_OFFSET_S        ((uint16_t)0x01<<1)
#define KEY_OFFSET_A 		((uint16_t)0x01<<2)
#define KEY_OFFSET_D        ((uint16_t)0x01<<3)
#define KEY_OFFSET_Q        ((uint16_t)0x01<<6)
#define KEY_OFFSET_E        ((uint16_t)0x01<<7)
#define KEY_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define KEY_OFFSET_CTRL     ((uint16_t)0x01<<5)

// Unused keys
// To use these keys, have to make new file ?
#define KEY_OFFSET_R        ((uint16_t)0x01<<8)
#define KEY_OFFSET_F        ((uint16_t)0x01<<9)
#define KEY_OFFSET_G        ((uint16_t)0x01<<10)
#define KEY_OFFSET_Z        ((uint16_t)0x01<<11)
#define KEY_OFFSET_X        ((uint16_t)0x01<<12)
#define KEY_OFFSET_C        ((uint16_t)0x01<<13)
#define KEY_OFFSET_V        ((uint16_t)0x01<<14)
#define KEY_OFFSET_B        ((uint16_t)0x01<<15)

#endif /* TASKS_INC_ROBOT_CONFIG_H_ */

