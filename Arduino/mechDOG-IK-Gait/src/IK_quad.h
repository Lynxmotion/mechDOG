/*
 *	Authors:		Eduardo Nunes
 *					Geraldine Barreto
 *	Version:		1.0
 *	Licence:		LGPL-3.0 (GNU Lesser General Public License)
 *	
 *	Description:	A library that performs inverse kinematics for a quadruped 
 *					robot.
 */

#define DEG(radians)  (radians*57.29578)
#define RADS(deg)  (deg/57.29578)

#ifndef IK_QUAD
#define IK_QUAD

#include "Arduino.h"
#include "string.h"
#include "LSS.h"
#include <math.h>

#define L1_MECHDOG 52
#define L2_MECHDOG 93.4
#define L3_MECHDOG 90
#define FOOT_RAD_MECHDOG 15
#define W_MECHDOG 75/2
#define L_MECHDOG 200/2
#define A_MECHDOG 45
#define A_MECHDOG_JOG 30
#define B_MECHDOG_DYNAMIC 20
#define B_MECHDOG_STATIC 20
#define CGX_MECHDOG_DYNAMIC 40
#define CGX_MECHDOG_STATIC 15
#define CGZ_MECHDOG 0
#define X_MECHDOG 0
#define Z_MECHDOG 0
#define Y_MECHDOG 140
//Limits
#define CGY_MECHDOG_LIMIT_MIN 60
#define CGY_MECHDOG_LIMIT_MAX 160
#define CGX_MECHDOG_LIMIT_MIN -45
#define CGX_MECHDOG_LIMIT_MAX 45
#define CGZ_MECHDOG_LIMIT_MIN -30
#define CGZ_MECHDOG_LIMIT_MAX 30
#define ROLL_MECHDOG_LIMIT_MIN -20 //degrees
#define ROLL_MECHDOG_LIMIT_MAX 20
#define PITCH_MECHDOG_LIMIT_MIN -20
#define PITCH_MECHDOG_LIMIT_MAX 20
#define YAW_MECHDOG_LIMIT_MIN -20
#define YAW_MECHDOG_LIMIT_MAX 20
#define BALANCE_DISTANCE_MECHDOG 30


//no tested yet
#define L1_DESKPET 			58
#define L2_DESKPET 			48.5
#define L3_DESKPET 			72.8
#define FOOT_RAD_DESKPET 	4.5
#define W_DESKPET 			51.5/2
#define L_DESKPET 			89.8/2
#define A_DESKPET 			20
#define A_DESKPET_JOG		10
#define B_DESKPET_DYNAMIC   15
#define B_DESKPET_STATIC	20
#define B_DESKPET 			20
#define X_DESKPET 			10
#define Z_DESKPET 			0
#define Y_DESKPET 			90
#define CGX_DESKPET_DYNAMIC 20
#define CGX_DESKPET_STATIC 10
//Limits
#define CGY_DESKPET_LIMIT_MIN 50
#define CGY_DESKPET_LIMIT_MAX 110
#define CGX_DESKPET_LIMIT_MIN -25
#define CGX_DESKPET_LIMIT_MAX 25
#define CGZ_DESKPET_LIMIT_MIN -20
#define CGZ_DESKPET_LIMIT_MAX 20
#define ROLL_DESKPET_LIMIT_MIN -20 //degrees
#define ROLL_DESKPET_LIMIT_MAX 20
#define PITCH_DESKPET_LIMIT_MIN -20
#define PITCH_DESKPET_LIMIT_MAX 20
#define YAW_DESKPET_LIMIT_MIN -20
#define YAW_DESKPET_LIMIT_MAX 20
#define BALANCE_DISTANCE_DESKPET 15

#define ROTANGLE 			0.1745 //10 deg in rads

#define StopWalk 			0
#define WalkForward 		360
#define WalkBackward 		180
#define WalkRight 			90
#define WalkLeft 			270

#define AngleRange 			60

enum min_max
{
	min,
	max
};

enum Rotation_Dir
{
    CCW = -1,
    StopRotation,
    CW
};

enum Foot_Trajectory
{
	Circular,
	Square
};

enum Gait_Type
{
	Dynamic = 1,
	Static = 3
};

enum LSS_Robot_Model
{
	DeskPet,
	MechDog
};

enum Special_Moves
{
	UP, 
	SIT, 
	LAY, 
	PAW, 
	WIGGLE, 
	TINKLE,
	STRETCH,
	JOG_ON,
	JOG_OFF
};

class Leg
{
	public:
		// Public attributes - Class
		float a1, a2, a3, xg, zg;
		float angle[3];
		uint8_t leg_ID;
		int8_t rest_pos_x, rest_pos_z, new_rest_pos_x, new_rest_pos_z;
		bool rightLeg = false;
		static float foot_rad, L1, L2, L3;
		// Public functions - Instance
		//> Constructors/destructor
		Leg(uint8_t leg_id=1, int16_t offset_x=0, int16_t offset_z=0, LSS_Robot_Model robot = MechDog);
		~Leg(void);
		// Public functions - Class
		void updateLegDimension(LSS_Robot_Model robot);
		void inverseKinematics(int16_t x, uint16_t y, int16_t z, int16_t joint_angles[4][3]);
		int16_t * Leg::inverseKinematics(int16_t x, uint16_t y, int16_t z);

	private:
		
};

class Joints
{
	public:
		int16_t joint_angles[4][3] =   {{0,0,0},
										{0,0,0},
										{0,0,0},
										{0,0,0}};
		Joints(LSS_Robot_Model robot = MechDog);
		~Joints(void);
		void updateJointsParams(LSS_Robot_Model robot);
		void moveServos(int8_t id);
		void moveServos(Leg leg);
		void moveServos(void);
		

	private:
		static int16_t mechdog_joint_offsets[3] = {0,745,155};
		static int16_t mechdog_joint_minmax[2][3] = {{-450,-600,0},	//min
													 {450,600,1800}}; //max
		static int16_t joint_offsets[3];
		static int16_t joint_minmax[2][3];
};

class Body
{
	public:
		Leg legs[4];
    	Joints joints; 
		// Gait variables
		float new_director_angle;
    	bool update_flag = false, stopped = true, new_jog_mode = false, jog_mode = false;
		int8_t new_move_state = 0;
		float roll = 0, pitch = 0, yaw = 0;
		int16_t cgx = 0, cgy = 90, cgz = 0;
		Rotation_Dir new_rot_angle = StopRotation;
		Foot_Trajectory trajectory_type = Circular;
		Gait_Type new_beta = Static;
		int16_t balance_distance, w, l, X, Z;
		static int16_t cgy_limits[2], cgx_limits[2], cgz_limits[2], roll_limits[2], pitch_limits[2], yaw_limits[2];
		static int16_t cgx_dynamic_gait, cgx_static_gait, cgy_std, cgz_std;
		static uint8_t foot_elevation, jog_foot_elevation, step_distance_dynamic, step_distance_static;
		//Special moves
		Special_Moves new_sp_move = UP, sp_move = UP;
		LSS_Robot_Model model;
		uint8_t spm_state = 0;
		Gait_Type beta = Static;
		Body(LSS_Robot_Model robot = MechDog);
		~Body();
		void walk(void);
		void specialMoves(void);
		void robotPostureInit(void);

	private:
		// Gait variables
		
    	int8_t points = 4, move_state = 0,  cont = 0, steps;
		uint8_t a, b;
		
		bool balancing = false;
		float director_angle;
		float rot_angle = 0;
		//Special moves
		void cgx_blocked(void);
		void getLegPos(uint8_t leg_ID, const float foot_positions[3], float leg_pos[3], bool mode);
		void updateRobotModel(LSS_Robot_Model robot = MechDog);
		void update_traj(void);
		void balance(void);
		bool trajectory(uint8_t leg_ID, uint8_t i, float* foot_pos);
		void updateleg(bool* legs_updated, float Y);
};

#endif