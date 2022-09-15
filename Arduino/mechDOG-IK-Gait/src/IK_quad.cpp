/*
 *	Authors:		Eduardo Nunes
 *					Geraldine Barreto
 *	Version:		1.0
 *	Licence:		LGPL-3.0 (GNU Lesser General Public License)
 *	
 *	Description:	A library that performs inverse kinematics for a quadruped 
 *					robot.
 */

#include "IK_quad.h"
#include<math.h>

int16_t Joints::mechdog_joint_offsets[3];
int16_t Joints::mechdog_joint_minmax[2][3];
int16_t Joints::joint_offsets[3];
int16_t Joints::joint_minmax[2][3];

Joints::Joints(LSS_Robot_Model robot = MechDog){
	this->updateJointsParams(robot);
}
Joints::~Joints(void){}

void Joints::updateJointsParams(LSS_Robot_Model robot){
	for (uint8_t joint = 0; joint < 3; joint++) {
		if (robot == MechDog){
			joint_offsets[joint] = mechdog_joint_offsets[joint];
			joint_minmax[0][joint] = mechdog_joint_minmax[0][joint];
			joint_minmax[1][joint] = mechdog_joint_minmax[1][joint];
		}else{
			joint_offsets[joint] = 0;
			joint_minmax[0][joint] = 0;
			joint_minmax[1][joint] = 0;
		}
	}
}

void Joints::moveServos(int8_t id)
{	
	uint8_t leg = id/10;
	uint8_t joint = id - leg*10;
	leg--;
	joint--;
	
	int16_t angle = joint_angles[leg][joint] - joint_offsets[joint];
	if (angle < joint_minmax[0][joint]) angle = joint_minmax[0][joint];
	if (angle > joint_minmax[1][joint]) angle = joint_minmax[1][joint];
	LSS(id).move(angle);
}

void Joints::moveServos(Leg leg)
{	
	uint8_t id;
	int16_t angle;
	uint8_t leg_id = leg.leg_ID-1;
	for (uint8_t joint = 0; joint < 3; joint++) {
		angle = joint_angles[leg_id][joint] - joint_offsets[joint];
		if (angle < joint_minmax[0][joint]) angle = joint_minmax[0][joint];
		if (angle > joint_minmax[1][joint]) angle = joint_minmax[1][joint];
		id = (leg.leg_ID)*10 + joint + 1;
		LSS(id).move(angle);
	}
}
void Joints::moveServos(void)
{	
	uint8_t id;
	int16_t angle;
	for (uint8_t leg = 0; leg < 4; leg++) {
		for (uint8_t joint = 0; joint < 3; joint++) {
			angle = joint_angles[leg][joint] - joint_offsets[joint];
			if (angle < joint_minmax[0][joint]) angle = joint_minmax[0][joint];
			if (angle > joint_minmax[1][joint]) angle = joint_minmax[1][joint];
			id = (leg+1)*10 + joint+1;
			LSS(id).move(angle);
		}
	}
}

float Leg::foot_rad, Leg::L1, Leg::L2, Leg::L3;

Leg::Leg(uint8_t leg_id, int16_t offset_x, int16_t offset_z, LSS_Robot_Model robot = MechDog)
{	
	this->updateLegDimension(robot);
	
	this->leg_ID = leg_id;

	// Right legs have an even ID
	if (this->leg_ID <= 2) this->rightLeg = true;
	else this->rightLeg = false;   
	

	// Resting offsets
	switch(leg_id){
		case 1:
			this->rest_pos_x = offset_x;
			this->rest_pos_z = offset_z;
			break;
		case 2:
			this->rest_pos_x = -offset_x;
			this->rest_pos_z = offset_z;
			break;
		case 3:
			this->rest_pos_x = offset_x;
			this->rest_pos_z = -offset_z;
			break;
		default:
			this->rest_pos_x = -offset_x;
			this->rest_pos_z = -offset_z;		
	}
	this->new_rest_pos_x = this->rest_pos_x;
	this->new_rest_pos_z = this->rest_pos_z;

	// Last leg position
	this->xg = this->rest_pos_x;
	this->zg = this->rest_pos_z;
	
}

Leg::~Leg(void){}

void Leg::updateLegDimension(LSS_Robot_Model robot){
	if (robot == MechDog){
		L1 = L1_MECHDOG;
		L2 = L2_MECHDOG;
		L3 = L3_MECHDOG;
		foot_rad = FOOT_RAD_MECHDOG;
	}else if(robot == DeskPet){
		L1 = L1_DESKPET;    			// Abduction to rotation distance
		L2 = L2_DESKPET;  				// Rotation to knee distance
		L3 = L3_DESKPET;   				// Knee to foot distance
		foot_rad = FOOT_RAD_DESKPET;	// Foot radius
	}
}

void Leg::inverseKinematics(int16_t x, uint16_t y, int16_t z,int16_t joint_angles[4][3])
{
	float yf = y - foot_rad;
    int16_t dz;
    
    // Lateral displacement
    if (this->rightLeg) dz = z;
    else dz = -z;
    float beta = atan((L1+dz)/yf);
    float h = yf/cos(beta);
    float gamma = acos(L1/h);
    float abduction_angle = beta+gamma-PI/2.0;
    float Yz = h*sin(gamma);
    
    // Frontal displacement
    float theta = atan(x/Yz);
    float Y = Yz/cos(theta);

    // Height
    float knee_angle = acos((pow(L2,2)+pow(L3,2)-pow(Y,2))/(2.0*L3*L2));
    float rotation_angle = acos((pow(L2,2)+pow(Y,2)-pow(L3,2))/(2.0*Y*L2)) + theta;
    
    if (!isnan(abduction_angle) && !isnan(rotation_angle) && !isnan(knee_angle)){
      	joint_angles[this->leg_ID-1][0] = DEG(abduction_angle)*10;
		joint_angles[this->leg_ID-1][1] = DEG(rotation_angle)*10;
		joint_angles[this->leg_ID-1][2] = DEG(knee_angle)*10;
    }
}


Body::Body(LSS_Robot_Model robot = MechDog){
		this->model = robot;
	    this->updateRobotModel(robot);
		this->roll = 0;
		this->pitch = 0;
		this->yaw = 0;
		// Gait variables
        this->beta = Static;                  //1 for dynamic gait and 3 for static
        this->points = 4;                //Simplified trajectory has 2 point in air
        this->steps = (1+this->beta)*this->points;   //Total of points for a period of the foot trajectory
        this->trajectory_type = Circular;
        
        this->director_angle = 0;     
        this->move_state = 0;            //0 = Stop moving;
        this->rot_angle = 0;
        this->balancing = false;
        this->stopped = true;
        
        this->cont = 0;
        this->update_flag = false;
        this->new_director_angle = 0; 
        this->new_move_state = StopWalk;         //0 = Stop moving; 1 = Moving 
        this->new_rot_angle = StopRotation;
        this->new_beta = Static;
        
        //Initialization
		this->robotPostureInit();
		this->joints = Joints(robot);		
        for(int i = 0; i<4; i++){
		  this->legs[i] = Leg(i+1,this->X,this->Z,robot);
        }
}

Body::~Body(){};

int16_t Body::cgy_limits[2], Body::cgx_limits[2], Body::cgz_limits[2], Body::roll_limits[2], Body::pitch_limits[2], Body::yaw_limits[2];
int16_t Body::cgx_dynamic_gait, Body::cgx_static_gait, Body::cgy_std, Body::cgz_std;
uint8_t Body::foot_elevation, Body::jog_foot_elevation, Body::step_distance_dynamic, Body::step_distance_static;

void Body::updateRobotModel(LSS_Robot_Model robot = MechDog){
	if (robot == MechDog){
		// MECHDOG
		this->w = W_MECHDOG;
		this->l = L_MECHDOG;
		cgy_limits[min] = CGY_MECHDOG_LIMIT_MIN;
		cgy_limits[max] = CGY_MECHDOG_LIMIT_MAX;
		cgx_limits[min] = CGX_MECHDOG_LIMIT_MIN;
		cgx_limits[max] = CGX_MECHDOG_LIMIT_MAX;
		cgz_limits[min] = CGZ_MECHDOG_LIMIT_MIN;
		cgz_limits[max] = CGZ_MECHDOG_LIMIT_MAX;
		roll_limits[min] = ROLL_MECHDOG_LIMIT_MIN;
		roll_limits[max] = ROLL_MECHDOG_LIMIT_MAX;
		pitch_limits[min] = PITCH_MECHDOG_LIMIT_MIN;
		pitch_limits[max] = PITCH_MECHDOG_LIMIT_MAX;
		yaw_limits[min] = YAW_MECHDOG_LIMIT_MIN;
		yaw_limits[max] = YAW_MECHDOG_LIMIT_MAX;
		cgx_dynamic_gait = CGX_MECHDOG_DYNAMIC;
		cgx_static_gait = CGX_MECHDOG_STATIC;
		cgy_std = Y_MECHDOG;
		cgz_std = Z_MECHDOG;
		foot_elevation = A_MECHDOG;
		jog_foot_elevation = A_MECHDOG_JOG;
		step_distance_dynamic = B_MECHDOG_DYNAMIC;
		step_distance_static = B_MECHDOG_STATIC;

		this->X = X_MECHDOG;
		this->Z = Z_MECHDOG;
		this->balance_distance = BALANCE_DISTANCE_MECHDOG;
		
	}else if(robot == DeskPet){
		this->w = W_DESKPET;
		this->l = L_DESKPET;
		cgy_limits[min] = CGY_DESKPET_LIMIT_MIN;
		cgy_limits[max] = CGY_DESKPET_LIMIT_MAX;
		cgx_limits[min] = CGX_DESKPET_LIMIT_MIN;
		cgx_limits[max] = CGX_DESKPET_LIMIT_MAX;
		cgz_limits[min] = CGZ_DESKPET_LIMIT_MIN;
		cgz_limits[max] = CGZ_DESKPET_LIMIT_MAX;
		roll_limits[min] = ROLL_DESKPET_LIMIT_MIN;
		roll_limits[max] = ROLL_DESKPET_LIMIT_MAX;
		pitch_limits[min] = PITCH_DESKPET_LIMIT_MIN;
		pitch_limits[max] = PITCH_DESKPET_LIMIT_MAX;
		yaw_limits[min] = YAW_DESKPET_LIMIT_MIN;
		yaw_limits[max] = YAW_DESKPET_LIMIT_MAX;

		cgx_dynamic_gait = CGX_DESKPET_DYNAMIC;
		cgx_static_gait = CGX_DESKPET_STATIC;
		cgy_std = Y_DESKPET;
		cgz_std = Z_DESKPET;
		foot_elevation = A_DESKPET;
		jog_foot_elevation = A_DESKPET_JOG;
		step_distance_dynamic = B_DESKPET_DYNAMIC;
		step_distance_static = B_DESKPET_STATIC;

		this->X = X_DESKPET;
		this->Z = Z_DESKPET;
		this->balance_distance = BALANCE_DISTANCE_DESKPET;
	}
}

bool Body::trajectory(uint8_t leg_ID, uint8_t i, float* foot_pos){
	bool rest_pos_update = false;
	uint8_t a = this->a;
	uint8_t b = this->b;
	
	if (this->move_state == 0) b = 0;
	if(this->jog_mode && this->director_angle == 0) 
	{
		a = A_MECHDOG_JOG;
		b = 0;
	}

	if (this->legs[leg_ID-1].rest_pos_x != this->legs[leg_ID-1].new_rest_pos_x || this->legs[leg_ID-1].rest_pos_z != this->legs[leg_ID-1].new_rest_pos_z){
		if(i == 0){
			this->legs[leg_ID-1].rest_pos_x = this->legs[leg_ID-1].new_rest_pos_x;
			this->legs[leg_ID-1].rest_pos_z = this->legs[leg_ID-1].new_rest_pos_z;
		}
		rest_pos_update = true;
	}
	
	float xft =  this->legs[leg_ID-1].rest_pos_x;
	float zft =  this->legs[leg_ID-1].rest_pos_z;
	

	float z = (-1)*b*sin(this->director_angle);
	float x = b*cos(this->director_angle);


	float Acl, L, Zy = 0, Xy = 0;
	float L1 = Leg::Leg().L1;
	
	// Rotation
	if (leg_ID == 4){
		Acl = atan((this->l-xft)/(this->w+L1-zft));
		L = (this->w+L1-zft)/cos(Acl);
		Zy = (-L*cos(Acl-this->rot_angle)+this->w) + L1;
		Xy = (-L*sin(Acl-this->rot_angle)+this->l);
	}
	else if (leg_ID == 2){
		Acl = atan((this->l-xft)/(this->w+L1+zft));
		L = (this->w+L1+zft)/cos(Acl);
		Zy = (L*cos(Acl+this->rot_angle)-this->w) - L1;
		Xy = (-L*sin(Acl+this->rot_angle)+this->l);
	}
	else if (leg_ID == 3){
		Acl = atan((l+xft)/(this->w+L1-zft));
		L = (this->w+L1-zft)/cos(Acl);
		Zy = (-L*cos(Acl+this->rot_angle)+this->w) + L1;
		Xy = (L*sin(Acl+this->rot_angle)-this->l);
	}
	else if (leg_ID == 1){
		Acl = atan((this->l+xft)/(this->w+L1+zft));
		L = (this->w+L1+zft)/cos(Acl);     
		Zy = (L*cos(Acl-this->rot_angle)-this->w) - L1;
		Xy = (L*sin(Acl-this->rot_angle)-this->l);
	}

	xft = x + (Xy - this->legs[leg_ID-1].rest_pos_x);
	zft = z + (Zy - this->legs[leg_ID-1].rest_pos_z);

	float stepx = xft/(this->beta*2); //beta: 3 stationary gait / 1 dynamic gait
	float stepz = zft/(this->beta*2);
	
	float constant = PI/this->points;
	float y = 0;
	
	if (i < this->points){                    //Foot on the air
		if (this->trajectory_type == Circular){
			y = a*sin((i+1)*constant);
			x = xft*cos((i+1)*constant);
			z = zft*cos((i+1)*constant);
		}
		else if (this->trajectory_type == Square){
			if (i == 0){
				y = a;
				x = xft;
				z = zft;
			}
			else if (i == 1){
				y = a;
				x = 0;
				z = 0;
			}
			else if (i == 2){
				y = a;
				x = -xft;
				z = -zft;
			}
			else if (i == 3){
				y = 0;
				x = -xft;
				z = -zft;
			}
		}
		x += this->legs[leg_ID-1].rest_pos_x;
		z += this->legs[leg_ID-1].rest_pos_z;
		if (i == 3){
			this->legs[leg_ID-1].xg = x;
			this->legs[leg_ID-1].zg = z;
			y = 15;
		}
	}
	else{
		y = 0;
		this->legs[leg_ID-1].xg += stepx;
		this->legs[leg_ID-1].zg += stepz;
		x = this->legs[leg_ID-1].xg;
		z = this->legs[leg_ID-1].zg;
	}
	if(abs(this->legs[leg_ID-1].xg - this->legs[leg_ID-1].rest_pos_x)< 1 && abs(this->legs[leg_ID-1].zg - this->legs[leg_ID-1].rest_pos_z)<1 && (this->move_state == 0) && (this->rot_angle == 0) && !rest_pos_update) {
		y = 0;
		//returned values
		foot_pos[0] = x;
		foot_pos[1] = y;
		foot_pos[2] = z;
		return true;
	}else{
		//returned values
		foot_pos[0] = x;
		foot_pos[1] = y;
		foot_pos[2] = z;
	 	return false;
	}
}

void Body::balance(void){
    if(!this->balancing && (this->move_state != StopWalk || this->rot_angle != StopRotation || this->X != this->legs[0].new_rest_pos_x || this->Z != this->legs[0].new_rest_pos_z) ) this->balancing = true;
	if(this->balancing){
		this->cgz = int16_t((-1)*this->balance_distance*sin(PI*(this->cont-3)/8));
    }
}

void Body::update_traj(void){
	
    this->update_flag = false;

    this->director_angle = RADS(this->new_director_angle);
    this->move_state = this->new_move_state;
	this->jog_mode = this->new_jog_mode;
	if(this->jog_mode) this->move_state = 1;

    if(this->new_rot_angle == CCW){
		this->rot_angle = -ROTANGLE;
	} 
	else if(this->new_rot_angle == CW){
		this->rot_angle = ROTANGLE;
	} 
	else if(this->new_rot_angle == StopRotation){
		this->rot_angle = StopRotation;
	} 

    this->legs[0].new_rest_pos_x = this->X;
    this->legs[0].new_rest_pos_z = this->Z;

    this->legs[1].new_rest_pos_x = -this->X;
    this->legs[1].new_rest_pos_z = this->Z;

    this->legs[2].new_rest_pos_x = this->X;
    this->legs[2].new_rest_pos_z = -this->Z;
 
    this->legs[3].new_rest_pos_x = -this->X;
    this->legs[3].new_rest_pos_z = -this->Z;

  }

void Body::walk(void){
	
	bool is_posture_change = false;
	if(this->new_move_state == 0 && this->new_rot_angle == StopRotation && this->stopped) is_posture_change = true;
	
	uint8_t leg_order[4] = {3,0,2,1};
	if (this->beta == Static){
		leg_order[0] = 0;
		leg_order[1] = 1;
		leg_order[2] = 2;
		leg_order[3] = 3;
		balance();
	}

	// static to dynamic gait
	if (((this->cont == 3 || this->cont == 11) || this->stopped) && this->beta!=this->new_beta){
		this->beta = this->new_beta;
		this->steps = (1+this->new_beta)*this->points;
		this->cgx_blocked();
	}

	this->stopped = false;

	bool leg_stopped[4] = {this->stopped,this->stopped,this->stopped,this->stopped};

	float back_front = 0;
	float left_right = 0;
	if (this->update_flag){
		if ((WalkForward - this->new_director_angle) < AngleRange || this->new_director_angle < AngleRange){
			back_front = 1;         
		}else if(abs(this->new_director_angle - WalkBackward) < AngleRange){
			back_front = -1;
		}
		if (abs(this->new_director_angle - WalkRight) < AngleRange){
			left_right = 1;        
		}else if(abs(this->new_director_angle - WalkLeft) < AngleRange){
			left_right = -1;
		}
	}

	uint8_t i;
	for (uint8_t leg_number = 0; leg_number < 4; leg_number ++){
		i  = (this->cont+(leg_order[leg_number])*this->points)%this->steps;

		//Wait for the appropriate moment and leg before changing the trajectory of movement
		if (this->update_flag && i == 1){				
			if ((back_front >= 0 && left_right >= 0 && leg_number == 1) || (this->new_rot_angle == CW)){      //Front Right leg
				this->update_traj();
			}
			else if ((back_front >= 0 && left_right <= 0 && leg_number == 3) || (this->new_rot_angle == CCW)){ //Front Left leg
				this->update_traj();
			}
			else if ((back_front <= 0 && left_right >= 0 && leg_number == 0) || (this->new_rot_angle == CW)){ //Back Right leg
				this->update_traj(); 
			}
			else if ((back_front <= 0 && left_right <= 0 && leg_number == 2) || (this->new_rot_angle == CCW)){ //Back Left leg
				this->update_traj();
			}
			if (this->new_move_state == StopWalk && this->new_rot_angle == StopRotation){ 
				this->update_traj();
			}
		} 

		if(!is_posture_change){
			this->cgx_blocked();
		}
		float foot_pos[3];
		float leg_pos[3];
		leg_stopped[leg_number] = this->trajectory(leg_number+1, i,foot_pos);
		this->getLegPos(leg_number+1,foot_pos,leg_pos,0);
		this->legs[leg_number].inverseKinematics(leg_pos[0],leg_pos[1],leg_pos[2],joints.joint_angles);
	}

	if(leg_stopped[0]&&leg_stopped[1]&&leg_stopped[2]&&leg_stopped[3]){
		if(this->beta == Static && (this->cgz == 0 || is_posture_change) && this->new_rot_angle == StopRotation && this->new_move_state == 0 && !this->update_flag){
			this->stopped = true;
			this->sp_move = this->new_sp_move;
			this->balancing = false;
		}else if(this->beta == Dynamic && this->new_rot_angle == StopRotation && this->new_move_state == 0){
			this->stopped = true;
			this->sp_move = this->new_sp_move;
		}
	}

	this->cont = this->cont + 1;
	if (this->cont >= this->steps) this->cont = 0;	 
}

void Body::getLegPos(uint8_t leg_ID, const float foot_positions[3], float leg_pos[3], bool mode){
	//Convert degrees to radians
	float Acl, L, Zy = 0, Xy = 0, ypdif, yrdif, xb, zb, Z, Lx, Ly, Lz;
	//Calculate the X and Z displacement & eliminate leg offset
	float xft = foot_positions[0] + this->cgx;
	float zft = foot_positions[2] + this->cgz;
	float L1 = Leg::Leg().L1;
	if (leg_ID == 4){
		Acl = atan((l-xft)/(this->w+L1-zft));
		L = (this->w+L1-zft)/cos(Acl);
		Zy = (-L*cos(Acl-this->yaw)+this->w) + L1;
		Xy = (-L*sin(Acl-this->yaw)+this->l);
	}
	else if (leg_ID == 2){
		Acl = atan((l-xft)/(this->w+L1+zft));
		L = (this->w+L1+zft)/cos(Acl);
		Zy = (L*cos(Acl+this->yaw)-this->w) - L1;
		Xy = (-L*sin(Acl+this->yaw)+this->l);
	}
	else if (leg_ID == 3){
		Acl = atan((l+xft)/(this->w+L1-zft));
		L = (this->w+L1-zft)/cos(Acl);
		Zy = (-L*cos(Acl+this->yaw)+this->w) + L1;
		Xy = (L*sin(Acl+this->yaw)-this->l);
	}
	else if (leg_ID == 1){
		Acl = atan((l+xft)/(this->w+L1+zft));
		L = (this->w+L1+zft)/cos(Acl);          
		Zy = (L*cos(Acl-this->yaw)-this->w) - L1;
		Xy = (L*sin(Acl-this->yaw)-this->l);
	}
	
	if (leg_ID == 3 || leg_ID == 1) ypdif = -this->l*sin(this->pitch);
	else ypdif = this->l*sin(this->pitch);
	if (leg_ID == 2 || leg_ID == 1) yrdif = -this->w*sin(this->roll);
	else yrdif = this->w*sin(this->roll);
	
	//PITCH
	float yb = this->cgy + ypdif + yrdif - foot_positions[1];
	if (leg_ID == 3 || leg_ID == 1) xb = (this->l*(1-cos(this->pitch)));
	else xb = (this->l*cos(this->pitch)-this->l);
	xb = xb + Xy;
	float ax = atan(xb/yb);
	float pitch_t = this->pitch + ax;
	float hx = yb/cos(ax);
	yb = hx*cos(pitch_t);
	float X = hx*sin(pitch_t);

	//ROLL
	if (leg_ID == 2 || leg_ID == 1) zb = this->w-this->w*cos(this->roll) + L1;
	else zb = this->w*cos(this->roll)-this->w - L1;
	zb = zb + Zy;
	float az = atan((zb)/yb);
	float yh = yb/cos(az);
	float roll_t = this->roll + az;
	float Y = yh*cos(roll_t);
	if (leg_ID == 2 || leg_ID == 1) Z = yh*sin(roll_t) - L1;
	else Z = yh*sin(roll_t) + L1;
	
	if (mode == 0){
		Lz = Z;
		Lx = X;
	}
	else{
		Lz = Z + this->cgz;
		Lx = X + this->cgx;
	}
	Ly = Y;
	leg_pos[0] = Lx;
	leg_pos[1] = Ly;
	leg_pos[2] = Lz;
}

void Body::robotPostureInit(void){
	if(this->beta == Dynamic){
		this->cgx = cgx_dynamic_gait;
		this->cgy = cgy_std;
		this->cgz = cgz_std;
		this->a = foot_elevation;
		this->b = step_distance_dynamic;
	}
	else{
		this->beta = Static;
		this->cgx = cgx_static_gait;
		this->cgy = cgy_std;
		this->cgz = cgz_std;
		this->a = foot_elevation;
		this->b = step_distance_static;
	}

}

void Body::cgx_blocked(void){
	switch (this->model)
	{
	case MechDog:
		if (this->beta == Dynamic){
			this->cgx = CGX_MECHDOG_DYNAMIC;
		}else if (this->beta == Static){
			this->cgx = CGX_MECHDOG_STATIC;
		}
		break;
	case DeskPet:
		/* code */
		break;
	default:
		break;
	}
}


void Body::specialMoves(void){
	int y = 0;
    bool legs_updated[4] = {false,false,false,false};
    int mode = 0;
    //if(this->new_move_state != 0) this->new_sp_move = UP;
    if (this->model == MechDog){
      switch(this->sp_move){
		case SIT:
			switch(this->spm_state){
				case 0: //SIT PART 1
					this->stopped = false;
					this->cgx = 0; 
					this->cgz = 0;
					this->pitch = 0;
					this->yaw = 0;
					this->roll = 0;
					this->cgy = 120;
					this->spm_state = 1;
					break;
				case 1: //SIT PART 2
					this->pitch = RADS(30);           
					this->cgx = -40;
					this->spm_state = 2;
					break;
				case 2: //MOVE PAW
					this->spm_state = 3;
					y = 60;
					this->legs[3].zg = 20;
					legs_updated[3]=true;
					break;
				case 3: //MOVE PAW
					this->spm_state = 4;
					y = 0;
					legs_updated[3]=true;
					break;
				case 4: //MOVE PAW
					this->spm_state = 5;
					y = 60;
					this->legs[1].zg = -20;
					legs_updated[1]=true;
					break;
				case 5: //MOVE PAW
					this->stopped = true;
					y = 0;
					legs_updated[1]=true;
					if(this->new_sp_move != SIT){
						this->spm_state = 6;
						this->stopped = false;}
					break;
				case 6: //MOVE PAW
					this->spm_state = 7;
					y = 60;
					this->legs[3].zg = this->legs[3].rest_pos_z;
					legs_updated[3]=true;
					break;
				case 7:
					this->spm_state = 8;
					y = 0;
					legs_updated[3]=true;
					break;
				case 8: 
					this->spm_state = 9;
					y = 60;
					this->legs[1].zg = this->legs[1].rest_pos_z;
					legs_updated[1]=true;
					break;
				case 9: 
					this->spm_state = 10;
					y = 0;
					legs_updated[1]=true;
					break;
				case 10: 
					this->pitch = 0;
					this->cgx = 0;
					this->spm_state = 11;
					break;
				case 11: 
					this->cgy = Y_MECHDOG;
					this->spm_state = 0;
					this->sp_move = this->new_sp_move;
					if(this->sp_move == UP) this->stopped = true;
					break;
				}
        	break;
		case PAW:
			switch(this->spm_state){
				case 0: 
					this->stopped = false;
					this->cgx = -10; 
					this->cgy = 135; 
					this->cgz = 20;
					this->yaw = 0;
					this->roll = 0;
					y = 50;
					this->pitch = RADS(15);
					legs_updated[1] = true;
					this->spm_state = 1;
					break;
				case 1: 
					y = Y_MECHDOG;
					this->stopped = true;
					this->legs[1].xg = -85;
					this->legs[1].zg = -20;
					legs_updated[1]=true;
					if(this->new_sp_move != PAW){
						y = 50;
						this->legs[1].xg = -20;
						this->spm_state = 2;
						this->stopped = false;
					}
					break;
				case 2:
					y = 0;
					this->pitch = 0;
					this->cgx = 0;
					this->cgz = 0;
					this->legs[1].xg = this->legs[1].rest_pos_x;
					this->legs[1].zg = this->legs[1].rest_pos_z;
					legs_updated[1]=true;
					this->spm_state = 3;
					break;
				case 3: 
					this->spm_state = 4;
					break;
				case 4: 
					this->sp_move = this->new_sp_move;
					this->spm_state = 0;
					if(this->sp_move == UP) this->stopped = true;
					break;
				}    
			break;
		case LAY:
			switch(this->spm_state){
				case 0:
				this->stopped = false;
				this->cgx = 0;
				this->cgy = Y_MECHDOG;
				this->cgz = 0;
				this->pitch = 0;
				this->yaw = 0;
				this->roll = 0;
				this->spm_state = 1;
				break;
			case 1:
				this->stopped = true;
				this->cgx = 0;
				this->cgy = 60;
				this->cgz = 0;
				this->pitch = 0;
				this->yaw = 0;
				this->roll = 0;
				if(this->new_sp_move != LAY){
					this->cgy = Y_MECHDOG;
					this->spm_state = 2;
					this->stopped = false;
				}
				break;
			case 2: // time to stabilize
				this->spm_state = 3;
				break;
			case 3: 
				this->sp_move = this->new_sp_move;
				this->spm_state = 0;
				if(this->sp_move == UP) this->stopped = true;
				break;
			}
          	break;
		case TINKLE:
			switch(this->spm_state){
				case 0: 
					this->stopped = false;
					this->cgx = 30; 
					this->cgy = Y_MECHDOG; 
					this->cgz = 30;
					this->pitch = 0;
					this->yaw = 0;
					this->roll = 0;
					this->spm_state = 1;
				break;
				case 1: 
					y = Y_MECHDOG-70;
					this->legs[0].zg = 30;
					this->legs[0].xg = 0;
					legs_updated[this->legs[0].leg_ID-1]=true;
					this->spm_state = 2;
					break;
				case 2:
					this->stopped = true; 
					this->legs[0].zg = 120;
					y = Y_MECHDOG-70;
					legs_updated[this->legs[0].leg_ID-1]=true;
					if(this->new_sp_move != TINKLE){
						this->legs[0].zg = 20;
						this->spm_state = 3;
						this->stopped = false;
					}
					break; 
				case 3: 
					this->spm_state = 4;
					y = 0;
					this->legs[0].zg = this->legs[0].rest_pos_z;
					this->legs[0].xg = this->legs[0].rest_pos_x;
					legs_updated[this->legs[0].leg_ID-1]=true;
				    this->cgy = Y_MECHDOG; 
					break;
				case 4: 
					this->spm_state = 5;
					this->cgz = 0;
					this->cgx = 0;
				    this->cgy = Y_MECHDOG; 
					break; 
				case 5: //time to stabilize
					this->spm_state = 6;
					break;
				case 6: 
					this->spm_state = 0;
					this->sp_move = this->new_sp_move;
					if(this->sp_move == UP) this->stopped = true;
					break;
				}
			break;
		case WIGGLE:
			switch(this->spm_state){
			case 0: 
				this->stopped = false;
				this->cgx = -20; 
				this->cgz = 0;
				this->pitch = RADS(-13);
				this->yaw = 0;
				this->roll = 0;
				this->cgy = 135; 
				this->spm_state = 1;
				break;
			case 1:
				mode = 1;
				this->yaw = RADS(10);
				this->spm_state = 2;
				break;
			case 2:
				mode = 1;
				this->yaw = RADS(-10);
				this->spm_state = 1;
				if(this->new_sp_move != WIGGLE){
					this->yaw = 0;
					this->spm_state = 3;
				}
				break;
			case 3:
				this->pitch = 0;
				this->spm_state = 4;
				break;
			case 4:
				this->cgx = 0; 
				this->cgz = 0;
				this->cgy = Y_MECHDOG; 
				this->spm_state = 5;
				break;
			case 5: //time to stabilize
				this->spm_state = 6;
				break;
			case 6:
				this->spm_state = 0;
				this->sp_move = this->new_sp_move;
				if(this->sp_move == UP) this->stopped = true;
				break;
			}
			break;
		case STRETCH:
			switch(this->spm_state){
				case 0: 
					this->stopped = false;
					this->cgx = 0; 
					this->cgz = 0;
					this->pitch = 0;
					this->yaw = 0;
					this->roll = 0;
					this->cgy = Y_MECHDOG; 
					this->spm_state = 1;
					break;
				case 1: 
					this->cgx = -20; 
					this->cgy = 110; 
					this->spm_state = 2;
					break;
				case 2: 
					this->cgx = -50; 				
					this->pitch = RADS(-14);
					this->cgy = 120; 
					this->spm_state = 3;
				case 3: 
					this->yaw = 0;
					this->roll = 0;
					this->spm_state = 7;
					break;
					break;
				case 4: 
					this->yaw = 5;
					this->roll = 5;
					this->spm_state = 5;
					break;
				case 5: 
					this->yaw = 0;
					this->roll = 0;
					this->spm_state = 6;
					break;
				case 6: 
					this->yaw = -5;
					this->roll = -5;
					this->spm_state = 7;
					break;	
				case 7: 
					this->yaw = 0;
					this->roll = 0;
					this->stopped = true;
					if(this->new_sp_move != STRETCH){
						this->pitch = 0;
						this->cgx = -20; 
						this->spm_state = 8;
						this->stopped = false;
						}
					break;	
				case 8: 
					this->cgx = 0; 
					this->cgz = 0;
					this->yaw = 0;
					this->cgy = Y_MECHDOG; 
					this->spm_state = 9;
					break;	
				case 9: 
					this->spm_state = 0;
					this->sp_move = this->new_sp_move;
					if(this->sp_move == UP) this->stopped = true;
					break;
			}
			break;
		default:
				this->new_sp_move = UP;
				this->sp_move = UP;
		}
    }

	float foot_pos[3] = {0,0,0};
	float leg_pos[3];
    for(uint8_t i = 0; i < 4; i++){
		if (legs_updated[i]){
			foot_pos[1] = y;
			Serial.print(" Leg updated "); Serial.print(i);
			Serial.print(" y: "); Serial.print(y); 
			Serial.println(); 
		}
		else foot_pos[1] = 0;
		foot_pos[0] = this->legs[i].xg;
		foot_pos[2] = this->legs[i].zg;
		this->getLegPos(i+1,foot_pos,leg_pos,0);
		this->legs[i].inverseKinematics(leg_pos[0],leg_pos[1],leg_pos[2],joints.joint_angles);
	}
}

void Body::updateleg(bool* legs_updated, float Y){
	// if leg == 0 update all legs, else only update the selected leg
	float y = 0;
	float leg_pos[3];
	float foot_pos[3] = {0,0,0};
	for(uint8_t i = 0; i < 4; i++){
		if (legs_updated[i]) foot_pos[1] = Y;
		else foot_pos[1] = 0;

		getLegPos(i+1,foot_pos,leg_pos,0);
		this->legs[i].inverseKinematics(leg_pos[0],leg_pos[1],leg_pos[2],joints.joint_angles);
	}

}

