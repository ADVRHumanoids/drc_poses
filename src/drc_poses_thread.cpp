/* Copyright [2014,2015] [Alessandro Settimi, Luca Muratore]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#include <yarp/os/all.h>

#include "drc_poses_thread.h"
#include "drc_poses_constants.h"
#include "paths.h"

#include <iostream>
#include <fstream>


using namespace walkman::drc::poses;
using namespace yarp::math;
using namespace std;

#define Max_Vel         0.5 // maximum joint velocity [rad/s]
#define Min_Texe        2.0 // minimum execution time for homing [sec]

#define RAD2DEG    (180.0/M_PI)
#define DEG2RAD    (M_PI/180.0)
#define PI         3.14

#define MIN_CLOSURE 0.0 * DEG2RAD
#define MAX_CLOSURE 600.0 * DEG2RAD

bool drc_poses_thread::move_hands(double close)
{   
    return move_hands(close, close);
}

bool drc_poses_thread::move_hands(double left_fraction, double right_fraction)
{   
  if (left_fraction <= 1.0 && left_fraction >= 0.0 && right_fraction <= 1.0 && right_fraction >= 0.0) 
  {
      yarp::sig::Vector q_right_hand(1), q_left_hand(1);
      q_right_hand[0]  = MIN_CLOSURE + right_fraction*(MAX_CLOSURE - MIN_CLOSURE); 
      q_left_hand[0]   = MIN_CLOSURE + left_fraction* (MAX_CLOSURE - MIN_CLOSURE);
      robot.left_hand.move(q_left_hand); 
      robot.right_hand.move(q_right_hand); 
      return true;
  }
  else
  {
      std::cout<<"closing amount is out of feasible bounds"<<std::endl;
  }
   return false;
}

// double left_offset[7] = {-0.001878101, -0.087266425, -0.00541460025, -0.04116454775, -0.0270602895, 0.05685963075, 0.05985226625};
// double right_offset[7] = {0.00496249625, 0.01221735225, 0.023223271, -0.01633125525, -0.04591635675, 0.0131223505, -0.0860596935};
double left_offset[7] = {0, 0, 0, 0, 0, 0, 0};
double right_offset[7] = {0, 0, 0, 0, 0, 0, 0};
yarp::sig::Vector left_arm_offset(7,left_offset);
yarp::sig::Vector right_arm_offset(7,right_offset);


drc_poses_thread::drc_poses_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr< paramHelp::ParamHelperServer > ph)
:control_thread( module_prefix, rf, ph ), cmd_interface(module_prefix), status_interface(module_prefix)
{
    kinematic_joints = robot.getNumberOfJoints();
    actuated_joints  = robot.getNumberOfJoints();
    right_arm_joints = robot.right_arm.getNumberOfJoints();
    left_arm_joints  = robot.left_arm.getNumberOfJoints();
    torso_joints     = robot.torso.getNumberOfJoints();
    right_leg_joints = robot.right_leg.getNumberOfJoints();
    left_leg_joints  = robot.left_leg.getNumberOfJoints();
    head_joints      = robot.head.getNumberOfJoints();
  
    std::string yamlPath = BUILD_PATH;
    yamlPath += "/poses.yaml";
    loadPoses(yamlPath);
    q_input.resize(kinematic_joints);
    delta_q.resize(kinematic_joints);
    q_output.resize(kinematic_joints);

    create_poses();
    updateCommandList();
    isInPoseSeriesMode = false;
    
    q_hands_desired.resize(2);
    
    
    next_time=0.0;
    final_time=Min_Texe;   //devel_JLee
    
    
    
    // wave 1
    
//     wave_q_torso_start.resize(3);
//     wave_q_torso_start[0] = 0.0*DEG2RAD;
//     wave_q_torso_start[1] = -10.0*DEG2RAD;
//     wave_q_torso_start[2] = 90.0*DEG2RAD;

    wave_q_right_arm_start.resize(right_arm_joints);
    wave_q_right_arm_start[0] = 20.0*DEG2RAD;
    wave_q_right_arm_start[1] = -80.0*DEG2RAD;
    wave_q_right_arm_start[2] = -80.0*DEG2RAD;
    wave_q_right_arm_start[3] = -100.0*DEG2RAD;
    wave_q_right_arm_start[4] = 0.0*DEG2RAD;
    wave_q_right_arm_start[5] = 0.0*DEG2RAD;
    wave_q_right_arm_start[6] = 0.0*DEG2RAD;
  
    wave_q_left_arm_start.resize(left_arm_joints);
    wave_q_left_arm_start[0] = 20.0*DEG2RAD;
    wave_q_left_arm_start[1] = 80.0*DEG2RAD;
    wave_q_left_arm_start[2] = 80.0*DEG2RAD;
    wave_q_left_arm_start[3] = -100.0*DEG2RAD;
    wave_q_left_arm_start[4] = 0.0*DEG2RAD;
    wave_q_left_arm_start[5] = 0.0*DEG2RAD;
    wave_q_left_arm_start[6] = 0.0*DEG2RAD;
    
    wave_q_head_start.resize(head_joints);
    wave_q_head_start[0] = -25.0*DEG2RAD;
    wave_q_head_start[1] = 0.0;
      
    // wave 2
    wave_q_right_arm_end.resize(right_arm_joints);
    wave_q_right_arm_end[0] = 20.0*DEG2RAD;
    wave_q_right_arm_end[1] = -80.0*DEG2RAD;
    wave_q_right_arm_end[2] = -80.0*DEG2RAD;
    wave_q_right_arm_end[3] = -140.0*DEG2RAD;
    wave_q_right_arm_end[4] = 0.0*DEG2RAD;
    wave_q_right_arm_end[5] = 0.0*DEG2RAD;
    wave_q_right_arm_end[6] = 0.0*DEG2RAD;
    
    wave_q_left_arm_end.resize(left_arm_joints);
    wave_q_left_arm_end[0] = 20.0*DEG2RAD;
    wave_q_left_arm_end[1] = 80.0*DEG2RAD;
    wave_q_left_arm_end[2] = 80.0*DEG2RAD;
    wave_q_left_arm_end[3] = -140.0*DEG2RAD;
    wave_q_left_arm_end[4] = 0.0*DEG2RAD;
    wave_q_left_arm_end[5] = 0.0*DEG2RAD;
    wave_q_left_arm_end[6] = 0.0*DEG2RAD;
    
    wave_q_head_end.resize(head_joints);
    wave_q_head_end[0] = 25.0*DEG2RAD;
    wave_q_head_end[1] = 0.0;
    
    //recover pose
    recover_q_right_arm.resize(right_arm_joints);
    recover_q_left_arm.resize(left_arm_joints);
    recover_q_torso.resize(torso_joints);
    recover_q_right_leg.resize(right_leg_joints);
    recover_q_left_leg.resize(left_leg_joints);
    recover_q_head.resize(head_joints);
    
    recover_q_right_arm[0]=60.0*DEG2RAD;
    recover_q_right_arm[1]=-10.0*DEG2RAD;
    recover_q_right_arm[2]=20.0*DEG2RAD;
    recover_q_right_arm[3]=-110.0*DEG2RAD;
    recover_q_right_arm[4]=0.0*DEG2RAD;
    recover_q_right_arm[5]=-30.0*DEG2RAD;
    recover_q_right_arm[6]=0.0*DEG2RAD;

    recover_q_left_arm[0]=60.0*DEG2RAD;
    recover_q_left_arm[1]=10.0*DEG2RAD;
    recover_q_left_arm[2]=-20.0*DEG2RAD;
    recover_q_left_arm[3]=-110.0*DEG2RAD;
    recover_q_left_arm[4]=0.0*DEG2RAD;
    recover_q_left_arm[5]=-30.0*DEG2RAD;
    recover_q_left_arm[6]=0.0*DEG2RAD;
    
    recover_q_torso[0] = 0.0;
    recover_q_torso[1] = 0.0;
    recover_q_torso[2] = 0.0;
    
    //walking pose
    walking_q_right_arm.resize(right_arm_joints);
    walking_q_left_arm.resize(left_arm_joints);
    walking_q_torso.resize(torso_joints);
    walking_q_right_leg.resize(right_leg_joints);
    walking_q_left_leg.resize(left_leg_joints);
    walking_q_head.resize(head_joints);
    
    walking_q_right_arm[0]=60.0*DEG2RAD;
    walking_q_right_arm[1]=-10.0*DEG2RAD;
    walking_q_right_arm[2]=20.0*DEG2RAD;
    walking_q_right_arm[3]=-110.0*DEG2RAD;
    walking_q_right_arm[4]=0.0*DEG2RAD;
    walking_q_right_arm[5]=-30.0*DEG2RAD;
    walking_q_right_arm[6]=50.0*DEG2RAD;

    walking_q_left_arm[0]=60.0*DEG2RAD;
    walking_q_left_arm[1]=10.0*DEG2RAD;
    walking_q_left_arm[2]=-20.0*DEG2RAD;
    walking_q_left_arm[3]=-110.0*DEG2RAD;
    walking_q_left_arm[4]=0.0*DEG2RAD;
    walking_q_left_arm[5]=-30.0*DEG2RAD;
    walking_q_left_arm[6]=-50.0*DEG2RAD;
    
    walking_q_torso[0] = 0.0;
    walking_q_torso[1] = 0.0;
    walking_q_torso[2] = 0.0;
    
    walking_q_head[0] = 0.0;
    walking_q_head[1] = 0.0;
    
    //driving pose
    drive_q_right_arm.resize(right_arm_joints);
    drive_q_left_arm.resize(left_arm_joints);
    drive_q_torso.resize(torso_joints);
    drive_q_right_leg.resize(right_leg_joints);
    drive_q_left_leg.resize(left_leg_joints);
    drive_q_head.resize(head_joints);
    
    drive_q_left_arm[0]=-10*DEG2RAD;
    drive_q_left_arm[1]=45*DEG2RAD;
    drive_q_left_arm[2]=20*DEG2RAD;
    drive_q_left_arm[3]=-100*DEG2RAD;
    drive_q_left_arm[4]=30*DEG2RAD;
    drive_q_left_arm[5]=-20*DEG2RAD;
    drive_q_left_arm[6]=0*DEG2RAD;
    
    drive_q_torso[0]=0*DEG2RAD;
    drive_q_torso[1]=-7*DEG2RAD;
    drive_q_torso[2]=45*DEG2RAD;
    
    //pre_homing pose
    pre_homing_q_right_arm.resize(right_arm_joints);
    pre_homing_q_left_arm.resize(left_arm_joints);
    pre_homing_q_torso.resize(torso_joints);
    pre_homing_q_right_leg.resize(right_leg_joints);
    pre_homing_q_left_leg.resize(left_leg_joints);
    pre_homing_q_head.resize(head_joints);
    pre_homing_q_left_hand.resize(1);
    pre_homing_q_right_hand.resize(1);
    
    pre_homing_q_left_arm[0]=0.0;
    pre_homing_q_left_arm[1]=0.4;
    pre_homing_q_left_arm[2]=0.8;
    pre_homing_q_left_arm[3]=-0.6;
    pre_homing_q_left_arm[4]=0.0;
    pre_homing_q_left_arm[5]=0.0;
    pre_homing_q_left_arm[6]=0.0;
    
    pre_homing_q_right_arm[0]=0.0;
    pre_homing_q_right_arm[1]=-0.4;
    pre_homing_q_right_arm[2]=-0.8;
    pre_homing_q_right_arm[3]=-0.6;
    pre_homing_q_right_arm[4]=0.0;
    pre_homing_q_right_arm[5]=0.0;
    pre_homing_q_right_arm[6]=0.0;

    pre_homing_q_left_hand[0]=0.0;
    pre_homing_q_right_hand[0]=0.0;
    
    pre_homing_q_torso[0]=0.0;
    pre_homing_q_torso[1]=0.0;
    pre_homing_q_torso[2]=0.0;

    pre_homing_q_left_leg[0]=0.2;
    pre_homing_q_right_leg[0]=-0.2;

    pre_homing_q_head[0]=0.0;
    pre_homing_q_head[1]=0.0;
}

bool drc_poses_thread::custom_init()
{
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);

    status_interface.start();
    status_interface.setStatus( "ready", status_seq_num++ );

    q_input=joint_sense();
    robot.idynutils.updateiDyn3Model(q_input, false);
    robot.setPositionDirectMode();

    q_initial = q_input;
    q_desired = q_input;

    return true;
}

yarp::sig::Vector drc_poses_thread::joint_sense()
{
    yarp::sig::Vector out(kinematic_joints);
    
    yarp::sig::Vector in = robot.senseMotorPosition(); //REAL_ROBOT
//     yarp::sig::Vector in = robot.sensePosition(); //SIMULATION

    for(int i=0;i<out.size();i++) out[i]=in[i];
    
    return out;
}

void drc_poses_thread::run()
{
    q_input=joint_sense();

    yarp::sig::Vector q_torso(3), q_left_arm(7), q_right_arm(7), q_left_leg(6), q_right_leg(6), q_head(2);
    robot.fromIdynToRobot(q_input, q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head);
    
    //OFFSET 
    q_left_arm = q_left_arm - left_arm_offset;
    q_right_arm = q_right_arm - right_arm_offset;
    
    robot.fromRobotToIdyn(q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head, q_input);

    robot.idynutils.updateiDyn3Model( q_input, false );

    if(action_completed()) 
    {
	busy=false;
	status_interface.setStatus( status_definitions.status_to_code.at("ready"), status_seq_num++ );
    }

    if(!busy)
    {
	std::string cmd, err="";
	if(cmd_interface.getCommand(cmd,cmd_seq_num) || demo_mode || isInPoseSeriesMode)
	{
	    if(demo_mode)
	    {
            // standard demo
            if(last_command=="demo") cmd="demo2";
            if(last_command=="demo0") cmd="demo1";
            if(last_command=="demo1") cmd="demo2";
            if(last_command=="demo2") cmd="demo3";
            if(last_command=="demo3") cmd="demo4";
            if(last_command=="demo4") cmd="demo5";
            if(last_command=="demo5") cmd="demo6";
            if(last_command=="demo6") cmd="demo7";
            if(last_command=="demo7") cmd="demo8";
            if(last_command=="demo8") cmd="demo9";
            if(last_command=="demo9") cmd="demo13"; // NOTE jump to 13
            if(last_command=="demo10") cmd="demo11";
            if(last_command=="demo11") cmd="demo12";
            if(last_command=="demo12") cmd="demo13";
            if(last_command=="demo13") cmd="demo14";
            if(last_command=="demo14") cmd="demo15";
            if(last_command=="demo15") cmd="demo16";
            if(last_command=="demo16") cmd="demo17";
            if(last_command=="demo17") demo_mode=false;
            
            // 1 HAND 
            if(last_command=="hello_1") cmd="wave_1";
            if(last_command=="wave_1") cmd="wave_2";
            if(last_command=="wave_2") cmd="wave_1";

            // 2 HANDS
            if(last_command=="hello_2") cmd="wave_2_1";
            if(last_command=="wave_2_1") cmd="wave_2_2";
            if(last_command=="wave_2_2") cmd="wave_2_1";
	    }

	    if( (cmd == "hello_1") || (cmd == "hello_2") || (cmd == "demo") )
	    {
            demo_mode=true;
	    }

	    last_command=cmd;

	    if(cmd=="help")
	    {
            print_help();
	    }
	    else if (cmd=="reloadYAML") {
            cout << "Reloading YAML poses file " << endl;
            std::string yamlPath = BUILD_PATH;
            yamlPath += "/poses.yaml";
            loadPoses(yamlPath);
            updateCommandList();
        } else if (cmd=="savePose") {
            static int poseCounter=0; 
            cout << "Saving pose to YAML file " << endl;

            yarp::sig::Vector q_in(kinematic_joints), tmp;

            tmp = robot.sensePosition(); //REAL_ROBOT
            for(int i=0;i<q_in.size();i++) q_in[i]=tmp[i];
            std::string yamlPath = BUILD_PATH;
            yamlPath += "/posesOut.yaml";
            
            savePose(q_in, yamlPath, poseCounter);
            poseCounter++;
        } else {
        err = (std::find(commands.begin(), commands.end(), cmd)!=commands.end())?"":"UNKWOWN";
        std::cout<<">> Received command ["<<cmd_seq_num<<"]: "<<cmd<<" "<<err<<std::endl;

		if(err=="" || isInPoseSeriesMode)
		{
		    busy=true;
		    initialized_time = yarp::os::Time::now();
		    q_initial = q_input;

		    if(cmd=="recover") //We don't want to move the legs in this case
		    {
			yarp::sig::Vector q(kinematic_joints);
			yarp::sig::Vector q_in(kinematic_joints);
			yarp::sig::Vector q_right_arm(right_arm_joints);
			yarp::sig::Vector q_left_arm(left_arm_joints);
			yarp::sig::Vector q_torso(torso_joints);
			yarp::sig::Vector q_right_leg(right_leg_joints);
			yarp::sig::Vector q_left_leg(left_leg_joints);
			yarp::sig::Vector q_head(head_joints);

			q_in=joint_sense();

			robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			recover_q_left_leg = q_left_leg;
			recover_q_right_leg = q_right_leg;
			recover_q_head = q_head;

			robot.fromRobotToIdyn(recover_q_right_arm,recover_q_left_arm,recover_q_torso,recover_q_right_leg,recover_q_left_leg,recover_q_head,q);
			poses["recover"] = q;
		    }
		    
		    if(cmd=="walking") //We don't want to move the legs in this case
		    {
			yarp::sig::Vector q(kinematic_joints);
            yarp::sig::Vector q_in(kinematic_joints);
            yarp::sig::Vector q_right_arm(right_arm_joints);
            yarp::sig::Vector q_left_arm(left_arm_joints);
            yarp::sig::Vector q_torso(torso_joints);
            yarp::sig::Vector q_right_leg(right_leg_joints);
            yarp::sig::Vector q_left_leg(left_leg_joints);
            yarp::sig::Vector q_head(head_joints);

            q_in=joint_sense();

            robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

            walking_q_left_leg = q_left_leg;
            walking_q_right_leg = q_right_leg;
            //walking_q_head = q_head;

            robot.fromRobotToIdyn(walking_q_right_arm,walking_q_left_arm,walking_q_torso,walking_q_right_leg,walking_q_left_leg,walking_q_head,q);
            poses["walking"] = q;
		    }
		    
		    if(cmd=="driving") //We don't want to move the legs in this case
		    {
			yarp::sig::Vector q(kinematic_joints);
			yarp::sig::Vector q_in(kinematic_joints);
			yarp::sig::Vector q_right_arm(right_arm_joints);
			yarp::sig::Vector q_left_arm(left_arm_joints);
			yarp::sig::Vector q_torso(torso_joints);
			yarp::sig::Vector q_right_leg(right_leg_joints);
			yarp::sig::Vector q_left_leg(left_leg_joints);
			yarp::sig::Vector q_head(head_joints);

			q_in=joint_sense();

			robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			drive_q_left_leg = q_left_leg;
			drive_q_right_leg = q_right_leg;
			drive_q_head = q_head;
			drive_q_right_arm = q_right_arm;

			robot.fromRobotToIdyn(drive_q_right_arm,drive_q_left_arm,drive_q_torso,drive_q_right_leg,drive_q_left_leg,drive_q_head,q);
			poses["driving"] = q;
		    }

		    if(cmd=="pre_homing") //We don't want to move the legs except for the first joint in this case
		    {
			yarp::sig::Vector q(kinematic_joints);
			yarp::sig::Vector q_in(kinematic_joints);
			yarp::sig::Vector q_right_arm(right_arm_joints);
			yarp::sig::Vector q_left_arm(left_arm_joints);
			yarp::sig::Vector q_torso(torso_joints);
			yarp::sig::Vector q_right_leg(right_leg_joints);
			yarp::sig::Vector q_left_leg(left_leg_joints);
			yarp::sig::Vector q_head(head_joints);

			q_in=joint_sense();

			robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			q_left_leg[0] = pre_homing_q_left_leg[0];
			pre_homing_q_left_leg = q_left_leg;
			q_right_leg[0] = pre_homing_q_right_leg[0];
			pre_homing_q_right_leg = q_right_leg;

			robot.fromRobotToIdyn(pre_homing_q_right_arm,pre_homing_q_left_arm,pre_homing_q_torso,pre_homing_q_right_leg,pre_homing_q_left_leg,pre_homing_q_head,q);
            robot.left_hand.move(pre_homing_q_left_hand);
            robot.right_hand.move(pre_homing_q_right_hand);
			poses["pre_homing"] = q;
		    }
		    
		    // HELLO 1 HAND start
		    if(cmd=="wave_1")
		    {
			yarp::sig::Vector q(kinematic_joints);
			yarp::sig::Vector q_in(kinematic_joints);
			yarp::sig::Vector q_right_arm(right_arm_joints);
			yarp::sig::Vector q_left_arm(left_arm_joints);
			yarp::sig::Vector q_torso(torso_joints);
			yarp::sig::Vector q_right_leg(right_leg_joints);
			yarp::sig::Vector q_left_leg(left_leg_joints);
			yarp::sig::Vector q_head(head_joints);

			q_in=joint_sense();

			robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			q_left_arm = wave_q_left_arm_start;
                        q_head = wave_q_head_start;
			//q_torso = wave_q_torso_start;

			robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);
			poses["wave_1"] = q;
		    }
		    // HELLO 1 HAND end
		    if(cmd=="wave_2")
		    {
			yarp::sig::Vector q(kinematic_joints);
			yarp::sig::Vector q_in(kinematic_joints);
			yarp::sig::Vector q_right_arm(right_arm_joints);
			yarp::sig::Vector q_left_arm(left_arm_joints);
			yarp::sig::Vector q_torso(torso_joints);
			yarp::sig::Vector q_right_leg(right_leg_joints);
			yarp::sig::Vector q_left_leg(left_leg_joints);
			yarp::sig::Vector q_head(head_joints);

			q_in=joint_sense();

			robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			q_left_arm = wave_q_left_arm_end;
                        q_head = wave_q_head_end;

			robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);
			poses["wave_2"] = q;
		    }
		    
		    // HELLO 2 HANDS start
                    if(cmd=="wave_2_1")
                    {
                        yarp::sig::Vector q(kinematic_joints);
                        yarp::sig::Vector q_in(kinematic_joints);
                        yarp::sig::Vector q_right_arm(right_arm_joints);
                        yarp::sig::Vector q_left_arm(left_arm_joints);
                        yarp::sig::Vector q_torso(torso_joints);
                        yarp::sig::Vector q_right_leg(right_leg_joints);
                        yarp::sig::Vector q_left_leg(left_leg_joints);
                        yarp::sig::Vector q_head(head_joints);

                        q_in=joint_sense();

                        robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

                        q_right_arm = wave_q_right_arm_start;
                        q_left_arm = wave_q_left_arm_start;
                        q_head = wave_q_head_start;
                        //q_torso = wave_q_torso_start;

                        robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);
                        poses["wave_2_1"] = q;
                    }
                    // HELLO 2 HANDS end
                    if(cmd=="wave_2_2")
                    {
                        yarp::sig::Vector q(kinematic_joints);
                        yarp::sig::Vector q_in(kinematic_joints);
                        yarp::sig::Vector q_right_arm(right_arm_joints);
                        yarp::sig::Vector q_left_arm(left_arm_joints);
                        yarp::sig::Vector q_torso(torso_joints);
                        yarp::sig::Vector q_right_leg(right_leg_joints);
                        yarp::sig::Vector q_left_leg(left_leg_joints);
                        yarp::sig::Vector q_head(head_joints);

                        q_in=joint_sense();

                        robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

                        q_right_arm = wave_q_right_arm_end;
                        q_left_arm = wave_q_left_arm_end;
                        q_head = wave_q_head_end;

                        robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);
                        poses["wave_2_2"] = q;
                    }

					for (int k=0; k<allPoseSeries.size(); k++) {
                            if(cmd==allPoseSeries[k].name)
                            {
                                cout << "Pose series: " << cmd << " --- > activated " << endl;
                                presentPoseSeries = allPoseSeries[k].poses;
                                isInPoseSeriesMode = true;      
                            } 
                        }
                        
                        
                        if (isInPoseSeriesMode) {
                          if (!presentPoseSeries.empty()) {
                            cmd = presentPoseSeries.front();
                            presentPoseSeries.pop_front();
                            cout << "Executing pose: " << cmd << endl;
                            if (std::find(commands.begin(), commands.end(), cmd)==commands.end()) {
                                cout << "Requested command doesn't exist! Series execution stopped!!" << endl;
                                isInPoseSeriesMode = false;
                                return;
                            }
                          } else {
                            isInPoseSeriesMode = false;
                            return;
                          }
                        }

                    for (int k=0; k<posesVector.size(); k++) {
                        if(cmd==posesVector[k].name)
                        {
                            yarp::sig::Vector q(kinematic_joints);
                            yarp::sig::Vector q_in(kinematic_joints);
                            yarp::sig::Vector q_right_arm(right_arm_joints);
                            yarp::sig::Vector q_left_arm(left_arm_joints);
                            yarp::sig::Vector q_torso(torso_joints);
                            yarp::sig::Vector q_right_leg(right_leg_joints);
                            yarp::sig::Vector q_left_leg(left_leg_joints);
                            yarp::sig::Vector q_head(head_joints);

                            q_in=joint_sense();

                            robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

                            if (posesVector[k].isMoving_right_arm)
                                q_right_arm = posesVector[k].right_arm;
                            if (posesVector[k].isMoving_left_arm)
                                q_left_arm = posesVector[k].left_arm;  
                            if (posesVector[k].isMoving_torso)
                                q_torso = posesVector[k].torso;
                            if (posesVector[k].isMoving_right_leg)
                                q_right_leg = posesVector[k].right_leg;
                            if (posesVector[k].isMoving_left_leg)
                                q_left_leg = posesVector[k].left_leg;
                            if (posesVector[k].isMoving_head)
                                q_head = posesVector[k].head;
                            
                            robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);
                            poses[posesVector[k].name] = q;
                            
                            if(posesVector[k].isMoving_hands) {
                                cout << "Moving hands to " << posesVector[k].hands(0) << " " << posesVector[k].hands(1) << endl;
                                move_hands(posesVector[k].hands(0), posesVector[k].hands(1));
                            }
                        } 
                    }
		    
		    q_desired = poses.at(cmd);

		    if(status_definitions.status_to_code.count(cmd))
			status_interface.setStatus( status_definitions.status_to_code.at(cmd), status_seq_num++ );

            yarp::sig::Vector q_disp;
            double q_max_disp, _max, _min;
            double tmp_fin_time;
            q_disp = q_desired - q_initial;
            _max = findMax( q_disp );
            _min = findMin( q_disp );
            //considering negative values
            if ( fabs( _max ) > fabs( _min ) ) {q_max_disp = fabs( _max);}
            else {q_max_disp = fabs( _min );}
            tmp_fin_time = q_max_disp/Max_Vel;

            if ( Min_Texe < tmp_fin_time) {final_time = tmp_fin_time;}
            else {final_time = Min_Texe;}
		}
		if(err=="UNKWOWN")
		{
		    std::cout<<">> send \"help\" to visualize possible commands"<<std::endl;
		}
	    }
	}
    }

    //std::cout<<"Final Time = "<<final_time<<std::endl;
    compute_delta_q();

    q_output = q_initial + delta_q;

    robot.fromIdynToRobot(q_output, q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head);
    
    //OFFSET 
    q_left_arm = q_left_arm + left_arm_offset;
    q_right_arm = q_right_arm + right_arm_offset;
    
    robot.fromRobotToIdyn(q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head, q_output);
    
    robot.move(q_output);
}

yarp::sig::Vector drc_poses_thread::compute_delta_q()
{
    next_time = yarp::os::Time::now()-initialized_time;

    if(next_time <= final_time)
    {
	for(int i=0;i<delta_q.size();i++) delta_q[i] = joints_traj_gen.polynomial_interpolation(poly,q_desired[i]-q_initial[i],next_time,final_time);
    }
    else
    {
	for(int i=0;i<delta_q.size();i++) delta_q[i]= joints_traj_gen.polynomial_interpolation(poly,q_desired[i]-q_initial[i],final_time,final_time);
    }

    return delta_q;
}

bool drc_poses_thread::action_completed()
{
    double joint_err = 0;

    bool ok=true;

    for(int i=0;i<q_input.size();i++)
    {
	ok = ok && (fabs(q_input[i]-q_desired[i])<0.05);
    }

    return ok;
}

void drc_poses_thread::print_help()
{
    std::cout<<"-------------------------------"<<std::endl;
    std::cout<<std::endl;
    std::cout<<"\tPossible Commands: "<<std::endl;
    std::cout<<std::endl;

    for(auto item:commands) std::cout<<"\t- "<<item<<std::endl;

    std::cout<<std::endl;
    std::cout<<"-------------------------------"<<std::endl;
}

/////////------------------------------------------------------- POSES DEFINITION --------------------------------------------------------

void drc_poses_thread::create_poses()
{
    
    yarp::sig::Vector q(kinematic_joints);
    poses["recover"] = q; //just to have it in the known commands
    poses["walking"] = q; //just to have it in the known commands
    poses["driving"] = q; //just to have it in the known commands
    poses["pre_homing"] = q; //just to have it in the known commands
    poses["wave_1"] = q;
    poses["wave_2"] = q;
    poses["wave_2_1"] = q;
    poses["wave_2_2"] = q;

    yarp::sig::Vector q_right_arm(right_arm_joints);
    yarp::sig::Vector q_left_arm(left_arm_joints);
    yarp::sig::Vector q_torso(torso_joints);
    yarp::sig::Vector q_right_leg(right_leg_joints);
    yarp::sig::Vector q_left_leg(left_leg_joints);
    yarp::sig::Vector q_head(head_joints);
    yarp::sig::Vector q_in(kinematic_joints);

    //---------------------- homing ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();

    q_right_arm[0]=60.0*DEG2RAD;
    q_right_arm[1]=-10.0*DEG2RAD;
    q_right_arm[2]=20.0*DEG2RAD;
    q_right_arm[3]=-110.0*DEG2RAD;
    q_right_arm[4]=0.0*DEG2RAD;
    q_right_arm[5]=-30.0*DEG2RAD;
    q_right_arm[6]=0.0*DEG2RAD;

    q_left_arm[0]=60.0*DEG2RAD;
    q_left_arm[1]=10.0*DEG2RAD;
    q_left_arm[2]=-20.0*DEG2RAD;
    q_left_arm[3]=-110.0*DEG2RAD;
    q_left_arm[4]=0.0*DEG2RAD;
    q_left_arm[5]=-30.0*DEG2RAD;
    q_left_arm[6]=0.0*DEG2RAD;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=2.2*DEG2RAD;
    q_right_leg[1]=0.15*DEG2RAD;
    q_right_leg[2]=-17.2*DEG2RAD;
    q_right_leg[3]=33.2*DEG2RAD;
    q_right_leg[4]=-16.0*DEG2RAD;
    q_right_leg[5]=-2.25*DEG2RAD;
    
    q_left_leg[0]=-2.2*DEG2RAD;
    q_left_leg[1]=-0.15*DEG2RAD;
    q_left_leg[2]=-17.2*DEG2RAD;
    q_left_leg[3]=33.2*DEG2RAD;
    q_left_leg[4]=-16.0*DEG2RAD;
    q_left_leg[5]=2.25*DEG2RAD;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["homing"] = q;
    
    //---------------------- stand ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[1]=-0.15;
    
    q_left_arm[1]=0.15;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["stand"] = q;
    
    //---------------------- homing_guard ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.52;
    q_right_arm[3]=-2.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=-0.52;
    q_left_arm[3]=-2.6;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["homing_guard"] = q;
    
    //---------------------- homing_guard_left ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.52;
    q_right_arm[3]=-2.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=-0.52;
    q_left_arm[3]=-2.6;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 1.57;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["homing_guard_left"] = q;
    
    //---------------------- homing_guard_right ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.52;
    q_right_arm[3]=-2.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=-0.52;
    q_left_arm[3]=-2.6;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = -1.57;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["homing_guard_right"] = q;
    
    //---------------------- lower ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-0.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-0.6;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.87;
    q_right_leg[3]=1.74;
    q_right_leg[4]=-0.87;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.87;
    q_left_leg[3]=1.74;
    q_left_leg[4]=-0.87;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["lower"] = q;

    //---------------------- squat ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-0.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-0.6;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-1.17;
    q_right_leg[3]=2.34;
    q_right_leg[4]=-1.17;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-1.17;
    q_left_leg[3]=2.34;
    q_left_leg[4]=-1.17;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["squat"] = q;
    
    //---------------------- debris ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=1.05;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-2.35;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=1.05;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-2.35;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-1.17;
    q_right_leg[3]=2.34;
    q_right_leg[4]=-1.17;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-1.17;
    q_left_leg[3]=2.34;
    q_left_leg[4]=-1.17;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["debris"] = q;

        //---------------------- driving_pose ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=-19.77*DEG2RAD;
    q_right_arm[1]=-43.47*DEG2RAD;
    q_right_arm[2]=8.83*DEG2RAD;
    q_right_arm[3]=-106.6*DEG2RAD;
    q_right_arm[4]=-90*DEG2RAD;
    q_right_arm[5]=0.3*DEG2RAD;
    q_right_arm[6]=-0.33*DEG2RAD;
    
    q_left_arm[0]=44*DEG2RAD;
    q_left_arm[1]=40.2*DEG2RAD;
    q_left_arm[2]=-0.5*DEG2RAD;
    q_left_arm[3]=-114.5*DEG2RAD;
    q_left_arm[4]=42*DEG2RAD;
    q_left_arm[5]=-65*DEG2RAD;
    q_left_arm[6]=1*DEG2RAD;
    
    q_torso[0] = 0.0*DEG2RAD;
    q_torso[1] = 0.0*DEG2RAD;
    q_torso[2] = 13.0*DEG2RAD;
    
    q_head[0] = 30.0*DEG2RAD;
    q_head[1] = 20.0*DEG2RAD;
    
    q_right_leg[0]=-6*DEG2RAD;
    q_right_leg[1]=-11*DEG2RAD;
    q_right_leg[2]=-91*DEG2RAD;
    q_right_leg[3]=89*DEG2RAD;
    q_right_leg[4]=4.5*DEG2RAD;
    q_right_leg[5]=4.0*DEG2RAD;
    
    q_left_leg[0]=0*DEG2RAD;
    q_left_leg[1]=6*DEG2RAD;
    q_left_leg[2]=-92.5*DEG2RAD;
    q_left_leg[3]=93*DEG2RAD;
    q_left_leg[4]=-30*DEG2RAD;
    q_left_leg[5]=-0.5*DEG2RAD;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["driving_pose"] = q;

    //---------------------- grasp_homing ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();

    q_right_arm[0]=60.0*DEG2RAD;
    q_right_arm[1]=-10.0*DEG2RAD;
    q_right_arm[2]=-20.0*DEG2RAD; //20
    q_right_arm[3]=-110.0*DEG2RAD;
    q_right_arm[4]=0.0*DEG2RAD;
    q_right_arm[5]=-30.0*DEG2RAD;
    q_right_arm[6]=0.0*DEG2RAD;

    q_left_arm[0]=60.0*DEG2RAD;
    q_left_arm[1]=10.0*DEG2RAD;
    q_left_arm[2]=20.0*DEG2RAD; //-20
    q_left_arm[3]=-110.0*DEG2RAD;
    q_left_arm[4]=0.0*DEG2RAD;
    q_left_arm[5]=-30.0*DEG2RAD;
    q_left_arm[6]=0.0*DEG2RAD;

    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;

    q_right_leg[0]=2.2*DEG2RAD;
    q_right_leg[1]=0.15*DEG2RAD;
    q_right_leg[2]=-17.2*DEG2RAD;
    q_right_leg[3]=33.2*DEG2RAD;
    q_right_leg[4]=-16.0*DEG2RAD;
    q_right_leg[5]=-2.25*DEG2RAD;

    q_left_leg[0]=-2.2*DEG2RAD;
    q_left_leg[1]=-0.15*DEG2RAD;
    q_left_leg[2]=-17.2*DEG2RAD;
    q_left_leg[3]=33.2*DEG2RAD;
    q_left_leg[4]=-16.0*DEG2RAD;
    q_left_leg[5]=2.25*DEG2RAD;

    //q_head[0] = 30.0*DEG2RAD; //left
    q_head[1] = 50.0*DEG2RAD; //down

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["grasp_homing"] = q;
    
    //---------------------- fall_homing ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    q_in.zero();
    
    
    q_in=joint_sense();
    robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

    q_right_arm[0]=60.0*DEG2RAD;
    q_right_arm[1]=-10.0*DEG2RAD;
    q_right_arm[2]=-20.0*DEG2RAD;
    q_right_arm[3]=-110.0*DEG2RAD;
    q_right_arm[4]=0.0*DEG2RAD;
    q_right_arm[5]=-30.0*DEG2RAD;
    q_right_arm[6]=0.0*DEG2RAD;

    q_left_arm[0]=60.0*DEG2RAD;
    q_left_arm[1]=10.0*DEG2RAD;
    q_left_arm[2]=20.0*DEG2RAD;
    q_left_arm[3]=-110.0*DEG2RAD;
    q_left_arm[4]=0.0*DEG2RAD;
    q_left_arm[5]=-30.0*DEG2RAD;
    q_left_arm[6]=0.0*DEG2RAD;

    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;

    q_right_leg[0]=2.2*DEG2RAD;
    q_right_leg[1]=0.15*DEG2RAD;
    q_right_leg[2]=-17.2*DEG2RAD;
    q_right_leg[3]=33.2*DEG2RAD;
    q_right_leg[4]=-16.0*DEG2RAD;
    q_right_leg[5]=-2.25*DEG2RAD;

    q_left_leg[0]=-2.2*DEG2RAD;
    q_left_leg[1]=-0.15*DEG2RAD;
    q_left_leg[2]=-17.2*DEG2RAD;
    q_left_leg[3]=33.2*DEG2RAD;
    q_left_leg[4]=-16.0*DEG2RAD;
    q_left_leg[5]=2.25*DEG2RAD;

    q_head[1] = 45.0*DEG2RAD; //down

	robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["fall_homing"] = q;
    
    //---------------------- walk ---------------------

    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    q_in.zero();
    
    q_in=joint_sense();
    robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

    q_right_arm[0]=60.0*DEG2RAD;
    q_right_arm[1]=-10.0*DEG2RAD;
    q_right_arm[2]=20.0*DEG2RAD;
    q_right_arm[3]=-110.0*DEG2RAD;
    q_right_arm[4]=0.0*DEG2RAD;
    q_right_arm[5]=-30.0*DEG2RAD;
    q_right_arm[6]=0.0*DEG2RAD;

    q_left_arm[0]=60.0*DEG2RAD;
    q_left_arm[1]=10.0*DEG2RAD;
    q_left_arm[2]=-20.0*DEG2RAD;
    q_left_arm[3]=-110.0*DEG2RAD;
    q_left_arm[4]=0.0*DEG2RAD;
    q_left_arm[5]=-30.0*DEG2RAD;
    q_left_arm[6]=0.0*DEG2RAD;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_head[0] = 0.0*DEG2RAD;
    q_head[1] = 0.0*DEG2RAD;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["walk"] = q;

    //------------------ POSES FOR DEMO ----------------
    //---------------------- demo0 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-0.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-0.6;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo0"] = q;
    
    //---------------------- demo1 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-0.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-0.6;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.87;
    q_right_leg[3]=1.74;
    q_right_leg[4]=-0.87;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.87;
    q_left_leg[3]=1.74;
    q_left_leg[4]=-0.87;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo1"] = q;
    
    //---------------------- demo2 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-0.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-0.6;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo2"] = q;
    
    //---------------------- demo3 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=-0.4;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=-0.7;
    q_right_arm[3]=-1.25;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=1;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.85;
    q_left_arm[3]=-2.25;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.6;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo3"] = q;
    
    //---------------------- demo4 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=1;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=-0.85;
    q_right_arm[3]=-2.25;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=-0.4;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.7;
    q_left_arm[3]=-1.25;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = -0.6;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo4"] = q;
    
    //---------------------- demo5 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-1.5;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-1.5;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo5"] = q;
    
    //---------------------- demo6 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-1.5;
    q_right_arm[4]=-0.7;
    q_right_arm[5]=0.7;
    q_right_arm[6]=-0.7;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-1.5;
    q_left_arm[4]=0.7;
    q_left_arm[5]=0.7;
    q_left_arm[6]=0.7;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;

    q_head[0] = 0.5;
//     q_head[1] = 0.3;

    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo6"] = q;
    
    //---------------------- demo7 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-1.5;
    q_right_arm[4]=0.7;
    q_right_arm[5]=-0.7;
    q_right_arm[6]=0.7;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-1.5;
    q_left_arm[4]=-0.7;
    q_left_arm[5]=-0.7;
    q_left_arm[6]=-0.7;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_head[0] = -0.5;
//     q_head[1] = -0.3;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo7"] = q;
    
    //---------------------- demo8 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-1.5;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-1.5;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;

    q_head[0] = 0.0;
    q_head[1] = 0.0;

    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo8"] = q;
    
//     //---------------------- demo9 ---------------------
//     q_right_arm.zero();
//     q_left_arm.zero();
//     q_torso.zero();
//     q_right_leg.zero();
//     q_left_leg.zero();
//     q_head.zero();
//     
//     q_right_arm[0]=0.3;
//     q_right_arm[1]=-0.15;
//     q_right_arm[2]=0.0;
//     q_right_arm[3]=-0.6;
//     q_right_arm[4]=0.0;
//     q_right_arm[5]=0.0;
//     q_right_arm[6]=0.0;
//     
//     q_left_arm[0]=0.3;
//     q_left_arm[1]=0.15;
//     q_left_arm[2]=0.0;
//     q_left_arm[3]=-0.6;
//     q_left_arm[4]=0.0;
//     q_left_arm[5]=0.0;
//     q_left_arm[6]=0.0;
//     
//     q_torso[0] = 0.0;
//     q_torso[1] = 0.0;
//     q_torso[2] = 0.0;
//     
//     q_right_leg[0]=0.0;
//     q_right_leg[1]=0.0;
//     q_right_leg[2]=-0.3;
//     q_right_leg[3]=0.6;
//     q_right_leg[4]=-0.3;
//     q_right_leg[5]=-0.0;
//     
//     q_left_leg[0]=-0.0;
//     q_left_leg[1]=0.0;
//     q_left_leg[2]=-0.3;
//     q_left_leg[3]=0.6;
//     q_left_leg[4]=-0.3;
//     q_left_leg[5]=0.0;
// 
//     robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);
// 
//     poses["demo9"] = q;    
//     
//     //---------------------- demo10 ---------------------
//     q_right_arm.zero();
//     q_left_arm.zero();
//     q_torso.zero();
//     q_right_leg.zero();
//     q_left_leg.zero();
//     q_head.zero();
//     
//     q_right_arm[0]=0.0;
//     q_right_arm[1]=-1.3;
//     q_right_arm[2]=0.25;
//     q_right_arm[3]=0.0;
//     q_right_arm[4]=0.0;
//     q_right_arm[5]=0.0;
//     q_right_arm[6]=0.0;
//     
//     q_left_arm[0]=0.0;
//     q_left_arm[1]=1.3;
//     q_left_arm[2]=-0.25;
//     q_left_arm[3]=0.0;
//     q_left_arm[4]=0.0;
//     q_left_arm[5]=0.0;
//     q_left_arm[6]=0.0;
//     
//     q_torso[0] = 0.0;
//     q_torso[1] = -0.25;
//     q_torso[2] = 0.0;
//     
//     q_right_leg[0]=0.0;
//     q_right_leg[1]=0.0;
//     q_right_leg[2]=-0.3;
//     q_right_leg[3]=0.6;
//     q_right_leg[4]=-0.3;
//     q_right_leg[5]=-0.0;
//     
//     q_left_leg[0]=-0.0;
//     q_left_leg[1]=0.0;
//     q_left_leg[2]=-0.3;
//     q_left_leg[3]=0.6;
//     q_left_leg[4]=-0.3;
//     q_left_leg[5]=0.0;
// 
//     robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);
// 
//     poses["demo10"] = q;    
//     
//     //---------------------- demo11 ---------------------
//     q_right_arm.zero();
//     q_left_arm.zero();
//     q_torso.zero();
//     q_right_leg.zero();
//     q_left_leg.zero();
//     q_head.zero();
//     
//     q_right_arm[0]=0.0;
//     q_right_arm[1]=-1.3;
//     q_right_arm[2]=0.25;
//     q_right_arm[3]=-2.6;
//     q_right_arm[4]=0.0;
//     q_right_arm[5]=0.0;
//     q_right_arm[6]=0.0;
//     
//     q_left_arm[0]=0.0;
//     q_left_arm[1]=1.3;
//     q_left_arm[2]=-0.25;
//     q_left_arm[3]=-2.6;
//     q_left_arm[4]=0.0;
//     q_left_arm[5]=0.0;
//     q_left_arm[6]=0.0;
//     
//     q_torso[0] = 0.0;
//     q_torso[1] = -0.25;
//     q_torso[2] = 0.0;
//     
//     q_right_leg[0]=0.0;
//     q_right_leg[1]=0.0;
//     q_right_leg[2]=-0.3;
//     q_right_leg[3]=0.6;
//     q_right_leg[4]=-0.3;
//     q_right_leg[5]=-0.0;
//     
//     q_left_leg[0]=-0.0;
//     q_left_leg[1]=0.0;
//     q_left_leg[2]=-0.3;
//     q_left_leg[3]=0.6;
//     q_left_leg[4]=-0.3;
//     q_left_leg[5]=0.0;
// 
//     robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);
// 
//     poses["demo11"] = q;   
//     
//     //---------------------- demo12 ---------------------
//     q_right_arm.zero();
//     q_left_arm.zero();
//     q_torso.zero();
//     q_right_leg.zero();
//     q_left_leg.zero();
//     q_head.zero();
//     
//     q_right_arm[0]=0.0;
//     q_right_arm[1]=-1.3;
//     q_right_arm[2]=0.0;
//     q_right_arm[3]=0.4;
//     q_right_arm[4]=0.0;
//     q_right_arm[5]=0.0;
//     q_right_arm[6]=0.0;
//     
//     q_left_arm[0]=0.0;
//     q_left_arm[1]=1.3;
//     q_left_arm[2]=0.0;
//     q_left_arm[3]=0.4;
//     q_left_arm[4]=0.0;
//     q_left_arm[5]=0.0;
//     q_left_arm[6]=0.0;
//     
//     q_torso[0] = 0.0;
//     q_torso[1] = -0.25;
//     q_torso[2] = 0.0;
//     
//     q_right_leg[0]=0.0;
//     q_right_leg[1]=0.0;
//     q_right_leg[2]=-0.3;
//     q_right_leg[3]=0.6;
//     q_right_leg[4]=-0.3;
//     q_right_leg[5]=-0.0;
//     
//     q_left_leg[0]=-0.0;
//     q_left_leg[1]=0.0;
//     q_left_leg[2]=-0.3;
//     q_left_leg[3]=0.6;
//     q_left_leg[4]=-0.3;
//     q_left_leg[5]=0.0;
// 
//     robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);
// 
//     poses["demo12"] = q;   
//     
    //---------------------- demo13 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.0;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=-0.25;
    q_right_arm[3]=-1.5;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.0;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.25;
    q_left_arm[3]=-1.5;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = -0.25;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo13"] = q;
    
    //---------------------- demo14 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.0;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=-0.35;
    q_right_arm[3]=-1.5;
    q_right_arm[4]=2.4;
    q_right_arm[5]=-1.3;
    q_right_arm[6]=-1.3;
    
    q_left_arm[0]=0.0;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.35;
    q_left_arm[3]=-1.5;
    q_left_arm[4]=-2.4;
    q_left_arm[5]=-1.3;
    q_left_arm[6]=1.3;
    
    q_torso[0] = 0.0;
    q_torso[1] = -0.25;
    q_torso[2] = 0.0;

    q_head[1] = 0.5;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo14"] = q;
    
    //---------------------- demo15 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.0;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=-0.25;
    q_right_arm[3]=-1.5;
    q_right_arm[4]=-2.4;
    q_right_arm[5]=1.3;
    q_right_arm[6]=1.3;
    
    q_left_arm[0]=0.0;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.25;
    q_left_arm[3]=-1.5;
    q_left_arm[4]=2.4;
    q_left_arm[5]=1.3;
    q_left_arm[6]=-1.3;
    
    q_torso[0] = 0.0;
    q_torso[1] = -0.25;
    q_torso[2] = 0.0;

    q_head[1] = 0.5;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo15"] = q;
  
    //---------------------- demo16 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.0;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-1.5;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.0;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-1.5;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = -0.25;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo16"] = q;    
    
    //---------------------- demo17 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.3;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-0.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.3;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-0.6;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.3;
    q_right_leg[3]=0.6;
    q_right_leg[4]=-0.3;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.3;
    q_left_leg[3]=0.6;
    q_left_leg[4]=-0.3;
    q_left_leg[5]=0.0;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo17"] = q;        
}

bool readYAML(yarp::sig::Vector &out, YAML::Node node)
{
  if (!node) return false;
  
  int size = node.size();
  if (size == 0) return false;
  out.resize(size);
  for (int i=0; i<size; i++) {
    out(i) = node[i].as<double>() * DEG2RAD;
    cout << out(i) << " ";
  }
  cout << endl;
  return true;
}

bool drc_poses_thread::loadPoses(std::string yamlFilename)
{
  cout << "Reading yaml file at: " << yamlFilename.c_str() << endl;
  posesVector.clear(); 
  
  YAML::Node config = YAML::LoadFile(yamlFilename.c_str());
  const YAML::Node& poses = config["poses"];
  
  for (std::size_t i = 0; i < poses.size(); i++) {
    pose tmpPose;
    const YAML::Node& poseNode = poses[i];
    if (poseNode["pose_name"]) {
        cout << poseNode["pose_name"].as<std::string>() << endl;
        tmpPose.name = poseNode["pose_name"].as<std::string>();
    }
    else return false;
    
    if (readYAML(tmpPose.right_arm, poseNode["right_arm"])) tmpPose.isMoving_right_arm = true;
    else cout << "right_arm is not present." << endl;
    
    if (readYAML(tmpPose.left_arm, poseNode["left_arm"])) tmpPose.isMoving_left_arm = true;
    else cout << "left_arm is not present." << endl;
    
    if (readYAML(tmpPose.right_leg, poseNode["right_leg"])) tmpPose.isMoving_right_leg = true;
    else cout << "right_leg is not present." << endl;
    
    if (readYAML(tmpPose.left_leg, poseNode["left_leg"])) tmpPose.isMoving_left_leg = true;
    else cout << "left_leg is not present." << endl;
    
    if (readYAML(tmpPose.torso, poseNode["torso"])) tmpPose.isMoving_torso = true;
    else cout << "torso is not present." << endl;

    if (readYAML(tmpPose.head, poseNode["head"])) tmpPose.isMoving_head = true;
    else cout << "head is not present." << endl;
    
    if (poseNode["hands"]){
        if (poseNode["hands"].size() == 2) { //Make sure there is a reference for left and right hand
            tmpPose.isMoving_hands = true;
            tmpPose.hands.resize(2);
            tmpPose.hands(0) = poseNode["hands"][0].as<double>()/100.0;
            tmpPose.hands(1) = poseNode["hands"][1].as<double>()/100.0;
            
            cout << "Hands: " << tmpPose.hands(0) << " " << tmpPose.hands(1) << endl;
        }
    }
    else cout << "Hands are not present." << endl;
    
    posesVector.push_back(tmpPose);
  }
  
  if (config["series"]) {
    const YAML::Node& poseSer = config["series"];
    allPoseSeries.clear();

    for (std::size_t i = 0; i < poseSer.size(); i++) {
        poseSeries tmpSeries;
        const YAML::Node& seriesNode = poseSer[i];
        if (seriesNode["series_name"]) {
            cout << "Loading series: " << seriesNode["series_name"].as<std::string>() << endl;
            tmpSeries.name = seriesNode["series_name"].as<std::string>();
        }
        else return false;
        
        cout << endl;
        for (int k=0; k<seriesNode["poses"].size(); k++) {
            cout << seriesNode["poses"][k].as<string>() << endl;
            tmpSeries.poses.push_back(seriesNode["poses"][k].as<string>());
            
        }    
        cout << endl;
        
        
        allPoseSeries.push_back(tmpSeries);
    }
  }
  
  return true;
}


bool drc_poses_thread::savePose(const yarp::sig::Vector &q_in, std::string filename, int poseNumber)
{
  cout << "Saving data to file: " << filename.c_str() << endl;
  
    yarp::sig::Vector q_right_arm(right_arm_joints);
    yarp::sig::Vector q_left_arm(left_arm_joints);
    yarp::sig::Vector q_torso(torso_joints);
    yarp::sig::Vector q_right_leg(right_leg_joints);
    yarp::sig::Vector q_left_leg(left_leg_joints);
    yarp::sig::Vector q_head(head_joints);


  robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

  ofstream myfile;
  myfile.open (filename.c_str(), ios::app | ios::out);
  myfile << " - pose_name : savedPose" << poseNumber << endl;
  
  myfile << "   right_arm : [";
  for (int i=0; i<q_right_arm.size()-1; i++) {
      myfile << round(q_right_arm(i)/PI*180) << ", ";
  }
  myfile << round(q_right_arm(q_right_arm.size()-1)/PI*180) << "]" << endl;
  
  myfile << "   left_arm  : [";
  for (int i=0; i<q_left_arm.size()-1; i++) {
      myfile << round(q_left_arm(i)/PI*180) << ", ";
  }
  myfile << round(q_left_arm(q_left_arm.size()-1)/PI*180) << "]" << endl;
  
  myfile << "   right_leg : [";
  for (int i=0; i<q_right_leg.size()-1; i++) {
      myfile << round(q_right_leg(i)/PI*180) << ", ";
  }
  myfile << round(q_right_leg(q_right_leg.size()-1)/PI*180) << "]" << endl;
  
  myfile << "   left_leg  : [";
  for (int i=0; i<q_left_leg.size()-1; i++) {
      myfile << round(q_left_leg(i)/PI*180) << ", ";
  }
  myfile << round(q_left_leg(q_left_leg.size()-1)/PI*180) << "]" << endl;
  
  myfile << "   head      : [";
  for (int i=0; i<q_head.size()-1; i++) {
      myfile << round(q_head(i)/PI*180) << ", ";
  }
  myfile << round(q_head(q_head.size()-1)/PI*180) << "]" << endl;
  
  myfile << "   torso     : [";
  for (int i=0; i<q_torso.size()-1; i++) {
      myfile << round(q_torso(i)/PI*180) << ", ";
  }
  myfile << round(q_torso(q_torso.size()-1)/PI*180) << "]" << endl << endl;
  
  myfile.close();
  
  return true;
}


void drc_poses_thread::updateCommandList()
{
  commands.clear();
  for(auto item:poses) commands.push_back(item.first);
  for(auto item:posesVector) commands.push_back(item.name); 
  for(auto item:allPoseSeries) commands.push_back(item.name); 
}



