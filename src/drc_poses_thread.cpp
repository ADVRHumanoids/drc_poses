#include <yarp/os/all.h>

#include "drc_poses_thread.h"
#include "drc_poses_constants.h"

using namespace walkman::drc::poses;
using namespace yarp::math;

#define Max_Vel         0.5 // maximum joint velocity [rad/s]
#define Min_Texe        2.0 // minimum execution time for homing [sec]

#define RAD2DEG    (180.0/M_PI)
#define DEG2RAD    (M_PI/180.0)

double left_offset[7] = {-0.001878101, -0.087266425, -0.00541460025, -0.04116454775, -0.0270602895, 0.05685963075, 0.05985226625};
double right_offset[7] = {0.00496249625, 0.01221735225, 0.023223271, -0.01633125525, -0.04591635675, 0.0131223505, -0.0860596935};
yarp::sig::Vector left_arm_offset(7,left_offset);
yarp::sig::Vector right_arm_offset(7,right_offset);

drc_poses_thread::drc_poses_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr< paramHelp::ParamHelperServer > ph)
:control_thread( module_prefix, rf, ph ), cmd_interface(module_prefix), status_interface(module_prefix)
{
    kinematic_joints = robot.getNumberOfKinematicJoints();
    actuated_joints = robot.getNumberOfActuatedJoints();
    right_arm_joints = 7;
    left_arm_joints = 7;
    torso_joints = 3;
    right_leg_joints = 6;
    left_leg_joints = 6;
    head_joints = 2;
  
    q_input.resize(kinematic_joints);
    delta_q.resize(kinematic_joints);
    q_output.resize(kinematic_joints);

    create_poses();
    for(auto item:poses) commands.push_back(item.first);

    next_time=0.0;
    final_time=Min_Texe;   //devel_JLee
    
        
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
    
//     yarp::sig::Vector in = robot.sensePositionRefFeedback(); //REAL_ROBOT
    yarp::sig::Vector in = robot.sensePosition(); //SIMULATION

    for(int i=0;i<out.size();i++) out[i]=in[i];
    
    return out;
}

void drc_poses_thread::run()
{
    q_input=joint_sense();

    yarp::sig::Vector q_torso(3), q_left_arm(7), q_right_arm(7), q_left_leg(6), q_right_leg(6), q_head(2);
    robot.fromIdynToRobot31(q_input, q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head);
    
    //OFFSET 
    q_left_arm = q_left_arm - left_arm_offset;
    q_right_arm = q_right_arm - right_arm_offset;
    
    robot.fromRobotToIdyn31(q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head, q_input);

    robot.idynutils.updateiDyn3Model( q_input, false );

    if(action_completed()) 
    {
	busy=false;
	status_interface.setStatus( status_definitions.status_to_code.at("ready"), status_seq_num++ );
    }

    if(!busy)
    {
	std::string cmd, err;
	if(cmd_interface.getCommand(cmd,cmd_seq_num) || demo_mode)
	{
	    if(demo_mode)
	    {
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
		if(last_command=="demo9") cmd="demo10";
		if(last_command=="demo10") cmd="demo11";
		if(last_command=="demo11") cmd="demo12";
		if(last_command=="demo12") cmd="demo13";
		if(last_command=="demo13") cmd="demo14";
		if(last_command=="demo14") cmd="demo15";
		if(last_command=="demo15") cmd="demo16";
		if(last_command=="demo16") cmd="demo17";
		if(last_command=="demo17") demo_mode=false;
	    }

	    if(cmd=="demo")
	    {
		demo_mode=true;
	    }

	    last_command=cmd;

	    if(cmd=="help")
	    {
		print_help();
	    }
	    else
	    {
		err = (std::find(commands.begin(), commands.end(), cmd)!=commands.end())?"":"UNKWOWN";
		std::cout<<">> Received command ["<<cmd_seq_num<<"]: "<<cmd<<" "<<err<<std::endl;

		if(err=="")
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

			robot.fromIdynToRobot31(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			recover_q_left_leg = q_left_leg;
			recover_q_right_leg = q_right_leg;
			recover_q_head = q_head;

			robot.fromRobotToIdyn31(recover_q_right_arm,recover_q_left_arm,recover_q_torso,recover_q_right_leg,recover_q_left_leg,recover_q_head,q);
			poses["recover"] = q;
		    }
		    
		    if(cmd=="recover_walking") //We don't want to move the legs in this case
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

			robot.fromIdynToRobot31(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			recover_q_left_leg = q_left_leg;
			recover_q_right_leg = q_right_leg;
			recover_q_head = q_head;
			
			// recover walking
			recover_q_right_arm[6]=50.0*DEG2RAD;
			recover_q_left_arm[6]= -50.0*DEG2RAD;
			
			robot.fromRobotToIdyn31(recover_q_right_arm,recover_q_left_arm,recover_q_torso,recover_q_right_leg,recover_q_left_leg,recover_q_head,q);
			poses["recover_walking"] = q;
			
			recover_q_right_arm[6]=0.0;
			recover_q_left_arm[6]=0.0;
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

			robot.fromIdynToRobot31(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			drive_q_left_leg = q_left_leg;
			drive_q_right_leg = q_right_leg;
			drive_q_head = q_head;
			drive_q_right_arm = q_right_arm;

			robot.fromRobotToIdyn31(drive_q_right_arm,drive_q_left_arm,drive_q_torso,drive_q_right_leg,drive_q_left_leg,drive_q_head,q);
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

			robot.fromIdynToRobot31(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			q_left_leg[0] = pre_homing_q_left_leg[0];
			pre_homing_q_left_leg = q_left_leg;
			q_right_leg[0] = pre_homing_q_right_leg[0];
			pre_homing_q_right_leg = q_right_leg;

			robot.fromRobotToIdyn31(pre_homing_q_right_arm,pre_homing_q_left_arm,pre_homing_q_torso,pre_homing_q_right_leg,pre_homing_q_left_leg,pre_homing_q_head,q);
			yarp::sig::Vector hands(2);
			hands[0]=pre_homing_q_right_hand[0];
			hands[1]=pre_homing_q_left_hand[0];
			robot.moveHands(hands);
			poses["pre_homing"] = q;
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

    robot.fromIdynToRobot31(q_output, q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head);
    
    //OFFSET 
    q_left_arm = q_left_arm + left_arm_offset;
    q_right_arm = q_right_arm + right_arm_offset;
    
    yarp::sig::Vector q_move(actuated_joints);
//     q_move.resize(31);
    robot.fromRobotToIdyn29(q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_move);

    robot.move29(q_move);
    robot.moveNeck(q_head);
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
    poses["recover_walking"] = q; //just to have it in the known commands
    poses["driving"] = q; //just to have it in the known commands
    poses["pre_homing"] = q; //just to have it in the known commands

    yarp::sig::Vector q_right_arm(right_arm_joints);
    yarp::sig::Vector q_left_arm(left_arm_joints);
    yarp::sig::Vector q_torso(torso_joints);
    yarp::sig::Vector q_right_leg(right_leg_joints);
    yarp::sig::Vector q_left_leg(left_leg_joints);
    yarp::sig::Vector q_head(head_joints);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["driving_pose"] = q;

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo8"] = q;
    
    //---------------------- demo9 ---------------------
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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo9"] = q;    
    
    //---------------------- demo10 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.0;
    q_right_arm[1]=-1.3;
    q_right_arm[2]=0.25;
    q_right_arm[3]=0.0;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.0;
    q_left_arm[1]=1.3;
    q_left_arm[2]=-0.25;
    q_left_arm[3]=0.0;
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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo10"] = q;    
    
    //---------------------- demo11 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.0;
    q_right_arm[1]=-1.3;
    q_right_arm[2]=0.25;
    q_right_arm[3]=-2.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.0;
    q_left_arm[1]=1.3;
    q_left_arm[2]=-0.25;
    q_left_arm[3]=-2.6;
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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo11"] = q;   
    
    //---------------------- demo12 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.0;
    q_right_arm[1]=-1.3;
    q_right_arm[2]=0.0;
    q_right_arm[3]=0.4;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.0;
    q_left_arm[1]=1.3;
    q_left_arm[2]=0.0;
    q_left_arm[3]=0.4;
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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo12"] = q;   
    
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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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

    robot.fromRobotToIdyn31(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo17"] = q;        
}
