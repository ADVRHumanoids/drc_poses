#include <yarp/os/all.h>

#include "drc_poses_thread.h"
#include "drc_poses_constants.h"

using namespace walkman::drc::poses;
using namespace yarp::math;

#define Max_Vel         0.3 // maximum joint velocity [rad/s]
#define Min_Texe        3.0 // minimum execution time for homing [sec]

#define RAD2DEG    (180.0/M_PI)
#define DEG2RAD    (M_PI/180.0)

drc_poses_thread::drc_poses_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr< paramHelp::ParamHelperServer > ph)
:control_thread( module_prefix, rf, ph ), cmd_interface(module_prefix), status_interface(module_prefix)
{
    q_input.resize(robot.getNumberOfJoints());
    delta_q.resize(robot.getNumberOfJoints());
    q_output.resize(robot.getNumberOfJoints());

    create_poses();
    for(auto item:poses) commands.push_back(item.first);

    next_time=0.0;
    final_time=Min_Texe;   //devel_JLee
    
        
    //recover pose
    recover_q_right_arm.resize(robot.right_arm.getNumberOfJoints());
    recover_q_left_arm.resize(robot.left_arm.getNumberOfJoints());
    recover_q_torso.resize(robot.torso.getNumberOfJoints());
    recover_q_right_leg.resize(robot.right_leg.getNumberOfJoints());
    recover_q_left_leg.resize(robot.left_leg.getNumberOfJoints());
    recover_q_head.resize(robot.head.getNumberOfJoints());
    
    recover_q_right_arm[0]=0.6;
    recover_q_right_arm[1]=-0.15;
    recover_q_right_arm[2]=0.0;
    recover_q_right_arm[3]=-1;
    recover_q_right_arm[4]=0.0;
    recover_q_right_arm[5]=0.0;
    recover_q_right_arm[6]=0.0;
    
    recover_q_left_arm[0]=0.6;
    recover_q_left_arm[1]=0.15;
    recover_q_left_arm[2]=0.0;
    recover_q_left_arm[3]=-1;
    recover_q_left_arm[4]=0.0;
    recover_q_left_arm[5]=0.0;
    recover_q_left_arm[6]=0.0;

    //driving pose
    drive_q_right_arm.resize(robot.right_arm.getNumberOfJoints());
    drive_q_left_arm.resize(robot.left_arm.getNumberOfJoints());
    drive_q_torso.resize(robot.torso.getNumberOfJoints());
    drive_q_right_leg.resize(robot.right_leg.getNumberOfJoints());
    drive_q_left_leg.resize(robot.left_leg.getNumberOfJoints());
    drive_q_head.resize(robot.head.getNumberOfJoints());
    
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
}

bool drc_poses_thread::custom_init()
{
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);

    status_interface.start();
    status_interface.setStatus( "ready", status_seq_num++ );

    q_input=robot.sensePosition();
    robot.idynutils.updateiDyn3Model(q_input, false);
    robot.setPositionDirectMode();
    
    q_initial = q_input;
    q_desired = q_input;

    return true;
}

void drc_poses_thread::run()
{
    q_input=robot.sensePosition();

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
	        if(last_command=="demo") cmd="demo0";
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
			yarp::sig::Vector q(robot.getNumberOfJoints());
			yarp::sig::Vector q_in(robot.getNumberOfJoints());
			yarp::sig::Vector q_right_arm(robot.right_arm.getNumberOfJoints());
			yarp::sig::Vector q_left_arm(robot.left_arm.getNumberOfJoints());
			yarp::sig::Vector q_torso(robot.torso.getNumberOfJoints());
			yarp::sig::Vector q_right_leg(robot.right_leg.getNumberOfJoints());
			yarp::sig::Vector q_left_leg(robot.left_leg.getNumberOfJoints());
			yarp::sig::Vector q_head(robot.head.getNumberOfJoints());

			q_in = robot.sensePosition();

			robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			recover_q_left_leg = q_left_leg;
			recover_q_right_leg = q_right_leg;
			recover_q_torso = q_torso;
			recover_q_head = q_head;

			robot.fromRobotToIdyn(recover_q_right_arm,recover_q_left_arm,recover_q_torso,recover_q_right_leg,recover_q_left_leg,recover_q_head,q);
			poses["recover"] = q;
		    }
		    
		    if(cmd=="driving") //We don't want to move the legs in this case
		    {
			yarp::sig::Vector q(robot.getNumberOfJoints());
			yarp::sig::Vector q_in(robot.getNumberOfJoints());
			yarp::sig::Vector q_right_arm(robot.right_arm.getNumberOfJoints());
			yarp::sig::Vector q_left_arm(robot.left_arm.getNumberOfJoints());
			yarp::sig::Vector q_torso(robot.torso.getNumberOfJoints());
			yarp::sig::Vector q_right_leg(robot.right_leg.getNumberOfJoints());
			yarp::sig::Vector q_left_leg(robot.left_leg.getNumberOfJoints());
			yarp::sig::Vector q_head(robot.head.getNumberOfJoints());

			q_in = robot.sensePosition();

			robot.fromIdynToRobot(q_in,q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head);

			drive_q_left_leg = q_left_leg;
			drive_q_right_leg = q_right_leg;
			drive_q_head = q_head;
			drive_q_right_arm = q_right_arm;

			robot.fromRobotToIdyn(drive_q_right_arm,drive_q_left_arm,drive_q_torso,drive_q_right_leg,drive_q_left_leg,drive_q_head,q);
			poses["driving"] = q;
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
    
    yarp::sig::Vector q(robot.getNumberOfJoints());
    poses["recover"] = q; //just to have it in the known commands
    poses["driving"] = q; //just to have it in the known commands

    yarp::sig::Vector q_right_arm(robot.right_arm.getNumberOfJoints());
    yarp::sig::Vector q_left_arm(robot.left_arm.getNumberOfJoints());
    yarp::sig::Vector q_torso(robot.torso.getNumberOfJoints());
    yarp::sig::Vector q_right_leg(robot.right_leg.getNumberOfJoints());
    yarp::sig::Vector q_left_leg(robot.left_leg.getNumberOfJoints());
    yarp::sig::Vector q_head(robot.head.getNumberOfJoints());

    //---------------------- homing ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.35;
    q_right_arm[1]=-0.15;
    q_right_arm[2]=0.0;
    q_right_arm[3]=-0.7;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.35;
    q_left_arm[1]=0.15;
    q_left_arm[2]=0.0;
    q_left_arm[3]=-0.7;
    q_left_arm[4]=0.0;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = 0.0;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=0.0;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-0.25;
    q_right_leg[3]=0.5;
    q_right_leg[4]=-0.25;
    q_right_leg[5]=-0.0;
    
    q_left_leg[0]=-0.0;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-0.25;
    q_left_leg[3]=0.5;
    q_left_leg[4]=-0.25;
    q_left_leg[5]=0.0;

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

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo9"] = q;    
    
    //---------------------- demo10 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.0;
    q_right_arm[1]=-1.6;
    q_right_arm[2]=0.25;
    q_right_arm[3]=0.0;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.0;
    q_left_arm[1]=1.6;
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

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo10"] = q;    
    
    //---------------------- demo11 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.0;
    q_right_arm[1]=-1.6;
    q_right_arm[2]=0.25;
    q_right_arm[3]=-2.6;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.0;
    q_left_arm[1]=1.6;
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

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["demo11"] = q;   
    
    //---------------------- demo12 ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=0.0;
    q_right_arm[1]=-1.6;
    q_right_arm[2]=0.0;
    q_right_arm[3]=0.4;
    q_right_arm[4]=0.0;
    q_right_arm[5]=0.0;
    q_right_arm[6]=0.0;
    
    q_left_arm[0]=0.0;
    q_left_arm[1]=1.6;
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

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

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
