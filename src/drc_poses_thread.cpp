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
	status_interface.setStatus( "ready", status_seq_num++ );
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
		if(last_command=="demo9") demo_mode=false;
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
		    q_desired = poses.at(cmd);
		    status_interface.setStatus( cmd, status_seq_num++ );

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

        //---------------------- driving ---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=-13.8*DEG2RAD;
    q_right_arm[1]=-44.4*DEG2RAD;
    q_right_arm[2]=31.4*DEG2RAD;
    q_right_arm[3]=-107.2*DEG2RAD;
    q_right_arm[4]=-90*DEG2RAD;
    q_right_arm[5]=0.1*DEG2RAD;
    q_right_arm[6]=-0.3*DEG2RAD;
    
    q_left_arm[0]=-35.0*DEG2RAD;
    q_left_arm[1]=42.9*DEG2RAD;
    q_left_arm[2]=49.0*DEG2RAD;
    q_left_arm[3]=-134.0*DEG2RAD;
    q_left_arm[4]=25.0*DEG2RAD;
    q_left_arm[5]=0.0;
    q_left_arm[6]=0.0;
    
    q_torso[0] = 0.0;
    q_torso[1] = -2.6*DEG2RAD;
    q_torso[2] = 0.0;
    
    q_right_leg[0]=-0.5*DEG2RAD;
    q_right_leg[1]=0.0;
    q_right_leg[2]=-90.0*DEG2RAD;
    q_right_leg[3]=88.0*DEG2RAD;
    q_right_leg[4]=-2.5*DEG2RAD;
    q_right_leg[5]=-0.16*DEG2RAD;
    
    q_left_leg[0]=-0.5*DEG2RAD;
    q_left_leg[1]=0.0;
    q_left_leg[2]=-90*DEG2RAD;
    q_left_leg[3]=88.0*DEG2RAD;
    q_left_leg[4]=-2.5*DEG2RAD;
    q_left_leg[5]=0.16*DEG2RAD;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["driving"] = q;

        //---------------------- car_exit---------------------
    q_right_arm.zero();
    q_left_arm.zero();
    q_torso.zero();
    q_right_leg.zero();
    q_left_leg.zero();
    q_head.zero();
    
    q_right_arm[0]=-34.9*DEG2RAD;
    q_right_arm[1]=-38.1*DEG2RAD;
    q_right_arm[2]=-55.2*DEG2RAD;
    q_right_arm[3]=-165.2*DEG2RAD;
    q_right_arm[4]=76.1*DEG2RAD;
    q_right_arm[5]=5.8*DEG2RAD;
    q_right_arm[6]=-1.83*DEG2RAD;
    
    q_left_arm[0]=-82.4*DEG2RAD;
    q_left_arm[1]=34.4*DEG2RAD;
    q_left_arm[2]=15.1*DEG2RAD;
    q_left_arm[3]=-104.1*DEG2RAD;
    q_left_arm[4]=25.0*DEG2RAD;
    q_left_arm[5]=0.3*DEG2RAD;
    q_left_arm[6]=0.1*DEG2RAD;
    
    q_torso[0] = 0.7*DEG2RAD;
    q_torso[1] = -2.0*DEG2RAD;
    q_torso[2] = 0.3*DEG2RAD;
    
    q_right_leg[0]=-1.2*DEG2RAD;
    q_right_leg[1]=-0.5*DEG2RAD;
    q_right_leg[2]=-89.1*DEG2RAD;
    q_right_leg[3]=88.0*DEG2RAD;
    q_right_leg[4]=-2.5*DEG2RAD;
    q_right_leg[5]=-0.15*DEG2RAD;
    
    q_left_leg[0]=18.5*DEG2RAD;
    q_left_leg[1]=20.7*DEG2RAD;
    q_left_leg[2]=-25.5*DEG2RAD;
    q_left_leg[3]=41.2*DEG2RAD;
    q_left_leg[4]=-3.5*DEG2RAD;
    q_left_leg[5]=0.1*DEG2RAD;

    robot.fromRobotToIdyn(q_right_arm,q_left_arm,q_torso,q_right_leg,q_left_leg,q_head,q);

    poses["car_exit"] = q;

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
    q_torso[2] = 1.0;
    
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
    q_torso[2] = -1.0;
    
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

    q_head[0] = 1.0;
    q_head[1] = 0.3;

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
    
    q_head[0] = -1.0;
    q_head[1] = -0.3;
    
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
}
