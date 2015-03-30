#include <yarp/os/all.h>

#include "drc_poses_thread.h"
#include "drc_poses_constants.h"

using namespace walkman::drc::poses;
using namespace yarp::math;

#define Max_Vel         0.3 // maximum joint velocity [rad/s]
#define Min_Texe        3.0 // minimum execution time for homing [sec]

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
	if(cmd_interface.getCommand(cmd,cmd_seq_num))
	{
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

    for(int i=0;i<q_input.size();i++)
    {
	joint_err += fabs(q_input[i]-q_desired[i]);
    }

    joint_err /= robot.getNumberOfJoints();

    return (joint_err < 0.1);
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
}
