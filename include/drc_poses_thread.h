#ifndef drc_poses_THREAD_H_
#define drc_poses_THREAD_H_

#include <GYM/control_thread.hpp>
#include <GYM/yarp_command_interface.hpp>
#include <GYM/yarp_status_interface.h>
#include <trajectory_generator/trajectory_generator.h>
#include "drc_shared/module_statuses/drc_poses_statuses.h"


#include "yaml-cpp/yaml.h"
#include <vector>

namespace walkman
{
namespace drc
{
namespace poses
{
  
struct pose {
    std::string name;
    yarp::sig::Vector right_arm;
    yarp::sig::Vector left_arm;
    yarp::sig::Vector head;
    yarp::sig::Vector torso;
    yarp::sig::Vector right_leg;
    yarp::sig::Vector left_leg;
    bool isMoving_right_arm;
    bool isMoving_left_arm;
    bool isMoving_head;
    bool isMoving_torso;
    bool isMoving_right_leg;
    bool isMoving_left_leg;
    pose () {
      isMoving_right_arm = false;
      isMoving_left_arm  = false;
      isMoving_head      = false;
      isMoving_torso     = false;
      isMoving_right_leg = false;
      isMoving_left_leg  = false;
    };
};


  
/**
 * @brief drc_poses control thread
 * 
 **/
class drc_poses_thread : public control_thread
{
private:
    walkman::yarp_command_interface cmd_interface;
    int cmd_seq_num=0;
    walkman::yarp_status_interface status_interface;
    int status_seq_num=0;
    walkman::drc::poses::status_definitions status_definitions;

    bool busy=false;

    yarp::sig::Vector q_input;
    yarp::sig::Vector delta_q;
    yarp::sig::Vector q_output;
    yarp::sig::Vector q_initial;

    yarp::sig::Vector compute_delta_q();

    std::vector<std::string> commands;

    trajectory_generator joints_traj_gen;
    polynomial_coefficients poly;
    double initialized_time,next_time,final_time;
    std::map<std::string,yarp::sig::Vector> poses;
    yarp::sig::Vector q_desired;
    std::string last_command;
    bool demo_mode=false;
    
    //Poses vector to be filled from yaml file
    std::vector<pose> posesVector;

    bool action_completed();
    void create_poses();
    void print_help();

//     yarp::sig::Vector  wave_q_torso_start;
    yarp::sig::Vector  wave_q_left_arm_start;
    yarp::sig::Vector  wave_q_left_arm_end;
    yarp::sig::Vector  wave_q_head_start;
    yarp::sig::Vector  wave_q_head_end;
    yarp::sig::Vector  wave_q_right_arm_start;
    yarp::sig::Vector  wave_q_right_arm_end;
    
    yarp::sig::Vector recover_q_right_arm;
    yarp::sig::Vector recover_q_left_arm;
    yarp::sig::Vector recover_q_torso;
    yarp::sig::Vector recover_q_right_leg;
    yarp::sig::Vector recover_q_left_leg;
    yarp::sig::Vector recover_q_head;

    yarp::sig::Vector walking_q_right_arm;
    yarp::sig::Vector walking_q_left_arm;
    yarp::sig::Vector walking_q_torso;
    yarp::sig::Vector walking_q_right_leg;
    yarp::sig::Vector walking_q_left_leg;
    yarp::sig::Vector walking_q_head;
    
    yarp::sig::Vector drive_q_right_arm;
    yarp::sig::Vector drive_q_left_arm;
    yarp::sig::Vector drive_q_torso;
    yarp::sig::Vector drive_q_right_leg;
    yarp::sig::Vector drive_q_left_leg;
    yarp::sig::Vector drive_q_head;

    yarp::sig::Vector pre_homing_q_right_arm;
    yarp::sig::Vector pre_homing_q_left_arm;
    yarp::sig::Vector pre_homing_q_torso;
    yarp::sig::Vector pre_homing_q_right_leg;
    yarp::sig::Vector pre_homing_q_left_leg;
    yarp::sig::Vector pre_homing_q_head;
    yarp::sig::Vector pre_homing_q_left_hand;
    yarp::sig::Vector pre_homing_q_right_hand;
    
    int kinematic_joints, actuated_joints;
    int left_arm_joints, right_arm_joints, torso_joints, head_joints, left_leg_joints, right_leg_joints;
    
    yarp::sig::Vector joint_sense();
    
    bool loadPoses(std::string yamlFilename);
    bool savePose(const yarp::sig::Vector &q_in, std::string filename, int poseNumber);
public:
    
    /**
     * @brief constructor
     * 
     * @param module_prefix the prefix of the module
     * @param rf resource finderce
     * @param ph param helper
     */
     drc_poses_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr<paramHelp::ParamHelperServer> ph );
    
    
    /**
     * @brief drc_poses control thread initialization
     * 
     * @return true on succes, false otherwise
     */
    virtual bool custom_init();
    
    /**
     * @brief drc_poses control thread main loop
     * 
     */
    virtual void run();
    
};
}
}
}
#endif