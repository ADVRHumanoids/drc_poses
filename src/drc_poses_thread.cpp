#include <yarp/os/all.h>

#include "drc_poses_thread.h"
#include "drc_poses_constants.h"

drc_poses_thread::drc_poses_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph )
{
    // TODO: skeleton constructor
}

bool drc_poses_thread::custom_init()
{
    // TODO: skeleton function   
    return true;
}

void drc_poses_thread::run()
{   
    // TODO: skeleton function
}