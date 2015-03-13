#include <yarp/os/all.h>
#include <cstdlib>
#include "drc_poses_module.hpp"

#define MODULE_PERIOD 1000

int main(int argc, char* argv[])
{
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cerr <<"yarpserver not running - run yarpserver"<< std::endl;
        exit(EXIT_FAILURE);
    }
    yarp.init();

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);

    rf.setDefaultConfigFile( "config.ini" ); 
    rf.setDefaultContext( "drc_poses" );  
    rf.configure(argc, argv);
    drc_poses_module drc_poses_mod = drc_poses_module( argc, argv, "drc_poses", MODULE_PERIOD, rf );

    drc_poses_mod.runModule( rf );

    yarp.fini();

    exit(EXIT_SUCCESS);
}