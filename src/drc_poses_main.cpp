#include <yarp/os/all.h>
#include <cstdlib>

#include "drc_poses_module.hpp"

// default module period
#define MODULE_PERIOD 1000 //[millisec]

int main(int argc, char* argv[])
{
    // yarp network declaration and check
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cerr <<"yarpserver not running - run yarpserver"<< std::endl;
        exit(EXIT_FAILURE);
    }
    // yarp network initialization
    yarp.init();

    // create rf
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    // set drc_poses_initial_config.ini as default
    // to specify another config file, run with this arg: --from your_config_file.ini 
    rf.setDefaultConfigFile( "config.ini" ); 
    rf.setDefaultContext( "drc_poses" );  
    rf.configure(argc, argv);
    // create my module
    drc_poses_module drc_poses_mod = drc_poses_module( argc, argv, "drc_poses", MODULE_PERIOD, rf );
    
    // run the module
    drc_poses_mod.runModule( rf );
    
    // yarp network deinitialization
    yarp.fini();
    
    exit(EXIT_SUCCESS);
}