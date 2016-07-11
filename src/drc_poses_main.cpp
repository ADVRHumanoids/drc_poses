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

    rf.setDefaultConfigFile( "robotology_config.ini" ); 
    rf.setDefaultContext( "drc_poses" );  
    rf.configure(argc, argv);
    drc_poses_module drc_poses_mod = drc_poses_module( argc, argv, "drc_poses", MODULE_PERIOD, rf );

    drc_poses_mod.runModule( rf );

    yarp.fini();

    exit(EXIT_SUCCESS);
}