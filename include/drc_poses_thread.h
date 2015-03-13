#ifndef drc_poses_THREAD_H_
#define drc_poses_THREAD_H_

#include <GYM/control_thread.hpp>

/**
 * @brief drc_poses control thread
 * 
 **/
class drc_poses_thread : public control_thread
{
private:   
    
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

#endif