//Required include files
#include "ros/ros.h"

#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

#include <stdio.h>	
#include <iostream>
#include "pubSysCls.h"

using namespace sFnd;	


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char* argv[]){

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  
  size_t portCount = 0;
  std::vector<std::string> comHubPorts;  
  SysManager myMgr; //Create System Manager myMgr
  
  try
    { 
      
      SysManager::FindComHubPorts(comHubPorts);
      printf("Found %d SC Hubs\n", comHubPorts.size());
      
      for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
	// define the first SC Hub port (port 0) to be associated 
	// with COM portnum (as seen in device manager)
	myMgr.ComHubPort(portCount, comHubPorts[portCount].c_str());
      }
      
      if (portCount > 0) {
	//printf("\n I will now open port \t%i \n \n", portnum);
	myMgr.PortsOpen(portCount); //Open the port
	
	for (size_t i = 0; i < portCount; i++) {
	  IPort &myPort = myMgr.Ports(i);
	  
	  printf(" Port[%d]: state=%d, nodes=%d\n",
		 myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());
 	}
	// Close down the ports
	myMgr.PortsClose();	
      }
      else {
	printf("Unable to locate SC hub port\n");	
	return -1; //This terminates the main program
      }
    }			
  catch(mnErr& theErr) //This catch statement will intercept any error from the Class library
    {
      printf("Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port\n");	
      //This statement will print the address of the error, the error code (defined by the mnErr class), 
      //as well as the corresponding error message.
      printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);          
      return -1; //This terminates the main program
    }


  ros::spin();
  
  return 0; //End program
}
//
