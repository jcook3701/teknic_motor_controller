//Required include files
// #include <Windows.h>
#include <unistd.h>
#include <stdio.h>	
#include <string>
#include <iostream>
#include "pubSysCls.h"	

using namespace sFnd;

// Send message and wait for newline
void msgUser(const char *msg) {
	std::cout << msg;
	getchar();
}

//*********************************************************************************
//This program will load configuration files onto each node connected to the port, then executes
//sequential repeated moves on each axis.
//*********************************************************************************

#define ACC_LIM_RPM_PER_SEC	20
#define VEL_LIM_RPM			200
#define MOVE_DISTANCE_CNTS	199080
#define NUM_MOVES			19
#define TIME_TILL_TIMEOUT	10000	//The timeout used for homing(ms)


int main(int argc, char* argv[])
{
	//	msgUser("Motion Example starting. Press Enter to continue.");

	int Node0_Cnts[] = { -19908, 0, 0, -2300, 40527, 42527, 700, 40527, 40527 , 5400, 40527,  41527, -1000, 40527,  41527, 5688, 0, 0, 28440 };
	int Node1_Cnts[] = { 19908, 0, 0, -2300, -40527, -42527, 700, -40527, -40527 , 5400,-40527, -41527, -1000, -40527, -41527, 5688, 0, 0, -28440 };

	int Node0_GPIO[] = { 0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0 };
	int Node1_GPIO[] = { 0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0 };

	printf("Global Cmd starting now");
	//Sleep(1000);
	size_t portCount = 0;
	std::vector<std::string> comHubPorts;


	//Create the SysManager object. This object will coordinate actions among various ports
	// and within nodes. In this example we use this object to setup and open our port.
	// SysManager* myMgr = SysManager::Instance();							//Create System Manager myMgr
	SysManager myMgr;
																		//This will try to open the port. If there is an error/exception during the port opening,
																		//the code will jump to the catch loop where detailed information regarding the error will be displayed;
																		//otherwise the catch loop is skipped over
	try
	{

		SysManager::FindComHubPorts(comHubPorts);
		printf("Found %d SC Hubs\n", comHubPorts.size());

		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {

			myMgr.ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated 
																			// with COM portnum (as seen in device manager)
		}

		if (portCount < 1) {

			printf("Unable to locate SC hub port\n");
			//Sleep(60000);
			msgUser("Press any key to continue no hub on port."); //pause so the user can see the error message; waits for user to press a key

			return -1;  //This terminates the main program
		}
		//printf("\n I will now open port \t%i \n \n", portnum);
		myMgr.PortsOpen(portCount);				//Open the port

		for (size_t i = 0; i < portCount; i++) {
			IPort &myPort = myMgr.Ports(i);

			printf(" Port[%d]: state=%d, nodes=%d\n",
				myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());



			//Once the code gets past this point, it can be assumed that the Port has been opened without issue
			//Now we can get a reference to our port object which we will use to access the node objects
			myPort.BrakeControl.BrakeSetting(0, GPO_OFF);
			myPort.BrakeControl.BrakeSetting(1, GPO_OFF);

			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				INode &theNode = myPort.Nodes(iNode);

				theNode.EnableReq(false);				//Ensure Node is disabled before loading config file

				myMgr.Delay(200);

				theNode.Status.AlertsClear();					//Clear Alerts on node 
				theNode.Motion.NodeStopClear();	//Clear Nodestops on Node  				
				theNode.EnableReq(true);					//Enable node 
															//At this point the node is enabled
				printf("Node \t%zi enabled\n", iNode);
				double timeout = myMgr.TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
																				//This will loop checking on the Real time values of the node's Ready status
				while (!theNode.Motion.IsReady()) {
					if (myMgr.TimeStampMsec() > timeout) {
						printf("Error Node Setup: Timed out waiting for Node %d to enable\n", iNode);
						sleep(60000);
						//msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}

			}

			///////////////////////////////////////////////////////////////////////////////////////
			//At this point we will execute 10 rev moves sequentially on each axis
			//////////////////////////////////////////////////////////////////////////////////////

			for (size_t i = 0; i < NUM_MOVES; i++)
			{
				printf("Cmd number %d running\n", i);
				if (i == 0) {
					myPort.BrakeControl.BrakeSetting(0, GPO_OFF);
					myPort.BrakeControl.BrakeSetting(1, GPO_OFF);
				}
				else {

					if (Node0_GPIO[i] == 0)
						myPort.BrakeControl.BrakeSetting(0, GPO_OFF);
					else
						myPort.BrakeControl.BrakeSetting(0, GPO_ON);

					if (Node1_GPIO[i] == 0)
						myPort.BrakeControl.BrakeSetting(1, GPO_OFF);
					else
						myPort.BrakeControl.BrakeSetting(1, GPO_ON);

					if ((Node0_GPIO[i - 1] != Node0_GPIO[i]) || (Node1_GPIO[i - 1] != Node1_GPIO[i]))
						sleep(20000); //wait for acctuator
				}

				for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
					// Create a shortcut reference for a node
					INode &theNode = myPort.Nodes(iNode);

					theNode.Motion.MoveWentDone();						//Clear the rising edge Move done register

					theNode.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
					theNode.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
					theNode.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
					theNode.Motion.VelLimit = VEL_LIM_RPM;				//Set Velocity Limit (RPM)
																		//					theNode.TrqUnit(35);
																		//					theNode.Limits.TrqGlobal = 0;

																		// Set the trigger group indicator
					theNode.Motion.Adv.TriggerGroup(1);

					printf("Moving Node \t%zi \n", iNode);
					if (iNode == 0)
						theNode.Motion.Adv.MovePosnStart(Node0_Cnts[i]);

					else
						theNode.Motion.Adv.MovePosnStart(Node1_Cnts[i]);

				} //nodes

				  //  motors to start.");

				INode &theNode0 = myPort.Nodes(0);
				INode &theNode1 = myPort.Nodes(1);

				theNode0.Motion.Adv.TriggerMovesInMyGroup();

				double timeout = myMgr.TimeStampMsec() + theNode0.Motion.MovePosnDurationMsec(MOVE_DISTANCE_CNTS) + 80000;

				while (!theNode0.Motion.MoveIsDone()) {
					if (myMgr.TimeStampMsec() > timeout) {
						printf("Error command 1: Timed out waiting for move to complete\n");
						sleep(20000);//						
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
				printf("Cmd number %d ran \n", i);
				// ********* add trim comand here
				if (Node0_Cnts[i] != 0) {
					std::cout << " Enter Trim value";
					int TrimCnt = 0;
					std::cin >> TrimCnt;
					msgUser("Press any key to execute Trim.");
					printf("Doing Trim at Value: %d \n", TrimCnt);

					if (TrimCnt != 0) {
						theNode0.Motion.Adv.MovePosnStart(TrimCnt);
						theNode1.Motion.Adv.MovePosnStart(TrimCnt);

						theNode0.Motion.Adv.TriggerMovesInMyGroup();

						while (!theNode0.Motion.MoveIsDone()) {
							if (myMgr.TimeStampMsec() > timeout) {
								printf("Error command 1: Timed out waiting for move to complete\n");
								sleep(20000);//						
								msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
								return -2;
							} //end if
						} //end while

					} // if doing trim - any value other than zero
				} // if skip trim altogether

				msgUser("Press any key to start next CMD.");
			} // for each move
			printf("Node \t%zi Move Done\n", 0);


			//////////////////////////////////////////////////////////////////////////////////////////////
			//After moves have completed Disable node, and close ports
			//////////////////////////////////////////////////////////////////////////////////////////////
			printf("Disabling nodes, and closing port\n");
			//Disable Nodes

			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				myPort.Nodes(iNode).EnableReq(false);
			}
		}
	}
	catch (mnErr& theErr)
	{
		printf("Failed to disable Nodes n\n");
		//This statement will print the address of the error, the error code (defined by the mnErr class), 
		//as well as the corresponding error message.
		printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);

		msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
		sleep(10000);
		return -5;  //This terminates the main program
	}

	// Close down the ports
	myMgr.PortsClose();
	printf("leaving soon");
	//Sleep(1000);
	msgUser("Press any key to continue - end of cmd."); //pause so the user can see the error message; waits for user to press a key
	return 5;			//End program
}

