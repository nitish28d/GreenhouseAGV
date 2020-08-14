//Required include files
#include <stdio.h>
#include <Windows.h>
#include <string>
#include <iostream>
#include "pubSysCls.h"	
#include <vector>
#include <sstream>



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

#define ACC_LIM_RPM_PER_SEC	100
#define VEL_LIM_RPM			700
#define MOVE_DISTANCE_CNTS	10000	
#define NUM_MOVES			5
#define TIME_TILL_TIMEOUT	10000	//The timeout used for homing(ms)

volatile int vel, num = 0;
char lpBuffer[] = "!R 2";
int vel1, confrm = 0;
volatile int var, tleft, tright, lm, rm, td, senscount, cppcount = 0;
int k = 0;
int a = 0;
int lmv = 0;
int error, integral, speed = 0;
float ki = .5;
float kp = .7;
HANDLE hComm;                          // Handle to the Serial port
char  ComPortName[] = "\\\\.\\COM8";  // Name of the Serial port(May Change) to be opened,
BOOL  Status;                          // Status of the various operations 
DWORD dwEventMask;                     // Event mask to trigger
DWORD  dNoOFBytestoWrite;              // No of bytes to write into the port
DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port
char  TempChar;                        // Temperory Character
char  SerialBuffer[256];               // Buffer Containing Rxed Data
DWORD NoBytesRead;                     // Bytes read by ReadFile()

int getsensval()
{
	int i = 0;

	Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception

	if (Status == FALSE)
		printf("\n\n    Error! in Setting CommMask");
	/*else
		printf("\n\n    Waiting for Data Reception");*/

	Status = WaitCommEvent(hComm, &dwEventMask, NULL); //Wait for the character to be received

													   /*-------------------------- Program will Wait here till a Character is received ------------------------*/

	if (Status == FALSE)
	{
		printf("\n    Error! in Setting WaitCommEvent()");
	}
	else //If  WaitCommEvent()==True Read the RXed data using ReadFile();
	{
		//printf("\n\n    Characters Received");
		confrm = 1;
		do
		{
			Status = ReadFile(hComm, &TempChar, sizeof(TempChar), &NoBytesRead, NULL);
			SerialBuffer[i] = TempChar;
			i++;
		} while (TempChar != '$');

		std::vector<int> vect;
		std::stringstream ss(SerialBuffer);

		int i;
		while (ss >> i)
		{
			vect.push_back(i);

			if (ss.peek() == ',')
				ss.ignore();
		}

		for (i = 0; i < vect.size(); i++) {
			num = vect.at(i);
			switch (i)
			{

			case 0:
				tright = num;
				//printf("\nTape Left: %d", tleft);
				break;
			case 1:
				tleft = num;
				//printf("\nTape Right: %d", tright);
				break;
			case 2:
				td = num;
				//printf("\nTape Detect: %d", td);
				break;
			case 3:
				lm = num;
				//printf("\nLeft Marker: %d", lm);
				break;
			case 4:
				rm = num;
				//printf("\nRight Marker: %d", rm);
				break;
			default:
				senscount = num;
				//printf("\nSenscount: %d", senscount);
				break;
			}
			/*SYSTEMTIME time;
			GetSystemTime(&time);
			float millis = (time.wSecond * 1000) + time.wMilliseconds;
			printf("\n time : %g", millis/1000);*/

		}
	}
	cppcount++;
	return tleft, tright, td, lm, rm, senscount;
}

int pidfwd(INode &tn1, INode &tn2) {
	if (tleft == 0) {
		vel1 = -70;
		vel = 70;
	}
	else if (tleft < 0) {
		tleft = -tleft;
		integral = integral + tleft;
		if (integral > 50) integral = 50;
		speed = (ki*integral) + (kp*tleft);
		vel1 = -(70 - speed);
		vel = 70;
		if (tleft < 10) integral = 0;
	}
	else if (tleft > 0) {
		integral = integral + tleft;
		if (integral > 50) integral = 50;
		speed = (ki*integral) + (kp*tleft);
		vel = (70 - speed);	//right motor
		vel1 = -70;		//left motor
		if (tleft < 10) integral = 0;
		tn1.Motion.MoveVelStart(vel);
		tn2.Motion.MoveVelStart(vel1);
	}
	return vel, vel1;
}

void stopsequence(INode &tn1, INode &tn2)
{
	strcpy(lpBuffer, "!R 0");		       // stop sensor
	dNoOFBytestoWrite = sizeof(lpBuffer); // Calculating the no of bytes to write into the port
	Status = WriteFile(hComm, lpBuffer, dNoOFBytestoWrite, &dNoOfBytesWritten, NULL);

	tn1.Motion.MoveVelStart(0);			//Move motor 1 
	tn2.Motion.MoveVelStart(0);			//Move motor 2 

	Sleep(5000);

	strcpy(lpBuffer, "!R");		       // start sensor
	dNoOFBytestoWrite = sizeof(lpBuffer); // Calculating the no of bytes to write into the port
	Status = WriteFile(hComm, lpBuffer, dNoOFBytestoWrite, &dNoOfBytesWritten, NULL);
}
int main(int argc, char* argv[])
{
	size_t portCount = 0;
	std::vector<std::string> comHubPorts;
	//Create the SysManager object. This object will coordinate actions among various ports
	// and within nodes. In this example we use this object to setup and open our port.
	SysManager* myMgr = SysManager::Instance();							//Create System Manager myMgr
	SysManager* SensorMgr = SysManager::Instance();						// Create Sensor Manager

																		//==========================================================================================================

	printf("\n\n +==========================================+");
	printf("\n |    Serial Port  Reception (Win32 API)    |  ");
	printf("\n +==========================================+\n");


	try
	{
		SysManager::FindComHubPorts(comHubPorts);
		printf("Found %d SC Hubs\n", comHubPorts.size());

		for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {
			myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str()); 	//define the first SC Hub port (port 0) to be associated 
		}

		if (portCount < 0) {

			printf("Unable to locate SC hub port\n");

			msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key

			return -1;  //This terminates the main program
		}
		//printf("\n I will now open port \t%i \n \n", portnum);
		myMgr->PortsOpen(portCount);				//Open the port

		for (size_t i = 0; i < portCount; i++) {
			IPort &myPort = myMgr->Ports(i);

			printf(" Port[%d]: state=%d, nodes=%d\n",
				myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());

			for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
				// Create a shortcut reference for a node
				INode &theNode = myPort.Nodes(iNode);

				theNode.EnableReq(false);				//Ensure Node is disabled before loading config file

				myMgr->Delay(200);

				//theNode.Setup.ConfigLoad("Config File path");

				printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
				printf("            userID: %s\n", theNode.Info.UserID.Value());
				printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
				printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
				printf("             Model: %s\n", theNode.Info.Model.Value());

				//The following statements will attempt to enable the node.  First,
				// any shutdowns or NodeStops are cleared, finally the node is enabled
				theNode.Status.AlertsClear();					//Clear Alerts on node 
				theNode.Motion.NodeStopClear();	//Clear Nodestops on Node  				
				theNode.EnableReq(true);					//Enable node 
															//At this point the node is enabled
				printf("Node \t%zi enabled\n", iNode);
				double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;	//define a timeout in case the node is unable to enable
																				//This will loop checking on the Real time values of the node's Ready status
				while (!theNode.Motion.IsReady()) {
					if (myMgr->TimeStampMsec() > timeout) {
						printf("Error: Timed out waiting for Node %d to enable\n", iNode);
						msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
						return -2;
					}
				}
				//homing code deleted
			}


			for (;;) {

				// Create a shortcut reference for a node
				size_t iNode = 1;
				INode &theNode = myPort.Nodes(0);

				//theNode.Motion.MoveWentDone();						//Clear the rising edge Move done register

				theNode.AccUnit(INode::RPM_PER_SEC);				//Set the units for Acceleration to RPM/SEC
				theNode.VelUnit(INode::RPM);						//Set the units for Velocity to RPM
																	//theNode.Motion.AccLimit = ACC_LIM_RPM_PER_SEC;		//Set Acceleration Limit (RPM/Sec)
																	//theNode.Motion.VelLimit = VEL_LIM_RPM;				//Set Velocity Limit (RPM)
				INode &theNode1 = myPort.Nodes(1);
				printf("Moving Node \t%zi \n", iNode);

				hComm = CreateFile(ComPortName,                  // Name of the Port to be Opened
					GENERIC_READ | GENERIC_WRITE, // Read/Write Access
					0,                            // No Sharing, ports cant be shared
					NULL,                         // No Security
					OPEN_EXISTING,                // Open existing port only
					0,                            // Non Overlapped I/O
					NULL);                        // Null for Comm Devices

				if (hComm == INVALID_HANDLE_VALUE)
					printf("\n    Error! - Port %s can't be opened\n", ComPortName);
				else
					printf("\n    Port %s Opened\n ", ComPortName);

				/*------------------------------- Setting the Parameters for the SerialPort ------------------------------*/

				DCB dcbSerialParams = { 0 };                         // Initializing DCB structure
				dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

				Status = GetCommState(hComm, &dcbSerialParams);      //retreives  the current settings

				if (Status == FALSE)
					printf("\n    Error! in GetCommState()");

				dcbSerialParams.BaudRate = CBR_115200;    // Setting BaudRate = 9600
				dcbSerialParams.ByteSize = 8;             // Setting ByteSize = 8
				dcbSerialParams.StopBits = ONESTOPBIT;    // Setting StopBits = 1
				dcbSerialParams.Parity = NOPARITY;        // Setting Parity = None 

				Status = SetCommState(hComm, &dcbSerialParams);  //Configuring the port according to settings in DCB 

				if (Status == FALSE)
				{
					printf("\n    Error! in Setting DCB Structure");
				}
				else //If Successfull display the contents of the DCB Structure
				{
					printf("\n\n    Setting DCB Structure Successfull\n");
					printf("\n       Baudrate = %d", dcbSerialParams.BaudRate);
					printf("\n       ByteSize = %d", dcbSerialParams.ByteSize);
					printf("\n       StopBits = %d", dcbSerialParams.StopBits);
					printf("\n       Parity   = %d", dcbSerialParams.Parity);
				}

				/*------------------------------------ Setting Timeouts --------------------------------------------------*/

				COMMTIMEOUTS timeouts = { 0 };
				timeouts.ReadIntervalTimeout = 50;
				timeouts.ReadTotalTimeoutConstant = 50;
				timeouts.ReadTotalTimeoutMultiplier = 10;
				timeouts.WriteTotalTimeoutConstant = 50;
				timeouts.WriteTotalTimeoutMultiplier = 10;

				if (SetCommTimeouts(hComm, &timeouts) == FALSE)
					printf("\n\n    Error! in Setting Time Outs");
				else
					printf("\n\n    Setting Serial Port Timeouts Successfull");
				/*---------------------------------- Opening the Serial Port -------------------------------------------*/
				strcpy(lpBuffer, "!R 2");		       // start sensor
				dNoOFBytestoWrite = sizeof(lpBuffer); // Calculating the no of bytes to write into the port
				Status = WriteFile(hComm,               // Handle to the Serialport
					lpBuffer,            // Data to be written to the port 
					dNoOFBytestoWrite,   // No of bytes to write into the port
					&dNoOfBytesWritten,  // No of bytes written to the port
					NULL);

				while (1) {
					getsensval();
					pidfwd(theNode, theNode1);

					if (lm == 1 && td == 1 && rm != 1) //left turn
					{
						lmv = 1;
						while (lmv) {
							getsensval();
							pidfwd(theNode, theNode1);
							printf("\n A VALUE: %d", a);
							printf("\n RM VALUE: %d", rm);

							//std::cin >> a;
							if ((rm == 1) && (a == 0)) {
								stopsequence(theNode, theNode1);
								lmv = 0;
								a = 1;
								printf("\n AFTER STOPPING A VALUE: %d", a);
								getsensval();
								//std::cin >> a;
								goto label;
							}

							getsensval();
							//pidfwd(theNode, theNode1);
							while (td == 1 && lm == 0)
							{
								getsensval();
								vel = vel1 = 25;
								theNode.Motion.MoveVelStart(vel1);			//Move motor 1 
								theNode1.Motion.MoveVelStart(vel);			//Move motor 2 
							}
							getsensval();
							while (td == 0)
							{
								vel = vel1 = 25;
								getsensval();
								theNode.Motion.MoveVelStart(vel1);			//Move motor 1 
								theNode1.Motion.MoveVelStart(vel);			//Move motor 2 
								lmv = 0;
							}
							getsensval();
							//while ((td == 1) && (tleft > (-20)))
							//{
							//	getsensval();
							//	while (td == 0 && tleft != 0) {
							//		getsensval();
							//		theNode.Motion.MoveVelStart(vel1);			//Move motor 1 
							//		theNode1.Motion.MoveVelStart(vel);			//Move motor 2 
							//	}
							//	vel = vel1 = 25;
							//	getsensval();
							//	theNode.Motion.MoveVelStart(vel1);			//Move motor 1 
							//	theNode1.Motion.MoveVelStart(vel);			//Move motor 2 
							//}
							//while (tleft < -30)
							//{
							//	vel = vel1 = 25;
							//	getsensval();
							//	theNode.Motion.MoveVelStart(vel1);			//Move motor 1 
							//	theNode1.Motion.MoveVelStart(vel);			//Move motor 2 
							//}
							//while (td == 0)
							//{
							//	vel = vel1 = 25;
							//	getsensval();
							//	theNode.Motion.MoveVelStart(vel1);			//Move motor 1 
							//	theNode1.Motion.MoveVelStart(vel);			//Move motor 2 
							//}

						}
					}
					lmv = 0;
					if (rm == 1 && td == 1 && lm != 1)
					{
						printf("\n Entered : %d", a);

						while (td == 1) {
							getsensval();
							vel = vel1 = -25;
							theNode.Motion.MoveVelStart(vel1);			//Move motor 1 
							theNode1.Motion.MoveVelStart(vel);			//Move motor 2 
						}
						while (td == 0) {
							vel = vel1 = -25;
							getsensval();
							theNode.Motion.MoveVelStart(vel1);			//Move motor 1 
							theNode1.Motion.MoveVelStart(vel);			//Move motor 2 
						}
						getsensval;
						while ((td == 1) && (tleft > (-20)))
						{
							getsensval();
							while (td == 0 && tleft != 0) {
								getsensval();
								theNode.Motion.MoveVelStart(vel1);			//Move motor 1 
								theNode1.Motion.MoveVelStart(vel);			//Move motor 2 
							}
							vel = vel1 = -25;
							getsensval();
							theNode.Motion.MoveVelStart(vel1);
							theNode1.Motion.MoveVelStart(vel);
						}
						while (tleft < -30)
						{
							vel = vel1 = -25;
							getsensval();
							theNode.Motion.MoveVelStart(vel1);
							theNode1.Motion.MoveVelStart(vel);
						}
						while (td == 0)
						{
							vel = vel1 = -25;
							getsensval();
							theNode.Motion.MoveVelStart(vel1);
							theNode1.Motion.MoveVelStart(vel);
						}
					}

					if (a == 0 && lm == 1 && rm == 1) {
						stopsequence(theNode, theNode1);
						printf("\n STOP SEQUENCE WHEN BOTH MARKERS DETECTED SAME TIME: %d", a);
						lmv = 0;
						a = 1;
						printf("\n AFTER STOPPING A VALUE: %d", a);
						getsensval();
						goto label;
					}
					if (a == 1) {
						if (lm == 0 && rm == 0) a = 0;
						getsensval();
					}

				label:
					getsensval();
					pidfwd(theNode, theNode1);
					printf("\nspeed %d", speed);
					printf("\n vel%d", vel);
					printf("\n vel1%d", vel1);
					theNode.Motion.MoveVelStart(vel);			//Move motor 1 
					theNode1.Motion.MoveVelStart(vel1);			//Move motor 2 
				}
				printf("Disabling nodes, and closing port\n");

				for (size_t iNode = 0; iNode < myPort.NodeCount(); iNode++) {
					// Create a shortcut reference for a node
					myPort.Nodes(iNode).EnableReq(false);
				}
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
		return 0;  //This terminates the main program
	}

	// Close down the ports
	myMgr->PortsClose();

	msgUser("Press any key to continue."); //pause so the user can see the error message; waits for user to press a key
	return 0;			//End program
}


