#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <conio.h>
#include <Windows.h>
#include "pose_to_length.h"
#include "Dependencies\sFoundation20\inc\pubSysCls.h"
#include "Dependencies\DynamixelSDK-3.7.31\include\dynamixel_sdk\dynamixel_sdk.h"

using namespace std;
using namespace sFnd;

int CheckMotorNetwork();

vector<string> comHubPorts;
vector<INode*> nodeList; // create a list for each node
vector<vector<double>> points;
vector<double> spcLimit;
unsigned int portCount;
const int nodeNum = 8; // !!!!! IMPORTANT !!!!! Put in the number of motors before compiling the programme
double step = 0.01; // in meters, for manual control
float targetTorque = -2.5; // in %, -ve for tension, also need to UPDATE in switch case 't'!!!!!!!!!
const int MILLIS_TO_NEXT_FRAME = 35; // note the basic calculation time is abt 16ms
double home[6] = {2.211, -3.482, 1.012, 0, 0, 0}; // home posisiton
double offset[8]; // L0, from "zero posi1tion", will be updated by "set home" command
double in1[6] = {2.211, -3.482, 1.012, 0, 0, 0};
double out1[8] = {2.87451, 2.59438, 2.70184, 2.40053, 2.46908, 2.15523, 2.65123, 2.35983}; // assume there are 8 motors
double a[6], b[6], c[6], d[6], e[6], f[6], g[6], tb[6]; // trajectory coefficients
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
uint8_t dxl_error = 0; // Dynamixel error
int32_t dxl1Pos = 0, dxl2Pos = 0;
int dxl_comm_result, rotationG, gripperG;
const int DXL1_ID = 10, DXL2_ID = 3; //dxl 1 is rotation motor, dxl 2 is gripper motor
const int ADDR_TORQUE_ENABLE = 64, ADDR_GOAL_POSITION = 116, ADDR_PRESENT_POSITION = 132, ADDR_GOAL_CURRENT = 102, ADDR_PRESENT_CURRENT = 126; // Control table adresses
const int LEN_GOAL_POSITION = 4, LEN_PRESENT_POSITION = 4, LEN_GOAL_CURRENT = 2, LEN_PRESENT_CURRENT = 2;

char cmd;

int main()
{   
    //// Check motor network
    SysManager* myMgr = SysManager::Instance();
    // Start the programme, scan motors in network
    try{
        if (CheckMotorNetwork() < 0){
            cout << "Motor network not available. Exit programme." << endl;
            return -1;
        }
    }
    catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
        printf("Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port\n");  
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        return -1;
    }
    IPort &myPort = myMgr->Ports(0);
    // cout << "Motor network available. Pick from menu for the next action:\nt - Tighten cables with Torque mode\ny - Loose the cables\ns - Set current position as home\nh - Move to Home\n8 - Manually adjust cable lengths\nu - Update current position from external file\ni - Info: show menu\nn - Move on to Next step" << endl;

    //// Initialize dynamexial gripper
    // Initialize PacketHandler and PacketHandler instance
    portHandler = dynamixel::PortHandler::getPortHandler("COM9");
    packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    // dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);       
    {    // Open port
        if (portHandler->openPort()) { cout << "Succeeded to open the port!\n"; }
        else { cout << "Failed to open the port!\nPress any key to terminate...\n"; getch(); return -1; }
        // Set port baudrate
        if (portHandler->setBaudRate(57600)) { cout << "Succeeded to change the baudrate!\n"; }
        else { cout << "Failed to change the baudrate!\nPress any key to terminate...\n"; getch(); return -1; }

        // Enable Dynamixel#1 Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        else { printf("Dynamixel#%d has been successfully connected \n", DXL1_ID); }

        Sleep(20);
        // Enable Dynamixel#2 Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        else { printf("Dynamixel#%d has been successfully connected \n", DXL2_ID); }

        // Setup Dynamixel#2 Current
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, 120, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Current Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        else { printf("Goal current of Dynamixel#%d is set \n", DXL2_ID); }

        // Add parameter storage for present position value
        if(!groupSyncRead.addParam(DXL1_ID)){ cout << DXL1_ID << " groupSyncRead addparam failed\n"; }
        if(!groupSyncRead.addParam(DXL2_ID)){ cout << DXL2_ID << " groupSyncRead addparam failed\n"; }
    }
    //// Read input .txt file

    //// Plan trajectory, include gripper motion

    //// Run motion and loop, update progress .txt
    cout << "Pick from menu:\nr - Read present position\na - Rotation angle\np - Gripper position\nq- quit\n";
    while(cmd != 'q'){
        cmd = getch();
        switch(cmd){
            case 'r':
                if(groupSyncRead.txRxPacket()){ cout << "Comm result: " << dxl_comm_result << endl; }
                // Get present position value
                dxl1Pos = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
                dxl2Pos = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
                printf("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\n", DXL1_ID, dxl1Pos, DXL2_ID, dxl2Pos);
                break;
            case 'a':
                cout << "Rotaiton: ";
                cin >> rotationG;
                if(cin.good()){
                    rotationG *= 11.26666;
                    if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, rotationG, &dxl_error)){ cout << "Error in 'a' command\n"; };
                }
                break;
            case 'p':
                cout << "Gripper position: ";
                cin >> gripperG;
                if(cin.good()){
                    cout << "hi\n";
                    if(packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, gripperG, &dxl_error)){ cout << "Error in 'p' command\n"; };
                }
                break;
            default:
                break;
        }
    }
    
    //// Reverse motion??

    // Safe system shut down and emegency shut down

    //// List of what-if-s??

    {   // Disable Dynamixel Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }

        // Close port
        portHandler->closePort();
    }
    cout << "Dynamixel is closed correctly\n";
    
    return 1;
}

int CheckMotorNetwork() {
    /*SysManager* myMgr = SysManager::Instance();

    sFnd::SysManager::FindComHubPorts(comHubPorts);

    cout << "Found " <<comHubPorts.size() << " SC Hubs\n";
    for (portCount = 0; portCount < comHubPorts.size(); portCount++) {
        myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());
    }
    if (portCount < 0) {
        cout << "Unable to locate SC hub port\n";
        return -1;
    }
    if(portCount==0) { return -1; } // do we need this?
    
    myMgr->PortsOpen(portCount);
    for (int i = 0; i < portCount; i++) { // check no. of nodes in each ports
        IPort &myPort = myMgr->Ports(i);
        myPort.BrakeControl.BrakeSetting(0, BRAKE_AUTOCONTROL); // do we need this?
        myPort.BrakeControl.BrakeSetting(1, BRAKE_AUTOCONTROL);
        printf(" Port[%d]: state=%d, nodes=%d\n", myPort.NetNumber(), myPort.OpenState(), myPort.NodeCount());
    
        for (int iNode = 0; iNode < myPort.NodeCount(); iNode++) {
            INode &theNode = myPort.Nodes(iNode);
            theNode.EnableReq(false); //Ensure Node is disabled before loading config file
            myMgr->Delay(200);

            printf("   Node[%d]: type=%d\n", int(iNode), theNode.Info.NodeType());
            printf("            userID: %s\n", theNode.Info.UserID.Value());
            printf("        FW version: %s\n", theNode.Info.FirmwareVersion.Value());
            printf("          Serial #: %d\n", theNode.Info.SerialNumber.Value());
            printf("             Model: %s\n", theNode.Info.Model.Value());

            theNode.Status.AlertsClear();               //Clear Alerts on node 
            theNode.Motion.NodeStopClear();             //Clear Nodestops on Node               
            theNode.EnableReq(true);                    //Enable node 
            theNode.Motion.PosnMeasured.AutoRefresh(true);
            theNode.Motion.TrqMeasured.AutoRefresh(true);
            printf("Node %d enabled. ", iNode);

            theNode.AccUnit(INode::RPM_PER_SEC);        //Set the units for Acceleration to RPM/SEC
            theNode.VelUnit(INode::RPM);                //Set the units for Velocity to RPM
            theNode.Motion.AccLimit = 40000;           //100000 Set Acceleration Limit (RPM/Sec)
            theNode.Motion.NodeStopDecelLim = 5000;
            theNode.Motion.VelLimit = 3000;             //700 Set Velocity Limit (RPM)
            theNode.Info.Ex.Parameter(98, 1);           //enable interrupting move
            cout << "AccLimit and VelLimit set" << endl;

            nodeList.push_back(&theNode);               // add node to list

            double timeout = myMgr->TimeStampMsec() + 2000; //TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable
            //This will loop checking on the Real time values of the node's Ready status
            while (!theNode.Motion.IsReady()) {
                if (myMgr->TimeStampMsec() > timeout) {
                    printf("Error: Timed out waiting for Node %d to enable\n", iNode);
                    return -2;
                }
            }
        }
    }*/
    return 0;
}
