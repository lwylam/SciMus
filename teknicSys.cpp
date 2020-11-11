#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <conio.h>
#include <Windows.h>
#include "pose_to_length.h"
#include "Dependencies\sFoundation20\inc\pubSysCls.h"
#include "Dependencies\DynamixelSDK-3.7.31\include\dynamixel_sdk\dynamixel_sdk.h"

using namespace std;
using namespace sFnd;

int CheckMotorNetwork();
int RunParaBlend(double point[7], bool showAttention = false);
void RunBricksTraj(dynamixel::GroupSyncRead groupSyncRead, int listOffset, bool showAttention = false);
void ReverseBricksTraj(dynamixel::GroupSyncRead groupSyncRead, int listOffset, bool showAttention = false);
void SendMotorGrp(bool IsTorque = false, bool IsLinearRail = false);
int32_t ToMotorCmd(int motorID, double length);
void TrjHome();
bool CheckLimits();
bool ReadBricksFile();
void HomeLinearRail(int n);
void MN_DECL AttentionDetected(const mnAttnReqReg &detected); // this is attention from teknic motors

vector<string> comHubPorts;
vector<INode*> nodeList; // create a list for each node
vector<vector<double>> brickPos;
vector<double> spcLimit;
unsigned int portCount;
char attnStringBuf[512]; // Create a buffer to hold the attentionReg information    
const int NodeNum = 8; // !!!!! IMPORTANT !!!!! Put in the number of motors before compiling the programme
const double RAIL_UP = 1.25, RAIL_DOWN = 0; // Linear rail upper and lower bound
const double endEffOffset = -0.145; // meters, offset from endeffector to ground
double step = 0.01; // in meters, for manual control
float targetTorque = -2.5; // in %, -ve for tension, also need to UPDATE in switch case 't'!!!!!!!!!
const int MILLIS_TO_NEXT_FRAME = 35; // note the basic calculation time is abt 16ms
double home[6] = {2.182, 1.9795, 0.9610, 0, 0, 0}; // home posisiton
double offset[12]; // L0, from "zero position", will be updated by "set home" command
double railOffset = 0; // linear rails offset
double in1[6] = {2.1977, 1.979, 0.9591, 0, 0, 0};
double out1[12] = {2.87451, 2.59438, 2.70184, 2.40053, 2.46908, 2.15523, 2.65123, 2.35983, 0, 0, 0, 0}; // assume there are 8 motors + 4 linear rails
double a[6], b[6], c[6], d[6], e[6], f[6], g[6], tb[6]; // trajectory coefficients
char limitType = 'C'; // A for home, B for limits, C for default
char quitType = 'r'; // q for emergency quit, f for finish traj, r for resume/default

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
uint8_t dxl_error = 0; // Dynamixel error
int32_t dxl1Pos = 0, dxl2Pos = 0, gpOpen = 800, gpClose = 1300, neutralRot = 1707; // define some position reading, and gripper commands. neutralRot is 1024.90 deg // 1707 is 150 deg
int dxl_comm_result, rotationG, gripperG;
const int DXL1_ID = 1, DXL2_ID = 2; //dxl 1 is rotation motor, dxl 2 is gripper motor
const int ADDR_TORQUE_ENABLE = 64, ADDR_GOAL_POSITION = 116, ADDR_PRESENT_POSITION = 132, ADDR_GOAL_CURRENT = 102, ADDR_PRESENT_CURRENT = 126; // Control table adresses
const int LEN_GOAL_POSITION = 4, LEN_PRESENT_POSITION = 4, LEN_GOAL_CURRENT = 2, LEN_PRESENT_CURRENT = 2, DXL_THRESHOLD = 10;


int main()
{   
    //// initiallize cable robot motor network
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
    IPort &myPort = myMgr->Ports(2);
    pose_to_length(home, offset); // save offset values according to home pose

    cout << "Motor network available. Pick from menu for the next action:\nt - Tighten cables with Torque mode\ny - Loose the cables\ns - Set current position as home\nh - Move to Home\n8 - Manually adjust cable lengths\nl - Linear rails motions\nu - Update current position from external file\nr - Reset Rotation to zero\ni - Info: show menu\nn - Move on to Next step" << endl;
    char cmd;
    try{
        do {
            bool allDone = false, stabilized = false;
            cin >> cmd;
            switch (cmd){
                case 'i':   // Show menu
                    cout << "Pick from menu for the next action:\nt - Tighten cables with Torque mode\ny - Loose the cables\ns - Set current position as home\nh - Move to Home\n8 - Manually adjust cable lengths\nl - Linear rails motions\nu - Update current position from external file\nr - Reset Rotation to zero\ni - Info: show menu\nn - Move on to Next step\n";
                    break;
                case 'y':   // Loosen cables using positive torque
                    targetTorque = 1;
                case 't':   // Tighten cables according to torque **Only for 8 motors
                    cout << "Current target torque = " << targetTorque << endl;
                    for(INode* n : nodeList){ n->Motion.AccLimit = 200; }
                    while(!stabilized) {
                        SendMotorGrp(true);
                        Sleep(50);
                        for (int n = 0; n < NodeNum; n++){
                            if(nodeList[n]->Motion.TrqCommanded.Value() > targetTorque) { break; }
                            stabilized = true;
                        }
                    } 
                    for(INode* n : nodeList){
                        n->Motion.TrqCommanded.Refresh();
                        cout << n->Motion.TrqCommanded.Value() << "\t";
                        n->Motion.MoveVelStart(0);
                        n->Motion.AccLimit = 40000;
                    }
                    cout << "\nTorque mode tightening completed" << endl;
                    targetTorque = -2.5;
                    break;
                case 's':   // Set zero
                    for (int n = 0; n < NodeNum; n++){
                        nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value()); // Zeroing the number space around the current Measured Position
                    }
                    copy(begin(home), end(home), begin(in1)); // copy home array into input array
                    cout << "Setting zero completed" << endl;
                    cout <<  "Home coordinates: " << in1[0] << ", " << in1[1] << ", " << in1[2] << ", " << in1[3] << ", " << in1[4] << ", " << in1[5] << endl;

                    cout << "Do you want to set current rail position as zero? (k - OK)\n";;
                    cin >> cmd;
                    if(cmd == 'k'){
                        for (int n = NodeNum; n < NodeNum+4; n++){
                            nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value());
                        }
                        cout << "Linear rails are set to zero.\n";
                    }
                    else{ cout << "Current rail position may not be at zero\n"; }
                    break;
                case 'h':   // Homing for all motors!! Including linear rail
                    allDone = false;
                    for (int n = 0; n<nodeList.size(); n++) { 
                        nodeList[n]->Motion.MoveWentDone();
                        nodeList[n]->Motion.MovePosnStart(0, true); // absolute position
                    }
                    while(!allDone) {
                        for (INode* n : nodeList) {
                            if(!n->Motion.MoveIsDone()) { break; }
                            allDone = true;
                        }
                    }
                    copy(begin(home), end(home), begin(in1)); // copy home array into input array
                    cout << "Homing completed" << endl;
                    break;
                case '8':   // Manual cable adjustment
                    cout << "0 to 7 - motor id to adjust cable length\n8 - move 4 linear rails together\na or d - increase or decrease cable length\nb - Back to previous menu\n";
                    while(cmd != 'b'){
                        cin >> cmd;
                        if('/' < cmd && cmd < NodeNum + 49){
                            int id = cmd - 48;
                            int sCount = ToMotorCmd(-1, step) / 5;
                            cout << "Motor "<< cmd <<" selected.\n";
                            do{
                                cmd = getch();
                                switch(cmd){
                                    case 'a':
                                        if(id == NodeNum){ for(int n = NodeNum; n<NodeNum+4; n++){ nodeList[n]->Motion.MovePosnStart(sCount); }}
                                        else { nodeList[id]->Motion.MovePosnStart(sCount); }
                                        break;
                                    case 'd':
                                        if(id == NodeNum){ for(int n = NodeNum; n<NodeNum+4; n++){ nodeList[n]->Motion.MovePosnStart(-sCount); }}
                                        else { nodeList[id]->Motion.MovePosnStart(-sCount); }
                                        break;
                                    case 'i':
                                        if(id == NodeNum){
                                            for(int n = NodeNum; n<NodeNum+4; n++){
                                                nodeList[n]->Motion.PosnMeasured.Refresh();
                                                cout << (double) nodeList[n]->Motion.PosnMeasured << endl;
                                            }
                                            cout << endl;
                                        }
                                        else{
                                            nodeList[id]->Motion.PosnMeasured.Refresh();
                                            cout << (double) nodeList[id]->Motion.PosnMeasured << endl;
                                        }
                                        break;
                                    case 'h':
                                        if(id == NodeNum){ cout << "Homing for linear rails are not implemented here.\n"; break; }
                                        nodeList[id]->Motion.VelLimit = 300;
                                        nodeList[id]->Motion.MoveWentDone();
                                        nodeList[id]->Motion.MovePosnStart(0, true);
                                        while(!nodeList[id]->Motion.MoveIsDone()){}
                                        cout << "Individual homing completed.\n";
                                        nodeList[id]->Motion.VelLimit = 3000;
                                        break;
                                }                        
                                Sleep(100); // do we need this?
                            }while(cmd =='a'|| cmd =='d' || cmd =='h' || cmd =='i');
                            cout << "Motor "<< id <<" deselected.\n";
                        }
                    }
                    cout << "Manual adjustment terminated" << endl;
                    break;
                case 'r':
                    cout << "Attenion: robot will rotate to 0,0,0...\n";
                    {
                        double point[7] = {0,0,0,0,0,0,3500}; // Default 3500 ms duaration to rotate to 0
                        copy(in1, in1+3, begin(point)); // copy x,y,z position
                        cout << "Goal coordinates: " << point[0] << ", " << point[1] << ", " << point[2] << ", " << point[3] << ", " << point[4] << ", " << point[5] << ", " << point[6] << endl;
                        if(RunParaBlend(point) < 0) { cout << "Trajectory aborted.\n"; break; }
                        cout << "Rotation reset completed.\n";      
                    }
                    break;
                case 'l':   // Linear rail motions
                    cout << "0 to 3 - home linear rail individually\n4 - home all 4 linear rails automatically\nb - Back to previous menu\nany other keys - stop the linear rail from current motion\n";
                    while(cmd != 'b'){
                        cin >> cmd;
                        if('/' < cmd && cmd < 52){ // homing individually
                            int id = cmd - 48;
                            cout << "Homing linear rail #"<< cmd <<".\n";
                            HomeLinearRail(id);
                        }
                        else if(cmd == 52){ // homing 4 all tgt
                            for (int i = 0; i < 4 ; i++){
                                HomeLinearRail(i);
                            }
                            cout << "All linear rails are homed.\n";
                        }
                    }
                    cout << "Linear rail homing terminated\n";
                    break; 
                case 'u':   // Update in1[] and offset[] from csv file
                    ifstream file ("lastPos.txt"); //"lastPos.txt" or "currentPos.csv"
                    string temp;
                    int count = 0;
                    if(file.is_open()){
                        try{
                            while (file >> temp){
                                if(count > 5) { railOffset = stod(temp); break; } // reading the rail offset, then break while loop
                                in1[count++] = stod(temp); // convert string to double stod()
                            }
                            cout << "Completed reading from external file" << endl; //"Completed updating from external pose file"
                        }
                        catch(int e){ cout << "Check if currentPos.csv matches the in1 input no." << endl; }
                        // do{
                        //     cout << "Current linear rail offset: ";
                        //     cin >> railOffset; // do we need other constraits? ie 0 <= railOffset < 2
                        // }while(!cin.good() || railOffset>2);
                        
                        pose_to_length(in1, out1, railOffset);
                        for (int n = 0; n < nodeList.size(); n++){
                            int32_t step = ToMotorCmd(n, out1[n]);
                            nodeList[n]->Motion.PosnMeasured.Refresh();
                            nodeList[n]->Motion.AddToPosition(-nodeList[n]->Motion.PosnMeasured.Value() + step);
                        }
                        cout << "Updating motor counts completed" << endl;
                        cout << "Current coordinates: " << in1[0] << ", " << in1[1] << ", " << in1[2] << ", " << in1[3] << ", " << in1[4] << ", " << in1[5] << endl;
                        cout << "Motor internal counts: ";
                        for (int id = 0; id < nodeList.size(); id++){
                            nodeList[id]->Motion.PosnMeasured.Refresh();
                            cout << (double) nodeList[id]->Motion.PosnMeasured << "\t";
                        }
                        cout << endl;
                        cout << "Linear rail offset: " << railOffset << endl;
                    }
                    break;
            }
        } while(cmd != 'n');
    }
    catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
        printf("ERROR: Motor command failed.\n");  
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);
        return -3;
    }
    
    //// Initialize dynamexial gripper
    portHandler = dynamixel::PortHandler::getPortHandler("\\\\.\\COM31"); // Initialize PacketHandler and PacketHandler instance
    packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    // dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);       
    {    // Open dynamixel port
        if (portHandler->openPort()) { cout << "Succeeded to open Dynamixel port\n"; }
        else { cout << "Failed to open the port!\nPress any key to terminate...\n"; getch(); return -1; }
        // Set port baudrate
        if (portHandler->setBaudRate(57600)) { cout << "Succeeded to change Dynamixel baudrate!\n"; }
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
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_CURRENT, 50, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Current Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        else { printf("Goal current of Dynamixel#%d is set \n", DXL2_ID); }

        // Add parameter storage for present position value
        if(!groupSyncRead.addParam(DXL1_ID)){ cout << DXL1_ID << " groupSyncRead addparam failed\n"; }
        if(!groupSyncRead.addParam(DXL2_ID)){ cout << DXL2_ID << " groupSyncRead addparam failed\n"; }

        cout << endl;
    }
    
    //// Read input .txt file
    cout << "Choose from menu for cable robot motion:\nt - Read from \"bricks.csv\" file for brick positions\nl - Loop through set num of bricks\nm - Manual input using w,a,s,d,r,f,v,g\ni - Info: show menu\nn - Prepare to disable motors and exit programme" << endl;
    do {
        cin >> cmd;
        switch (cmd){
            case 'i':    // Show menu
                cout << "Choose from menu for cable robot motion:\nt - Read from \"bricks.csv\" file for brick positions\nl - Loop through set num of bricks\nm - Manual input using w,a,s,d,r,f,v,g\ni - Info: show menu\nn - Prepare to disable motors and exit programme" << endl;
                break;
            case 't':   // Read brick file, plan trajectory
            case 'T':
                if(!ReadBricksFile()){ continue; } // Read "bricks.csv"
                RunBricksTraj(groupSyncRead, 0, true);
                break;
            case 'm':   // Manual wasdrf
            case 'M':
                cout << "Press 'q' to quit manual input anytime.\n'h' for Homing.\n'x' to adjust increment step size.\n";
                while(cmd != 'q' && cmd != 'Q'){
                    cmd = getch();
                    switch(cmd){
                        case 'I':
                        case 'i':
                            if(packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, gpClose, &dxl_error)){ cout << "Error in closing gripper\n"; }
                            continue;
                        case 'O':
                        case 'o':
                            if(packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; }
                            continue;
                        case 'P':
                        case 'p':
                            cout << "Rotation: ";
                            cin >> rotationG;
                            if(cin.good()){
                                rotationG *= 11.26666;
                                if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, rotationG, &dxl_error)){ cout << "Error in gripper rotation command\n"; };
                            }
                            continue;
                        case 'W':
                        case 'w':
                            in1[1] += step;
                            break;
                        case 'S':
                        case 's':
                            in1[1] -= step;
                            break;
                        case 'A':
                        case 'a':
                            in1[0] -= step;
                            break;
                        case 'D':
                        case 'd':
                            in1[0] += step;
                            break;
                        case 'R':
                        case 'r':
                            in1[2] += step;
                            break;
                        case 'F':
                        case 'f':
                            in1[2] -= step;
                            break;
                        case 'G':
                        case 'g':
                            railOffset += step/20;
                            if(railOffset > RAIL_UP) { cout << "WARNING! Rail offset beyond upper bound!\n"; railOffset -= step/20; }
                            cout << "Current rail offset: " << railOffset << endl;
                            pose_to_length(in1, out1, railOffset);
                            SendMotorGrp(false, true);
                            continue;
                        case 'V':
                        case 'v':
                            railOffset -= step/20;
                            if(railOffset < RAIL_DOWN) { cout << "WARNING! Rail offset beyond lower bound!\n"; railOffset += step/20; }
                            cout << "Current rail offset: " << railOffset << endl;
                            pose_to_length(in1, out1, railOffset);
                            SendMotorGrp(false, true);
                            continue;
                        case 'H':
                        case 'h':
                            cout << "Homing...\n"; 
                            TrjHome();
                            break;
                        case 'X':
                        case 'x':
                            cout << "Current step size: " << step << "m. Please enter new step size: ";
                            cin >> step;
                            if (!cin.good()){ cout << "Invalid input!"; }
                            else if (abs(step) > 0.1){ cout << "Warning! Step size is too large, may cause vigorious motions.";}
                            else { cout << "New step size: " << step << endl; break; }
                            cout << " Step size is now set to 0.01m.\n";
                            step = 0.01;
                            break;
                        case 'C':
                        case 'c':
                            cout << "Are you sure to clear warnings from triggered linear rail limit switch? Type \"CLEAR\" to confirm.\n";
                            {
                                string reply;
                                cin >> reply;
                                if (reply == "CLEAR"){ limitType = 'C'; cout << "Warning dismissed.\n"; }
                                else { cout << "Warning is not cleared.\n"; }
                            }
                            continue;
                    }
                    cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
                    if(CheckLimits()){
                        pose_to_length(in1, out1, railOffset);
                        cout << "OUT: "<<  out1[0] << " " << out1[1] << " " << out1[2] << " " << out1[3] << " " <<  out1[4] << " " << out1[5] << " " << out1[6] << " " << out1[7] << " " <<  out1[8] << " " << out1[9] << " " << out1[10] << " " << out1[11] << endl;
                        SendMotorGrp();
                        
                        Sleep(step*1000);
                    }
                    else{
                        cout << "WARNING: Intended position out of bound!\n";
                        switch(cmd){
                            case 'W':
                            case 'w':
                                in1[1] -= step;
                                break;
                            case 'S':
                            case 's':
                                in1[1] += step;
                                break;
                            case 'A':
                            case 'a':
                                in1[0] += step;
                                break;
                            case 'D':
                            case 'd':
                                in1[0] -= step;
                                break;
                            case 'R':
                            case 'r':
                                in1[2] -= step;
                                break;
                            case 'F':
                            case 'f':
                                in1[2] += step;
                                break;
                        }
                    }
                }
                cout << "Quit manual control\n";
                break;
            case 'l':   // Read brick file, loop through a set no. of bricks
            case 'L':
                int loopNum = 30; // Define the no. of bricks to loop here!!!
                // Read input file for traj-gen
                quitType = 'r';
                if(!ReadBricksFile()){ continue; } // Read "bricks.csv"
                cout << "Bricks to loop: " << loopNum << endl;
                if(brickPos.size()<loopNum){ cout << "Warning! Defined brick file is not long enough for looping.\n"; break; }
                int listOffset = brickPos.size() - loopNum;
                brickPos.erase(brickPos.begin(), brickPos.end()-loopNum); // Only need the last elements
                while(quitType != 'f' && quitType != 'F'){ // if not running the final loop.....
                    // always reverse from a complete built, then rebuild it
                    ReverseBricksTraj(groupSyncRead, listOffset, true);
                    if(quitType == 'q' || quitType == 'Q'){ break; }
                    RunBricksTraj(groupSyncRead, listOffset, true);
                    if(quitType == 'q' || quitType == 'Q'){ break; }
                }
                cout << "Quit looping trajectory.\n";
                break;
        }
    } while(cmd != 'n');

    
    //// Safe system shut down, safe last pos and emegency shut down
    // Saving last position before quiting programme
    cout << "Saving last position...\n";
    ofstream myfile;
    myfile.open ("lastPos.txt");
    myfile << in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
    myfile << railOffset << endl;
    for(INode* n : nodeList){
        n->Motion.PosnMeasured.Refresh();
        myfile << n->Motion.PosnMeasured.Value() << " ";
    }
    myfile.close();
    
    //// List of what-if-s??

    {   // Disable Dynamixel Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }
        Sleep(20);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {cout << "Torque Comm result: " << dxl_comm_result <<endl; }
        else if (dxl_error != 0) { cout << "Error: " << dxl_error << endl; }

        // Close port
        portHandler->closePort();
    }
    cout << "Dynamixel is closed correctly\n";

    for(int i = 0; i < nodeList.size(); i++){ //Disable Nodes
        nodeList[i]->EnableReq(false);
    }
    nodeList.clear();
    myMgr->PortsClose(); // Close down the ports
    cout << "Teknic motors are disabled\n";
    return 1;
}

int CheckMotorNetwork() {
    SysManager* myMgr = SysManager::Instance();

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
            cout << "AccLimit and VelLimit set." << endl;

            nodeList.push_back(&theNode);               // add node to list

            double timeout = myMgr->TimeStampMsec() + 2000; //TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable
            //This will loop checking on the Real time values of the node's Ready status
            while (!theNode.Motion.IsReady()) {
                if (myMgr->TimeStampMsec() > timeout) {
                    printf("Error: Timed out waiting for Node %d to enable\n", iNode);
                    return -2;
                }
            }

            if (i == 2) { // Set the Node's Attention Mask to generate attentions on "Enabled" on every linear rail nodes
                attnReg attnReq;
                attnReq.cpm.InA = 1;
                attnReq.cpm.InB = 1;
                theNode.Adv.Attn.Mask = attnReq;

                theNode.Adv.Attn.ClearAttn(attnReq);
                theNode.EnableReq(true);
            }
        }
        if (i == 2){ // For linear rail port
            myPort.Adv.Attn.Enable(true); // Enable the attentions for this port, ie Port#2 for linear rails
            myPort.Adv.Attn.AttnHandler(AttentionDetected); // Register handler at the port level, just for illustrative purposes.
        }
    }
    return 0;
}

int RaiseRailTo(double target){ // !!! Define velocity limit !!!
    if (target < 0 || target > 1.25) { cout << "WARNING! Intended rail offset is out of bound!\n"; return -2; }

    double velLmt = 0.002; // meters per sec
    double dura = abs(target - railOffset)/velLmt*1000;// *1000 to change unit to ms
    double a, b, c; // coefficients for cubic spline trajectory
    cout << "\nATTENTION: Raise rail function is called. Estimated time: " << dura << "\n";
    
    // Solve coefficients of equations for cubic
    a = railOffset;
    b = 3 / (dura * dura) * (target - railOffset);
    c = -2 / (dura * dura * dura) * (target - railOffset);
    
    // Run the trajectory till the given time is up
    double t = 0;
    auto start = chrono::steady_clock::now();
    long dur = 0;
    while (t <= dura){        
        // CUBIC equation, per time step pose
        railOffset = a + b * t * t + c * t * t * t; // update new rail offset

        // get absolute cable lengths and rail positions in meters
        pose_to_length(in1, out1, railOffset);
        cout << "OUT: "<<  out1[4] << "\t" << out1[5] << "\t" << out1[6] << "\t" << out1[7] << "\t" <<  out1[8] << "\t" << out1[9] << "\t" << out1[10] << "\t" << out1[11] << endl;

        SendMotorGrp(false, true);

        dur = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now()-start).count();
        double dif = MILLIS_TO_NEXT_FRAME - dur - 1;
        if(dif > 0) { Sleep(dif);}
        t += MILLIS_TO_NEXT_FRAME;
        start = chrono::steady_clock::now(); // refresh the ending time in loop

        if(kbhit()){ // Emergency quit during trajectory control
            cout << "\nSystem interrupted!! Do you want to quit the trajectory control?\nq - Quit trajectory\nf - Finish this loop of motion\nr - Resume trajectory\n";
            cin >> quitType;
            if(quitType == 'q' || quitType == 'Q'){
                cout << "Trajectory emergency quit\n";
                return -1;
            }
        }
        if(limitType != 'C'){
            cout << "WARNING! Linear rail limits triggered. Please quit the programme and check the system.\n";
            return -3;
        }
    }
    return 0;
}

int RunParaBlend(double point[7], bool showAttention){
    float vMax[6] = {.4, .4, .4, 0.8, 0.8, 0.8}; // m/s, define the maximum velocity for each DoF
    float aMax[6] = {80, 80, 80, 10, 10, 10}; // m/s^2, define the maximum acceleration for each DoF
    double sQ[6], Q[6], o[6];
    double dura = point[6];
    cout << "Will run for " << dura << "ms...\n";
    
    // Solve parabolic blend coefficients for each DoF
    for(int i = 0; i < 6; i++){
        sQ[i] = in1[i]; // start point, from current position
        Q[i] = point[i]; // end point, from goal position
        vMax[i] /= 1000; // change the velocity unit to meter per ms
        aMax[i] /= 1000000; // change the unit to meter per ms square
        tb[i] = dura - (Q[i] - sQ[i]) / vMax[i];
        if(tb[i] < 0) {
            cout << "WARNING: Intended trajectory exceeds velocity limit in DoF "<< i << ".\n";
            return -1;
        }
        else if (tb[i] > dura / 2){
            if (showAttention){ cout << "ATTENTION: Trajectory for DoF " << i << " will be in cubic form.\n"; }
            tb[i] = dura / 2;
            vMax[i] = 2 * (Q[i] - sQ[i]) / dura;
        }
        o[i] = vMax[i] / 2 / tb[i];
        if(abs(o[i]*2) > aMax[i]){
            cout << "WARNING: Intended trajectory acceleration <" << abs(o[i]*2) << "> exceeds limit in DoF "<< i << ".\n";
            return -1;
        }
        
        // solve coefficients of equations for parabolic
        a[i] = sQ[i];
        b[i] = o[i];

        c[i] = sQ[i] - vMax[i] * tb[i] / 2;
        d[i] = vMax[i];

        e[i] = Q[i] - o[i] * dura * dura;
        f[i] = 2 * o[i] * dura;
        g[i] = -o[i];
    }
    
    // Run the trajectory till the given time is up
    double t = 0;
    while (t <= point[6]){
        auto start = chrono::steady_clock::now();
        long dur = 0;
        
        // PARABOLIC BLEND equation, per time step pose
        for (int j = 0; j < 6; j++){
            if (t <= tb[j]){
                in1[j] = a[j] + b[j] * t * t;
            }
            else if(t <= point[6]-tb[j]){
                in1[j] = c[j] + d[j] * t;
            }
            else{
                in1[j] = e[j] + f[j] * t + g[j] * t * t;
            }
        }
        // get absolute cable lengths in meters
        // cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
        pose_to_length(in1, out1, railOffset);
        // cout << "OUT: "<<  out1[0] << "\t" << out1[1] << "\t" << out1[2] << "\t" << out1[3] << "\t" <<  out1[4] << "\t" << out1[5] << "\t" << out1[6] << "\t" << out1[7] << endl;

        SendMotorGrp();

        auto end = chrono::steady_clock::now();
        dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
        
        double dif = MILLIS_TO_NEXT_FRAME - dur - 1;
        if(dif > 0) { Sleep(dif);}
        t += MILLIS_TO_NEXT_FRAME;

        if(kbhit()){ // Emergency quit during trajectory control
            cout << "\nSystem interrupted!! Do you want to quit the trajectory control?\nq - Quit trajectory\nf - Finish this loop of motion\nr - Resume trajectory\n";
            cin >> quitType;
            if(quitType == 'q' || quitType == 'Q'){
                cout << "Trajectory emergency quit\n";
                return -2;
            }
        }
        if(limitType != 'C'){
            cout << "WARNING! Linear rail limits triggered. Please quit the programme and check the system.\n";
            return -3;
        }
    }
    return 0;
}

void RunBricksTraj(dynamixel::GroupSyncRead groupSyncRead, int listOffset, bool showAttention){
    double brickPickUp[7] = {0.7123, 1.3295, 1.641, 0, 0, 0, 10}; // !!!! Define the brick pick up point !!!!, the last digit is a dummy number for time duration.
    double safePt[3] = {0.8123, 1.3295, 1.661}; // a safe area near to the arm
    double goalPos[7] = {2, 2, 1, 0, 0, 0, 10}; // updated according to brick position
    double velLmt = 0.22; // meters per second
    double safeT = 1500; // in ms, time to raise to safety height
    double safeH = 0.06; // meter, safety height from building brick level
    double currentBrkLvl = railOffset; // meter, check if the rail offset is the same as target BrkLvl
    double dura = 0;
    
    // Go through the given bricks
    for (int i = 0; i < brickPos.size(); i++) {
        // Check if rails need to be raised
        if(brickPos[i][2] - 0.04 != currentBrkLvl){
            currentBrkLvl = brickPos[i][2] - 0.04; // Offset one brick height from building levei, ie 0.04m
            if(RaiseRailTo(currentBrkLvl) < 0) { cout << "Trajectory aborted.\n"; return; } // raise rail to the building brick level
        }

        // Update index file for grasshopper display
        ofstream myfile;
        myfile.open ("indexBrk.txt");
        myfile << i + 1 + listOffset;
        myfile.close();

        // Go pick up a brick
        if(showAttention) { cout << "Picking up a brick\n"; }
        if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, neutralRot, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        Sleep(10); // short break between RS-485 communication
        if(packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        brickPickUp[6] = sqrt(pow(brickPickUp[0]-in1[0],2)+pow(brickPickUp[1]-in1[1],2)+pow(brickPickUp[2]-in1[2],2))/velLmt*1000; // calculate time
        if(RunParaBlend(brickPickUp) < 0) { cout << "Trajectory aborted.\n"; return; }
        Sleep(2500); //////////// FOR TESTING ONYL, delete later!!!!!!!!!!!!!!!!!!
        if(packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, gpClose, &dxl_error)){ cout << "Error in closing gripper\n"; return; }
        do{ // wait till gripper is closed
            if(dxl_comm_result = groupSyncRead.txRxPacket()){ cout << "Comm error in reading packet: " << dxl_comm_result << endl; } 
            // Get present position value
            dxl1Pos = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            dxl2Pos = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            printf("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\n", DXL1_ID, dxl1Pos, DXL2_ID, dxl2Pos);
        }while((abs(neutralRot - dxl1Pos) > DXL_THRESHOLD) || (abs(gpClose - dxl2Pos) > DXL_THRESHOLD));
        Sleep(500);

        // Go to building level
        if(showAttention) { cout << "Going to building level\n"; }
        copy(brickPickUp, brickPickUp+3, begin(goalPos));
        goalPos[2] += 0.16;
        goalPos[6] = safeT;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // raise the brick from robot arm
        Sleep(1500); // Wait for ABB to leave
        rotationG = brickPos[i][3] * 11.26666;; // conversion from angle to motor command
        if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, rotationG, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        copy(safePt, safePt+2, begin(goalPos)); // safe x,y position
        goalPos[2] = brickPos[i][2] + endEffOffset + safeH; // brick level with safe height
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2)+pow(goalPos[2]-in1[2],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // raise the brick to building level

        // Go to brick placing position
        if(showAttention) { cout << "Going to brick position\n"; }
        goalPos[0] = brickPos[i][0];                       
        goalPos[1] = brickPos[i][1];
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }

        // Place brick
        if(showAttention) { cout << "Placing brick\n"; }
        goalPos[2] -= safeH;
        goalPos[6] = safeT;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        if(packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        Sleep(2000); //Wait a while after placing brick
        
        // Rise and leave building area, stand by for next brick pick up
        if(showAttention) { cout << "Going to stand by position\n"; }
        goalPos[2] += safeH;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // leave building level
        if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, neutralRot, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        copy(begin(safePt), end(safePt), begin(goalPos)); // safe x,y position
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2)+pow(goalPos[2]-in1[2],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // return to safe point 
        
        cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
        cout << "----------Completed brick #" << i + 1 + listOffset <<"----------" << endl;
    }
}

void ReverseBricksTraj(dynamixel::GroupSyncRead groupSyncRead, int listOffset, bool showAttention){
    double brickDropOff[7] = {0.717, 2.14, 1.88, 0, 0, 0, 10}; // !!!! Define the brick drop off point !!!!, the last digit is a dummy number for time duration.
    double safePt[3] = {0.988, 2.14, 1.9}; // a safe area near the drop off
    double goalPos[7] = {2, 2, 1, 0, 0, 0, 10}; // updated according to brick position
    double velLmt = 0.18; // meters per second
    double safeT = 1500; // in ms, time to raise to safety height
    double safeH = 0.06; // meter, safety height from building brick level
    double currentBrkLvl = railOffset; // meter, check if the rail offset is the same as target BrkLvl
    double dura = 0;
    
    // Go through the given bricks
    for (int i = brickPos.size()-1; i > -1; i--) {
        // Check if rails need to be raised
        if(brickPos[i][2] - 0.04 != currentBrkLvl){
            currentBrkLvl = brickPos[i][2] - 0.04; // Offset one brick height from building levei, ie 0.04m
            if(RaiseRailTo(currentBrkLvl) < 0) { cout << "Trajectory aborted.\n"; return; } // raise rail to the building brick level
        }

        // Update index file for grasshopper display
        ofstream myfile;
        myfile.open ("indexBrk.txt");
        myfile << i + 1 + listOffset;
        myfile.close();

        // Go to building level
        if(showAttention) { cout << "Going to building level\n"; }
        if(packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        copy(in1, in1+2, begin(goalPos));
        goalPos[2] = brickPos[i][2] + endEffOffset + safeH; // brick level
        goalPos[6] = sqrt(pow(goalPos[2]-in1[2],2))/velLmt*1000; // calculate time
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // raise from current pos to builing level
        
        // Go to brick placing position
        if(showAttention) { cout << "Going to brick position\n"; }
        goalPos[0] = brickPos[i][0];                       
        goalPos[1] = brickPos[i][1];
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        rotationG = brickPos[i][3] * 11.26666;; // conversion from angle to motor command
        if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, rotationG, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        
        // Retrieve brick
        if(showAttention) { cout << "Retrieving brick\n"; }
        goalPos[2] -= safeH;
        goalPos[6] = safeT;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; }
        if(packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, gpClose, &dxl_error)){ cout << "Error in closing gripper\n"; return; }
        do{ // wait till gripper is closed
            if(dxl_comm_result = groupSyncRead.txRxPacket()){ cout << "Comm error in reading packet: " << dxl_comm_result << endl; } 
            // Get present position value
            dxl1Pos = groupSyncRead.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            dxl2Pos = groupSyncRead.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            printf("[ID:%03d] PresPos:%03d\t[ID:%03d] PresPos:%03d\n", DXL1_ID, dxl1Pos, DXL2_ID, dxl2Pos);
        }while((abs(gpClose - dxl2Pos) > DXL_THRESHOLD));

        // Rise and leave building area, stand by for next brick pick up
        if(showAttention) { cout << "Going to drop off position\n"; }
        goalPos[2] += safeH;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // leave building level
        if(packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, neutralRot, &dxl_error)){ cout << "Error in rotating gripper\n"; return; }
        copy(begin(safePt), end(safePt), begin(goalPos)); // safe x,y position
        goalPos[6] = sqrt(pow(goalPos[0]-in1[0],2)+pow(goalPos[1]-in1[1],2)+pow(goalPos[2]-in1[2],2))/velLmt*1000;
        if(RunParaBlend(goalPos) < 0) { cout << "Trajectory aborted.\n"; return; } // return to safe point 
        
        // Dropping off a brick
        if(showAttention) { cout << "Dropping off the brick\n"; }
        brickDropOff[6] = sqrt(pow(brickDropOff[0]-in1[0],2)+pow(brickDropOff[1]-in1[1],2)+pow(brickDropOff[2]-in1[2],2))/velLmt*1000; // calculate time
        if(RunParaBlend(brickDropOff) < 0) { cout << "Trajectory aborted.\n"; return; }
        if(packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, gpOpen, &dxl_error)){ cout << "Error in opening gripper\n"; return; }
        Sleep(1000); // wait a little after dropping brick
        
        cout << "----------Retrieved brick #" << i + 1 + listOffset <<"----------" << endl;
    }
}

int32_t ToMotorCmd(int motorID, double length){ // applicable for all 12 motors
    double scale = 509283.772; //509295.818; // 6400 encoder count per revoltion, 25 times gearbox, 50mm spool radias. ie 6400*25/(2*pi*0.05) 
    if(motorID >= NodeNum) {
        scale = 38400000; // 38400000; // 6400 encoder count per revoltion, 30 times gearbox, linear rail pitch 5mm. ie 6400*30/0.005 
        return length * scale;
    }
    else if(motorID == -1) { return length * scale; }
    return (length - offset[motorID]) * scale;
}

void SendMotorCmd(int n){
    // convert to absolute cable length command
    try{
        int32_t step = ToMotorCmd(n, out1[n]);
        nodeList[n]->Motion.MoveWentDone();
        nodeList[n]->Motion.MovePosnStart(step, true, true); // absolute position
        nodeList[n]->Motion.Adv.TriggerGroup(1);
    }
    catch(sFnd::mnErr& theErr) {    //This catch statement will intercept any error from the Class library
        cout << "\nERROR: Motor [" << n << "] command failed.\n";  
        printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);    
    }
}

void SendMotorTrq(int n){
    nodeList[n]->Motion.TrqCommanded.Refresh();
    float currentTorque = nodeList[n]->Motion.TrqCommanded.Value();

    if(currentTorque > targetTorque){ nodeList[n]->Motion.MoveVelStart(-300); }
    else if (currentTorque < targetTorque - 1.8){ nodeList[n]->Motion.MoveVelStart(150); cout << "Too much torque!!\n";}
    else{ nodeList[n]->Motion.MoveVelStart(-10);}
    printf("Node[%d], current torque: %f\n", n, currentTorque);
}

void SendMotorGrp(bool IsTorque, bool IsLinearRail){
    SysManager* myMgr = SysManager::Instance();
    IPort &myPort = myMgr->Ports(0);
    void (*func)(int){ SendMotorCmd };
    if(IsTorque){ func = SendMotorTrq; }
    int n = IsLinearRail? 4 : 0; // offset in nodeList
    
    thread nodeThreads[NodeNum];
    for(int i = 0; i < NodeNum; i++){
        nodeThreads[i] = thread((*func), i + n); 
    }
    for(int i = 0; i < NodeNum; i++){
        nodeThreads[i].join();
    }
    myPort.Adv.TriggerMovesInGroup(1);
}

void TrjHome(){// !!! Define the task space velocity limit for homing !!!
    double velLmt = 0.05; // unit in meters per sec
    double dura = sqrt(pow(in1[0]-home[0],2)+pow(in1[1]-home[1],2)+pow(in1[2]-home[2],2))/velLmt*1000; // *1000 to change unit to ms
    double t = 0;
    cout << "Expected homing duration: " << dura <<"ms\n";
    if (dura == 0){ return; }

    for(int i = 0; i < 6; i++){
        // solve coefficients of equations for cubic
        a[i] = in1[i];
        b[i] = 3 / (dura * dura) * (home[i] - in1[i]);
        c[i] = -2 / (dura * dura * dura) * (home[i] - in1[i]);
    }
    while (t <= dura){
        auto start = chrono::steady_clock::now();
        long dur = 0;
        
        // CUBIC equation
        for (int j = 0; j < 6; j++){
            in1[j] = a[j] + b[j] * t * t + c[j] * t * t * t;
        }
        cout << "IN: "<< in1[0] << " " << in1[1] << " " << in1[2] << " " << in1[3] << " " << in1[4] << " " << in1[5] << endl;
        pose_to_length(in1, out1, railOffset);
        // cout << "OUT: "<<  out1[0] << " " << out1[1] << " " << out1[2] << " " << out1[3] << endl;
        
        SendMotorGrp();
        
        auto end = chrono::steady_clock::now();
        dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
        cout << " Before sleep: " << dur << endl;
        
        double dif = MILLIS_TO_NEXT_FRAME - dur;
        if(dif > 0) { Sleep(dif); }
        // Sleep(MILLIS_TO_NEXT_FRAME);

        end = chrono::steady_clock::now();
        dur = chrono::duration_cast<chrono::milliseconds>(end-start).count();
        cout << " Time elasped: " << dur << "\tTime left: " << dura - t << endl;
        t += MILLIS_TO_NEXT_FRAME;
        if(kbhit()){ // Emergency quit during trajectory control
            cout << "\nSystem interrupted!! Do you want to quit the trajectory control?\nq - Quit trajectory\nr - Resume trajectory\n";
            char cmd;
            cin >> cmd;
            if(cmd == 'q' || cmd == 'Q'){
                cout << "Trajectory control aborted.\n";
                t = dura;
                break;
            }
        }
        if(limitType != 'C'){
            cout << "WARNING! Linear rail limits triggered. Please quit the programme and check the system.\n";
            return;
        }
    }
    cout << "Homing with trajectory completed\n";
}

bool CheckLimits(){
    if(spcLimit.empty()){
        ifstream file ("limit.csv"); // Read limit file
        string temp;
        if(file.is_open()){
            while (file >> temp){
                spcLimit.push_back(stod(temp)); // convert string to double stod()
            }
            cout << "Completed reading external limit file" << endl;
        }
    }
    for (int i = 0; i < 3; i++){
        if(spcLimit[i*2]>in1[i] || in1[i]>spcLimit[i*2+1]){ return false; }
    }
    return true;
}

bool ReadBricksFile(){ // Define which file to read here !!!
    ifstream file ("bricks.csv");
    vector<double> row;
    string line, word, temp;

    brickPos.clear();
    if(file.is_open()){
        while (getline(file, line)){
            row.clear();
            stringstream s(line);
            while (s >> word){
                row.push_back(stod(word)); // convert string to double stod()
            }
            for(int i=0; i<3; i++){ row[i] /= 1000; } // convert mm to m unit for xyz
            row[3] += 90; // convert Adam's file to robot rotation, 90deg offset
            if(row[3]<0){ row[3] += 180; } // convert -ve rotation to +ve
            brickPos.push_back(row);
        }
        cout << "Completed reading brick position input file" << endl;
    }
    else{ cout << "Failed to read input file. Please check \"bricks.csv\" file." << endl; return false;
    }
    return true;
}

void HomeLinearRail(int n){
    double velLmt = -2000; // IMPORTANT!!!!! Set the linear homing speed here!!
    double hLmtOffset[4] = {-34000, -34000, -34000, -34000}; // Set offset from home switch to real "home" in motor counts units

    SysManager* myMgr = SysManager::Instance();
    INode &theNode = myMgr->Ports(2).Nodes(n);
    theNode.Motion.VelocityReachedTarget();
    theNode.Motion.MoveVelStart(velLmt);
            
    while(!kbhit()){
        switch(limitType){
        case 'A':
            cout << "InA detected!\n";
            theNode.Motion.MoveVelStart(0);
            while(!theNode.Motion.VelocityAtTarget()){} // wait till motor stopped
            theNode.Motion.AddToPosition(-theNode.Motion.PosnMeasured.Value()+hLmtOffset[n]); // Add offset here if applicable
            cout << "Reached homed swtich.\n";
            theNode.Motion.MovePosnStart(0, true);
            while(!theNode.Motion.MoveIsDone()){}
            cout << "Homing completed.\n";
            limitType = 'C';
            return;
        case 'B':
            cout << "InB detected!\n";
            velLmt *= -1;
            theNode.Motion.MoveVelStart(velLmt);
            if (velLmt > 0) { // ie moving upwards
                while(limitType != 'A') {} //wait till home switch is passed
                Sleep(2000);
                velLmt *= -1;
                theNode.Motion.MoveVelStart(velLmt);
                limitType = 'C'; // wait till the home is hit from upward side
                break;
            } // else, just break and wait till homed
            break;
        }
    }
    theNode.Motion.MoveVelStart(0);
    cout << "Linear rail motion interrupted\n";
}

void MN_DECL AttentionDetected(const mnAttnReqReg &detected)
{
    // Make a local, non-const copy for printing purposes
    mnAttnReqReg myAttns = detected;
    // Load the buffer with the string representation of the attention information
    myAttns.AttentionReg.StateStr(attnStringBuf, 512);
    // Print it out to the console
    printf("ATTENTION: port %d, node=%d, attn=%s\n", detected.MultiAddr >> 4, detected.MultiAddr, attnStringBuf);

    limitType = attnStringBuf[strlen(attnStringBuf)-2];
}