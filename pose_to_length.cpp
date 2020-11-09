#include "pose_to_length.h"
#include "Dependencies\eigen-3.3.7\Eigen\Dense"
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;

void pose_to_length(double pose[], double lengths[], double rail_offset){
////////////////////////////////// Define the cable robot parameters here !! //////////////////////////////////
    const int CABLE_NUM = 8;
    const int direction[8] = {1,1,1,1,-1,-1,-1,-1};

    Vector3d frmOut[CABLE_NUM]; // coordinates of the attachment points on frame at the beginning
    frmOut[0] << 0.098, -0.217, 2.443;
    frmOut[1] << 3.572, -0.205, 2.439;
    frmOut[2] << 3.568, 3.457, 2.443;
    frmOut[3] << 0.097,-3.450, 2.446;
    frmOut[4] << 0.00, -0.033, -0.050 + rail_offset; //motors connected to linear rails, coordinates when rail offset is 0
    frmOut[5] << 3.662, -0.033, -0.053 + rail_offset;
    frmOut[6] << 3.678, 3.278, -0.057 + rail_offset;
    frmOut[7] << 0.018, 3.282, -0.053 + rail_offset;

    Vector3d frmOutUnitV[CABLE_NUM]; // unit vectors/directions of the fixed cable attachments on frame
    for(int i = 0; i < CABLE_NUM; i++){ frmOutUnitV[i] << 0, 0, direction[i]; }

    Vector3d endOut[CABLE_NUM]; // local coordinates of cable attachment points on end-effector, ie ^er_B
    endOut[0] << -0.08487, -0.088698, -0.0785;
    endOut[1] << 0.08487, -0.088698, -0.0785;
    endOut[2] << 0.08487, 0.088698, -0.0785;
    endOut[3] << -0.08487, 0.088698, -0.0785;
    endOut[4] << -0.115081, -0.124702, 0.0785;
    endOut[5] << 0.115081, -0.124702, 0.0785;
    endOut[6] << 0.115081, 0.124702, 0.0785;
    endOut[7] << -0.115081, 0.124702, 0.0785;

    const double pRadius = 0.024175; // radius of rotating pulley on frame //0.025

////////////////////////////////// End of manual model defination !! //////////////////////////////////    
    // local variables
    Vector3d orB[CABLE_NUM]; // vector from frame 0 origin to end effector cable outlet
    Vector3d orA[CABLE_NUM]; // vector from frame 0 origin to cable outlet point on rotating pulley
    Matrix3d oe_R; // rotation matrix from end effector to frame 0
    Matrix3d Ra, Rb, Rc;
    double a = pose[3], b = pose[4], c = pose[5];
    
    Ra << 1, 0, 0,
          0, cos(a), -sin(a),
          0, sin(a), cos(a);
    Rb << cos(b), 0,  sin(b),
          0, 1, 0,
          -sin(b), 0, cos(b);
    Rc << cos(c), -sin(c), 0, 
          sin(c), cos(c), 0, 
          0, 0, 1;
    oe_R = Ra * Rb * Rc; // Rotation matrix from given pose rotation
    Vector3d endEfr(pose[0], pose[1], pose[2]); // Translation of end-effector
    for(int i = 0; i < CABLE_NUM; i++){
        ///// Calculate orB[8] vectors /////
        orB[i] = oe_R*endOut[i] + endEfr;
                
        ///// Calculate cable outlet point on rotating pulley, ie point A /////
        Vector3d plNormal = frmOutUnitV[i].cross(orB[i] - frmOut[i]); // normal vector of the plane that the rotating pulley is in
        Vector3d VecC = plNormal.cross(frmOutUnitV[i]); // direction from fixed point towards pulley center
        Vector3d orC = VecC/VecC.norm()*pRadius + frmOut[i]; // vector from frame 0 origin to rotating pulley center
        double triR = pRadius / (orB[i] - orC).norm(); // triangle ratio??
        orA[i] = orC + triR*triR*(orB[i] - orC) - triR*sqrt(1 - triR*triR)*((plNormal/plNormal.norm()).cross(orB[i] - orC));
        
        ///// Calculate arc length /////
        Vector3d UVecCF = -VecC / VecC.norm(); // unit vector of CF
        Vector3d UVecCA = (orA[i]- orC) / (orA[i]- orC).norm(); // unit vector of CA
        double l_arc = pRadius * acos(UVecCF.dot(UVecCA));

        ///// Sum the total cable length /////
        lengths[i] = i < 4 ? l_arc + (orA[i] - orB[i]).norm() : l_arc + (orA[i] - orB[i]).norm() - rail_offset; // Need to subtract the rail offset for the bottom 4 motors 
    }
    lengths[CABLE_NUM] = lengths[CABLE_NUM+1] = lengths[CABLE_NUM+2] = lengths[CABLE_NUM+3] = rail_offset;
}