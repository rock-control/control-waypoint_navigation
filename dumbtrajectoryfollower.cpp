#include "dumbtrajectoryfollower.hpp"
#include <iostream>

DumbTrajectoryFollower::DumbTrajectoryFollower()
{
    stopAndTurnAngle = 30.0 / 180.0 * M_PI;
    tvP = 0.5;
    rvP = 0.3;
    wayPointReachedDistance = 0.1;
    wayPointLeftDistance = 0.2;
    aligning = false;
	
}   

//lame implementation for drive behaviour
//TODO perhaps add a nice polynom or something
void DumbTrajectoryFollower::getMovementCommand ( double& tv, double& rv )
{
    std::cout << "Target Position in World Coordinates :" << targetPosition << std::endl;

    //calculate vector to target position
    Eigen::Vector3d driveVector = targetPosition - position;
    
    tv = tvP * driveVector.norm();

    if(tv < wayPointReachedDistance) {
	aligning = true;
    }

    if(tv > wayPointLeftDistance) {
	aligning = false;
    }
    
    Eigen::Vector3d targetPosRobot(0,0,0);
    
    if(!aligning) {
	//convert target position into robot coordinate system
	targetPosRobot = orientation * targetPosition;
	std::cout << "Target Position in Robot Coordinates :" << targetPosRobot << std::endl;

    } else {
	//fit orientation of the robot to the wanted orientation

	//calculate oriintation difference to wanted orientation
	//orientation  * orientation.inv() == identity;
	//identity * targetOrientation == targetOrientation;
	//orientation * orientation.inv() * targetOrientation == targetOrientation
	//so orientation.inv() * targetOrientation is the difference from orientation to targetOrientation
	Eigen::Quaterniond driveOrientation = orientation.inverse() * targetOrientation;

	//rotate forward vector about the difference of both orienatations
	//and feed it to the "normal" drive function
	targetPosRobot = driveOrientation * Eigen::Vector3d(0,1.0, 0);
    }
	
    double angleToTarget = 0;
    
    //calulate angle to target position
    if(targetPosRobot.y() == 0 && targetPosRobot.x() == 0) {
	//special case, we are alligned
	angleToTarget = 0;
    } else {
	//robot is allways alligned to (0 1 0) therefor angle is atan - PI/2
	angleToTarget = atan2(targetPosRobot.y(), targetPosRobot.x()) - M_PI/2.0;
    }
    std::cout << "Angle to Target Position :" << angleToTarget << " " << angleToTarget + M_PI/2.0 << std::endl;

    //if missalignment of the robot is too big stop an point turn
    if(fabs(angleToTarget) > stopAndTurnAngle || aligning) {
	tv = 0;
    }
    
    rv = rvP * angleToTarget;
}

void DumbTrajectoryFollower::setPose(Eigen::Vector3d p, Eigen::Quaterniond o)
{
    aligning = false;
    position = p;
    orientation =o;
}

void DumbTrajectoryFollower::setTargetPose(Eigen::Vector3d p, Eigen::Quaterniond o) 
{
    aligning = false;
    targetPosition = p;
    targetOrientation = o;
}


