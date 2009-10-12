#include "localplanner.hpp"

LocalPlanner::LocalPlanner()
{

}

void LocalPlanner::getMovementCommand ( double& tv, double& rv )
{
    const double stopAndTurnAngle = 30.0 / 180.0 * M_PI;
    const double tvP = 0.5;
    const double rvP = 0.3;
    
    //calculate vector to target position
    Eigen::Vector3d driveVector = targetPosition - position;
    
    tv = tvP * driveVector.norm();
    
    //calculate oriintation difference to wanted orientation
    //orientation  * orientation.inv() == identity;
    //identity * targetOrientation == targetOrientation;
    //orientation * orientation.inv() * targetOrientation == targetOrientation
    //so orientation.inv() * targetOrientation is the difference from orientation to targetOrientation
    Eigen::Quaterniond driveOrientation = orientation.inverse() * targetOrientation;
    
    
    
    //convert target position into robot coordinate system
    Eigen::Vector3d targetPosRobot = orientation * targetPosition;
    
    //calculate direction vector
    Eigen::Vector3d directionRobot = orientation * Eigen::Vector3d(0,1.0,1);
    
    //calulate angle to target position
    double angleToTarget = atan2(directionRobot.x() - targetPosRobot.x(), directionRobot.y() - targetPosRobot.y());
    
    
    
    //lame implementation for drive behaviour
    //TODO perhaps add a nice polynom or something
    
    //if missalignment of the robot is too big stop an point turn
    if(fabs(angleToTarget) > stopAndTurnAngle) {
	tv = 0;
    }
    
    rv = rvP * angleToTarget;
    
    
    
}

void LocalPlanner::setPose(Eigen::Vector3d p, Eigen::Quaterniond o)
{
    position = p;
    orientation =o;
}

void LocalPlanner::setTargetPose(Eigen::Vector3d p, Eigen::Quaterniond o) 
{
    targetPosition = p;
    targetOrientation = o;
}


