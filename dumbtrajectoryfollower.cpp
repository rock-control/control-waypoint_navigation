#include "dumbtrajectoryfollower.hpp"
#include <iostream>

DumbTrajectoryFollower::DumbTrajectoryFollower()
{
    stopAndTurnAngle = 30.0 / 180.0 * M_PI;
    tvP = 1.0;
    rvP = 1.0;
    wayPointReachedDistance = 0.1;
    wayPointLeftDistance = 0.2;
    maxDisalignment = 10.0 / 180.0 * M_PI;
    aligning = false;
    targetSet = true;
    poseSet = true;
}   

//lame implementation for drive behaviour
//TODO perhaps add a nice polynom or something
void DumbTrajectoryFollower::getMovementCommand ( double& tv, double& rv )
{
    if(!targetSet || !poseSet) {
	std::cout << "No target or pose specified" << std::endl;
	tv = 0;
	rv = 0;
	return;
    }
    
    std::cout << "Target Position in World Coordinates : X: " << targetPosition.x() << " Y: " << targetPosition.y() << " Z: " << targetPosition.z() << std::endl;
    std::cout << "Own Position in World Coordinates    : X: " << position.x() << " Y: " << position.y() << " Z: " << position.z() << std::endl;

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
	std::cout << "Not aligning" << std::endl;
	//convert target position into robot coordinate system
	targetPosRobot = orientation.inverse() * (targetPosition - position);
	std::cout << "Target Position in Robot Coordinates : X: " << targetPosRobot.x() << " Y: " << targetPosRobot.y() << " Z: " << targetPosRobot.z() << std::endl;;

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
	angleToTarget = atan2(-targetPosRobot.x(), targetPosRobot.y());
    }
    std::cout << "Angle to Target Position :" << angleToTarget << std::endl;

    //if missalignment of the robot is too big stop an point turn
    if(fabs(angleToTarget) > stopAndTurnAngle || aligning) {
	tv = 0;
    }
    
    rv = rvP * angleToTarget;
    std::cout << "RV intern :" << rv << std::endl;

}

void DumbTrajectoryFollower::setPose(Eigen::Vector3d p, Eigen::Quaterniond o)
{
    std::cout << "DTF: Got new pose " << p;
    poseSet = true;
    position = p;
    orientation =o;
}

void DumbTrajectoryFollower::setTargetPose(Eigen::Vector3d p, Eigen::Quaterniond o) 
{
    std::cout << "DTF: Got new target pose " << p;
    targetSet = true;
    aligning = false;
    targetPosition = p;
    targetOrientation = o;
}


void DumbTrajectoryFollower::testSetNextWaypoint()
{
    if(trajectory.empty()) 
    {
	std::cout << "Trajectory is empty" << std::endl;
	return;
    }
    
    if(waypointReached((*currentWaypoint)->position, (*currentWaypoint)->orientation)) {
	std::cout << "TARGET REACHED " << std::endl;
	std::vector<Pose *>::iterator nextWp = currentWaypoint;
	nextWp++;

	if(nextWp != trajectory.end()) 
	{
	    currentWaypoint++;
	    setTargetPose((*currentWaypoint)->position, (*currentWaypoint)->orientation);
	}
    }
    
}


void DumbTrajectoryFollower::setTrajectory(std::vector< DumbTrajectoryFollower::Pose *>& t )
{
    targetSet = false;
    for(std::vector< DumbTrajectoryFollower::Pose *>::iterator it = trajectory.begin(); it != trajectory.end(); it++) 
    {
	delete *it;
    }
    trajectory.clear();
    trajectory = t;
    currentWaypoint = trajectory.begin();
    
    if(!trajectory.empty()) {
	setTargetPose((*currentWaypoint)->position, (*currentWaypoint)->orientation);
    }
}


bool DumbTrajectoryFollower::waypointReached(Eigen::Vector3d& target, Eigen::Quaterniond& targetOrientation) const
{
    
    if((position - target).norm() > wayPointReachedDistance)
	return false;
	
    Eigen::Quaterniond oriDiffToTarget = orientation.inverse() * targetOrientation;
    
    Eigen::Vector3d t = oriDiffToTarget * Eigen::Vector3d(1.0,0,0);
    
    double anglediff = atan2(t.y(), t.x());
    
    if(fabs(anglediff) < maxDisalignment)
	return true;
    
    return false;
}


