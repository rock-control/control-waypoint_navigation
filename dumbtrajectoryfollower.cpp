#include "dumbtrajectoryfollower.hpp"
#include <iostream>

DumbTrajectoryFollower::DumbTrajectoryFollower()
{
    stopAndTurnAngle = 30.0 / 180.0 * M_PI;
    tvP = 1.0;
    rvP = 1.0;
    maxDisalignment = 10.0 / 180.0 * M_PI;
    aligning = false;
    targetSet = true;
    poseSet = true;
    reachedHysteresisRatio = 2.0;
}   

//lame implementation for drive behaviour
//TODO perhaps add a nice polynom or something
void DumbTrajectoryFollower::getMovementCommand ( double& tv, double& rv )
{
    
    //check if std deviation is bigger than the specified std deviation for the current target pose
    for (int i = 0; i<3; i++) {
	//not no sqrt, as both values are sqared, and > is vallid in this case
	if(curPose.covariancePosition(i,i) > targetPose.covariancePosition(i,i)) {
	    std::cout << "Variance of " << i << " is to high " << curPose.covariancePosition(i,i) << " should be smaller than " << targetPose.covariancePosition(i,i) << std::endl; 
	    tv = 0;
	    rv = 0;
	    return;
	}
    }
    
    if(!targetSet || !poseSet) {
	std::cout << "No target or pose specified" << std::endl;
	tv = 0;
	rv = 0;
	return;
    }
    
    std::cout << "Target Position in World Coordinates : X: " << targetPose.position.x() << " Y: " << targetPose.position.y() << " Z: " << targetPose.position.z() << std::endl;
    std::cout << "Own Position in World Coordinates    : X: " << curPose.position.x() << " Y: " << curPose.position.y() << " Z: " << curPose.position.z() << std::endl;

    //calculate vector to target position
    Eigen::Vector3d driveVector = targetPose.position - curPose.position;
    double distToTarget = driveVector.norm();
    
    tv = tvP * distToTarget;

    bool reachedInnerPoint = true;
    bool reachedOuterPoint = true;
    //check if we reached target position in respect, to covariance of target position
    for(int i = 0; i< 3; i++){
	double dist = fabs(curPose.position[i] - targetPose.position[i]);
	double reached_dist = sqrt(targetPose.covariancePosition(i,i));
	if(dist > reached_dist / reachedHysteresisRatio)
	    reachedInnerPoint = false;
	if(dist > reached_dist)
	    reachedOuterPoint = false;
    }

    //start aligning if we reached inner point
    if(reachedInnerPoint)
	aligning = true;
    
    //stop aligning, if we left variance of target point
    if(!reachedOuterPoint)
	aligning = false;
    
    Eigen::Vector3d targetPosRobot(0,0,0);
     
    if(!aligning) {
	std::cout << "Not aligning" << std::endl;
	//convert target position into robot coordinate system
	targetPosRobot = curPose.orientation.inverse() * (targetPose.position - curPose.position);
	std::cout << "Target Position in Robot Coordinates : X: " << targetPosRobot.x() << " Y: " << targetPosRobot.y() << " Z: " << targetPosRobot.z() << std::endl;;

    } else {
	//fit orientation of the robot to the wanted orientation

	//calculate oriintation difference to wanted orientation
	//orientation  * orientation.inv() == identity;
	//identity * targetOrientation == targetOrientation;
	//orientation * orientation.inv() * targetOrientation == targetOrientation
	//so orientation.inv() * targetOrientation is the difference from orientation to targetOrientation
	Eigen::Quaterniond driveOrientation = curPose.orientation.inverse() * targetPose.orientation;

	//rotate forward vector about the difference of both orienatations
	//and feed it to the "normal" drive function
	targetPosRobot = driveOrientation * Eigen::Vector3d(0,1.0, 0);
	
	//just rotate, not translate
	tv = 0;
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

void DumbTrajectoryFollower::setPose(DumbTrajectoryFollower::Pose& pose)
{
    std::cout << "DTF: Got new pose " << pose.position;
    poseSet = true;
    curPose = pose;
}

void DumbTrajectoryFollower::setTargetPose(DumbTrajectoryFollower::Pose& pose) 
{
    std::cout << "DTF: Got new target pose " << pose.position;
    targetSet = true;
    aligning = false;
    targetPose = pose;
}


void DumbTrajectoryFollower::testSetNextWaypoint()
{
    if(trajectory.empty()) 
    {
	std::cout << "Trajectory is empty" << std::endl;
	return;
    }
    
    while(waypointReached(**currentWaypoint)) {
	std::cout << "TARGET REACHED " << std::endl;
	std::vector<Pose *>::iterator nextWp = currentWaypoint;
	nextWp++;

	if(nextWp != trajectory.end()) 
	{
	    currentWaypoint++;
	    setTargetPose(**currentWaypoint);
	} else {
	    break;
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
	setTargetPose(**currentWaypoint);
    }
}


bool DumbTrajectoryFollower::waypointReached(DumbTrajectoryFollower::Pose& target) const
{
    //check if we reached target position in respect, to covariance of target position
    for(int i = 0; i< 3; i++){
	double dist = fabs(curPose.position[i] - target.position[i]);
	double reached_dist = sqrt(target.covariancePosition(i,i));
	std::cout << "Dist " << i << " is " << dist << " reached dist is " << reached_dist << std::endl;
	if(dist > reached_dist)
	    return false;
    }
	
    std::cout << "Waypoint reached " << std::endl;
    //return true;
	
    Eigen::Quaterniond oriDiffToTarget = curPose.orientation.inverse() * targetPose.orientation;
    
    Eigen::Vector3d t = oriDiffToTarget * Eigen::Vector3d(1.0,0,0);
    
    double anglediff = atan2(t.y(), t.x());
    
    if(fabs(anglediff) < maxDisalignment)
	return true;
    
    return false;
}


