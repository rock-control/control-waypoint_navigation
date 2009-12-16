#ifndef DUMBTRAJECTORYFOLLOWER_HPP
#define DUMBTRAJECTORYFOLLOWER_HPP

#define EIGEN_DONT_ALIGN
#include <Eigen/Geometry>
#include <vector>
class DumbTrajectoryFollower
{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	DumbTrajectoryFollower();
	
	class Pose {
	    public:
		Pose() : position(Eigen::Vector3d(0,0,0)), orientation(Eigen::Quaterniond::Identity()), covariancePosition(Eigen::Matrix3d::Identity()), covarianceOrientation(Eigen::Matrix3d::Identity()) {};
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Vector3d position;
	     	Eigen::Quaterniond orientation;
		Eigen::Matrix3d covariancePosition;
		Eigen::Matrix3d covarianceOrientation;
	};
	
	/**
	* set positon and orientation, where to drive to
	*/
	void setTargetPose(Pose &pose);
	
	/**
	* Set current orientation and position
	*/
	void setPose(Pose &pose);
	
	/**
	* calculates a translational and rotational velocity
	* that should drive the robot from the current
	* pose to the target pose
	*/
	void getMovementCommand(double &tv, double &rv);
	
	/**
	* sets the trajecory the robot should follow
	*/
	void setTrajectory(std::vector<DumbTrajectoryFollower::Pose *> &t);
	
	/**
	* tests if the current Waypoint was reached and switches to
	* the next waypoint in the trajectory
	*/
	void testSetNextWaypoint();
	
    private:
	/**
	* Calculates if the given waypoint was reached
	*/
	bool waypointReached(Pose &target) const;
	

	double stopAndTurnAngle;
	double tvP;
	double rvP;
	double maxDisalignment;
	double reachedHysteresisRatio;
	bool aligning;
	bool targetSet;
	bool poseSet;
	Pose curPose;
	Pose targetPose;
	
	std::vector<DumbTrajectoryFollower::Pose *> trajectory;
	std::vector<DumbTrajectoryFollower::Pose *>::iterator currentWaypoint;
};

#endif 
