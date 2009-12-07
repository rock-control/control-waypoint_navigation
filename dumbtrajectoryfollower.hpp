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
		Pose() {};
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Vector3d position;
	     	Eigen::Quaterniond orientation;
	};
	
	/**
	* set positon and orientation, where to drive to
	*/
	void setTargetPose(Eigen::Vector3d p, Eigen::Quaterniond o);
	
	/**
	* Set current orientation and position
	* TODO add uncertaintys
	*/
	void setPose(Eigen::Vector3d p, Eigen::Quaterniond o);
	
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
	bool waypointReached(Eigen::Vector3d &target, Eigen::Quaterniond &targetOrientation) const;
	

	double stopAndTurnAngle;
	double tvP;
	double rvP;
	double wayPointReachedDistance;
	double wayPointLeftDistance;
	double maxDisalignment;
	bool aligning;
	bool targetSet;
	bool poseSet;
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
	
	Eigen::Vector3d targetPosition;
	Eigen::Quaterniond targetOrientation;
	
	std::vector<DumbTrajectoryFollower::Pose *> trajectory;
	std::vector<DumbTrajectoryFollower::Pose *>::iterator currentWaypoint;
};

#endif 
