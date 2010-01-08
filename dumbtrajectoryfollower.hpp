#ifndef DUMBTRAJECTORYFOLLOWER_HPP
#define DUMBTRAJECTORYFOLLOWER_HPP
#include <Eigen/Geometry>
#include <vector>
#include <base/samples/rigid_body_state.h>
#include <base/waypoint.h>

class DumbTrajectoryFollower
{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	DumbTrajectoryFollower();
	
	/**
	* set positon and orientation, where to drive to
	*/
	void setTargetPose(base::Waypoint &pose);
	
	/**
	* Set current orientation and position
	*/
	void setPose(base::samples::RigidBodyState &pose);
	
	/**
	* calculates a translational and rotational velocity
	* that should drive the robot from the current
	* pose to the target pose
	*/
	void getMovementCommand(double &tv, double &rv);
	
	/**
	* sets the trajecory the robot should follow
	*/
	void setTrajectory(std::vector<base::Waypoint *> &t);
	
	/**
	* tests if the current Waypoint was reached and switches to
	* the next waypoint in the trajectory
	*
	* returns true if new waypoint was selected
	*/
	bool testSetNextWaypoint();
	
	/**
	* returns an iterator, that points to the current waypoint in
	* the trajectory
	*/
	std::vector<base::Waypoint *>::const_iterator getCurrentWaypoint() const {
	    return currentWaypoint;
	}
	
	/**
	* returns the trajectory
	*/
	const std::vector<base::Waypoint *> &getTrajectory() const {
	    return trajectory;
	}
	
    private:
	/**
	* Calculates if the given waypoint was reached
	*/
	bool waypointReached(base::Waypoint &target) const;
	
	bool newWaypoint;
	double stopAndTurnAngle;
	double tvP;
	double rvP;
	double maxDisalignment;
	double reachedHysteresisRatio;
	bool aligning;
	bool targetSet;
	bool poseSet;
	base::samples::RigidBodyState curPose;
	base::Waypoint targetPose;
	
	std::vector<base::Waypoint *> trajectory;
	std::vector<base::Waypoint *>::iterator currentWaypoint;
};

#endif 
