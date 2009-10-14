#ifndef LOCALPLANNER_HPP
#define LOCALPLANNER_HPP

#include <Eigen/Geometry>

class LocalPlanner
{
    //TODO add eigen allign constructor
    public:
	LocalPlanner();
	
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
	
    private:
	double stopAndTurnAngle;
	double tvP;
	double rvP;
	double wayPointReachedDistance;
	double wayPointLeftDistance;
	bool aligning;
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
	
	Eigen::Vector3d targetPosition;
	Eigen::Quaterniond targetOrientation;
};

#endif // LOCALPLANNER_HPP
