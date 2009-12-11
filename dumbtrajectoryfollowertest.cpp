#include "dumbtrajectoryfollowertest.hpp"
#include "dumbtrajectoryfollower.hpp"
#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;

int main() {
    DumbTrajectoryFollower lp;
    
    
    DumbTrajectoryFollower::Pose robotPose;
    DumbTrajectoryFollower::Pose targetPose;
    

    robotPose.covariancePosition = Eigen::Matrix3d::Identity() * 0.001;
    
    double tv, rv;
    

    //test case target in front of robot
    robotPose.orientation.setIdentity();
    targetPose.orientation.setIdentity();
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(0,1,0);
    
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(tv > 0 && fabs(rv) < 0.001);
    std::cout << "Test 1 PASSED" << std::endl << std::endl;
    
    //test case target left of robot
    robotPose.orientation.setIdentity();
    targetPose.orientation.setIdentity();
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(-1,0,0);
    
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0);
    std::cout << "Test 2 PASSED" << std::endl << std::endl;
    
    //test case target right of robot
    robotPose.orientation.setIdentity();
    targetPose.orientation.setIdentity();
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(1,0,0);
    
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv < 0);
    std::cout << "Test 3 PASSED" << std::endl << std::endl;
    
    //test case target right of robot
    robotPose.orientation.setIdentity();
    targetPose.orientation.setIdentity();
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(1,0.1,0);
    
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv < 0);
    std::cout << "Test 4 PASSED" << std::endl << std::endl;
    
    //test case target equals own position
    robotPose.position = Eigen::Vector3d(0,0,0);
    robotPose.orientation.setIdentity();
    targetPose.orientation.setIdentity();
    lp.setPose(robotPose);
    lp.setTargetPose(robotPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && fabs(rv) < 0.001);
    std::cout << "Test 5 PASSED" << std::endl << std::endl;
    
    
    //test case robot needs to be alligned
    robotPose.orientation.setIdentity();
    targetPose.orientation = Quaterniond(AngleAxisd(M_PI/2.0, Vector3d::UnitZ()));
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(0,0,0);
    
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0);
    std::cout << "Test 6 PASSED" << std::endl << std::endl;
    
    //test case robot needs to be alligned
    robotPose.orientation = Quaterniond(AngleAxisd(M_PI/4.0, Vector3d::UnitZ()));
    targetPose.orientation = Quaterniond(AngleAxisd(M_PI/2.0, Vector3d::UnitZ()));
    robotPose.position = Eigen::Vector3d(0,0,0);
    targetPose.position = Eigen::Vector3d(0,0,0);
    
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0);
    std::cout << "Test 7 PASSED" << std::endl << std::endl;    

    //test case robot is in front of target and rear points toward target
    robotPose.orientation.setIdentity();
    targetPose.orientation.setIdentity();
    robotPose.position = Eigen::Vector3d(0,1,0);
    targetPose.position = Eigen::Vector3d(0,0,0);
    lp.setPose(robotPose);
    lp.setTargetPose(targetPose);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert((fabs(tv) < 0.001) && ((rv > 0.3) || (rv < -0.3)));
    std::cout << "Test 8 PASSED" << std::endl << std::endl;    
    
}

