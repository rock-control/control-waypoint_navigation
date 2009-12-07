#include "dumbtrajectoryfollowertest.hpp"
#include "dumbtrajectoryfollower.hpp"
#include <Eigen/Geometry>
#include <iostream>

using namespace Eigen;

int main() {
    DumbTrajectoryFollower lp;
    
    Eigen::Vector3d robotPos(0,0,0);
    Eigen::Quaterniond robotOri;
    Eigen::Vector3d targetPos(0,0,0);
    Eigen::Quaterniond targetOri;
        
    double tv, rv;
    

    //test case target in front of robot
    robotOri.setIdentity();
    targetOri.setIdentity();
    robotPos = Eigen::Vector3d(0,0,0);
    targetPos = Eigen::Vector3d(0,1,0);
    
    lp.setPose(robotPos, robotOri);
    lp.setTargetPose(targetPos, targetOri);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(tv > 0 && fabs(rv) < 0.001);
    std::cout << "Test 1 PASSED" << std::endl << std::endl;
    
    //test case target left of robot
    robotOri.setIdentity();
    targetOri.setIdentity();
    robotPos = Eigen::Vector3d(0,0,0);
    targetPos = Eigen::Vector3d(-1,0,0);
    
    lp.setPose(robotPos, robotOri);
    lp.setTargetPose(targetPos, targetOri);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0);
    std::cout << "Test 2 PASSED" << std::endl << std::endl;
    
    //test case target right of robot
    robotOri.setIdentity();
    targetOri.setIdentity();
    robotPos = Eigen::Vector3d(0,0,0);
    targetPos = Eigen::Vector3d(1,0,0);
    
    lp.setPose(robotPos, robotOri);
    lp.setTargetPose(targetPos, targetOri);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv < 0);
    std::cout << "Test 3 PASSED" << std::endl << std::endl;
    
    //test case target right of robot
    robotOri.setIdentity();
    targetOri.setIdentity();
    robotPos = Eigen::Vector3d(0,0,0);
    targetPos = Eigen::Vector3d(1,0.1,0);
    
    lp.setPose(robotPos, robotOri);
    lp.setTargetPose(targetPos, targetOri);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv < 0);
    std::cout << "Test 4 PASSED" << std::endl << std::endl;
    
    //test case target equals own position
    robotPos = Eigen::Vector3d(0,0,0);
    robotOri.setIdentity();
    lp.setPose(robotPos, robotOri);
    lp.setTargetPose(robotPos, robotOri);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && fabs(rv) < 0.001);
    std::cout << "Test 5 PASSED" << std::endl << std::endl;
    
    
    //test case robot needs to be alligned
    robotOri.setIdentity();
    targetOri.setIdentity();
    targetOri = Quaterniond(AngleAxisd(M_PI/2.0, Vector3d::UnitZ()));
    robotPos = Eigen::Vector3d(0,0,0);
    targetPos = Eigen::Vector3d(0,0,0);
    
    lp.setPose(robotPos, robotOri);
    lp.setTargetPose(targetPos, targetOri);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0);
    std::cout << "Test 6 PASSED" << std::endl << std::endl;
    
    //test case robot needs to be alligned
    robotOri.setIdentity();
    robotOri = Quaterniond(AngleAxisd(M_PI/4.0, Vector3d::UnitZ()));
    targetOri.setIdentity();
    targetOri = Quaterniond(AngleAxisd(M_PI/2.0, Vector3d::UnitZ()));
    robotPos = Eigen::Vector3d(0,0,0);
    targetPos = Eigen::Vector3d(0,0,0);
    
    lp.setPose(robotPos, robotOri);
    lp.setTargetPose(targetPos, targetOri);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0);
    std::cout << "Test 7 PASSED" << std::endl << std::endl;    

    //test case robot is in front of target and rear points toward target
    robotOri.setIdentity();
    targetOri.setIdentity();
    robotPos = Eigen::Vector3d(0,1,0);
    targetPos = Eigen::Vector3d(0,0,0);
    lp.setPose(robotPos, robotOri);
    lp.setTargetPose(targetPos, targetOri);
    lp.getMovementCommand(tv, rv);
    std::cout << "Tv: " << tv << " Rv: " << rv << std::endl;
    assert(fabs(tv) < 0.001 && rv > 0.3 || rv < -0.3);
    std::cout << "Test 8 PASSED" << std::endl << std::endl;    
    
}

