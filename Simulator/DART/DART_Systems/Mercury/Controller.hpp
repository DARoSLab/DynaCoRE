#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "worldnode.hpp"
#include "Configuration.h"

class Controller
{
public:
	Controller(const dart::dynamics::SkeletonPtr& robot): robot_(robot){
		nDofs_ = robot_->getNumDofs();
		Forces_ = Eigen::VectorXd::Zero(nDofs_);

		Kp_ = Eigen::MatrixXd::Identity(nDofs_,nDofs_);
		Kd_ = Eigen::MatrixXd::Identity(nDofs_,nDofs_);

		// virtual joints
		for(int i(0); i<6 ;i++){
			Kp_(i,i) = 0.0;
			Kd_(i,i) = 0.0;
		}
	}

	void clearForces(){
		Forces_.setZero();
	}

	void setPDGains(){
		for(int i(6); i<robot_->getNumDofs(); ++i){
			Kp_(i,i) = 1000;
			Kd_(i,i) = 50;
		}
		int Ank_num1= robot_->getDof("ankle_r")->getIndexInSkeleton();
		int Ank_num2= robot_->getDof("ankle_l")->getIndexInSkeleton();
		Kp_(Ank_num1, Ank_num1) = 0.0;
		Kd_(Ank_num2, Ank_num2) = 0.0;
	}

	void setPDForces(Eigen::VectorXd& JPos_des){
		Eigen::VectorXd q= robot_->getPositions();
		Eigen::VectorXd dq = robot_->getVelocities();

		Forces_ = Kp_*(JPos_des - q)  -  Kd_*dq;
	}

private:
	dart::dynamics::SkeletonPtr robot_;

	Eigen::MatrixXd Kp_;
	Eigen::MatrixXd Kd_;

public:
	int nDofs_;
	Eigen::VectorXd Forces_;
};