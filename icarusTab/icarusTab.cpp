/*
 * This file: Copyright (c) 2013 Gustavo Gomes (github at guvux dot com dot br)
 * This software is derived from planningTab.cpp
 * 
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "icarusTab.h"

using namespace std;

// Define IDs for buttons
enum DynamicSimulationTabEvents {
	id_button_RestartObjects,
	id_button_ShowStart,
	id_button_ShowGoal,
	id_button_Plan
};

// Handler for events
BEGIN_EVENT_TABLE(icarusTab, wxPanel)
	EVT_COMMAND (id_button_RestartObjects, wxEVT_COMMAND_BUTTON_CLICKED, icarusTab::onButtonRestartObjects)
	EVT_COMMAND (id_button_ShowStart, wxEVT_COMMAND_BUTTON_CLICKED, icarusTab::onButtonShowStart)
	EVT_COMMAND (id_button_ShowGoal, wxEVT_COMMAND_BUTTON_CLICKED, icarusTab::onButtonShowGoal)
	EVT_COMMAND (id_button_Plan, wxEVT_COMMAND_BUTTON_CLICKED, icarusTab::onButtonAction)
END_EVENT_TABLE()

IMPLEMENT_DYNAMIC_CLASS(icarusTab, GRIPTab)

icarusTab::icarusTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style) :
  GRIPTab(parent, id, pos, size, style) {
	// Create user interface
	wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);
  
	wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Setup planning problem"));
	wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Check"));
	wxStaticBox* ss3Box = new wxStaticBox(this, -1, wxT("Execute"));
  
	wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
	wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);
	wxStaticBoxSizer* ss3BoxS = new wxStaticBoxSizer(ss3Box, wxVERTICAL);

	ss1BoxS->Add(new wxButton(this, id_button_RestartObjects, wxT("Restart objects")), 0, wxALL, 1);
	ss2BoxS->Add(new wxButton(this, id_button_ShowStart, wxT("Show Start")), 0, wxALL, 1); 
	ss2BoxS->Add(new wxButton(this, id_button_ShowGoal, wxT("Show Goal")), 0, wxALL, 1); 
	ss3BoxS->Add(new wxButton(this, id_button_Plan, wxT("Do Action")), 0, wxALL, 1); 

	sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss3BoxS, 1, wxEXPAND | wxALL, 6);

	SetSizer(sizerFull);

	// Set predefined start and goal configuration
	mPredefStartConf.resize(6);
	mPredefGoalConf.resize(6);
	mPredefStartConf << -0.858702, -0.674395, 0.0, -0.337896, 0.0, 0.0;
	mPredefGoalConf << -0.69115, 0.121475, 0.284977, -1.02486, 0.0, 0.0;
	mStartConf = mPredefStartConf;
	mGoalConf = mPredefGoalConf;
	mAlreadyReplan = false;
	mController = NULL;
}

/// Before each simulation step we set the torques the controller applies to the joints
void icarusTab::GRIPEventSimulationBeforeTimestep() {
	Eigen::VectorXd torques = mController->getTorques(mRobot->getPose(), mRobot->getPoseVelocity(), mWorld->getTime());
	// section here to control the fingers for force-based grasping
    // instead of position-based grasping
	mRobot->setInternalForces(torques);

	//check object position and replan only if it hasnt been done already to save computing power
    if (!mAlreadyReplan) {
		grasper->findClosestGraspingPoint(currentGraspPoint, selectedNode);
		Vector3d diff = currentGraspPoint - grasper->getGraspingPoint();
        
		// Note: Error bound must be < 0.09 as Jacobian translation fails to reach when it's too close to target
		if (diff.norm() >= 0.006) {
			ECHO("\tNote: Re-planning grasp!");
			this->retryGrasp();
			mAlreadyReplan = true;
		}
	}
}

/// Gets triggered after a world is loaded
void icarusTab::GRIPEventSceneLoaded() {
	// Find robot and set initial configuration for the legs
	mRobot = mWorld->getSkeleton("GolemHubo");
	assert(mRobot);

	// Set initial configuration for the legs
	int legDofsArray[] = {19, 20, 23, 24, 27, 28};
	vector<int> legDofs(legDofsArray, legDofsArray + 6);
	Eigen::VectorXd legValues(6);
	legValues << -10.0, -10.0, 20.0, 20.0, -10.0, -10.0;
	legValues *= M_PI / 180.0;
	mRobot->setConfig(legDofs, legValues);

	// Define right arm nodes
	const string armNodes[] = {"Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "Body_RWP"}; 
	mArmDofs.resize(6);
	for(int i = 0; i < mArmDofs.size(); i++) {
		mArmDofs[i] = mRobot->getNode(armNodes[i].c_str())->getDof(0)->getSkelIndex();
	}

	//Define palm effector name; Note: this is robot dependent!
	eeName = "Body_RWP";
    // Initialize Grasper
    grasper = new planning::Grasper(mWorld, mRobot, eeName);
	if (mWorld->getSkeleton("yellowCube")) {
		selectedNode = mWorld->getSkeleton("yellowCube")->getRoot();
	}
	else {
		cout << "There is not yellow Cube in this world" << endl;
	}
}

/// Store selected node in tree-view data as grasper's objective
void icarusTab::GRIPStateChange() {
	if (!selectedTreeNode) {
        return;
    }
    switch (selectedTreeNode->dType) {
		case Return_Type_Robot:
			selectedNode = ((kinematics::Skeleton*)selectedTreeNode->data)->getRoot();
			break;
		case Return_Type_Node:
			selectedNode = (kinematics::BodyNode*)selectedTreeNode->data;
			break;
		default:
			fprintf(stderr, "someone else's problem.");
			assert(0);
			exit(1);
    }
}

/// Move the cubes to the initial position
void icarusTab::onButtonRestartObjects(wxCommandEvent & _evt) {

  dynamics::SkeletonDynamics* orangeCube = mWorld->getSkeleton("orangeCube");
  dynamics::SkeletonDynamics* yellowCube = mWorld->getSkeleton("yellowCube");
  dynamics::SkeletonDynamics* cyanCube = mWorld->getSkeleton("cyanCube");
  dynamics::SkeletonDynamics* redCube = mWorld->getSkeleton("redCube");
  
  if(!orangeCube || !yellowCube || !cyanCube || !redCube) {
    cout << "Did not find orange or yellow or cyan or red object. Exiting and no moving anything" << endl;
    return;
  }
  
  Eigen::Matrix<double, 6, 1> pose; 
  pose << 0.30, -0.30, 0.83, 0.0, 0.0, 0.0;
  orangeCube->setPose(pose);
  pose << 0.30, -0.30, 0.935, 0.0, 0.0, 0.0;
  yellowCube->setPose(pose);
  pose << 0.30, -0.15, 0.83, 0.0, 0.0, 0.0;
  cyanCube->setPose(pose);
  pose << 0.30, -0.15, 0.935, 0.0, 0.0, 0.0;
  redCube->setPose(pose);

  viewer->DrawGLScene();
}


/// Show the currently set start configuration
void icarusTab::onButtonShowStart(wxCommandEvent & _evt) {
  cout << "Showing start conf for right arm: " << mStartConf.transpose() << endl;
  mRobot->setConfig(mArmDofs, mStartConf);
  viewer->DrawGLScene();
}


/// Show the currently set goal configuration
void icarusTab::onButtonShowGoal(wxCommandEvent & _evt) {
  cout << "Showing goal conf for right arm: " << mGoalConf.transpose() << endl;
  mRobot->setConfig(mArmDofs, mGoalConf);
  viewer->DrawGLScene();
}

/// Test currently implemented action
void icarusTab::onButtonAction(wxCommandEvent & _evt) {
	if(!mWorld || mRobot == NULL){
        cout << "No world loaded or world does not contain a robot" << endl;
        return;
    }
    grasp();
}

/// Set initial dynamic parameters and call grasp planner and controller
void icarusTab::grasp() {
    
    if (selectedNode == NULL || mStartConf.size() == 0) {
		cout << "\tERROR: Must select an object to grasp first!!" << endl;
		return;
	}
    // Perform memory management to allow for continuous grasping tests
    if (mController != NULL) {
        delete mController;
        delete grasper;
        //re-init grasper
        grasper = new planning::Grasper(mWorld, mRobot, eeName);
    }
    // Store the actuated joints (all except the first 6 which are only a convenience to locate the robot in the world)
    std::vector<int> actuatedDofs(mRobot->getNumDofs() - 6);
    for (unsigned int i = 0; i < actuatedDofs.size(); i++) {
        actuatedDofs[i] = i + 6;
    }

    // Define PD controller gains
    Eigen::VectorXd kI = 100.0 * Eigen::VectorXd::Ones(mRobot->getNumDofs());
    Eigen::VectorXd kP = 500.0 * Eigen::VectorXd::Ones(mRobot->getNumDofs());
    Eigen::VectorXd kD = 100.0 * Eigen::VectorXd::Ones(mRobot->getNumDofs());

    // Define gains for the ankle PD
    std::vector<int> ankleDofs(2);
    ankleDofs[0] = 27;
    ankleDofs[1] = 28;
    const Eigen::VectorXd anklePGains = -1000.0 * Eigen::VectorXd::Ones(2);
    const Eigen::VectorXd ankleDGains = -200.0 * Eigen::VectorXd::Ones(2);
    
    // Update robot's pose
    mRobot->setConfig(mArmDofs, mStartConf);
    
    // Create controller
    mController = new planning::Controller(mRobot, actuatedDofs, kP, kD, ankleDofs, anklePGains, ankleDGains);
   
    // Setup grasper with a step = 0.02 mainly for JointMover
    grasper->init(mArmDofs, mStartConf, selectedNode, 0.02);
    
    // Perform grasp planning; now really it's just Jacobian translation
    std::list<Eigen::VectorXd> path;
    std::vector<int> mTotalDofs;
    grasper->plan(path, mTotalDofs);
    
    // CHECK
    cout << "Offline Plan Size: " << path.size() << endl;
    
    // Create trajectory; no need to shorten path here
    const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mTotalDofs.size());
    const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mTotalDofs.size());
    planning::Trajectory* trajectory = new planning::PathFollowingTrajectory(path, maxVelocity, maxAcceleration);
    
    std::cout << "Trajectory duration: " << trajectory->getDuration() << endl;
    mController->setTrajectory(trajectory, 0, mTotalDofs);
    
    printf("Controller time: %f \n", mWorld->getTime());
}

/// Replan in the middle of simulation according to accuracy measures
void icarusTab::retryGrasp() {
    // Setup grasper by updating startConfig to be current robot's config
    grasper->init(mArmDofs, mRobot->getConfig(mArmDofs), selectedNode, 0.02);
    
    // Perform grasp planning; now really it's just Jacobian translation
    std::list<Eigen::VectorXd> path;
    std::vector<int> mTotalDofs;
    grasper->plan(path, mTotalDofs);
    
    // CHECK
    cout << "\tReplanned Path Size: " << path.size()<< endl;
     
    // Create trajectory; no need to shorten path here
    const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mTotalDofs.size());
    const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mTotalDofs.size());
    planning::Trajectory* trajectory = new planning::PathFollowingTrajectory(path, maxVelocity, maxAcceleration);
    
    cout << "\tReplanned Trajectory Duration: " << trajectory->getDuration() << endl;
    mController->setTrajectory(trajectory, 0, mTotalDofs);
    
    printf("\tReplanned Controller Time: %f \n", mWorld->getTime());
}
