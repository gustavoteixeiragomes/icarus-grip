/*
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

#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN 1
#endif
#include <wx/wx.h>
#include <WinSock2.h>
#include <ws2tcpip.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <iostream>
#include <sys/types.h>

#include "icarusTab.h"

/*******************************
* Temp
*******************************/
//#include "socket.h"
//#include <fstream>
//#include <process.h>
/*******************************
* Temp
*******************************/


#include <collision/CollisionSkeleton.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <planning/PathPlanner.h>
#include <planning/PathShortener.h>
#include <planning/PathFollowingTrajectory.h>
#include "Controller.h"

using namespace std;

HANDLE hThread;
unsigned threadID;
bool threadStop;

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
EVT_COMMAND (id_button_Plan, wxEVT_COMMAND_BUTTON_CLICKED, icarusTab::onButtonPlan)
END_EVENT_TABLE()

IMPLEMENT_DYNAMIC_CLASS(icarusTab, GRIPTab)

icarusTab::icarusTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style) :
  GRIPTab(parent, id, pos, size, style)
{
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
  ss3BoxS->Add(new wxButton(this, id_button_Plan, wxT("Do Planning")), 0, wxALL, 1); 

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
  
}


/// Gets triggered after a world is loaded
void icarusTab::GRIPEventSceneLoaded() {
  mRobot = mWorld->getSkeleton("GolemHubo");

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
}

/// Start the server on the simulation start
void icarusTab::GRIPEventSimulationStart() {
	std::cout << "Start Server" << endl;
	icarusTab::startServer();
}

/// Stop the server on the simulation stop
void icarusTab::GRIPEventSimulationStop() {
	std::cout << "Stop Server" << endl;
	icarusTab::stopServer();
}

/// Before each simulation step we set the torques the controller applies to the joints
void icarusTab::GRIPEventSimulationBeforeTimestep() {
	Eigen::VectorXd torques = mController->getTorques(mRobot->getPose(), mRobot->getPoseVelocity(), mWorld->getTime());
	mRobot->setInternalForces(torques);
}

/// Move the cube to the initial position
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


/// Set initial dynamic parameters and call planner and controller
void icarusTab::onButtonPlan(wxCommandEvent & _evt) {

  // Store the actuated joints (all except the first 6 which are only a convenience to locate the robot in the world)
  std::vector<int> actuatedDofs(mRobot->getNumDofs() - 6);
  for(unsigned int i = 0; i < actuatedDofs.size(); i++) {
    actuatedDofs[i] = i + 6;
  }
  
  // Deactivate collision checking between the feet and the ground during planning
  dynamics::SkeletonDynamics* ground = mWorld->getSkeleton("ground");
  //mWorld->getCollisionHandle()->getCollisionChecker()->deactivatePair(mRobot->getNode("Body_LAR"), ground->getNode(1));
  //mWorld->getCollisionHandle()->getCollisionChecker()->deactivatePair(mRobot->getNode("Body_RAR"), ground->getNode(1));
  
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

  // Set robot to start configuration
  mRobot->setConfig(mArmDofs, mStartConf);

  // Create controller
  mController = new planning::Controller(mRobot, actuatedDofs, kP, kD, ankleDofs, anklePGains, ankleDGains);

  // Call path planner
  planning::PathPlanner<> pathPlanner(*mWorld);
  std::list<Eigen::VectorXd> path;
  if(!pathPlanner.planPath(mRobot, mArmDofs, mStartConf, mGoalConf, path)) {
    std::cout << "Path planner could not find a path." << std::endl;
  }
  else {
    // Call path shortener
    planning::PathShortener pathShortener(mWorld, mRobot, mArmDofs);
    pathShortener.shortenPath(path);

    // Convert path into time-parameterized trajectory satisfying acceleration and velocity constraints
    const Eigen::VectorXd maxVelocity = 0.6 * Eigen::VectorXd::Ones(mArmDofs.size());
    const Eigen::VectorXd maxAcceleration = 0.6 * Eigen::VectorXd::Ones(mArmDofs.size());
    planning::Trajectory* trajectory = new planning::PathFollowingTrajectory(path, maxVelocity, maxAcceleration);
    std::cout << "-- Trajectory duration: " << trajectory->getDuration() << endl;
    mController->setTrajectory(trajectory, 0.0, mArmDofs);
  }
  
  // Reactivate collision of feet with floor
  //mWorld->getCollisionHandle()->getCollisionChecker()->activatePair(mRobot->getNode("Body_LAR"), ground->getNode(1));
  //mWorld->getCollisionHandle()->getCollisionChecker()->activatePair(mRobot->getNode("Body_RAR"), ground->getNode(1));
}

/*=====================================================================================================================
=======================================================================================================================
=======================================================================================================================
=======================================================================================================================
=======================================================================================================================
=======================================================================================================================
=====================================================================================================================*/

//***************************** Move to external class

// Serialize a Eigen::VectorXd
std::string icarusTab::serializeVectorXd(const Eigen::VectorXd &vectorXd)
{
	//std::string output;
	int size = vectorXd.size();
	std::stringstream output;
	output << "a:" << size;
	if (size > 0) {
		output << ":{";
	}
	for (int i = 0; i < size; i++) {
		output << "i:" << i << ";" << "d:" << vectorXd.array()[i] << ";";
	}
	if (size > 0) {
		output << "}";
	}
	return output.str();
}

// Run when the server accept a connection
unsigned __stdcall Connection(void* arg) {
	icarusTab* icarusTabObj = (icarusTab*)arg;
	// Create socket
	SocketInterface::SocketServer server(SERVER_PORT, 1, SocketInterface::NonBlockingSocket);
	
	 while (!threadStop) {
		// Wait for the client
		std::cout << "Waiting on port " << SERVER_PORT << endl;
		int connectionResult = WSAEWOULDBLOCK;
		SocketInterface::Socket* clientConnection;
		while(connectionResult==WSAEWOULDBLOCK && !threadStop)
		{
			//std::cout<<"Waiting for incoming connections...\r\n";
			clientConnection = server.Accept();
			connectionResult=WSAGetLastError();
		}
		while (!threadStop) {
			std::string inputTemp = clientConnection->ReceiveLine();
			std::string inputData = inputTemp.substr(0, inputTemp.size()-1);
			//cout << inputData.length() << " | " << inputData << endl;
			int nError=WSAGetLastError();
			//cout << nError << endl;
			// Wait for receive data
			if (nError==WSAEWOULDBLOCK) {
				//cout << "WSAEWOULDBLOCK" << endl;
			}
			// winsock error
			//else if (nError!=WSAEWOULDBLOCK && nError!=0 && inputData.empty() && nError!=6) {
			else if (nError!=WSAEWOULDBLOCK && nError!=0 && inputData.empty()) {
				//cout<<"Winsock error code: "<<nError<<"\r\n";
				cout<<"Server disconnected!\r\n";
				// Close our socket entirely
				clientConnection->Close();

				break;
			}
			// Information OK
			else {
				if (inputData == "close()") {
					cout << "closing" << endl;
					break;
				}
				/*
				std::string outputData;
				outputData = icarusTabObj->serializeVectorXd(mWorld->getState().transpose());
				clientConnection->SendLine(outputData);
				*/
				else if (inputData == "update()") {
					cout << "Showing start conf for right arm: " << icarusTabObj->mStartConf.transpose() << endl;
					icarusTabObj->mRobot->setConfig(icarusTabObj->mArmDofs, icarusTabObj->mStartConf);
					clientConnection->SendLine("update");
				}
				else if (inputData == "getObjects()") {
					std::stringstream outputData;
						
					// getYellowCube
					dynamics::SkeletonDynamics* yellowCube = mWorld->getSkeleton("yellowCube");
					
					if(yellowCube) {
						Eigen::VectorXd pose, dimension;
						pose = yellowCube->getPose();
						dimension = yellowCube->getNode(0)->getShape()->getDim();
					
						outputData << "(block yellowCube xpos " << pose.x() << " ypos " << pose.y() << " zpos " << pose.z();
						outputData << " roll " << pose[3] << " pitch " << pose[4] << " yaw " << pose[5];
						outputData << " width " << dimension.x() << " height " << dimension.y() << " length  " << dimension.z() << " shade " << pose[5] << ")";
						cout << "sending yellowCube" << endl;
					}
				
					// getOrangeCube()
					dynamics::SkeletonDynamics* orangeCube = mWorld->getSkeleton("orangeCube");
					
					if (orangeCube) {
						Eigen::VectorXd pose, dimension;
						pose = orangeCube->getPose();
						dimension = orangeCube->getNode(0)->getShape()->getDim();
						
						outputData << "(block orangeCube xpos " << pose.x() << " ypos " << pose.y() << " zpos " << pose.z();
						outputData << " roll " << pose[3] << " pitch " << pose[4] << " yaw " << pose[5];
						outputData << " width " << dimension.x() << " height " << dimension.y() << " length  " << dimension.z() << " shade " << 0 << ")";
						cout << "sending OrangeCube" << endl;
					}

					// getGlassDiningTable2()
					dynamics::SkeletonDynamics* diningTable = mWorld->getSkeleton("glassDiningTable2");
					
					if (diningTable) {
						Eigen::VectorXd pose, dimension;
						pose = diningTable->getPose();
						dimension = diningTable->getNode(0)->getShape()->getDim();
						
						outputData << "(block glassDiningTable2 xpos " << pose.x() << " ypos " << pose.y() << " zpos " << pose.z();
						outputData << " roll " << pose[3] << " pitch " << pose[4] << " yaw " << pose[5] << ")";;
						cout << "sending glassDiningTable2" << endl;
					}
				
					// getGolemHubo()
					dynamics::SkeletonDynamics* golemHubo = mWorld->getSkeleton("GolemHubo");
					
					if (golemHubo) {
						Eigen::VectorXd pose, dimension;
						pose = golemHubo->getPose();
						dimension = golemHubo->getNode(0)->getShape()->getDim();
						
						outputData << "(block GolemHubo xpos " << pose.x() << " ypos " << pose.y() << " zpos " << pose.z();
						outputData << " roll " << pose[3] << " pitch " << pose[4] << " yaw " << pose[5] << ")";;
						cout << "sending GolemHubo" << endl;
					}

					clientConnection->SendLine( outputData.str() );
				}
				else if (inputData == "wait()") {
					cout << "Waiting" << endl;
					clientConnection->SendLine("wait");
				}
				else if (!inputData.empty()) {
					cout << "Unknow command" << endl;
					clientConnection->SendLine("Unknow command");
				}
			}
		}
		if (clientConnection) {
			clientConnection->Close();
		}
	}
	server.Close();
	return 0;
}

// Start the server
void icarusTab::startServer() {
	try {
		// Signal to continue the thread.
		threadStop = false;
		hThread = (HANDLE)_beginthreadex(0, 0, Connection, this, 0, &threadID);
	}
	catch (char *s) {
		cerr << s << endl;
	} 
	catch (std::string s) {
		cerr << s << endl;
	}
	catch (std::exception& e) {
		cerr << "Standard exception: " << e.what() << endl;
	}
	catch (...) {
		cerr << "unhandled exception\n";
	}
}

// Stop the server
void icarusTab::stopServer() {
	try {
		// Signal to stop the thread.
		threadStop = true;
		// Destroy the thread object.
		CloseHandle( hThread );
	}
	catch (char *s) {
		cerr << s << endl;
	} 
	catch (std::string s) {
		cerr << s << endl;
	}
	catch (std::exception& e) {
		cerr << "Standard exception: " << e.what() << endl;
	}
	catch (...) {
		cerr << "unhandled exception\n";
	}
}

// Local Variables:
// c-basic-offset: 2
// End:
