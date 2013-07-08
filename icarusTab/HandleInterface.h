#pragma once
#ifndef __HANDLE_INTERFACE__
	#define __HANDLE_INTERFACE__

#include <string>
#include <sstream>
#include <iomanip>
#include <GUI/GUI.h>  // extern mWorld
#include <simulation/World.h>
#include <dynamics/BodyNodeDynamics.h>
#include <dynamics/SkeletonDynamics.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include <kinematics/Shape.h>

namespace handleInterface {
	// Handle with the requests from the client Icarus in the server
	class Handle {
		public:
			Handle();
			Handle(std::string request);
			int execute();
			std::string getRequest();
			void setRequest(std::string request);
			std::string getAnswer();
			void clear();
			//simulation::World* world_;
			
		private:
			enum { waitingRequest, waitingExecution, done } state_;
			std::string request_;
			std::string answer_;
	};
} // Namespace icarusInterface

#endif

/*/ Serialize a Eigen::VectorXd
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
*/