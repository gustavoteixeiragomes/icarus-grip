
#include "HandleInterface.h"

using namespace std;
using namespace Eigen;
using namespace simulation;

handleInterface::Handle::Handle() {
	clear();
}

handleInterface::Handle::Handle(std::string request) {
	setRequest(request);
}

int handleInterface::Handle::execute() {
	if (state_ == waitingExecution) {
		if (request_ == "getObject()") {
			answer_ = "Hello World!";
		}
		else if (request_ == "getObjects()") {
			std::stringstream outputData;
			outputData << std::fixed << std::setprecision(2);

			if (mWorld) {
				int numObjects = mWorld->getNumSkeletons();
				if (numObjects > 0) {
					for (int i = 0; i < numObjects; i++) {
						// getObject
						dynamics::SkeletonDynamics* object = mWorld->getSkeleton(i);
						cout << i << ": sending " << object->getName() << endl;
						if(object) {
							if (object->getName() != "ground") {
								Eigen::VectorXd pose, dimension;
								pose = object->getPose();

								outputData << "(block " << object->getName() << " xpos " << pose.x() << " ypos " << pose.y() << " zpos " << pose.z();
								outputData << " roll " << pose[3] << " pitch " << pose[4] << " yaw " << pose[5];
								if (object->getName() != "GolemHubo" && object->getName() != "glassDiningTable2") {
									dimension = object->getNode(0)->getShape()->getDim();
									outputData << " width " << dimension.x() << " height " << dimension.y() << " length  " << dimension.z() << " shade " << pose[5];
								}
							}
							else {
								outputData << "(block " << object->getName();
							}
							outputData << ")";
							cout << "sending " << object->getName() << endl;
						}
					}
					answer_ = outputData.str();
					// verificar o envio de strings grandes....
					cout << answer_ << "| " << answer_.size() <<  endl;
				}
				else {
					answer_ = "No objects in this world";
				}
				//cout << "out" << endl;
			}
			else {
				answer_ = "The world is not ready";
			}
		}
		else {
			answer_ = "Unknown command";
		}
		state_ = done;
	}
	return state_;
}

std::string handleInterface::Handle::getRequest() {
	return request_;
}

void handleInterface::Handle::setRequest(std::string request) {
	request_ = request;
	answer_.clear();
	state_ = waitingExecution;
}

std::string handleInterface::Handle::getAnswer() {
	return answer_;
}

void handleInterface::Handle::clear() {
	request_.clear();
	answer_.clear();
	state_ = waitingRequest;
}