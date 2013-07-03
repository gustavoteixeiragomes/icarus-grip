
#include "HandleInterface.h"

handleInterface::Handle::Handle() {
	clear();
}

handleInterface::Handle::Handle(std::string request) {
	setRequest(request);
}

int handleInterface::Handle::execute() {
	if (state_ == waitingExecution) {
		if (request_ == "getObjects()") {
			answer_ = "Hello World!";
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