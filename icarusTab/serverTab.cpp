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

#include "serverTab.h"

using namespace std;

HANDLE hThread;
unsigned threadID;
bool threadStop;

// Define IDs for buttons
enum DynamicSimulationTabEvents {
  id_button_Start,
  id_button_Stop,
  id_button_ChangePort
};

// Handler for events
BEGIN_EVENT_TABLE(serverTab, wxPanel)
EVT_COMMAND (id_button_Start, wxEVT_COMMAND_BUTTON_CLICKED, serverTab::onButtonStart)
EVT_COMMAND (id_button_Stop, wxEVT_COMMAND_BUTTON_CLICKED, serverTab::onButtonStop)
EVT_COMMAND (id_button_ChangePort, wxEVT_COMMAND_BUTTON_CLICKED, serverTab::onButtonChangePort)
END_EVENT_TABLE()

IMPLEMENT_DYNAMIC_CLASS(serverTab, GRIPTab)

serverTab::serverTab(wxWindow *parent, const wxWindowID id, const wxPoint& pos, const wxSize& size, long style) :
  GRIPTab(parent, id, pos, size, style)
{
	// Create user interface
	wxSizer* sizerFull= new wxBoxSizer(wxHORIZONTAL);

	wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Execute"));
	wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Config"));

	wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
	wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);

	ss1BoxS->Add(new wxButton(this, id_button_Start, wxT("Start")), 0, wxALL, 1);
	ss1BoxS->Add(new wxButton(this, id_button_Stop, wxT("Stop")), 0, wxALL, 1); 
	ss2BoxS->Add(new wxButton(this, id_button_ChangePort, wxT("Change Port")), 0, wxALL, 1); 

	sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);

	SetSizer(sizerFull);

	// Set predefined port
	defPort = SERVER_PORT;
}

/// Gets triggered after a world is loaded
void serverTab::GRIPEventSceneLoaded() {
	state_ = SceneLoaded;
}

/// Start the server on the simulation start
void serverTab::GRIPEventSimulationStart() {
	startServer();
	state_ = SimulationStart;
}

/// Stop the server on the simulation stop
void serverTab::GRIPEventSimulationStop() {
	stopServer();
	state_ = SimulationStop;
}

/// Enable the server to start
void serverTab::onButtonStart(wxCommandEvent & _evt) {
	state_ = ServerStart;
}

/// Stop the server
void serverTab::onButtonStop(wxCommandEvent & _evt) {
	state_ = ServerStop;
}

/// Set the server port
void serverTab::onButtonChangePort(wxCommandEvent & _evt) {
	if (state_ == ServerStop)
	{
		defPort++;
	}
}

// Run when the server accept a connection
unsigned __stdcall Connection(void* arg) {
	serverTab* serverTabObj = (serverTab*)arg;
	boost::asio::io_service io_service;
	BoostServer::server s(io_service, SERVER_PORT);
	
	io_service.run();
}
// Start the server
void serverTab::startServer() {
	std::cout << "Start Server" << endl;
	try
	{
		/*
		GRIPThread threadServer(this);
		threadServer.CreateThread();
		threadServer.Entry();
		*/
		// Initialize
		
		
		hThread = (HANDLE)_beginthreadex(0, 0, Connection, this, 0, &threadID);
		//boost::thread t();
		//boost::bind(&boost::asio::io_service::run, &io_service);
	}
	catch (char *s) {
		cerr << s << endl;
	} 
	catch (std::string s) {
		cerr << s << endl;
	}
	catch (std::exception& e)
	{
		std::cerr << "Standard Exception: " << e.what() << "\n";
	}
	catch (...) {
		cerr << "unhandled exception\n";
	}
}

// Stop the server
void serverTab::stopServer() {
	std::cout << "Stop Server" << endl;
	try {
		// Signal to stop the thread.
		//threadStop = true;
		//boost::this_thread::interruption_point();
		//threadControl.interrupt();
		//threadControl.join();
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
