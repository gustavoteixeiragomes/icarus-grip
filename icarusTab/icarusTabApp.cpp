/**
 * @file pushDemoTabApp.h
 * @brief Creates application for pushDemoTab
 * @author A. Huaman Q.
 */
#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN 1
#endif

#include "GRIPApp.h"
#include "BoostServer.h"
#include "icarusTab.h"

extern wxNotebook* tabView;
HANDLE hThread;
unsigned threadID;

// Run when the server accept a connection
unsigned __stdcall Connection(void* arg) {
	//serverTab* serverTabObj = (serverTab*)arg;
	boost::asio::io_service io_service;
	BoostServer::server s(io_service, SERVER_PORT);
	//serverTabObj->serverRunning_ = 1;
	// WIN32 conflict wxWidgets with boost::thread
	//boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service));
	io_service.run();
	return 0;
}

class icarusTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new icarusTab(tabView), wxT("ICARUS Tab"));
		//tabView->AddPage(new serverTab(tabView), wxT("Server Tab"));
		std::cout << "Start Server" << endl;
		try {
			hThread = (HANDLE)_beginthreadex(0, 0, Connection, this, 0, &threadID);	
		}
		catch (char *s) {
			cerr << s << endl;
		} 
		catch (std::string s) {
			cerr << s << endl;
		}
		catch (std::exception& e) {
			std::cerr << "Standard Exception: " << e.what() << "\n";
		}
		catch (...) {
			cerr << "unhandled exception\n";
		}
	}
	virtual ~icarusTabApp() {
		CloseHandle( hThread );
	};
};

IMPLEMENT_APP(icarusTabApp)
