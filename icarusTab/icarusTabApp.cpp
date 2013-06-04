/**
 * @file pushDemoTabApp.h
 * @brief Creates application for pushDemoTab
 * @author A. Huaman Q.
 */
#include "GRIPApp.h"
#include "icarusTab.h"

extern wxNotebook* tabView;

class icarusTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new icarusTab(tabView), wxT("ICARUS Tab"));
	}
};

IMPLEMENT_APP(icarusTabApp)
