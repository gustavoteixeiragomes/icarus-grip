#pragma comment(lib, "Ws2_32.lib")

#include "socket.h"

#include <iostream>

using namespace std;

int main() {

  try {
    SocketInterface::SocketClient server(SERVER_IP, SERVER_PORT);

    //server.SendLine("update()");
    server.SendLine("getCube()");
	server.SendLine("");

    while (1) {
      string l = server.ReceiveLine();
	  cout << "Buffer: " << l.size() << endl;
      if (l.empty()) break;
      cout << l << endl;
      cout.flush();
    }

  } 
  catch (const char* s) {
    cerr << s << endl;
  } 
  catch (std::string s) {
    cerr << s << endl;
  } 
  catch (...) {
    cerr << "unhandled exception\n";
  }
  system("pause");
  return 0;
}