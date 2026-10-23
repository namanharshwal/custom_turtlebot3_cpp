// ==============================================================================
// Creator & Maintainer: Naman Harshwal (namanharshwal@gmail.com)
// Owner/Supervisor:     Pradeep Singh Chauhan
// Workspace:            psnh_turtlebot_cpp_ws
// Package:              serial
// License:              Apache License 2.0 (c) 2026 Naman Harshwal
//
// Description: Modern C++17 native serial port test utility.
// ==============================================================================

#include <iostream>
#include <string>
#include "serial/serial.h"

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Usage: cpp_serial_test <port_name like: /dev/ttyUSB0>" << std::endl;
    return 1;
  }

  std::string port = argv[1];
  try {
    serial::Serial my_serial(port, 115200, serial::Timeout::simpleTimeout(250));
    std::cout << "Successfully opened serial port: " << port << std::endl;
    my_serial.write("Testing.");
    std::string response = my_serial.read(8);
    std::cout << "Received: " << response << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "Serial exception: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
