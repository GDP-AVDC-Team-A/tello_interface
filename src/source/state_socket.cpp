#include "tello_driver.hpp"

#include <regex>

using namespace std;

  // Goals:
  // * make the data useful by parsing all documented fields
  // * some future SDK version might introduce new field types, so don't parse undocumented fields
  // * send the raw string as well

  StateSocket::StateSocket(unsigned short data_port) : TelloSocket2(data_port)
  {
    buffer_ = std::vector<unsigned char>(1024);
    //listen();
  }

// Process a state packet from the drone, runs at 10Hz
  std::string StateSocket::process_packet(size_t r)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    // Split on ";" and ":" and generate a key:value map
    std::map<std::string, std::string> fields;
    std::string raw(buffer_.begin(), buffer_.begin() + r);
    std::regex re("([^:]+):([^;]+);");
    for (auto i = std::sregex_iterator(raw.begin(), raw.end(), re); i != std::sregex_iterator(); ++i) {
      auto match = *i;
      fields[match[1]] = match[2];
    }

    // First message?
    if (!receiving_) {
      receiving_ = true;
    }
    
    return raw;
  }
