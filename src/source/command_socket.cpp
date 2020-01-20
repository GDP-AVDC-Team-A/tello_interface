#include "tello_driver.hpp"

using namespace std;

  CommandSocket::CommandSocket(std::string drone_ip,
                               unsigned short drone_port, unsigned short command_port) :
    TelloSocket2(command_port),
    remote_endpoint_(asio::ip::address_v4::from_string(drone_ip), drone_port)
  {
    buffer_ = std::vector<unsigned char>(1024);
    //listen();
  }

  void CommandSocket::timeout()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    receiving_ = false;

    if (waiting_) {
      complete_command("error: command timed out");
    }
  }

  bool CommandSocket::waiting()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return waiting_;
  }

  void CommandSocket::initiate_command(std::string command, bool respond)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!waiting_) {
      socket_.send_to(asio::buffer(command), remote_endpoint_);

      // Wait for a response for all commands except "rc"
      if (command.rfind("rc", 0) != 0) {
        respond_ = respond;
        waiting_ = true;
        complete_command(command);
      }
    }
  }

  void CommandSocket::complete_command(std::string str)
  {
    if (respond_) {
        
    }
    waiting_ = false;
  }

  std::string CommandSocket::process_packet(size_t r)
  {
    std::lock_guard<std::mutex> lock(mtx_);


    if (!receiving_) {
      receiving_ = true;
    }

    std::string str = std::string(buffer_.begin(), buffer_.begin() + r);
    if (waiting_) {
      complete_command(str);
    } else {
        
    }
    return "test";
  }

