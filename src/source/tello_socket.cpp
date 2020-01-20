#include "tello_driver.hpp"

using namespace std;

  void TelloSocket2::listen()
  {
    
    thread_ = std::thread(
      [this]()
      {
        for (;;) {
          size_t r = socket_.receive(asio::buffer(buffer_));
          process_packet(r);
        }
      });
      return;
  }

  std::string TelloSocket2::listen_once()
  {
      size_t r = socket_.receive(asio::buffer(buffer_));
      return process_packet(r);
  }

  bool TelloSocket2::receiving()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return receiving_;
  }

  void TelloSocket2::timeout()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    receiving_ = false;
  }

