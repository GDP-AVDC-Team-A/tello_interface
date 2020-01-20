#ifndef TELLO_DRIVER_H
#define TELLO_DRIVER_H

#include <asio.hpp>
#include <thread>
#include <mutex>
#include "h264decoder.hpp"
#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"

using asio::ip::udp;

#define TELLO_CLIENT_ADDRESS "192.168.10.1"
#define TELLO_SERVER_ADDRESS "192.168.10.2"
#define TELLO_COMMAND_PORT 8889
#define PC_COMMAND_PORT 38065
#define TELLO_STATE_PORT 8890
#define TELLO_CAMERA_PORT 11111

class CommandSocket;
class StateSocket;
class VideoSocket;


//=====================================================================================
// Abstract socket
//=====================================================================================

class TelloSocket2
{
public:

  TelloSocket2(unsigned short port) :
    socket_(io_service_, udp::endpoint(udp::v4(), port)) {}

  bool receiving();
  virtual void timeout();
  void listen();
  std::string listen_once();

protected:

  virtual std::string process_packet(size_t r) = 0;

  asio::io_service io_service_;         // Manages IO for this socket
  udp::socket socket_;                  // The socket
  std::thread thread_;                  // Each socket receives on it's own thread
  std::mutex mtx_;                      // All public calls must be guarded
  bool receiving_ = false;              // Are we receiving packets on this socket?
  std::vector<unsigned char> buffer_;   // Packet buffer
};

//=====================================================================================
// State socket
//=====================================================================================

class StateSocket : public TelloSocket2
{
public:

  StateSocket(unsigned short data_port);

private:

  std::string process_packet(size_t r) override;
};

//=====================================================================================
// Command socket
//=====================================================================================

class CommandSocket : public TelloSocket2
{
public:

  CommandSocket(std::string drone_ip, unsigned short drone_port, unsigned short command_port);

  void timeout() override;
  bool waiting();
  void initiate_command(std::string command, bool respond);

private:

  std::string process_packet(size_t r) override;
  void complete_command(std::string str);

  udp::endpoint remote_endpoint_;

  bool respond_;            // Send response on tello_response_pub_
  bool waiting_ = false;    // Are we waiting for a response?
};

//=====================================================================================
// Video socket
//=====================================================================================

class VideoSocket : public TelloSocket2
{
public:

  VideoSocket(unsigned short video_port, ros::Publisher pub);

private:

  std::string process_packet(size_t r) override;
  void decode_frames();

  std::vector<unsigned char> seq_buffer_;   // Collect video packets into a larger sequence
  size_t seq_buffer_next_ = 0;              // Next available spot in the sequence buffer
  int seq_buffer_num_packets_ = 0;          // How many packets we've collected, for debugging

  H264Decoder decoder_;                     // Decodes h264
  ConverterRGB24 converter_;                // Converts pixels from YUV420P to BGR24

  ros::Publisher camera_pub;
};

#endif