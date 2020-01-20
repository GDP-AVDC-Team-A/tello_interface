#include <libavutil/frame.h>
#include <opencv2/highgui.hpp>
#include <std_msgs/Header.h>

#include "camera_calibration_parsers/parse.h"
#include "tello_driver.hpp"

using namespace std;
  VideoSocket::VideoSocket(unsigned short video_port, ros::Publisher pub) :
    TelloSocket2(video_port), camera_pub(pub)
  {
    buffer_ = std::vector<unsigned char>(2048);
    seq_buffer_ = std::vector<unsigned char>(65536);
    listen();
  }

  // Process a video packet from the drone
  std::string VideoSocket::process_packet(size_t r)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!receiving_) {
      // First packet
      receiving_ = true;
      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
    }

    if (seq_buffer_next_ + r >= seq_buffer_.size()) {
      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
      return "";
    }

    std::copy(buffer_.begin(), buffer_.begin() + r, seq_buffer_.begin() + seq_buffer_next_);
    seq_buffer_next_ += r;
    seq_buffer_num_packets_++;

    // If packet < 1460 B last packet
        if (r < 1460) {
      decode_frames();

      seq_buffer_next_ = 0;
      seq_buffer_num_packets_ = 0;
    }

    return "";
  }

  // Decode frames
  void VideoSocket::decode_frames()
  {
    size_t next = 0;

    try {
      while (next < seq_buffer_next_) {
        // Parse H264
        ssize_t consumed = decoder_.parse(seq_buffer_.data() + next, seq_buffer_next_ - next);

        if (decoder_.is_frame_available()) {
            // Decode the frame
            const AVFrame &frame = decoder_.decode_frame();

            // YUV420P to BGR24
            int size = converter_.predict_size(frame.width, frame.height);
            unsigned char bgr24[size];
            converter_.convert(frame, bgr24);

            cv::Mat mat{frame.height, frame.width, CV_8UC3, bgr24};

            std_msgs::Header header{};
            header.frame_id = "camera_frame";
            ros::Time current_timestamp = ros::Time::now();
            header.stamp = current_timestamp;
            cv_bridge::CvImage image{header, sensor_msgs::image_encodings::BGR8, mat};
            camera_pub.publish(image.toImageMsg());
        }

        next += consumed;
      }
    }
    catch (std::runtime_error e) {
    }
  }
