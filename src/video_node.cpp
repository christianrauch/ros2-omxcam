#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <omxcam.h>

void onBufferCallback(omxcam_buffer_t buffer);

class OMXVidPublisher : public rclcpp::Node {
public:
    OMXVidPublisher() :
        Node("omxcam"),
        pub_vid(create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed"))
    {
        omxcam_video_init(&settings);
        settings.camera.width = 640;
        settings.camera.height = 480;
        settings.camera.framerate = 90;

        settings.on_data = onBufferCallback;

        omxcam_video_start(&settings, OMXCAM_CAPTURE_FOREVER);
    }

    ~OMXVidPublisher() {
        omxcam_video_stop();
    }

    void onData(omxcam_buffer_t buffer) {
        sensor_msgs::msg::CompressedImage vid_frame;
        vid_frame.format = "H264";
        vid_frame.data.insert(vid_frame.data.end(), buffer.data, buffer.data+buffer.length);
        pub_vid->publish(vid_frame);
    }

private:
    omxcam_video_settings_t settings;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_vid;
};

OMXVidPublisher* omx_pub_ptr;
void onBufferCallback(omxcam_buffer_t buffer) {
  return omx_pub_ptr->onData(buffer);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<OMXVidPublisher>();
    omx_pub_ptr = node_ptr.get();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();

    return 0;
}