#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <omxcam.h>

void onBufferCallback(omxcam_buffer_t buffer);

class OMXCamPublisher : public rclcpp::Node {
public:
    OMXCamPublisher() :
        Node("omxcam"),
        pub_img(create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed"))
    {
        omxcam_still_init (&settings);

        // void (*on_data)(omxcam_buffer_t buffer);
        //settings.on_data = std::bind(&OMXCamPublisher::onData, this, std::placeholders::_1);
        settings.on_data = onBufferCallback;

        thread = std::thread([this](){
            while(true) {
                captureImage();
            }
        });
    }

    ~OMXCamPublisher() {
        omxcam_still_stop();
    }

    int captureImage() {
        jpeg_buffer.clear();
        omxcam_still_start(&settings);

        sensor_msgs::msg::CompressedImage img;
        img.format = "jpeg";
        img.data = jpeg_buffer;

        pub_img->publish(img);
    }

    void onData(omxcam_buffer_t buffer) {
        jpeg_buffer.insert(jpeg_buffer.end(), buffer.data, buffer.data+buffer.length);
    }

private:
    omxcam_still_settings_t settings;
    std::thread thread;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_img;
    std::vector<uint8_t> jpeg_buffer;

};

OMXCamPublisher* omx_pub_ptr;
void onBufferCallback(omxcam_buffer_t buffer) {
  return omx_pub_ptr->onData(buffer);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<OMXCamPublisher>();
    omx_pub_ptr = node_ptr.get();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();

    return 0;
}