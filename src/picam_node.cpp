#include <rclcpp/rclcpp.hpp>

#include <omxcam.h>

class OMXCamPublisher : public rclcpp::Node {
public:
    OMXCamPublisher() : Node("omxcam") {
        omxcam_still_init (&settings);
    }

private:
    omxcam_still_settings_t settings;

};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OMXCamPublisher>());
    rclcpp::shutdown();

    return 0;
}