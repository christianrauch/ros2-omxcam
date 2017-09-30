#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <omxcam.h>
extern "C" {
#include <libavcodec/avcodec.h>
}

void onBufferCallback(omxcam_buffer_t buffer);

class OMXVidPublisher : public rclcpp::Node {
public:
    OMXVidPublisher() :
        Node("omxcam"),
        pub_vid(create_publisher<sensor_msgs::msg::CompressedImage>("image/h264_frames")),
        pub_img(create_publisher<sensor_msgs::msg::Image>("image"))
    {
        omxcam_video_init(&settings);
        settings.camera.width = 640;
        settings.camera.height = 480;
        settings.camera.framerate = 90;

        settings.on_data = onBufferCallback;

        avcodec_register_all();
        //avcodec_register(AV_CODEC_ID_H264);
        codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if(!codec) {
            throw std::runtime_error("Could not find H264 codec!");
        }

        context = avcodec_alloc_context3(codec);

        if (codec->capabilities & AV_CODEC_CAP_TRUNCATED) {
            context->flags |= AV_CODEC_FLAG_TRUNCATED;
        }
        else {
            std::cerr << "truncated frames not available" << std::endl;
        }

        context->width = settings.camera.width;
        context->height = settings.camera.height;

        if (avcodec_open2(context, codec, NULL) < 0)
            throw std::runtime_error("Could not open codec!");

        frame = av_frame_alloc();

        av_init_packet(&avpkt);

        std::cout << "starting" << std::endl;
        omxcam_video_start(&settings, OMXCAM_CAPTURE_FOREVER);

        while(1) { };
        std::cout << "done" << std::endl;
    }

    ~OMXVidPublisher() {
        omxcam_video_stop();

        //avcodec_close(context);
        //av_free(context);
        avcodec_free_context(&context);
        av_frame_free(&frame);
    }

    void onData(omxcam_buffer_t buffer) {
        std::cout << "activated" << std::endl;
        //sensor_msgs::msg::CompressedImage vid_frame;
        //vid_frame.format = "H264";
        //vid_frame.data.insert(vid_frame.data.end(), buffer.data, buffer.data+buffer.length);

        //av_packet_from_data(&avpkt, buffer.data, buffer.length);
        //AVPacket avpkt;
        //av_init_packet(&avpkt);
        
        avpkt.data = buffer.data;
        avpkt.size = buffer.length;

        int got_frame;
        const int ret = avcodec_decode_video2(context, frame, &got_frame, &avpkt);
        //avcodec_send_packet(context, &avpkt);
        //int ret = avcodec_receive_frame(context, frame);
        std::cout << "decoded " << ret << std::endl;
        if(ret<0) {
            //throw std::runtime_error("len<0");
            std::cerr << "len<0" << std::endl;
        }

        //if(ret==0) {
        if(got_frame!=0) {
            // ok
            std::cout << "got frame" << std::endl;

            size_t frame_size = frame->width * frame->height * frame->linesize[0];

            sensor_msgs::msg::Image img_msg;
            img_msg.width = frame->width;
            img_msg.height = frame->height;
            img_msg.data.insert(img_msg.data.end(), *frame->data, *frame->data+frame_size);

            pub_img->publish(img_msg);
        }

        std::cout << "done onData" << std::endl;


        //pub_vid->publish(vid_frame);
    }

private:
    omxcam_video_settings_t settings;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_vid;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img;

    AVCodec* codec;
    AVCodecContext* context;
    AVFrame* frame;
    AVPacket avpkt;
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