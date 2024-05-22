#include "rclcpp/rclcpp.hpp" //ROS2 사용할 헤더파일
#include "sensor_msgs/msg/compressed_image.hpp" //이미지 처리를 위한 헤더 파일
#include "cv_bridge/cv_bridge.h" //브릿지 사용할 헤더 파일
#include "opencv2/opencv.hpp" //opencv를 사용할 헤더 파일
#include <memory> 
#include <chrono>

// GStreamer 파이프라인을 설정
std::string src = "nvarguscamerasrc sensor-id=0 ! \
	video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
	videoconvert ! video/x-raw, format=(string)BGR ! appsink"; 

int main(int argc, char * argv[]) //main 함수
{
    rclcpp::init(argc, argv); //ROS2 초기화
    auto node = std::make_shared<rclcpp::Node>("campub"); // ROS2 노드 선언 
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); //통신의 품질을 설정 ( best_effort()가 가장 좋은 품질로 됨 )
    auto mypub = node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile ); // 퍼블리셔 생성
    
    std_msgs::msg::Header hdr; //
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    rclcpp::WallRate loop_rate(40.0);

    cv::VideoCapture cap(src, cv::CAP_GSTREAMER); // 카메라를 사용할 객체
    if (!cap.isOpened()) { // 오류가 발생시 처리
        RCLCPP_ERROR(node->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return -1;
    }
    cv::Mat frame; // 영상을 담을 객체

    while(rclcpp::ok()) // 반복문
    {
        cap >> frame; // 영상을 프레임마다 전송
        if (frame.empty()) { RCLCPP_ERROR(node->get_logger(), "frame empty"); break;} // 오류 해결 문
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg(); // OpenCV의 이미지를 ROS 2 메시지 형식으로 변환
        mypub->publish(*msg); // 이미지 메시지를 발행
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
