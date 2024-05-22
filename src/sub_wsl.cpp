#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1; // 콜백 함수에 사용
cv::VideoWriter video_writer; //OpenCV의 비디오 라이터 객체를 전역변수로 선언
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR); // 이미지를 COLOR로 받는다
    cv::imshow("frame",frame); // 화면에 이미지를 출력
    cv::waitKey(1); // imshow 할 때 필요
    if (!video_writer.isOpened()) // 이미지를 열 수 없으면 중지
    {
        RCLCPP_ERROR(node->get_logger(), "Could not open the output video file for write");
        return;
    }
    video_writer.write(frame); // 비디오 파일로 저장
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols); //디버깅 및 모니터링 목적으로 사용
}
int main(int argc, char* argv[]) // main 함수
{
    rclcpp::init(argc, argv); // ROS2 초기화
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl"); // 노드 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); // 통신의 품질을 설정 ( best_effort()가 가장 좋은 품질로 됨 )
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn; // 
    int frame_width = 640; // 영상의 가로길이
    int frame_height = 360; // 영상의 세로길이
    video_writer.open("output.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, cv::Size(frame_width, frame_height)); // " " 안의 이름으로 파일 저장
    fn = std::bind(mysub_callback, node, _1); // bind로 콜백 함수에 전달할 인자를 설정
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",qos_profile,fn); // 구독자를 생성하는 역할
    rclcpp::spin(node);
    rclcpp::shutdown();

    video_writer.release();
    
    return 0;
}
