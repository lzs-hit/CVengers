#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <memory>
#include <opencv2/opencv.hpp>

#include "image_processor.h"
#include "armor_detector.h"

/**
 * @brief 视觉处理主节点
 * 
 * 负责ROS2通信架构，可以集成多种检测任务
 */
class ArmorNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    ArmorNode() : Node("armor_node") {
        RCLCPP_INFO(this->get_logger(), "初始化装甲板检测……");

        // 初始化图像处理工具
        image_processor_ = std::make_unique<ImageProcessor>();
        
        // 初始化装甲板检测器
        armor_detector_ = std::make_unique<ArmorDetector>(this);

        // 创建图像订阅器
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ArmorNode::image_callback, this, std::placeholders::_1));

        // 创建目标发布器
        auto target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
            "/vision/target", 10);
        armor_detector_->set_publisher(target_pub);

        // 初始化显示窗口
        cv::namedWindow("Raw Image", cv::WINDOW_AUTOSIZE);

        RCLCPP_INFO(this->get_logger(), "正在检测装甲板……");
    }

    /**
     * @brief 析构函数
     */
    ~ArmorNode() {
        cv::destroyWindow("Raw Image");
    }

private:
    /**
     * @brief 图像回调函数
     * @param msg ROS图像消息
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 转换图像消息为OpenCV格式
            auto image = image_processor_->convert_image_msg(msg);
            
            // 使用装甲板检测器处理图像
            auto display_image = armor_detector_->process_frame(image);
            
            // 显示处理结果
            cv::imshow("Raw Image", display_image);
            cv::waitKey(1);

        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    // 图像处理工具
    std::unique_ptr<ImageProcessor> image_processor_;
    
    // 检测器（可以扩展为多个检测器）
    std::unique_ptr<ArmorDetector> armor_detector_;
    
    // ROS2通信组件
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    // 帧计数器
    int frame_count_ = 0;
};

/**
 * @brief 主函数
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorNode>();
    RCLCPP_INFO(node->get_logger(), "Starting ArmorNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
