#pragma once

#include "image_processor.h"
#include <rclcpp/rclcpp.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <vector>
#include <memory>

/**
 * @brief 装甲板检测类
 * 
 * 专门用于检测和识别装甲板，包含粗检测、精检测、坐标计算等特定功能
 */
class ArmorDetector {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针，用于日志记录和参数获取
     */
    ArmorDetector(rclcpp::Node* node);
    
    /**
     * @brief 默认析构函数
     */
    ~ArmorDetector() = default;

    /**
     * @brief 处理单帧图像，执行完整的装甲板检测流程
     * @param image 输入图像
     * @return 处理后的图像（包含可视化结果）
     */
    cv::Mat process_frame(const cv::Mat& image);

    /**
     * @brief 发布装甲板检测结果
     * @param armor_points 装甲板四个角点坐标
     */
    void publish_armor_result(const std::vector<cv::Point2f>& armor_points);

    /**
     * @brief 设置目标发布器
     * @param publisher ROS2发布器
     */
    void set_publisher(rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr publisher);

private:
    /**
     * @brief 粗检测模块：通过黑色区域检测初步定位装甲板位置
     * @param image 输入图像
     * @return 检测到的装甲板旋转矩形列表
     */
    std::vector<cv::RotatedRect> coarse_detection(const cv::Mat& image);

    /**
     * @brief 精检测模块：通过红色灯条检测精确识别装甲板角点
     * @param mask_image 掩码图像
     * @return 装甲板四个角点坐标
     */
    std::vector<cv::Point2f> fine_detection(const cv::Mat& mask_image);

    /**
     * @brief 坐标计算模块：根据灯条位置计算装甲板四个角点坐标
     * @param all_rectangles_points 所有矩形顶点
     * @return 装甲板四个角点坐标
     */
    std::vector<cv::Point2f> calculate_armor_points(const std::vector<cv::Point2f>& all_rectangles_points);

    /**
     * @brief 透视变换，提取装甲板数字区域
     * @param img 输入图像
     * @param armor_points 装甲板四个角点
     * @return 变换后的数字区域图像
     */
    cv::Mat WarpMat(const cv::Mat& img, const std::vector<cv::Point2f>& armor_points);

    /**
     * @brief 装甲板数字识别流程
     * @param armor_points 装甲板角点
     * @param display_image 显示图像
     * @return 识别到的数字ID
     */
    int Armor_ID(std::vector<cv::Point2f> armor_points, cv::Mat display_image);

    /**
     * @brief 数字识别核心方法
     * @param img 数字区域图像
     * @return 识别到的数字（0-9），-1表示识别失败
     */
    int num_recognition(cv::Mat img);

    /**
     * @brief 从文件加载数字模板
     * @param template_dir 模板目录路径
     */
    void load_templates_from_file(const std::string& template_dir = "./templates/");

    /**
     * @brief 预处理数字图像
     * @param number_roi 数字区域图像
     * @return 预处理后的图像
     */
    cv::Mat preprocess_number_image(const cv::Mat& number_roi);

    // ROS2相关
    rclcpp::Node* node_;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr target_pub_;
    
    // 图像处理工具
    std::unique_ptr<ImageProcessor> image_processor_;

    // 数字识别相关成员变量
    std::vector<cv::Mat> number_templates_;  // 存储0-9的模板
    bool templates_loaded_ = false;          // 模板是否已加载
    double match_threshold_ = 0.3;           // 匹配阈值
    
    // 检测参数
    float expand_ratio_ = 1.1f;
    double min_contour_area_ = 5.0;
    cv::Scalar black_lower_ = cv::Scalar(0, 0, 0);
    cv::Scalar black_upper_ = cv::Scalar(50, 50, 50);
    cv::Scalar red_lower1_ = cv::Scalar(0, 50, 50);
    cv::Scalar red_upper1_ = cv::Scalar(10, 255, 255);
    cv::Scalar red_lower2_ = cv::Scalar(170, 50, 50);
    cv::Scalar red_upper2_ = cv::Scalar(180, 255, 255);
    int morph_kernel_size_ = 3;
};
