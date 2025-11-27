#pragma once

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>
#include <iomanip>

/**
 * @brief 通用图像处理工具类
 * 
 * 提供基础的图像处理功能
 */
class ImageProcessor {
public:
    /**
     * @brief 默认构造函数
     */
    ImageProcessor() = default;
    
    /**
     * @brief 默认析构函数
     */
    ~ImageProcessor() = default;

    /**
     * @brief 将ROS图像消息转换为OpenCV格式
     * @param msg ROS图像消息
     * @return 转换后的BGR格式图像
     */
    cv::Mat convert_image_msg(const sensor_msgs::msg::Image::SharedPtr& msg);

    /**
     * @brief 创建扩展的掩码图像
     * @param original_image 原始图像
     * @param armor_rect 装甲板旋转矩形
     * @param expand_ratio 扩展比例，默认1.1
     * @return 掩码图像
     */
    cv::Mat create_expanded_armor_mask(const cv::Mat& original_image, 
                                     const cv::RotatedRect& armor_rect,
                                     float expand_ratio = 1.1f);

    /**
     * @brief 在图像上绘制检测结果
     * @param image 原始图像
     * @param armor_points 装甲板四个角点
     * @param armor_id 装甲板ID，默认0
     * @return 绘制后的图像
     */
    cv::Mat draw_detection_results(const cv::Mat& image, 
                                 const std::vector<cv::Point2f>& armor_points,
                                 int armor_id = 0);

    /**
     * @brief 颜色空间转换：BGR转HSV
     * @param bgr_image BGR图像
     * @return HSV图像
     */
    cv::Mat bgr_to_hsv(const cv::Mat& bgr_image);

    /**
     * @brief 根据HSV范围创建掩码
     * @param hsv_image HSV图像
     * @param lower_bound 下限
     * @param upper_bound 上限
     * @return 二值掩码
     */
    cv::Mat create_hsv_mask(const cv::Mat& hsv_image, 
                          const cv::Scalar& lower_bound, 
                          const cv::Scalar& upper_bound);

    /**
     * @brief 形态学操作
     * @param image 输入图像
     * @param operation 形态学操作类型
     * @param kernel_size 核大小
     * @param kernel_type 核类型，默认矩形
     * @return 处理后的图像
     */
    cv::Mat morphological_operation(const cv::Mat& image, 
                                  int operation, 
                                  int kernel_size, 
                                  int kernel_type = cv::MORPH_RECT);

    /**
     * @brief 轮廓检测
     * @param binary_image 二值图像
     * @param mode 轮廓检索模式
     * @param method 轮廓近似方法
     * @return 轮廓列表
     */
    std::vector<std::vector<cv::Point>> find_contours(const cv::Mat& binary_image, 
                                                    int mode = cv::RETR_EXTERNAL, 
                                                    int method = cv::CHAIN_APPROX_SIMPLE);

    /**
     * @brief 获取轮廓的最小外接矩形
     * @param contour 轮廓点集
     * @return 旋转矩形
     */
    cv::RotatedRect get_min_area_rect(const std::vector<cv::Point>& contour);

    /**
     * @brief 计算轮廓面积
     * @param contour 轮廓点集
     * @return 面积
     */
    double calculate_contour_area(const std::vector<cv::Point>& contour);

private:
    // 可视化参数
    cv::Scalar bbox_color_ = cv::Scalar(0, 255, 0);      // 边界框颜色
    cv::Scalar text_color_ = cv::Scalar(255, 255, 255);  // 文本颜色
    cv::Scalar label_color_ = cv::Scalar(255, 255, 0);   // 标签颜色
    int bbox_thickness_ = 2;                             // 边界框线宽
    double text_scale_ = 0.3;                            // 文本大小
    double label_scale_ = 0.5;                           // 标签大小
};
