#include "armor_detector.h"
#include <rclcpp/rclcpp.hpp>

ArmorDetector::ArmorDetector(rclcpp::Node* node) : node_(node) {
    image_processor_ = std::make_unique<ImageProcessor>();
}

void ArmorDetector::set_publisher(rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr publisher) {
    target_pub_ = publisher;
}

cv::Mat ArmorDetector::process_frame(const cv::Mat& image) {
    cv::Mat display_image = image.clone();
    
    if (image.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Received empty image");
        return display_image;
    }

    // 步骤1：粗检测 - 通过黑色区域检测初步定位装甲板位置
    auto armor_rects = coarse_detection(image);
    
    RCLCPP_INFO(node_->get_logger(), "检测到 %zu 个可能的装甲板", armor_rects.size());

    // 步骤2：对每个检测到的装甲板进行精细处理
    for (size_t i = 0; i < armor_rects.size(); i++) {
        // 创建装甲板区域掩码
        cv::Mat armor_mask = image_processor_->create_expanded_armor_mask(image, armor_rects[i], expand_ratio_);
        
        if (!armor_mask.empty()) {
            RCLCPP_INFO(node_->get_logger(), "对装甲板 %zu 进行坐标识别", i);
            
            // 步骤3：精检测 - 通过红色灯条检测精确识别装甲板角点
            std::vector<cv::Point2f> armor_points = fine_detection(armor_mask);
            
            if (!armor_points.empty()) {
                // 步骤4：可视化 - 在图像上绘制检测结果
                display_image = image_processor_->draw_detection_results(display_image, armor_points, i);
                
                // 步骤5：发布检测结果
                publish_armor_result(armor_points);
                
            } else {
                RCLCPP_WARN(node_->get_logger(), "装甲板 %zu 的坐标提取失败", i);
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "装甲板 %zu 的掩码图像创建失败", i);
        }
    }
    
    return display_image;
}

void ArmorDetector::publish_armor_result(const std::vector<cv::Point2f>& armor_points) {
    if (!target_pub_) {
        RCLCPP_ERROR(node_->get_logger(), "发布器未设置，无法发布结果");
        return;
    }
    
    if (armor_points.size() != 4) {
        RCLCPP_ERROR(node_->get_logger(), "坐标点数错误: %zu，需要4个点", armor_points.size());
        return;
    }
    
    auto msg_object = std::make_shared<referee_pkg::msg::MultiObject>();
    msg_object->header.stamp = node_->now();
    msg_object->header.frame_id = "camera_frame";

    referee_pkg::msg::Object obj;
    obj.target_type = "armor_red_1";
    
    for (int j = 0; j < 4; j++) {
        geometry_msgs::msg::Point corner;
        corner.x = armor_points[j].x;
        corner.y = armor_points[j].y;
        corner.z = 0.0;
        obj.corners.push_back(corner);
    }
    
    msg_object->objects.push_back(obj);
    target_pub_->publish(*msg_object);
    
    RCLCPP_INFO(node_->get_logger(), "发布装甲板坐标: (%.1f,%.1f),(%.1f,%.1f),(%.1f,%.1f),(%.1f,%.1f)", 
                armor_points[0].x, armor_points[0].y,
                armor_points[1].x, armor_points[1].y,
                armor_points[2].x, armor_points[2].y,
                armor_points[3].x, armor_points[3].y);
}

std::vector<cv::RotatedRect> ArmorDetector::coarse_detection(const cv::Mat& image) {
    std::vector<cv::RotatedRect> armor_rects;
    
    // 转换为HSV颜色空间
    cv::Mat hsv = image_processor_->bgr_to_hsv(image);
    
    // 创建黑色区域掩码
    cv::Mat mask = image_processor_->create_hsv_mask(hsv, black_lower_, black_upper_);
    
    // 形态学操作优化掩码
    mask = image_processor_->morphological_operation(mask, cv::MORPH_CLOSE, morph_kernel_size_);
    
    // 轮廓检测
    auto contours = image_processor_->find_contours(mask, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    RCLCPP_INFO(node_->get_logger(), "找到%zu个装甲板", contours.size());
    
    // 过滤有效轮廓并获取旋转矩形
    for (size_t i = 0; i < contours.size(); i++) {
        double area = image_processor_->calculate_contour_area(contours[i]);
        if (area > min_contour_area_) {
            cv::RotatedRect minRect = image_processor_->get_min_area_rect(contours[i]);
            armor_rects.push_back(minRect);
        }
    }
    
    return armor_rects;
}

std::vector<cv::Point2f> ArmorDetector::fine_detection(const cv::Mat& mask_image) {
    std::vector<cv::Point2f> all_rectangles_points;
    
    // 转换为HSV颜色空间
    cv::Mat hsv = image_processor_->bgr_to_hsv(mask_image);
    
    // 创建红色区域掩码
    cv::Mat mask1 = image_processor_->create_hsv_mask(hsv, red_lower1_, red_upper1_);
    cv::Mat mask2 = image_processor_->create_hsv_mask(hsv, red_lower2_, red_upper2_);
    cv::Mat mask = mask1 | mask2;
    
    // 形态学操作
    mask = image_processor_->morphological_operation(mask, cv::MORPH_CLOSE, morph_kernel_size_);
    
    // 轮廓检测 - 尝试多种检测方法
    auto contours = image_processor_->find_contours(mask, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() == 0) {
        contours = image_processor_->find_contours(mask, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    }
    if (contours.size() == 0) {
        contours = image_processor_->find_contours(mask, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    }
    
    RCLCPP_INFO(node_->get_logger(), "在掩码图像中找到 %zu 个轮廓(红色灯条)", contours.size());
    
    // 处理每个检测到的轮廓
    for (size_t i = 0; i < contours.size(); i++) {
        double area = image_processor_->calculate_contour_area(contours[i]);
        if (area > min_contour_area_) {
            cv::RotatedRect minRect = image_processor_->get_min_area_rect(contours[i]);
            cv::Point2f vertices[4];
            minRect.points(vertices);
            
            // 存储矩形顶点
            for (int k = 0; k < 4; k++) {
                all_rectangles_points.push_back(vertices[k]);
            }
        }
    }
    
    // 坐标计算
    return calculate_armor_points(all_rectangles_points);
}

std::vector<cv::Point2f> ArmorDetector::calculate_armor_points(const std::vector<cv::Point2f>& all_rectangles_points) {
    std::vector<cv::Point2f> armor_points;
    
    if(all_rectangles_points.size() >= 8){
        // 按Y坐标降序排序
        auto sorted_points = all_rectangles_points;
        std::sort(sorted_points.begin(), sorted_points.end(),
            [](const cv::Point2f& a, const cv::Point2f& b){
                return a.y >= b.y;
            });
            
        // 根据顶点分布情况选择不同的角点计算策略
        if(sorted_points[0].y > sorted_points[1].y){
            // 策略1：处理垂直分布的灯条
            std::vector<cv::Point2f> tool_points;
            tool_points.push_back(sorted_points[0]);
            tool_points.push_back(sorted_points[3]);
            tool_points.push_back(sorted_points[4]);
            tool_points.push_back(sorted_points[7]);
            
            if(tool_points[3].x < tool_points[1].x){
                armor_points.push_back(sorted_points[0]);//左下
                armor_points.push_back(sorted_points[1]);//右下
                armor_points.push_back(sorted_points[3]);//右上
                armor_points.push_back(sorted_points[2]);//左上
            } else {
                armor_points.push_back(sorted_points[1]);//左下
                armor_points.push_back(sorted_points[0]);//右下
                armor_points.push_back(sorted_points[2]);//右上
                armor_points.push_back(sorted_points[3]);//左上
            }
        } else {
            // 策略2：处理水平分布的灯条
            std::vector<cv::Point2f> first_four(sorted_points.begin(), sorted_points.begin() + 4);
            std::vector<cv::Point2f> last_four(sorted_points.end() - 4, sorted_points.end());
            
            // 按X坐标排序
            std::sort(first_four.begin(), first_four.end(),
            [](const cv::Point2f& a, const cv::Point2f& b){
                return a.x < b.x;
            });
            
            std::sort(last_four.begin(), last_four.end(),
            [](const cv::Point2f& a, const cv::Point2f& b){
                return a.x < b.x;
            });
            
            // 构建装甲板四个角点
            armor_points.push_back(first_four[0]);//左下
            armor_points.push_back(first_four[3]);//右下
            armor_points.push_back(last_four[3]);//右上
            armor_points.push_back(last_four[0]);//左上
        }
        RCLCPP_INFO(node_->get_logger(), "成功提取装甲板坐标，共 %zu 个点", armor_points.size());
    } else {
        RCLCPP_WARN(node_->get_logger(), "掩码图像中点数不足，无法提取坐标");
    }
    
    // 输出坐标信息
    if (armor_points.size() >= 4) {
        RCLCPP_INFO(node_->get_logger(), 
                "点1(%.1f,%.1f),点2(%.1f,%.1f),点3(%.1f,%.1f),点4(%.1f,%.1f)", 
                armor_points[0].x, armor_points[0].y,
                armor_points[1].x, armor_points[1].y,
                armor_points[2].x, armor_points[2].y,
                armor_points[3].x, armor_points[3].y);
    }
    return armor_points;
}

cv::Mat ArmorDetector::WarpMat(const cv::Mat& img,const std::vector<cv::Point2f>& light_points){
    if (light_points.size() != 4) {
        RCLCPP_ERROR(node_->get_logger(), "未能提取出数字影像");
        return cv::Mat();
    }

    cv::Mat matrix, imgWarp;
	float w = 128, h = 128;

    cv::Point2f src[4] ;
    for(int i=0;i<4;i++){
        src[i]=light_points[i];
    }
	cv::Point2f dst[4] = { {-0.4375*w,0.75*h},{0.4375*w,0.75*h},{0.4375*w,0.25*h},{-0.4375*w,0.25*h} };

    matrix = getPerspectiveTransform(src, dst);
	warpPerspective(img, imgWarp, matrix, cv::Point(w, h));

    return imgWarp;
}
