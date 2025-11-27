#include "image_processor.h"
#include <rclcpp/rclcpp.hpp>

cv::Mat ImageProcessor::convert_image_msg(const sensor_msgs::msg::Image::SharedPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    if (msg->encoding == "rgb8" || msg->encoding == "R8G8B8") {
        cv::Mat image(msg->height, msg->width, CV_8UC3,
                      const_cast<unsigned char*>(msg->data.data()));
        cv::Mat bgr_image;
        cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
        cv_ptr = std::make_shared<cv_bridge::CvImage>();
        cv_ptr->header = msg->header;
        cv_ptr->encoding = "bgr8";
        cv_ptr->image = bgr_image;
    } else {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    return cv_ptr->image;
}

cv::Mat ImageProcessor::create_expanded_armor_mask(const cv::Mat& original_image, 
                                                 const cv::RotatedRect& armor_rect,
                                                 float expand_ratio) {
    cv::Mat mask_image = cv::Mat::zeros(original_image.size(), original_image.type());
    cv::RotatedRect expanded_rect = armor_rect;
    expanded_rect.size.width *= expand_ratio;
    expanded_rect.size.height *= expand_ratio;
    
    cv::Point2f vertices[4];
    expanded_rect.points(vertices);
    
    std::vector<cv::Point> polygon_points;
    for (int i = 0; i < 4; i++) {
        polygon_points.push_back(cv::Point(static_cast<int>(vertices[i].x), 
                                         static_cast<int>(vertices[i].y)));
    }
    
    cv::Mat polygon_mask = cv::Mat::zeros(original_image.size(), CV_8UC1);
    cv::fillConvexPoly(polygon_mask, polygon_points, cv::Scalar(255));
    original_image.copyTo(mask_image, polygon_mask);
    
    return mask_image;
}

cv::Mat ImageProcessor::draw_detection_results(const cv::Mat& image, 
                                             const std::vector<cv::Point2f>& armor_points,
                                             int armor_id) {
    cv::Mat display_image = image.clone();
    
    if (armor_points.size() == 4) {
        // 绘制绿色边界框
        for(int j = 0; j < 4; j++) {
            cv::line(display_image, armor_points[j], armor_points[(j+1)%4], 
                     bbox_color_, bbox_thickness_);
        }
        
        // 添加坐标文本标注
        for(int j = 0; j < 4; j++) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(1) << armor_points[j].x << "," << armor_points[j].y;
            std::string coord_text = ss.str();
            
            if(j == 0){
                cv::putText(display_image, coord_text, 
                           cv::Point(static_cast<int>(armor_points[j].x)-10, 
                                     static_cast<int>(armor_points[j].y)+10),
                           cv::FONT_HERSHEY_SIMPLEX, text_scale_, text_color_);
            } else if(j == 1){
                cv::putText(display_image, coord_text, 
                           cv::Point(static_cast<int>(armor_points[j].x)-10, 
                                     static_cast<int>(armor_points[j].y)-10),
                           cv::FONT_HERSHEY_SIMPLEX, text_scale_, text_color_);
            } else if(j == 2){
                cv::putText(display_image, coord_text, 
                           cv::Point(static_cast<int>(armor_points[j].x)+10, 
                                     static_cast<int>(armor_points[j].y)-10),
                           cv::FONT_HERSHEY_SIMPLEX, text_scale_, text_color_);
            } else if(j == 3){
                cv::putText(display_image, coord_text, 
                           cv::Point(static_cast<int>(armor_points[j].x)+10, 
                                     static_cast<int>(armor_points[j].y)+10),
                           cv::FONT_HERSHEY_SIMPLEX, text_scale_, text_color_);
            }
        }
        
        // 添加装甲板编号标签
        std::string armor_text = "Armor " + std::to_string(armor_id);
        cv::putText(display_image, armor_text,
                   cv::Point(static_cast<int>(armor_points[0].x), 
                             static_cast<int>(armor_points[0].y) - 20),
                   cv::FONT_HERSHEY_SIMPLEX, label_scale_, label_color_, 1);
    }
    
    return display_image;
}

cv::Mat ImageProcessor::bgr_to_hsv(const cv::Mat& bgr_image) {
    cv::Mat hsv_image;
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
    return hsv_image;
}

cv::Mat ImageProcessor::create_hsv_mask(const cv::Mat& hsv_image, 
                                      const cv::Scalar& lower_bound, 
                                      const cv::Scalar& upper_bound) {
    cv::Mat mask;
    cv::inRange(hsv_image, lower_bound, upper_bound, mask);
    return mask;
}

cv::Mat ImageProcessor::morphological_operation(const cv::Mat& image, 
                                              int operation, 
                                              int kernel_size, 
                                              int kernel_type) {
    cv::Mat result;
    cv::Mat kernel = cv::getStructuringElement(kernel_type, cv::Size(kernel_size, kernel_size));
    cv::morphologyEx(image, result, operation, kernel);
    return result;
}

std::vector<std::vector<cv::Point>> ImageProcessor::find_contours(const cv::Mat& binary_image, 
                                                                int mode, 
                                                                int method) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_image, contours, hierarchy, mode, method);
    return contours;
}

cv::RotatedRect ImageProcessor::get_min_area_rect(const std::vector<cv::Point>& contour) {
    return cv::minAreaRect(contour);
}

double ImageProcessor::calculate_contour_area(const std::vector<cv::Point>& contour) {
    return cv::contourArea(contour);
}
