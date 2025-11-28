#include "libtorch_digit_recognizer.h"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <filesystem>

LibtorchDigitRecognizer::LibtorchDigitRecognizer() {
    model_loaded_ = false;
}

bool LibtorchDigitRecognizer::loadModel(const std::string& model_path) {
    try {
        // 详细检查模型文件
        if (!std::filesystem::exists(model_path)) {
            RCLCPP_ERROR(rclcpp::get_logger("libtorch_digit_recognizer"), 
                        "模型文件不存在: %s", model_path.c_str());
            return false;
        }
        
        auto file_size = std::filesystem::file_size(model_path);
        RCLCPP_INFO(rclcpp::get_logger("libtorch_digit_recognizer"), 
                   "模型文件大小: %.2f MB", file_size / (1024.0 * 1024.0));

        // 加载模型
        RCLCPP_INFO(rclcpp::get_logger("libtorch_digit_recognizer"), 
                   "开始加载模型...");
        
        module_ = torch::jit::load(model_path);
        
        RCLCPP_INFO(rclcpp::get_logger("libtorch_digit_recognizer"), 
                   "模型加载成功，设置评估模式");
        
        module_.eval();
        
        // 检查模型方法
        auto methods = module_.get_methods();
        RCLCPP_INFO(rclcpp::get_logger("libtorch_digit_recognizer"), 
                   "模型方法数量: %zu", methods.size());
        
        for (const auto& method : methods) {
            RCLCPP_INFO(rclcpp::get_logger("libtorch_digit_recognizer"), 
                       "模型方法: %s", method.name().c_str());
        }
        
        model_loaded_ = true;
        RCLCPP_INFO(rclcpp::get_logger("libtorch_digit_recognizer"), 
                   "✅ Libtorch模型完全加载成功: %s", model_path.c_str());
        return true;
        
    } catch (const c10::Error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("libtorch_digit_recognizer"), 
                    "❌ 加载Libtorch模型失败: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("libtorch_digit_recognizer"), 
                    "❌ 加载模型时发生异常: %s", e.what());
        return false;
    }
}

cv::Mat LibtorchDigitRecognizer::preprocessImage(const cv::Mat& image) {
    cv::Mat processed;
    
    // 转换为灰度图
    if (image.channels() == 3) {
        cv::cvtColor(image, processed, cv::COLOR_BGR2GRAY);
    } else {
        processed = image.clone();
    }
    
    // 调整大小到128x128
    cv::resize(processed, processed, cv::Size(128, 128));
    
    // 归一化到[0,1]并标准化到[-1,1]
    processed.convertTo(processed, CV_32F, 1.0 / 255.0);
    processed = (processed - 0.5) / 0.5;
    
    return processed;
}

int LibtorchDigitRecognizer::recognizeDigit(const cv::Mat& warped_armor_image, double& confidence) {
    if (!model_loaded_) {
        RCLCPP_ERROR(rclcpp::get_logger("libtorch_digit_recognizer"), "模型未加载!");
        confidence = 0.0;
        return -1;
    }
    
    try {
        // 预处理图像
        cv::Mat processed = preprocessImage(warped_armor_image);
        
        // 转换为torch tensor
        torch::Tensor tensor_image = torch::from_blob(processed.data, {128, 128, 1}, torch::kFloat32);
        tensor_image = tensor_image.permute({2, 0, 1}); // [C, H, W] -> [1, 128, 128]
        tensor_image = tensor_image.unsqueeze(0); // 添加batch维度 [1, 1, 128, 128]
        
        // 修复：正确格式化tensor形状
        std::ostringstream input_shape_stream;
        input_shape_stream << tensor_image.sizes();
        RCLCPP_DEBUG(rclcpp::get_logger("libtorch_digit_recognizer"), 
                    "输入Tensor形状: %s", input_shape_stream.str().c_str());
        
        // 禁用梯度计算
        torch::NoGradGuard no_grad;
        
        // 推理 - 使用更健壮的方式
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(tensor_image);
        
        RCLCPP_DEBUG(rclcpp::get_logger("libtorch_digit_recognizer"), 
                    "开始模型推理...");
        
        at::Tensor output;
        
        // 尝试多种调用方式
        try {
            // 方式1: 直接调用forward方法
            output = module_.get_method("forward")(inputs).toTensor();
            RCLCPP_DEBUG(rclcpp::get_logger("libtorch_digit_recognizer"), 
                        "使用forward方法成功");
        } catch (const c10::Error& e) {
            RCLCPP_DEBUG(rclcpp::get_logger("libtorch_digit_recognizer"), 
                        "forward方法失败，尝试直接调用: %s", e.what());
            try {
                // 方式2: 直接调用模块
                output = module_.forward(inputs).toTensor();
                RCLCPP_DEBUG(rclcpp::get_logger("libtorch_digit_recognizer"), 
                            "使用forward成员函数成功");
            } catch (const c10::Error& e2) {
                RCLCPP_DEBUG(rclcpp::get_logger("libtorch_digit_recognizer"), 
                            "forward成员函数失败，尝试operator(): %s", e2.what());
                try {
                    // 方式3: 使用operator()
                    output = module_(inputs).toTensor();
                    RCLCPP_DEBUG(rclcpp::get_logger("libtorch_digit_recognizer"), 
                                "使用operator()成功");
                } catch (const c10::Error& e3) {
                    RCLCPP_ERROR(rclcpp::get_logger("libtorch_digit_recognizer"), 
                                "所有调用方式都失败: %s", e3.what());
                    confidence = 0.0;
                    return -1;
                }
            }
        }
        
        // 修复：正确格式化输出tensor形状
        std::ostringstream output_shape_stream;
        output_shape_stream << output.sizes();
        RCLCPP_DEBUG(rclcpp::get_logger("libtorch_digit_recognizer"), 
                    "输出Tensor形状: %s", output_shape_stream.str().c_str());
        
        // 应用softmax获取概率
        auto softmax_output = torch::softmax(output, 1);
        
        // 获取预测结果和置信度
        auto max_result = softmax_output.max(1);
        auto max_prob = std::get<0>(max_result);
        auto predicted_class = std::get<1>(max_result);
        
        confidence = max_prob.item<double>();
        int digit = predicted_class.item<int>();
        
        RCLCPP_INFO(rclcpp::get_logger("libtorch_digit_recognizer"), 
                   "预测数字: %d, 置信度: %.3f", digit, confidence);
        
        return digit;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("libtorch_digit_recognizer"), 
                    "❌ 数字识别失败: %s", e.what());
        confidence = 0.0;
        return -1;
    }
}
