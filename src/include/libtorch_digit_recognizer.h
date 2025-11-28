#pragma once

#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

class LibtorchDigitRecognizer {
public:
    LibtorchDigitRecognizer();
    ~LibtorchDigitRecognizer() = default;

    /**
     * @brief 加载训练好的模型
     * @param model_path 模型文件路径
     * @return 是否加载成功
     */
    bool loadModel(const std::string& model_path);

    /**
     * @brief 识别装甲板数字
     * @param warped_armor_image 矫正后的装甲板图像
     * @param confidence 输出置信度
     * @return 识别的数字 (0-9)，-1表示识别失败
     */
    int recognizeDigit(const cv::Mat& warped_armor_image, double& confidence);

    /**
     * @brief 检查模型是否已加载
     */
    bool isModelLoaded() const { return model_loaded_; }

private:
    torch::jit::script::Module module_;
    bool model_loaded_ = false;
    
    /**
     * @brief 图像预处理（与训练时一致）
     */
    cv::Mat preprocessImage(const cv::Mat& image);
};
