#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

// 定义与PyTorch完全相同的CNN模型结构
struct SimpleDigitCNNImpl : torch::nn::Module {
    SimpleDigitCNNImpl() :
        // 卷积层
        conv1(register_module("conv1", torch::nn::Conv2d(torch::nn::Conv2dOptions(1, 32, 3).padding(1)))),
        conv2(register_module("conv2", torch::nn::Conv2d(torch::nn::Conv2dOptions(32, 64, 3).padding(1)))),
        conv3(register_module("conv3", torch::nn::Conv2d(torch::nn::Conv2dOptions(64, 128, 3).padding(1)))),
        // 全连接层
        fc1(register_module("fc1", torch::nn::Linear(128 * 16 * 16, 256))),
        fc2(register_module("fc2", torch::nn::Linear(256, 10))),
        // Dropout层
        dropout1(register_module("dropout1", torch::nn::Dropout(0.25))),
        dropout2(register_module("dropout2", torch::nn::Dropout(0.25))),
        dropout3(register_module("dropout3", torch::nn::Dropout(0.25))),
        dropout4(register_module("dropout4", torch::nn::Dropout(0.5)))
    {
        std::cout << "CNN模型初始化完成" << std::endl;
    }

    torch::Tensor forward(torch::Tensor x) {
        // 第一层卷积块
        x = torch::relu(conv1(x));
        x = torch::max_pool2d(x, 2);
        x = dropout1(x);

        // 第二层卷积块
        x = torch::relu(conv2(x));
        x = torch::max_pool2d(x, 2);
        x = dropout2(x);

        // 第三层卷积块
        x = torch::relu(conv3(x));
        x = torch::max_pool2d(x, 2);
        x = dropout3(x);

        // 全连接层
        x = x.view({x.size(0), -1}); // 展平
        x = torch::relu(fc1(x));
        x = dropout4(x);
        x = fc2(x);

        return x;
    }

    torch::nn::Conv2d conv1, conv2, conv3;
    torch::nn::Linear fc1, fc2;
    torch::nn::Dropout dropout1, dropout2, dropout3, dropout4;
};

TORCH_MODULE(SimpleDigitCNN);

// 自定义数据集
class DigitDataset : public torch::data::Dataset<DigitDataset> {
private:
    std::vector<std::string> image_paths_;
    std::vector<int64_t> labels_;

public:
    DigitDataset(const std::string& data_dir, const std::string& phase = "train") {
        std::string phase_dir = data_dir + "/" + phase;
        std::cout << "加载数据从: " << phase_dir << std::endl;
        
        for (int digit = 0; digit < 10; digit++) {
            std::string digit_dir = phase_dir + "/" + std::to_string(digit);
            if (!fs::exists(digit_dir)) {
                std::cout << "警告: 目录不存在 " << digit_dir << std::endl;
                continue;
            }
            
            int count = 0;
            for (const auto& entry : fs::directory_iterator(digit_dir)) {
                if (entry.is_regular_file()) {
                    std::string ext = entry.path().extension().string();
                    if (ext == ".png" || ext == ".jpg") {
                        image_paths_.push_back(entry.path().string());
                        labels_.push_back(digit);
                        count++;
                    }
                }
            }
            std::cout << "数字 " << digit << ": " << count << " 张图片" << std::endl;
        }
        std::cout << "总共加载: " << image_paths_.size() << " 张图片" << std::endl;
    }

    torch::data::Example<> get(size_t index) override {
        // 读取图像
        cv::Mat image = cv::imread(image_paths_[index], cv::IMREAD_GRAYSCALE);
        if (image.empty()) {
            std::cerr << "错误: 无法读取图像 " << image_paths_[index] << std::endl;
            // 返回黑色图像作为占位符
            image = cv::Mat::zeros(128, 128, CV_8UC1);
        }

        // 预处理 - 与PyTorch训练时一致
        cv::Mat resized, float_img;
        cv::resize(image, resized, cv::Size(128, 128));
        resized.convertTo(float_img, CV_32F, 1.0 / 255.0); // 归一化到 [0,1]
        float_img = (float_img - 0.5) / 0.5; // 标准化到 [-1,1]

        // 转换为torch tensor
        torch::Tensor tensor_image = torch::from_blob(float_img.data, {128, 128, 1}, torch::kFloat32);
        tensor_image = tensor_image.permute({2, 0, 1}); // [C, H, W]
        tensor_image = tensor_image.clone(); // 确保数据独立

        torch::Tensor tensor_label = torch::full({1}, labels_[index], torch::kInt64);

        return {tensor_image, tensor_label};
    }

    torch::optional<size_t> size() const override {
        return image_paths_.size();
    }
};

// 训练函数
void trainModel() {
    std::cout << "开始训练数字识别模型..." << std::endl;
    
    // 设置设备
    torch::Device device(torch::kCPU);
    std::cout << "使用设备: CPU" << std::endl;

    // 创建模型 - 注意这里直接实例化，不是shared_ptr
    SimpleDigitCNN model;
    model->to(device);  // 使用 -> 操作符

    // 优化器
    torch::optim::Adam optimizer(model->parameters(), torch::optim::AdamOptions(0.001));  // 使用 -> 操作符

    // 数据集路径
    std::string data_dir = "/home/lzs/Vision_Arena_2025_main/number_data";
    
    // 创建数据集
    auto train_dataset = DigitDataset(data_dir, "train")
        .map(torch::data::transforms::Stack<>());
    
    auto test_dataset = DigitDataset(data_dir, "test")
        .map(torch::data::transforms::Stack<>());

    // 数据加载器
    auto train_loader = torch::data::make_data_loader(
        std::move(train_dataset),
        torch::data::DataLoaderOptions().batch_size(32).workers(2)
    );

    auto test_loader = torch::data::make_data_loader(
        std::move(test_dataset), 
        torch::data::DataLoaderOptions().batch_size(32).workers(2)
    );

    // 训练循环
    int num_epochs = 20;
    double best_accuracy = 0.0;

    for (int epoch = 0; epoch < num_epochs; epoch++) {
        // 训练模式 - 使用 -> 操作符
        model->train();
        double running_loss = 0.0;
        int batch_count = 0;

        for (auto& batch : *train_loader) {
            auto data = batch.data.to(device);
            auto targets = batch.target.squeeze().to(device);

            // 前向传播 - 使用 -> 操作符
            auto output = model->forward(data);
            auto loss = torch::nn::functional::cross_entropy(output, targets);

            // 反向传播
            optimizer.zero_grad();
            loss.backward();
            optimizer.step();

            running_loss += loss.item<double>();
            batch_count++;
        }

        // 评估模式 - 使用 -> 操作符
        model->eval();
        int correct = 0;
        int total = 0;

        for (auto& batch : *test_loader) {
            auto data = batch.data.to(device);
            auto targets = batch.target.squeeze().to(device);

            // 前向传播 - 使用 -> 操作符
            auto output = model->forward(data);
            auto pred = output.argmax(1);
            correct += pred.eq(targets).sum().item<int>();
            total += targets.size(0);
        }

        double accuracy = 100.0 * correct / total;
        double avg_loss = running_loss / batch_count;

        std::cout << "Epoch [" << (epoch + 1) << "/" << num_epochs << "], "
                  << "Loss: " << avg_loss << ", "
                  << "准确率: " << accuracy << "%" << std::endl;

        // 保存最佳模型
        if (accuracy > best_accuracy) {
            best_accuracy = accuracy;
            torch::save(model, "digit_model_libtorch.pt");  // 直接保存模型对象
            std::cout << "✅ 保存最佳模型! 准确率: " << accuracy << "%" << std::endl;
        }
    }

    std::cout << "训练完成! 最佳准确率: " << best_accuracy << "%" << std::endl;
    std::cout << "模型保存为: digit_model_libtorch.pt" << std::endl;
}

int main() {
    try {
        std::cout << "=== Libtorch数字识别训练程序 ===" << std::endl;
        trainModel();
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
