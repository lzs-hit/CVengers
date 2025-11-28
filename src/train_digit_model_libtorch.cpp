#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

// 定义CNN模型结构
struct SimpleDigitCNNImpl : torch::nn::Module {
    SimpleDigitCNNImpl() :
        conv1(register_module("conv1", torch::nn::Conv2d(torch::nn::Conv2dOptions(1, 32, 3).padding(1)))),
        conv2(register_module("conv2", torch::nn::Conv2d(torch::nn::Conv2dOptions(32, 64, 3).padding(1)))),
        conv3(register_module("conv3", torch::nn::Conv2d(torch::nn::Conv2dOptions(64, 128, 3).padding(1)))),
        fc1(register_module("fc1", torch::nn::Linear(128 * 16 * 16, 256))),
        fc2(register_module("fc2", torch::nn::Linear(256, 10))),
        dropout1(register_module("dropout1", torch::nn::Dropout(0.25))),
        dropout2(register_module("dropout2", torch::nn::Dropout(0.25))),
        dropout3(register_module("dropout3", torch::nn::Dropout(0.25))),
        dropout4(register_module("dropout4", torch::nn::Dropout(0.5)))
    {}

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
        x = x.view({x.size(0), -1});
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

// 自定义数据集类
class DigitDataset : public torch::data::Dataset<DigitDataset> {
public:
    DigitDataset(const std::string& data_dir, bool is_train = true) {
        std::string phase_dir = is_train ? (data_dir + "/train") : (data_dir + "/test");
        
        // 检查目录是否存在
        if (!std::filesystem::exists(phase_dir)) {
            std::cerr << "错误: 数据目录不存在: " << phase_dir << std::endl;
            return;
        }

        for (int digit = 0; digit < 10; digit++) {
            std::string digit_dir = phase_dir + "/" + std::to_string(digit);
            
            if (!std::filesystem::exists(digit_dir)) {
                std::cerr << "警告: 数字目录不存在: " << digit_dir << std::endl;
                continue;
            }

            int count = 0;
            for (const auto& entry : std::filesystem::directory_iterator(digit_dir)) {
                if (entry.is_regular_file() && 
                    (entry.path().extension() == ".png" || 
                     entry.path().extension() == ".jpg" ||
                     entry.path().extension() == ".jpeg")) {
                    image_paths_.push_back(entry.path().string());
                    labels_.push_back(digit);
                    count++;
                }
            }
            std::cout << "数字 " << digit << ": " << count << " 张图片" << std::endl;
        }
        
        std::cout << "总共加载: " << image_paths_.size() << " 张图片" << std::endl;
    }

    torch::data::Example<> get(size_t index) override {
        if (index >= image_paths_.size()) {
            throw std::out_of_range("索引超出范围");
        }

        std::string file_path = image_paths_[index];
        int label = labels_[index];

        // 使用OpenCV读取图像
        cv::Mat img = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            std::cerr << "错误: 无法加载图像 " << file_path << std::endl;
            // 返回一个随机图像作为占位符
            cv::Mat placeholder = cv::Mat::zeros(128, 128, CV_8UC1);
            cv::randu(placeholder, 0, 255);
            img = placeholder;
        }

        // 预处理：调整大小、归一化、标准化
        cv::Mat resized, float_img;
        cv::resize(img, resized, cv::Size(128, 128));
        resized.convertTo(float_img, CV_32F, 1.0 / 255.0);
        float_img = (float_img - 0.5) / 0.5;

        // 将cv::Mat转换为torch::Tensor
        torch::Tensor tensor_image = torch::from_blob(float_img.data, {128, 128, 1}, torch::kFloat32);
        tensor_image = tensor_image.permute({2, 0, 1});
        tensor_image = tensor_image.clone(); // 确保数据独立

        torch::Tensor tensor_label = torch::full({1}, label, torch::kLong);

        return {tensor_image, tensor_label};
    }

    torch::optional<size_t> size() const override {
        return image_paths_.size();
    }

private:
    std::vector<std::string> image_paths_;
    std::vector<int64_t> labels_;
};

void train() {
    std::cout << "=== 开始数字识别模型训练 ===" << std::endl;
    
    // 设置设备
    torch::Device device(torch::kCPU);
    if (torch::cuda::is_available()) {
        std::cout << "使用GPU进行训练" << std::endl;
        device = torch::Device(torch::kCUDA);
    } else {
        std::cout << "使用CPU进行训练" << std::endl;
    }

    // 创建模型
    auto model = SimpleDigitCNN();
    model->to(device);
    
    // 创建优化器
    torch::optim::Adam optimizer(model->parameters(), torch::optim::AdamOptions(0.001));

    // 加载数据集
    std::string data_dir = "/home/lzs/Vision_Arena_2025_main/number_data";
    std::cout << "加载数据从: " << data_dir << std::endl;
    
    auto train_dataset = DigitDataset(data_dir, true);
    if (train_dataset.size().value_or(0) == 0) {
        std::cerr << "错误: 没有找到训练数据!" << std::endl;
        return;
    }
    
    auto train_loader = torch::data::make_data_loader(
        train_dataset.map(torch::data::transforms::Stack<>()),
        torch::data::DataLoaderOptions().batch_size(32).workers(2)
    );

    // 损失函数
    auto criterion = torch::nn::CrossEntropyLoss();

    // 训练参数
    size_t num_epochs = 20;
    double best_accuracy = 0.0;

    std::cout << "开始训练..." << std::endl;
    
    for (size_t epoch = 0; epoch < num_epochs; epoch++) {
        model->train();
        double running_loss = 0.0;
        int correct_predictions = 0;
        int total_samples = 0;

        for (auto& batch : *train_loader) {
            auto data = batch.data.to(device);
            auto targets = batch.target.squeeze().to(device);

            // 前向传播
            auto output = model->forward(data);
            auto loss = criterion(output, targets);

            // 反向传播和优化
            optimizer.zero_grad();
            loss.backward();
            optimizer.step();

            running_loss += loss.item<double>();
            
            // 计算准确率
            auto predictions = output.argmax(1);
            correct_predictions += predictions.eq(targets).sum().item<int>();
            total_samples += targets.size(0);
        }

        double epoch_loss = running_loss / std::distance(train_loader->begin(), train_loader->end());
        double accuracy = 100.0 * correct_predictions / total_samples;
        
        std::cout << "Epoch [" << (epoch + 1) << "/" << num_epochs 
                  << "], Loss: " << epoch_loss 
                  << ", 准确率: " << accuracy << "%" << std::endl;

        // 保存最佳模型
        if (accuracy > best_accuracy) {
            best_accuracy = accuracy;
            
            // 关键修复：使用TorchScript保存模型
            model->eval();
            torch::jit::Module traced_model;
            try {
                // 创建示例输入用于跟踪
                auto example_input = torch::randn({1, 1, 128, 128}).to(device);
                
                // 跟踪模型
                traced_model = torch::jit::trace(model, example_input);
                
                // 保存为TorchScript格式
                std::string model_path = "/home/lzs/CVengers_challenge/models/digit_model_libtorch.pt";
                traced_model.save(model_path);
                
                std::cout << "✅ 保存最佳模型! 准确率: " << accuracy << "%" << std::endl;
                std::cout << "模型保存为: " << model_path << std::endl;
                
            } catch (const std::exception& e) {
                std::cerr << "❌ 模型保存失败: " << e.what() << std::endl;
            }
        }
    }

    std::cout << "=== 训练完成 ===" << std::endl;
    std::cout << "最佳准确率: " << best_accuracy << "%" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "数字识别模型训练程序" << std::endl;
    std::cout << "====================" << std::endl;
    
    try {
        train();
        std::cout << "🎉 训练成功完成!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "❌ 训练失败: " << e.what() << std::endl;
        return -1;
    }
}
