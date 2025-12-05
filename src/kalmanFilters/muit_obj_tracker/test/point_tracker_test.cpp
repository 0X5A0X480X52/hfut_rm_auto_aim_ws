/**
 * @file point_tracker_test.cpp
 * @brief PointTracker 测试程序
 * 
 * 该测试程序支持:
 * 1. 从 YAML 配置文件加载卡尔曼滤波器参数
 * 2. 生成模拟轨迹数据进行测试
 * 3. 测试 PointTracker 的跟踪性能
 * 
 * 使用方法:
 *   ./point_tracker_test <config_file> [--visualize]
 */

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <random>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <map>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "muit_obj_tracker/tracker/point_tracker.hpp"
#include "muit_obj_tracker/utils/types.hpp"
#include "models/model_factory.h"

// 包含 basic_models 头文件
#include "basic_models/basic_model_factories.h"

// 手动注册工厂函数
namespace {
    void registerBasicModelFactories() {
        auto& registry = ModelFactoryRegistry::getInstance();
        
        // 只有在尚未注册时才注册
        if (!registry.hasFactory("CV_KF")) {
            registry.registerFactory("CV_KF", std::make_shared<CV_KF_Factory>());
        }
        if (!registry.hasFactory("CA_KF")) {
            registry.registerFactory("CA_KF", std::make_shared<CA_KF_Factory>());
        }
        if (!registry.hasFactory("CS_KF")) {
            registry.registerFactory("CS_KF", std::make_shared<CS_KF_Factory>());
        }
        if (!registry.hasFactory("CTRV_EKF")) {
            registry.registerFactory("CTRV_EKF", std::make_shared<CTRV_EKF_Factory>());
        }
        if (!registry.hasFactory("Singer_KF")) {
            registry.registerFactory("Singer_KF", std::make_shared<Singer_KF_Factory>());
        }
    }
    
    // 在程序启动时调用
    struct FactoryInitializer {
        FactoryInitializer() { registerBasicModelFactories(); }
    } factoryInitializer;
}

namespace test_utils {

/**
 * @brief YAML 配置读取器
 */
class ConfigReader {
public:
    /**
     * @brief 从 YAML 文件加载配置
     * @param filename YAML 配置文件路径
     * @return 模型配置
     */
    static ModelConfig loadModelConfig(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        ModelConfig model_config;
        
        if (!config["filter"]) {
            throw std::runtime_error("配置文件中没有 'filter' 节");
        }
        
        YAML::Node filter = config["filter"];
        
        if (filter["T"]) {
            model_config.T = filter["T"].as<double>();
        }
        
        if (filter["Dim"]) {
            model_config.Dim = filter["Dim"].as<int>();
        }
        
        // 读取测量噪声协方差矩阵 R
        if (filter["R"]) {
            auto r_node = filter["R"];
            int rows = r_node.size();
            int cols = r_node[0].size();
            
            model_config.R = Eigen::MatrixXd(rows, cols);
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    model_config.R(i, j) = r_node[i][j].as<double>();
                }
            }
        }
        
        // 读取初始状态向量 X_0
        if (filter["X_0"]) {
            auto x0_node = filter["X_0"];
            int size = x0_node.size();
            model_config.X_0 = Eigen::VectorXd(size);
            for (int i = 0; i < size; ++i) {
                model_config.X_0(i) = x0_node[i].as<double>();
            }
        }
        
        // 读取额外参数
        if (filter["process_noise_std"]) {
            model_config.extra_params["process_noise_std"] = filter["process_noise_std"].as<double>();
        }
        
        return model_config;
    }
    
    /**
     * @brief 获取滤波器类型
     * @param filename YAML 配置文件路径
     * @return 滤波器类型字符串
     */
    static std::string getFilterType(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        if (config["filter"] && config["filter"]["type"]) {
            return config["filter"]["type"].as<std::string>();
        }
        throw std::runtime_error("配置文件中未指定滤波器类型");
    }
    
    /**
     * @brief 加载跟踪器参数
     * @param filename YAML 配置文件路径
     * @param max_age 输出: 最大未更新帧数
     * @param min_hits 输出: 最小匹配次数
     * @param distance_threshold 输出: 距离阈值
     */
    static void loadTrackerParams(const std::string& filename, 
                                   int& max_age, 
                                   int& min_hits, 
                                   double& distance_threshold) {
        YAML::Node config = YAML::LoadFile(filename);
        
        // 使用默认值
        max_age = 30;
        min_hits = 3;
        distance_threshold = 100.0;
        
        if (config["tracker"]) {
            YAML::Node tracker = config["tracker"];
            if (tracker["max_age"]) {
                max_age = tracker["max_age"].as<int>();
            }
            if (tracker["min_hits"]) {
                min_hits = tracker["min_hits"].as<int>();
            }
            if (tracker["distance_threshold"]) {
                distance_threshold = tracker["distance_threshold"].as<double>();
            }
        }
    }
};

/**
 * @brief CSV 检测数据读取器
 * 
 * 从 CSV 文件读取检测数据，格式：
 * frame_id, feature_id, x, y, width, height, confidence
 */
class CsvDetectionReader {
public:
    /**
     * @brief 从 CSV 文件读取检测数据
     * @param filename CSV 文件路径
     * @return 每帧的检测结果列表
     */
    static std::vector<std::vector<muit_obj_tracker::Detection>> readDetections(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法打开文件: " + filename);
        }
        
        std::map<int, std::vector<muit_obj_tracker::Detection>> frame_detections;
        std::string line;
        
        // 跳过表头
        std::getline(file, line);
        
        int max_frame_id = 0;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            
            while (std::getline(ss, cell, ',')) {
                row.push_back(cell);
            }
            
            if (row.size() >= 7) {
                int frame_id = std::stoi(row[0]);
                int feature_id = std::stoi(row[1]);
                float x = std::stof(row[2]);
                float y = std::stof(row[3]);
                int width = std::stoi(row[4]);
                int height = std::stoi(row[5]);
                float confidence = std::stof(row[6]);
                
                muit_obj_tracker::Detection det;
                det.id = feature_id;
                det.bbox = cv::Rect(static_cast<int>(x), static_cast<int>(y), width, height);
                det.confidence = confidence;
                
                frame_detections[frame_id].push_back(det);
                max_frame_id = std::max(max_frame_id, frame_id);
            }
        }
        
        // 转换为按帧索引的向量
        std::vector<std::vector<muit_obj_tracker::Detection>> result(max_frame_id + 1);
        for (auto& [frame_id, dets] : frame_detections) {
            result[frame_id] = std::move(dets);
        }
        
        return result;
    }
    
    /**
     * @brief 读取真值数据用于评估
     * @param filename CSV 文件路径
     * @return 每帧每个特征的真值位置 map<frame_id, map<feature_id, (x, y)>>
     */
    static std::map<int, std::map<int, std::pair<float, float>>> readGroundTruth(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法打开文件: " + filename);
        }
        
        std::map<int, std::map<int, std::pair<float, float>>> result;
        std::string line;
        
        // 跳过表头
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            
            while (std::getline(ss, cell, ',')) {
                row.push_back(cell);
            }
            
            if (row.size() >= 4) {
                int frame_id = std::stoi(row[0]);
                int feature_id = std::stoi(row[1]);
                float x = std::stof(row[2]);
                float y = std::stof(row[3]);
                
                result[frame_id][feature_id] = {x, y};
            }
        }
        
        return result;
    }
};

/**
 * @brief 轨迹生成器 - 生成模拟的目标轨迹
 */
class TrajectoryGenerator {
public:
    enum class TrajectoryType {
        LINEAR,         // 直线运动
        CIRCULAR,       // 圆周运动
        ZIGZAG,         // 之字形运动
        RANDOM_WALK     // 随机游走
    };
    
    /**
     * @brief 生成轨迹
     * @param type 轨迹类型
     * @param num_frames 帧数
     * @param start_x 起始 x 坐标
     * @param start_y 起始 y 坐标
     * @param noise_std 测量噪声标准差
     * @return 检测结果列表（每帧）
     */
    static std::vector<std::vector<muit_obj_tracker::Detection>> generateTrajectory(
        TrajectoryType type,
        int num_frames,
        double start_x,
        double start_y,
        double noise_std = 2.0) 
    {
        std::vector<std::vector<muit_obj_tracker::Detection>> all_detections;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> noise(0.0, noise_std);
        
        for (int i = 0; i < num_frames; ++i) {
            std::vector<muit_obj_tracker::Detection> frame_dets;
            
            double x = 0, y = 0;
            double t = static_cast<double>(i);
            
            switch (type) {
                case TrajectoryType::LINEAR:
                    x = start_x + t * 5.0;  // 速度 5 像素/帧
                    y = start_y + t * 3.0;  // 速度 3 像素/帧
                    break;
                    
                case TrajectoryType::CIRCULAR:
                    x = start_x + 100.0 * std::cos(t * 0.1);
                    y = start_y + 100.0 * std::sin(t * 0.1);
                    break;
                    
                case TrajectoryType::ZIGZAG:
                    x = start_x + t * 5.0;
                    y = start_y + 50.0 * std::sin(t * 0.2);
                    break;
                    
                case TrajectoryType::RANDOM_WALK:
                    {
                        static double rx = start_x, ry = start_y;
                        std::uniform_real_distribution<> dist(-5.0, 5.0);
                        rx += dist(gen);
                        ry += dist(gen);
                        x = rx;
                        y = ry;
                    }
                    break;
            }
            
            // 添加测量噪声
            x += noise(gen);
            y += noise(gen);
            
            muit_obj_tracker::Detection det;
            det.id = 0;
            det.bbox = cv::Rect(static_cast<int>(x - 20), static_cast<int>(y - 20), 40, 40);
            det.confidence = 0.9f;
            
            frame_dets.push_back(det);
            all_detections.push_back(frame_dets);
        }
        
        return all_detections;
    }
    
    /**
     * @brief 生成多目标轨迹
     * @param num_targets 目标数量
     * @param num_frames 帧数
     * @param noise_std 测量噪声标准差
     * @return 检测结果列表（每帧）
     */
    static std::vector<std::vector<muit_obj_tracker::Detection>> generateMultiTargetTrajectory(
        int num_targets,
        int num_frames,
        double noise_std = 2.0)
    {
        std::vector<std::vector<muit_obj_tracker::Detection>> all_detections(num_frames);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> noise(0.0, noise_std);
        
        for (int target = 0; target < num_targets; ++target) {
            double start_x = 100.0 + target * 200.0;
            double start_y = 100.0 + target * 100.0;
            double vx = (target % 2 == 0) ? 4.0 : -3.0;
            double vy = (target % 2 == 0) ? 2.0 : 5.0;
            
            for (int frame = 0; frame < num_frames; ++frame) {
                double t = static_cast<double>(frame);
                double x = start_x + vx * t + noise(gen);
                double y = start_y + vy * t + noise(gen);
                
                muit_obj_tracker::Detection det;
                det.id = target;
                det.bbox = cv::Rect(static_cast<int>(x - 20), static_cast<int>(y - 20), 40, 40);
                det.confidence = 0.9f;
                
                all_detections[frame].push_back(det);
            }
        }
        
        return all_detections;
    }
    
    /**
     * @brief 生成带遮挡的轨迹（某些帧检测丢失）
     * @param num_frames 帧数
     * @param occlusion_start 遮挡开始帧
     * @param occlusion_end 遮挡结束帧
     * @param noise_std 测量噪声标准差
     * @return 检测结果列表（每帧）
     */
    static std::vector<std::vector<muit_obj_tracker::Detection>> generateOccludedTrajectory(
        int num_frames,
        int occlusion_start,
        int occlusion_end,
        double noise_std = 2.0)
    {
        std::vector<std::vector<muit_obj_tracker::Detection>> all_detections;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> noise(0.0, noise_std);
        
        double start_x = 100.0, start_y = 200.0;
        double vx = 5.0, vy = 2.0;
        
        for (int frame = 0; frame < num_frames; ++frame) {
            std::vector<muit_obj_tracker::Detection> frame_dets;
            
            // 在遮挡期间不产生检测
            if (frame >= occlusion_start && frame <= occlusion_end) {
                all_detections.push_back(frame_dets);
                continue;
            }
            
            double t = static_cast<double>(frame);
            double x = start_x + vx * t + noise(gen);
            double y = start_y + vy * t + noise(gen);
            
            muit_obj_tracker::Detection det;
            det.id = 0;
            det.bbox = cv::Rect(static_cast<int>(x - 20), static_cast<int>(y - 20), 40, 40);
            det.confidence = 0.9f;
            
            frame_dets.push_back(det);
            all_detections.push_back(frame_dets);
        }
        
        return all_detections;
    }
};

/**
 * @brief 测试结果统计
 */
struct TestMetrics {
    int total_frames = 0;
    int tracked_frames = 0;
    int id_switches = 0;
    double avg_position_error = 0.0;
    double max_position_error = 0.0;
    double processing_time_ms = 0.0;
    
    void print() const {
        std::cout << "\n=== 测试统计结果 ===" << std::endl;
        std::cout << "  总帧数: " << total_frames << std::endl;
        std::cout << "  成功跟踪帧数: " << tracked_frames << std::endl;
        std::cout << "  跟踪率: " << std::fixed << std::setprecision(2) 
                  << (100.0 * tracked_frames / total_frames) << "%" << std::endl;
        std::cout << "  ID 切换次数: " << id_switches << std::endl;
        std::cout << "  平均位置误差: " << std::fixed << std::setprecision(3) 
                  << avg_position_error << " 像素" << std::endl;
        std::cout << "  最大位置误差: " << std::fixed << std::setprecision(3) 
                  << max_position_error << " 像素" << std::endl;
        std::cout << "  总处理时间: " << std::fixed << std::setprecision(2) 
                  << processing_time_ms << " ms" << std::endl;
        std::cout << "  平均每帧处理时间: " << std::fixed << std::setprecision(3) 
                  << (processing_time_ms / total_frames) << " ms" << std::endl;
    }
};

/**
 * @brief 从状态向量提取位置
 * 
 * 不同滤波器类型的状态向量结构：
 * - CV_KF:    [x, vx, y, vy]        (4 状态) - y 在索引 2
 * - CA_KF:    [x, vx, ax, y, vy, ay] (6 状态) - y 在索引 3
 * - CS_KF:    [x, vx, ax, y, vy, ay] (6 状态) - y 在索引 3
 * - Singer_KF: [x, vx, ax, y, vy, ay] (6 状态) - y 在索引 3
 * - CTRV_EKF: [x, y, v, theta, omega] (5 状态) - x 在索引 0, y 在索引 1
 */
class StateExtractor {
public:
    static cv::Point2f getPosition(const Eigen::VectorXd& state, const std::string& filter_type) {
        float x = 0, y = 0;
        
        if (filter_type == "CV_KF") {
            // CV: [x, vx, y, vy]
            if (state.size() >= 4) {
                x = static_cast<float>(state(0));
                y = static_cast<float>(state(2));
            }
        } else if (filter_type == "CA_KF" || filter_type == "CS_KF" || filter_type == "Singer_KF") {
            // CA/CS/Singer: [x, vx, ax, y, vy, ay]
            if (state.size() >= 6) {
                x = static_cast<float>(state(0));
                y = static_cast<float>(state(3));
            }
        } else if (filter_type == "CTRV_EKF") {
            // CTRV: [x, y, v, theta, omega]
            if (state.size() >= 2) {
                x = static_cast<float>(state(0));
                y = static_cast<float>(state(1));
            }
        } else {
            // 默认：假设 [x, y, ...]
            if (state.size() >= 2) {
                x = static_cast<float>(state(0));
                y = static_cast<float>(state(1));
            }
        }
        
        return cv::Point2f(x, y);
    }
};

} // namespace test_utils

void printUsage(const char* program_name) {
    std::cout << "用法: " << program_name << " <config_file> [options]\n";
    std::cout << "\n";
    std::cout << "参数:\n";
    std::cout << "  config_file   : YAML 配置文件路径\n";
    std::cout << "\n";
    std::cout << "选项:\n";
    std::cout << "  --visualize   : 启用可视化显示\n";
    std::cout << "  --test-type N : 测试类型 (0=直线, 1=圆周, 2=之字形, 3=多目标, 4=遮挡, 5=CSV文件)\n";
    std::cout << "  --frames N    : 测试帧数 (默认: 200)\n";
    std::cout << "  --noise N     : 测量噪声标准差 (默认: 2.0)\n";
    std::cout << "  --csv FILE    : 从CSV文件读取检测数据 (需要 --test-type 5)\n";
    std::cout << "  --gt FILE     : 真值CSV文件 (可选，用于评估)\n";
    std::cout << "  --output FILE : 保存跟踪结果到CSV文件\n";
    std::cout << "\n";
    std::cout << "示例:\n";
    std::cout << "  " << program_name << " config/cv_kf_2d.yaml\n";
    std::cout << "  " << program_name << " config/ca_kf_2d.yaml --visualize --test-type 1\n";
    std::cout << "  " << program_name << " config/cv_kf_2d.yaml --test-type 5 --csv data/detections.csv\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    
    std::string config_file = argv[1];
    bool visualize = false;
    int test_type = 0;
    int num_frames = 200;
    double noise_std = 2.0;
    std::string csv_file;
    std::string gt_file;
    std::string output_file;
    
    // 解析命令行参数
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--visualize") {
            visualize = true;
        } else if (arg == "--test-type" && i + 1 < argc) {
            test_type = std::stoi(argv[++i]);
        } else if (arg == "--frames" && i + 1 < argc) {
            num_frames = std::stoi(argv[++i]);
        } else if (arg == "--noise" && i + 1 < argc) {
            noise_std = std::stod(argv[++i]);
        } else if (arg == "--csv" && i + 1 < argc) {
            csv_file = argv[++i];
        } else if (arg == "--gt" && i + 1 < argc) {
            gt_file = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_file = argv[++i];
        }
    }
    
    try {
        std::cout << "=== PointTracker 测试 ===" << std::endl;
        std::cout << "配置文件: " << config_file << std::endl;
        std::cout << std::endl;
        
        // 1. 加载配置
        std::cout << "正在加载配置..." << std::endl;
        std::string filter_type = test_utils::ConfigReader::getFilterType(config_file);
        ModelConfig model_config = test_utils::ConfigReader::loadModelConfig(config_file);
        
        int max_age, min_hits;
        double distance_threshold;
        test_utils::ConfigReader::loadTrackerParams(config_file, max_age, min_hits, distance_threshold);
        
        std::cout << "  滤波器类型: " << filter_type << std::endl;
        std::cout << "  采样时间 (T): " << model_config.T << " s" << std::endl;
        std::cout << "  维度: " << model_config.Dim << "D" << std::endl;
        std::cout << "  初始状态维度: " << model_config.X_0.size() << std::endl;
        std::cout << "  跟踪器参数: max_age=" << max_age 
                  << ", min_hits=" << min_hits 
                  << ", distance_threshold=" << distance_threshold << std::endl;
        std::cout << std::endl;
        
        // 2. 创建 PointTracker
        std::cout << "正在创建 PointTracker..." << std::endl;
        muit_obj_tracker::PointTracker tracker(max_age, min_hits, distance_threshold, 
                                               filter_type, model_config);
        std::cout << "  PointTracker 创建成功" << std::endl;
        std::cout << std::endl;
        
        // 3. 生成测试数据
        std::cout << "正在生成测试数据..." << std::endl;
        std::vector<std::vector<muit_obj_tracker::Detection>> all_detections;
        std::string test_name;
        std::map<int, std::map<int, std::pair<float, float>>> ground_truth;
        
        switch (test_type) {
            case 0:
                test_name = "直线运动";
                all_detections = test_utils::TrajectoryGenerator::generateTrajectory(
                    test_utils::TrajectoryGenerator::TrajectoryType::LINEAR, 
                    num_frames, 100.0, 200.0, noise_std);
                break;
            case 1:
                test_name = "圆周运动";
                all_detections = test_utils::TrajectoryGenerator::generateTrajectory(
                    test_utils::TrajectoryGenerator::TrajectoryType::CIRCULAR, 
                    num_frames, 400.0, 300.0, noise_std);
                break;
            case 2:
                test_name = "之字形运动";
                all_detections = test_utils::TrajectoryGenerator::generateTrajectory(
                    test_utils::TrajectoryGenerator::TrajectoryType::ZIGZAG, 
                    num_frames, 100.0, 300.0, noise_std);
                break;
            case 3:
                test_name = "多目标跟踪";
                all_detections = test_utils::TrajectoryGenerator::generateMultiTargetTrajectory(
                    3, num_frames, noise_std);
                break;
            case 4:
                test_name = "遮挡恢复";
                all_detections = test_utils::TrajectoryGenerator::generateOccludedTrajectory(
                    num_frames, num_frames / 3, num_frames / 3 + 10, noise_std);
                break;
            case 5:
                // 从 CSV 文件读取
                test_name = "CSV文件数据";
                if (csv_file.empty()) {
                    throw std::runtime_error("使用 --test-type 5 时必须指定 --csv 参数");
                }
                std::cout << "  正在从 CSV 文件读取检测数据: " << csv_file << std::endl;
                all_detections = test_utils::CsvDetectionReader::readDetections(csv_file);
                num_frames = all_detections.size();
                
                // 读取真值数据（如果提供）
                if (!gt_file.empty()) {
                    std::cout << "  正在读取真值数据: " << gt_file << std::endl;
                    ground_truth = test_utils::CsvDetectionReader::readGroundTruth(gt_file);
                }
                break;
            default:
                test_name = "直线运动";
                all_detections = test_utils::TrajectoryGenerator::generateTrajectory(
                    test_utils::TrajectoryGenerator::TrajectoryType::LINEAR, 
                    num_frames, 100.0, 200.0, noise_std);
        }
        
        std::cout << "  测试类型: " << test_name << std::endl;
        std::cout << "  帧数: " << num_frames << std::endl;
        std::cout << "  测量噪声标准差: " << noise_std << std::endl;
        std::cout << std::endl;
        
        // 4. 运行跟踪测试
        std::cout << "正在运行跟踪测试..." << std::endl;
        
        test_utils::TestMetrics metrics;
        metrics.total_frames = num_frames;
        
        int last_track_id = -1;
        double total_error = 0.0;
        int error_count = 0;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        cv::Mat vis_img;
        if (visualize) {
            vis_img = cv::Mat(720, 1280, CV_8UC3, cv::Scalar(30, 30, 30));
        }
        
        for (int frame = 0; frame < num_frames; ++frame) {
            // 预测
            tracker.predict();
            
            // 更新
            tracker.update(all_detections[frame]);
            
            // 获取跟踪结果
            auto tracks = tracker.getTracks();
            
            if (!tracks.empty()) {
                metrics.tracked_frames++;
                
                // 检查 ID 切换
                if (last_track_id != -1 && tracks[0].track_id != last_track_id) {
                    metrics.id_switches++;
                }
                last_track_id = tracks[0].track_id;
                
                // 计算位置误差（如果有对应的检测）
                if (!all_detections[frame].empty()) {
                    const auto& det = all_detections[frame][0];
                    cv::Point2f det_center(det.bbox.x + det.bbox.width / 2.0f,
                                          det.bbox.y + det.bbox.height / 2.0f);
                    
                    // 使用 StateExtractor 从状态向量提取估计位置
                    cv::Point2f est_pos = test_utils::StateExtractor::getPosition(tracks[0].state, filter_type);
                    
                    double error = std::sqrt(std::pow(det_center.x - est_pos.x, 2) + 
                                            std::pow(det_center.y - est_pos.y, 2));
                    total_error += error;
                    error_count++;
                    metrics.max_position_error = std::max(metrics.max_position_error, error);
                }
            }
            
            // 可视化
            if (visualize) {
                vis_img.setTo(cv::Scalar(30, 30, 30));
                
                // 绘制检测
                for (const auto& det : all_detections[frame]) {
                    cv::rectangle(vis_img, det.bbox, cv::Scalar(0, 255, 0), 2);
                    cv::Point2f center(det.bbox.x + det.bbox.width / 2.0f,
                                      det.bbox.y + det.bbox.height / 2.0f);
                    cv::circle(vis_img, center, 3, cv::Scalar(0, 255, 0), -1);
                }
                
                // 绘制跟踪
                for (const auto& track : tracks) {
                    cv::rectangle(vis_img, track.bbox, cv::Scalar(0, 0, 255), 2);
                    
                    // 使用 StateExtractor 从状态向量提取估计位置
                    cv::Point2f est_pos = test_utils::StateExtractor::getPosition(track.state, filter_type);
                    cv::circle(vis_img, cv::Point(static_cast<int>(est_pos.x), static_cast<int>(est_pos.y)), 
                              5, cv::Scalar(0, 0, 255), -1);
                    
                    // 绘制 ID
                    cv::putText(vis_img, "ID: " + std::to_string(track.track_id),
                               cv::Point(track.bbox.x, track.bbox.y - 5),
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                }
                
                // 绘制信息
                cv::putText(vis_img, "Frame: " + std::to_string(frame) + "/" + std::to_string(num_frames),
                           cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                cv::putText(vis_img, "Filter: " + filter_type,
                           cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                cv::putText(vis_img, "Test: " + test_name,
                           cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                cv::putText(vis_img, "Tracks: " + std::to_string(tracks.size()),
                           cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                
                cv::imshow("PointTracker Test", vis_img);
                int key = cv::waitKey(30);
                if (key == 27) break;  // ESC 退出
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        metrics.processing_time_ms = duration.count() / 1000.0;
        
        if (error_count > 0) {
            metrics.avg_position_error = total_error / error_count;
        }
        
        // 5. 输出结果
        metrics.print();
        
        // 多目标统计
        if (test_type == 3 || test_type == 5) {
            int total_dets = 0;
            for (const auto& dets : all_detections) {
                total_dets += dets.size();
            }
            std::cout << "\n=== 多目标统计 ===" << std::endl;
            std::cout << "  总检测数: " << total_dets << std::endl;
            std::cout << "  平均每帧检测数: " << std::fixed << std::setprecision(2) 
                      << (double)total_dets / num_frames << std::endl;
        }
        
        // 保存跟踪结果
        if (!output_file.empty()) {
            std::cout << "\n正在保存跟踪结果到: " << output_file << std::endl;
            std::ofstream out(output_file);
            out << "frame_id,track_id,x,y,width,height,state_x,state_y\n";
            
            // 重新运行跟踪以保存结果
            tracker.reset();
            for (int frame = 0; frame < num_frames; ++frame) {
                tracker.predict();
                tracker.update(all_detections[frame]);
                auto tracks = tracker.getTracks();
                
                for (const auto& track : tracks) {
                    cv::Point2f est_pos = test_utils::StateExtractor::getPosition(track.state, filter_type);
                    out << frame << "," << track.track_id << ","
                        << track.bbox.x << "," << track.bbox.y << ","
                        << track.bbox.width << "," << track.bbox.height << ","
                        << est_pos.x << "," << est_pos.y << "\n";
                }
            }
            out.close();
            std::cout << "  跟踪结果已保存" << std::endl;
        }
        
        if (visualize) {
            cv::destroyAllWindows();
        }
        
        std::cout << "\n=== 测试完成 ===" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
}
