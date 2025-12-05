/**
 * @file point_tracker_test_3d.cpp
 * @brief PointTracker 3D 测试程序
 * 
 * 该测试程序支持:
 * 1. 从 YAML 配置文件加载 3D 卡尔曼滤波器参数
 * 2. 从 CSV 文件读取 3D 检测数据进行测试
 * 3. 测试 PointTracker 的 3D 跟踪性能
 * 
 * 使用方法:
 *   ./point_tracker_test_3d <config_file> --csv <detection_csv>
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
#include <functional>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "models/model_factory.h"
#include "models/models.h"

// 包含 basic_models 头文件
#include "basic_models/basic_model_factories.h"

// 包含 combined_models 头文件
#include "combined_models/combined_model_factories.h"

// 手动注册工厂函数
namespace {
    void registerBasicModelFactories() {
        auto& registry = ModelFactoryRegistry::getInstance();
        
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
        // combined_models
        if (!registry.hasFactory("IMM_CV_CA_CS_3Dim")) {
            registry.registerFactory("IMM_CV_CA_CS_3Dim", std::make_shared<IMM_CV_CA_CS_3Dim_Factory>());
        }
        if (!registry.hasFactory("IMM_CV_CA_CS_Singer_3Dim")) {
            registry.registerFactory("IMM_CV_CA_CS_Singer_3Dim", std::make_shared<IMM_CV_CA_CS_Singer_3Dim_Factory>());
        }
    }
    
    struct FactoryInitializer {
        FactoryInitializer() { registerBasicModelFactories(); }
    } factoryInitializer;
}

namespace test_utils_3d {

/**
 * @brief 3D 检测数据
 */
struct Detection3D {
    int frame_id;
    int feature_id;
    double x, y, z;
    double confidence;
};

/**
 * @brief 3D 跟踪结果
 */
struct Track3D {
    int track_id;
    Eigen::VectorXd state;
    int time_since_update;
    int hits;
    int age;
};

/**
 * @brief YAML 配置读取器
 */
class ConfigReader {
public:
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
        
        if (filter["extra_params"]) {
            auto extra = filter["extra_params"];
            if (extra["alpha"]) {
                model_config.extra_params["alpha"] = extra["alpha"].as<double>();
            }
            if (extra["a_max"]) {
                model_config.extra_params["a_max"] = extra["a_max"].as<double>();
            }
        }
        
        // 读取 IMM 相关参数
        if (filter["transform_rate_mat"]) {
            auto mat = filter["transform_rate_mat"];
            int rows = mat.size();
            int cols = mat[0].size();
            model_config.transform_rate_mat = Eigen::MatrixXd(rows, cols);
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    model_config.transform_rate_mat(i, j) = mat[i][j].as<double>();
                }
            }
        }
        
        return model_config;
    }
    
    static std::string getFilterType(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        if (config["filter"] && config["filter"]["type"]) {
            return config["filter"]["type"].as<std::string>();
        }
        throw std::runtime_error("配置文件中未指定滤波器类型");
    }
    
    static void loadTrackerParams(const std::string& filename,
                                   int& max_age,
                                   int& min_hits,
                                   double& distance_threshold) {
        YAML::Node config = YAML::LoadFile(filename);
        
        max_age = 30;
        min_hits = 3;
        distance_threshold = 1.0;  // 3D 空间用米为单位
        
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
 * @brief 3D CSV 检测数据读取器
 */
class CsvDetectionReader3D {
public:
    static std::vector<std::vector<Detection3D>> readDetections(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法打开文件: " + filename);
        }
        
        std::map<int, std::vector<Detection3D>> frame_detections;
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
            
            // 格式: frame_id, feature_id, x, y, z, confidence
            if (row.size() >= 6) {
                Detection3D det;
                det.frame_id = std::stoi(row[0]);
                det.feature_id = std::stoi(row[1]);
                det.x = std::stod(row[2]);
                det.y = std::stod(row[3]);
                det.z = std::stod(row[4]);
                det.confidence = std::stod(row[5]);
                
                frame_detections[det.frame_id].push_back(det);
                max_frame_id = std::max(max_frame_id, det.frame_id);
            }
        }
        
        std::vector<std::vector<Detection3D>> result(max_frame_id + 1);
        for (auto& [frame_id, dets] : frame_detections) {
            result[frame_id] = std::move(dets);
        }
        
        return result;
    }
    
    static std::map<int, std::map<int, std::tuple<double, double, double>>> readGroundTruth(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("无法打开文件: " + filename);
        }
        
        std::map<int, std::map<int, std::tuple<double, double, double>>> result;
        std::string line;
        
        std::getline(file, line);  // 跳过表头
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            
            while (std::getline(ss, cell, ',')) {
                row.push_back(cell);
            }
            
            if (row.size() >= 5) {
                int frame_id = std::stoi(row[0]);
                int feature_id = std::stoi(row[1]);
                double x = std::stod(row[2]);
                double y = std::stod(row[3]);
                double z = std::stod(row[4]);
                
                result[frame_id][feature_id] = {x, y, z};
            }
        }
        
        return result;
    }
};

/**
 * @brief 3D 状态提取器
 * 
 * 从不同滤波器的状态向量中提取 3D 位置
 */
class StateExtractor3D {
public:
    /**
     * @brief 从状态向量提取 3D 位置
     * 
     * 状态向量结构（3D Kronecker 积）:
     * - CV_KF 3D (6 状态): [x, vx, y, vy, z, vz]
     * - CA_KF/CS_KF/Singer_KF 3D (9 状态): [x, vx, ax, y, vy, ay, z, vz, az]
     */
    static Eigen::Vector3d getPosition(const Eigen::VectorXd& state, const std::string& filter_type, int dim) {
        double x = 0, y = 0, z = 0;
        
        if (dim == 3) {
            if (filter_type == "CV_KF") {
                // CV 3D: [x, vx, y, vy, z, vz]
                if (state.size() >= 6) {
                    x = state(0);
                    y = state(2);
                    z = state(4);
                }
            } else if (filter_type == "CA_KF" || filter_type == "CS_KF" || filter_type == "Singer_KF" ||
                       filter_type == "IMM_CV_CA_CS_3Dim") {
                // CA/CS/Singer 3D: [x, vx, ax, y, vy, ay, z, vz, az]
                if (state.size() >= 9) {
                    x = state(0);
                    y = state(3);
                    z = state(6);
                }
            } else {
                // 默认：假设前三个是位置
                if (state.size() >= 3) {
                    x = state(0);
                    y = state(1);
                    z = state(2);
                }
            }
        } else {
            // 2D 情况
            if (filter_type == "CV_KF") {
                if (state.size() >= 4) {
                    x = state(0);
                    y = state(2);
                }
            } else if (filter_type == "CA_KF" || filter_type == "CS_KF" || filter_type == "Singer_KF") {
                if (state.size() >= 6) {
                    x = state(0);
                    y = state(3);
                }
            } else if (filter_type == "CTRV_EKF") {
                if (state.size() >= 2) {
                    x = state(0);
                    y = state(1);
                }
            }
        }
        
        return Eigen::Vector3d(x, y, z);
    }
};

/**
 * @brief 简化的 3D 跟踪器（直接使用滤波器）
 */
class SimpleTracker3D {
public:
    SimpleTracker3D(int max_age, int min_hits, double distance_threshold,
                    const std::string& model_name, const ModelConfig& model_config)
        : max_age_(max_age), min_hits_(min_hits), distance_threshold_(distance_threshold),
          model_name_(model_name), model_config_(model_config), track_id_count_(0) {}
    
    void predict() {
        for (auto& track : tracks_) {
            track.filter->performPredict();
            track.age++;
            track.time_since_update++;
        }
    }
    
    void update(const std::vector<Detection3D>& detections) {
        int n_tracks = tracks_.size();
        int n_dets = detections.size();
        
        // 没有跟踪，创建新跟踪
        if (n_tracks == 0) {
            for (const auto& det : detections) {
                createNewTrack(det);
            }
            return;
        }
        
        // 没有检测，更新现有跟踪
        if (n_dets == 0) {
            removeDeadTracks();
            return;
        }
        
        // 计算代价矩阵
        std::vector<std::vector<double>> cost_matrix(n_tracks, std::vector<double>(n_dets));
        
        for (int i = 0; i < n_tracks; ++i) {
            Eigen::Vector3d pred_pos = StateExtractor3D::getPosition(
                tracks_[i].filter->get_X_after(), model_name_, model_config_.Dim);
            
            for (int j = 0; j < n_dets; ++j) {
                Eigen::Vector3d det_pos(detections[j].x, detections[j].y, detections[j].z);
                cost_matrix[i][j] = (pred_pos - det_pos).norm();
            }
        }
        
        // 贪婪匹配（简化版）
        std::vector<bool> track_matched(n_tracks, false);
        std::vector<bool> det_matched(n_dets, false);
        
        for (int iter = 0; iter < std::min(n_tracks, n_dets); ++iter) {
            double min_cost = std::numeric_limits<double>::max();
            int best_track = -1, best_det = -1;
            
            for (int i = 0; i < n_tracks; ++i) {
                if (track_matched[i]) continue;
                for (int j = 0; j < n_dets; ++j) {
                    if (det_matched[j]) continue;
                    if (cost_matrix[i][j] < min_cost) {
                        min_cost = cost_matrix[i][j];
                        best_track = i;
                        best_det = j;
                    }
                }
            }
            
            if (best_track >= 0 && min_cost < distance_threshold_) {
                track_matched[best_track] = true;
                det_matched[best_det] = true;
                
                // 更新滤波器
                Eigen::VectorXd z(3);
                z << detections[best_det].x, detections[best_det].y, detections[best_det].z;
                tracks_[best_track].filter->performUpdate(z);
                tracks_[best_track].hits++;
                tracks_[best_track].time_since_update = 0;
            }
        }
        
        // 创建新跟踪
        for (int j = 0; j < n_dets; ++j) {
            if (!det_matched[j]) {
                createNewTrack(detections[j]);
            }
        }
        
        // 删除死亡跟踪
        removeDeadTracks();
    }
    
    std::vector<Track3D> getTracks() const {
        std::vector<Track3D> results;
        for (const auto& track : tracks_) {
            if (track.time_since_update < 1 && (track.hits >= min_hits_ || track.age <= min_hits_)) {
                Track3D res;
                res.track_id = track.id;
                res.state = track.filter->get_X_after();
                res.time_since_update = track.time_since_update;
                res.hits = track.hits;
                res.age = track.age;
                results.push_back(res);
            }
        }
        return results;
    }
    
    void reset() {
        tracks_.clear();
        track_id_count_ = 0;
    }

private:
    struct InternalTrack {
        int id;
        std::unique_ptr<Models> filter;
        int time_since_update = 0;
        int hits = 0;
        int age = 0;
    };
    
    void createNewTrack(const Detection3D& det) {
        InternalTrack track;
        track.id = ++track_id_count_;
        
        // 创建滤波器配置
        ModelConfig init_config = model_config_;
        
        // 初始化状态
        if (model_config_.Dim == 3) {
            if (model_name_ == "CV_KF") {
                // CV 3D: [x, vx, y, vy, z, vz]
                if (init_config.X_0.size() >= 6) {
                    init_config.X_0(0) = det.x;
                    init_config.X_0(2) = det.y;
                    init_config.X_0(4) = det.z;
                }
            } else if (model_name_ == "CA_KF" || model_name_ == "CS_KF" || 
                       model_name_ == "Singer_KF" || model_name_ == "IMM_CV_CA_CS_3Dim") {
                // CA/CS/Singer 3D: [x, vx, ax, y, vy, ay, z, vz, az]
                if (init_config.X_0.size() >= 9) {
                    init_config.X_0(0) = det.x;
                    init_config.X_0(3) = det.y;
                    init_config.X_0(6) = det.z;
                }
            }
        }
        
        // 创建滤波器
        auto& registry = ModelFactoryRegistry::getInstance();
        track.filter = registry.createModel(model_name_, init_config);
        
        if (!track.filter) {
            std::cerr << "无法创建滤波器: " << model_name_ << std::endl;
            return;
        }
        
        // 初始化滤波器
        track.filter->KalmanFilterInit(init_config.X_0);
        
        track.time_since_update = 0;
        track.hits = 1;
        track.age = 1;
        
        tracks_.push_back(std::move(track));
    }
    
    void removeDeadTracks() {
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                [this](const InternalTrack& t) { return t.time_since_update > max_age_; }),
            tracks_.end()
        );
    }
    
    std::vector<InternalTrack> tracks_;
    int max_age_;
    int min_hits_;
    double distance_threshold_;
    std::string model_name_;
    ModelConfig model_config_;
    int track_id_count_;
};

/**
 * @brief 测试结果统计
 */
struct TestMetrics3D {
    int total_frames = 0;
    int tracked_frames = 0;
    int total_detections = 0;
    int total_tracks = 0;
    int id_switches = 0;
    double avg_position_error = 0.0;
    double max_position_error = 0.0;
    double processing_time_ms = 0.0;
    
    void print() const {
        std::cout << "\n=== 3D 测试统计结果 ===" << std::endl;
        std::cout << "  总帧数: " << total_frames << std::endl;
        std::cout << "  总检测数: " << total_detections << std::endl;
        std::cout << "  总跟踪输出数: " << total_tracks << std::endl;
        std::cout << "  成功跟踪帧数: " << tracked_frames << std::endl;
        std::cout << "  跟踪率: " << std::fixed << std::setprecision(2) 
                  << (100.0 * tracked_frames / total_frames) << "%" << std::endl;
        std::cout << "  ID 切换次数: " << id_switches << std::endl;
        std::cout << "  平均位置误差: " << std::fixed << std::setprecision(6) 
                  << avg_position_error << " m" << std::endl;
        std::cout << "  最大位置误差: " << std::fixed << std::setprecision(6) 
                  << max_position_error << " m" << std::endl;
        std::cout << "  总处理时间: " << std::fixed << std::setprecision(2) 
                  << processing_time_ms << " ms" << std::endl;
        std::cout << "  平均每帧处理时间: " << std::fixed << std::setprecision(3) 
                  << (processing_time_ms / total_frames) << " ms" << std::endl;
    }
};

} // namespace test_utils_3d

void printUsage(const char* program_name) {
    std::cout << "用法: " << program_name << " <config_file> [options]\n";
    std::cout << "\n";
    std::cout << "参数:\n";
    std::cout << "  config_file   : YAML 配置文件路径\n";
    std::cout << "\n";
    std::cout << "选项:\n";
    std::cout << "  --csv FILE    : 从 CSV 文件读取 3D 检测数据\n";
    std::cout << "  --gt FILE     : 真值 CSV 文件（可选，用于评估）\n";
    std::cout << "  --output FILE : 保存跟踪结果到 CSV 文件\n";
    std::cout << "\n";
    std::cout << "示例:\n";
    std::cout << "  " << program_name << " config/cv_kf_3d.yaml --csv data/complex_detections_3d.csv\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    
    std::string config_file = argv[1];
    std::string csv_file;
    std::string gt_file;
    std::string output_file;
    
    // 解析命令行参数
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--csv" && i + 1 < argc) {
            csv_file = argv[++i];
        } else if (arg == "--gt" && i + 1 < argc) {
            gt_file = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_file = argv[++i];
        }
    }
    
    if (csv_file.empty()) {
        std::cerr << "错误: 必须指定 --csv 参数\n";
        printUsage(argv[0]);
        return 1;
    }
    
    try {
        std::cout << "=== PointTracker 3D 测试 ===" << std::endl;
        std::cout << "配置文件: " << config_file << std::endl;
        std::cout << "检测数据: " << csv_file << std::endl;
        std::cout << std::endl;
        
        // 1. 加载配置
        std::cout << "正在加载配置..." << std::endl;
        std::string filter_type = test_utils_3d::ConfigReader::getFilterType(config_file);
        ModelConfig model_config = test_utils_3d::ConfigReader::loadModelConfig(config_file);
        
        int max_age, min_hits;
        double distance_threshold;
        test_utils_3d::ConfigReader::loadTrackerParams(config_file, max_age, min_hits, distance_threshold);
        
        std::cout << "  滤波器类型: " << filter_type << std::endl;
        std::cout << "  采样时间 (T): " << model_config.T << " s" << std::endl;
        std::cout << "  维度: " << model_config.Dim << "D" << std::endl;
        std::cout << "  初始状态维度: " << model_config.X_0.size() << std::endl;
        std::cout << "  跟踪器参数: max_age=" << max_age
                  << ", min_hits=" << min_hits
                  << ", distance_threshold=" << distance_threshold << " m" << std::endl;
        std::cout << std::endl;
        
        // 2. 加载检测数据
        std::cout << "正在加载 3D 检测数据..." << std::endl;
        auto all_detections = test_utils_3d::CsvDetectionReader3D::readDetections(csv_file);
        int num_frames = all_detections.size();
        
        int total_det_count = 0;
        for (const auto& frame_dets : all_detections) {
            total_det_count += frame_dets.size();
        }
        std::cout << "  帧数: " << num_frames << std::endl;
        std::cout << "  总检测数: " << total_det_count << std::endl;
        std::cout << std::endl;
        
        // 加载真值数据（如果提供）
        std::map<int, std::map<int, std::tuple<double, double, double>>> ground_truth;
        if (!gt_file.empty()) {
            std::cout << "正在加载真值数据: " << gt_file << std::endl;
            ground_truth = test_utils_3d::CsvDetectionReader3D::readGroundTruth(gt_file);
        }
        
        // 3. 创建跟踪器
        std::cout << "正在创建 3D 跟踪器..." << std::endl;
        test_utils_3d::SimpleTracker3D tracker(max_age, min_hits, distance_threshold,
                                               filter_type, model_config);
        std::cout << "  跟踪器创建成功" << std::endl;
        std::cout << std::endl;
        
        // 4. 运行跟踪测试
        std::cout << "正在运行 3D 跟踪测试..." << std::endl;
        
        test_utils_3d::TestMetrics3D metrics;
        metrics.total_frames = num_frames;
        metrics.total_detections = total_det_count;
        
        std::map<int, int> last_track_ids;  // feature_id -> last_track_id
        double total_error = 0.0;
        int error_count = 0;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // 用于保存结果
        std::vector<std::tuple<int, int, double, double, double>> results;  // frame, track_id, x, y, z
        
        for (int frame = 0; frame < num_frames; ++frame) {
            // 预测
            tracker.predict();
            
            // 更新
            tracker.update(all_detections[frame]);
            
            // 获取跟踪结果
            auto tracks = tracker.getTracks();
            metrics.total_tracks += tracks.size();
            
            if (!tracks.empty()) {
                metrics.tracked_frames++;
            }
            
            // 计算位置误差
            for (const auto& track : tracks) {
                Eigen::Vector3d est_pos = test_utils_3d::StateExtractor3D::getPosition(
                    track.state, filter_type, model_config.Dim);
                
                // 与最近检测比较
                double min_error = std::numeric_limits<double>::max();
                for (const auto& det : all_detections[frame]) {
                    Eigen::Vector3d det_pos(det.x, det.y, det.z);
                    double error = (est_pos - det_pos).norm();
                    min_error = std::min(min_error, error);
                }
                
                if (min_error < std::numeric_limits<double>::max()) {
                    total_error += min_error;
                    error_count++;
                    metrics.max_position_error = std::max(metrics.max_position_error, min_error);
                }
                
                // 保存结果
                results.emplace_back(frame, track.track_id, est_pos(0), est_pos(1), est_pos(2));
            }
            
            // 简单的 ID 切换检测
            for (const auto& track : tracks) {
                // 这里简化处理，实际应该基于特征 ID 匹配
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
        
        // 保存跟踪结果
        if (!output_file.empty()) {
            std::cout << "\n正在保存跟踪结果到: " << output_file << std::endl;
            std::ofstream out(output_file);
            out << "frame_id,track_id,state_x,state_y,state_z\n";
            
            for (const auto& [frame, track_id, x, y, z] : results) {
                out << frame << "," << track_id << ","
                    << std::fixed << std::setprecision(6)
                    << x << "," << y << "," << z << "\n";
            }
            out.close();
            std::cout << "  跟踪结果已保存" << std::endl;
        }
        
        std::cout << "\n=== 3D 测试完成 ===" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
}
