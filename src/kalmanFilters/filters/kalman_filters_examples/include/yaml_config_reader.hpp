#ifndef YAML_CONFIG_READER_HPP
#define YAML_CONFIG_READER_HPP

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "models/model_factory.h"

/**
 * @brief YAML configuration reader for Kalman filters
 */
class YamlConfigReader {
public:
    /**
     * @brief Load configuration from YAML file
     * @param filename Path to YAML file
     * @return ModelConfig structure
     */
    static ModelConfig loadConfig(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        ModelConfig model_config;
        
        if (!config["filter"]) {
            throw std::runtime_error("No 'filter' section in config file");
        }
        
        YAML::Node filter = config["filter"];
        
        // Read basic parameters
        if (filter["type"]) {
            // Type is stored separately and returned via getType()
        }
        
        if (filter["T"]) {
            model_config.T = filter["T"].as<double>();
        }
        
        if (filter["Dim"]) {
            model_config.Dim = filter["Dim"].as<int>();
        }
        
        // Read measurement noise covariance matrix R
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
        
        // Read initial state vector X_0
        if (filter["X_0"]) {
            auto x0_node = filter["X_0"];
            int size = x0_node.size();
            model_config.X_0 = Eigen::VectorXd(size);
            for (int i = 0; i < size; ++i) {
                model_config.X_0(i) = x0_node[i].as<double>();
            }
        }
        
        // Read transition probability matrix (for IMM)
        if (filter["transform_rate_mat"]) {
            auto mat_node = filter["transform_rate_mat"];
            int rows = mat_node.size();
            int cols = mat_node[0].size();
            
            model_config.transform_rate_mat = Eigen::MatrixXd(rows, cols);
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    model_config.transform_rate_mat(i, j) = mat_node[i][j].as<double>();
                }
            }
        }
        
        // Read extra parameters
        if (filter["extra_params"]) {
            auto params_node = filter["extra_params"];
            for (auto it = params_node.begin(); it != params_node.end(); ++it) {
                std::string key = it->first.as<std::string>();
                double value = it->second.as<double>();
                model_config.extra_params[key] = value;
            }
        }
        
        return model_config;
    }
    
    /**
     * @brief Get filter type from config file
     * @param filename Path to YAML file
     * @return Filter type string
     */
    static std::string getFilterType(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        if (config["filter"] && config["filter"]["type"]) {
            return config["filter"]["type"].as<std::string>();
        }
        throw std::runtime_error("No filter type specified in config file");
    }
};

#endif // YAML_CONFIG_READER_HPP
