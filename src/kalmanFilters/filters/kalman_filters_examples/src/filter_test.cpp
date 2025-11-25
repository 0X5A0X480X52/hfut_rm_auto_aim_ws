/**
 * @file filter_test.cpp
 * @brief Kalman Filter Test Program
 * 
 * This program tests various Kalman filter models by:
 * 1. Reading configuration from YAML file
 * 2. Loading test data from CSV file
 * 3. Running the filter
 * 4. Saving results to CSV file
 */

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include "models/model_factory.h"
#include "yaml_config_reader.hpp"
#include "csv_reader.hpp"
#include "csv_writer.hpp"

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <config_file> <input_csv> <output_csv> [meas_dim]\n";
    std::cout << "\n";
    std::cout << "Arguments:\n";
    std::cout << "  config_file  : Path to YAML configuration file\n";
    std::cout << "  input_csv    : Path to input CSV file with test data\n";
    std::cout << "  output_csv   : Path to output CSV file for results\n";
    std::cout << "  meas_dim     : Measurement dimension (default: auto-detect from config)\n";
    std::cout << "\n";
    std::cout << "Example:\n";
    std::cout << "  " << program_name << " config/cv_kf_2d.yaml test_data/cv_2d.csv results/cv_2d_result.csv\n";
}

int main(int argc, char** argv) {
    if (argc < 4) {
        printUsage(argv[0]);
        return 1;
    }
    
    std::string config_file = argv[1];
    std::string input_csv = argv[2];
    std::string output_csv = argv[3];
    
    try {
        std::cout << "=== Kalman Filter Test ===" << std::endl;
        std::cout << "Config file: " << config_file << std::endl;
        std::cout << "Input data:  " << input_csv << std::endl;
        std::cout << "Output file: " << output_csv << std::endl;
        std::cout << std::endl;
        
        // 1. Load configuration
        std::cout << "Loading configuration..." << std::endl;
        std::string filter_type = YamlConfigReader::getFilterType(config_file);
        ModelConfig config = YamlConfigReader::loadConfig(config_file);
        
        std::cout << "  Filter type: " << filter_type << std::endl;
        std::cout << "  Sampling time (T): " << config.T << " s" << std::endl;
        std::cout << "  Dimension: " << config.Dim << "D" << std::endl;
        std::cout << "  Initial state dim: " << config.X_0.size() << std::endl;
        std::cout << std::endl;
        
        // 2. Create filter model
        std::cout << "Creating filter model..." << std::endl;
        auto model = ModelFactoryRegistry::getInstance().createModel(filter_type, config);
        if (!model) {
            throw std::runtime_error("Failed to create model: " + filter_type);
        }
        std::cout << "  Model created successfully" << std::endl;
        std::cout << std::endl;
        
        // 3. Read input data
        std::cout << "Reading input data..." << std::endl;
        std::vector<std::string> headers;
        Eigen::MatrixXd all_data = CsvReader::readAll(input_csv, headers);
        
        std::cout << "  Data shape: " << all_data.rows() << " x " << all_data.cols() << std::endl;
        std::cout << "  Headers: ";
        for (size_t i = 0; i < headers.size(); ++i) {
            std::cout << headers[i];
            if (i < headers.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
        
        // Extract measurements (last Dim columns are measurements)
        int meas_dim = config.Dim;
        int meas_start_col = all_data.cols() - meas_dim;
        
        std::vector<Eigen::VectorXd> measurements;
        for (int i = 0; i < all_data.rows(); ++i) {
            Eigen::VectorXd meas(meas_dim);
            for (int j = 0; j < meas_dim; ++j) {
                meas(j) = all_data(i, meas_start_col + j);
            }
            measurements.push_back(meas);
        }
        
        std::cout << "  Extracted " << measurements.size() << " measurements" << std::endl;
        std::cout << "  Measurement dimension: " << meas_dim << std::endl;
        std::cout << std::endl;
        
        // 4. Run filter
        std::cout << "Running filter..." << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now();
        
        Eigen::MatrixXd filtered_states = model->KalmanFilterWholeProcess(measurements);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        std::cout << "  Processed " << filtered_states.rows() << " steps" << std::endl;
        std::cout << "  State dimension: " << filtered_states.cols() << std::endl;
        std::cout << "  Processing time: " << duration.count() / 1000.0 << " ms" << std::endl;
        std::cout << "  Average time per step: " << duration.count() / (double)measurements.size() << " μs" << std::endl;
        std::cout << std::endl;
        
        // 5. Prepare output data
        std::cout << "Preparing output..." << std::endl;
        
        // Combine time, measurements, and filtered states
        int state_dim = filtered_states.cols();
        Eigen::MatrixXd output_data(all_data.rows(), 1 + meas_dim + state_dim);
        
        for (int i = 0; i < all_data.rows(); ++i) {
            output_data(i, 0) = all_data(i, 0); // time
            
            // measurements
            for (int j = 0; j < meas_dim; ++j) {
                output_data(i, 1 + j) = measurements[i](j);
            }
            
            // filtered states
            for (int j = 0; j < state_dim; ++j) {
                output_data(i, 1 + meas_dim + j) = filtered_states(i, j);
            }
        }
        
        // Generate headers
        std::vector<std::string> output_headers;
        output_headers.push_back("time");
        
        // Measurement headers
        for (int i = 0; i < meas_dim; ++i) {
            if (meas_dim == 2) {
                output_headers.push_back(i == 0 ? "meas_x" : "meas_y");
            } else if (meas_dim == 3) {
                std::string suffix = (i == 0) ? "x" : (i == 1) ? "y" : "z";
                output_headers.push_back("meas_" + suffix);
            } else {
                output_headers.push_back("meas_" + std::to_string(i));
            }
        }
        
        // State headers (depends on filter type)
        if (filter_type == "CV_KF") {
            // CV: [x, vx, y, vy] or [x, vx, y, vy, z, vz]
            for (int d = 0; d < meas_dim; ++d) {
                std::string axis = (d == 0) ? "x" : (d == 1) ? "y" : "z";
                output_headers.push_back("est_" + axis);
                output_headers.push_back("est_v" + axis);
            }
        } else if (filter_type == "CA_KF" || filter_type == "CS_KF" || filter_type == "Singer_KF") {
            // CA/CS/Singer: [x, vx, ax, y, vy, ay] or [..., z, vz, az]
            for (int d = 0; d < meas_dim; ++d) {
                std::string axis = (d == 0) ? "x" : (d == 1) ? "y" : "z";
                output_headers.push_back("est_" + axis);
                output_headers.push_back("est_v" + axis);
                output_headers.push_back("est_a" + axis);
            }
        } else if (filter_type == "CTRV_EKF") {
            // CTRV: [x, y, v, theta, omega]
            output_headers.push_back("est_x");
            output_headers.push_back("est_y");
            output_headers.push_back("est_v");
            output_headers.push_back("est_theta");
            output_headers.push_back("est_omega");
        } else {
            // Generic state naming
            for (int i = 0; i < state_dim; ++i) {
                output_headers.push_back("est_state_" + std::to_string(i));
            }
        }
        
        // 6. Write output
        std::cout << "Writing results to " << output_csv << "..." << std::endl;
        CsvWriter::write(output_csv, output_headers, output_data);
        
        std::cout << "  Output shape: " << output_data.rows() << " x " << output_data.cols() << std::endl;
        std::cout << std::endl;
        
        std::cout << "=== Test completed successfully ===" << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
