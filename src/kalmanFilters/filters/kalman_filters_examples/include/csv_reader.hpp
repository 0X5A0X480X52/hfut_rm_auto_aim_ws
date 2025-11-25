#ifndef CSV_READER_HPP
#define CSV_READER_HPP

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

/**
 * @brief CSV file reader for test data
 */
class CsvReader {
public:
    /**
     * @brief Read CSV file and extract measurements
     * @param filename Path to CSV file
     * @param meas_start_col Starting column index for measurements
     * @param meas_dim Dimension of measurements
     * @return Vector of measurement vectors
     */
    static std::vector<Eigen::VectorXd> readMeasurements(
        const std::string& filename, 
        int meas_start_col, 
        int meas_dim) 
    {
        std::vector<Eigen::VectorXd> measurements;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename);
        }
        
        std::string line;
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<double> row;
            
            // Read all columns
            while (std::getline(ss, cell, ',')) {
                row.push_back(std::stod(cell));
            }
            
            // Extract measurements
            if (row.size() >= static_cast<size_t>(meas_start_col + meas_dim)) {
                Eigen::VectorXd meas(meas_dim);
                for (int i = 0; i < meas_dim; ++i) {
                    meas(i) = row[meas_start_col + i];
                }
                measurements.push_back(meas);
            }
        }
        
        file.close();
        return measurements;
    }
    
    /**
     * @brief Read all data from CSV file
     * @param filename Path to CSV file
     * @param headers Output: column headers
     * @return Matrix where each row is a data row
     */
    static Eigen::MatrixXd readAll(const std::string& filename, 
                                   std::vector<std::string>& headers) 
    {
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename);
        }
        
        std::string line;
        
        // Read header
        std::getline(file, line);
        std::stringstream header_ss(line);
        std::string header;
        headers.clear();
        while (std::getline(header_ss, header, ',')) {
            headers.push_back(header);
        }
        
        // Read data
        std::vector<std::vector<double>> data;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<double> row;
            
            while (std::getline(ss, cell, ',')) {
                row.push_back(std::stod(cell));
            }
            data.push_back(row);
        }
        
        file.close();
        
        // Convert to Eigen matrix
        if (data.empty()) {
            return Eigen::MatrixXd();
        }
        
        int rows = data.size();
        int cols = data[0].size();
        Eigen::MatrixXd matrix(rows, cols);
        
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                matrix(i, j) = data[i][j];
            }
        }
        
        return matrix;
    }
};

#endif // CSV_READER_HPP
