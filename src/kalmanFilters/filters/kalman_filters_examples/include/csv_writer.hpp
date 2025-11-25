#ifndef CSV_WRITER_HPP
#define CSV_WRITER_HPP

#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

/**
 * @brief CSV file writer for filter results
 */
class CsvWriter {
public:
    /**
     * @brief Write filter results to CSV file
     * @param filename Output filename
     * @param headers Column headers
     * @param data Data matrix (rows x cols)
     */
    static void write(const std::string& filename,
                     const std::vector<std::string>& headers,
                     const Eigen::MatrixXd& data)
    {
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            throw std::runtime_error("Cannot create file: " + filename);
        }
        
        // Write headers
        for (size_t i = 0; i < headers.size(); ++i) {
            file << headers[i];
            if (i < headers.size() - 1) file << ",";
        }
        file << "\n";
        
        // Write data
        for (int i = 0; i < data.rows(); ++i) {
            for (int j = 0; j < data.cols(); ++j) {
                file << data(i, j);
                if (j < data.cols() - 1) file << ",";
            }
            file << "\n";
        }
        
        file.close();
    }
    
    /**
     * @brief Write filter results with time column
     * @param filename Output filename
     * @param headers Column headers (excluding time)
     * @param time_data Time vector
     * @param data Data matrix
     */
    static void writeWithTime(const std::string& filename,
                             const std::vector<std::string>& headers,
                             const Eigen::VectorXd& time_data,
                             const Eigen::MatrixXd& data)
    {
        std::ofstream file(filename);
        
        if (!file.is_open()) {
            throw std::runtime_error("Cannot create file: " + filename);
        }
        
        // Write headers
        file << "time";
        for (const auto& header : headers) {
            file << "," << header;
        }
        file << "\n";
        
        // Write data
        for (int i = 0; i < data.rows(); ++i) {
            file << time_data(i);
            for (int j = 0; j < data.cols(); ++j) {
                file << "," << data(i, j);
            }
            file << "\n";
        }
        
        file.close();
    }
};

#endif // CSV_WRITER_HPP
