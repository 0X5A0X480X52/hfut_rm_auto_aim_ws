#include "muit_obj_tracker/utils/hungarian.hpp"
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>

namespace muit_obj_tracker {

double HungarianAlgorithm::Solve(const std::vector<std::vector<double>>& distMatrix, std::vector<int>& assignment) {
    int n = distMatrix.size();
    if (n == 0) return 0.0;
    int m = distMatrix[0].size();
    if (m == 0) return 0.0;
    
    assignment.assign(n, -1);
    
    int dim = std::max(n, m);
    std::vector<std::vector<double>> a(dim + 1, std::vector<double>(dim + 1, 0));
    
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            a[i+1][j+1] = distMatrix[i][j];
        }
    }
    
    std::vector<double> u(dim + 1, 0), v(dim + 1, 0);
    std::vector<int> p(dim + 1, 0), way(dim + 1, 0);
    std::vector<double> minv(dim + 1);
    std::vector<bool> used(dim + 1);
    
    for (int i = 1; i <= dim; ++i) {
        p[0] = i;
        int j0 = 0;
        std::fill(minv.begin(), minv.end(), std::numeric_limits<double>::max());
        std::fill(used.begin(), used.end(), false);
        
        do {
            used[j0] = true;
            int i0 = p[j0], j1 = 0;
            double delta = std::numeric_limits<double>::max();
            
            for (int j = 1; j <= dim; ++j) {
                if (!used[j]) {
                    double cur = a[i0][j] - u[i0] - v[j];
                    if (cur < minv[j]) minv[j] = cur, way[j] = j0;
                    if (minv[j] < delta) delta = minv[j], j1 = j;
                }
            }
            
            for (int j = 0; j <= dim; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }
    
    double cost = 0;
    for (int j = 1; j <= dim; ++j) {
        if (p[j] != 0) {
            int row = p[j] - 1;
            int col = j - 1;
            if (row < n && col < m) {
                assignment[row] = col;
                cost += distMatrix[row][col];
            }
        }
    }
    
    return cost;
}

} // namespace muit_obj_tracker
