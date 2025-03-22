#include "armor_solver/KM.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

const double INF = 1e9;
const double eps = 1e-8;

KMAlgorithm::KMAlgorithm(int maxN) : maxN(maxN) {
  w.resize(maxN, std::vector<double>(maxN));
  lx.resize(maxN);
  ly.resize(maxN);
  slack.resize(maxN);
  match.resize(maxN);
  visx.resize(maxN);
  visy.resize(maxN);
}

double KMAlgorithm::distance(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  return (a - b).norm();
}

bool KMAlgorithm::dfs(int x) {
  visx[x] = true;
  for (int y = 0; y < n; y++) {
    if (visy[y]) continue;
    double t = lx[x] + ly[y] - w[x][y];
    if (fabs(t) < eps) {
      visy[y] = true;
      if (match[y] == -1 || dfs(match[y])) {
        match[y] = x;
        return true;
      }
    } else {
      slack[y] = std::min(slack[y], t);
    }
  }
  return false;
}

std::vector<std::pair<int, int>> KMAlgorithm::compute(std::vector<Eigen::Vector3d>& setA, std::vector<Eigen::Vector3d>& setB, double* totalDist) {
    n = setA.size();
    // 构建权值矩阵
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            w[i][j] = -distance(setA[i], setB[j]); // 取负是为了转换最小值问题为最大值问题
        }
    }
    
    // 初始化顶标
    for (int i = 0; i < n; i++) {
        lx[i] = -INF;
        ly[i] = 0;
        for (int j = 0; j < n; j++) {
            lx[i] = std::max(lx[i], w[i][j]);
        }
    }
    
    std::fill(match.begin(), match.begin() + n, -1);
    
    // 对每个点寻找匹配
    for (int i = 0; i < n; i++) {
        std::fill(slack.begin(), slack.begin() + n, INF);
        while (true) {
            std::fill(visx.begin(), visx.begin() + n, false);
            std::fill(visy.begin(), visy.begin() + n, false);
            if (dfs(i)) break;
            
            double d = INF;
            for (int j = 0; j < n; j++) {
                if (!visy[j]) d = std::min(d, slack[j]);
            }
            for (int j = 0; j < n; j++) {
                if (visx[j]) lx[j] -= d;
                if (visy[j]) ly[j] += d;
                else slack[j] -= d;
            }
        }
    }
    
    // 生成匹配结果
    std::vector<std::pair<int, int>> result;
    *totalDist = 0;
    for (int i = 0; i < n; i++) {
        if (match[i] != -1) {
            result.emplace_back(match[i], i);
            *totalDist += distance(setA[match[i]], setB[i]);
        }
    }
    return result;
}

