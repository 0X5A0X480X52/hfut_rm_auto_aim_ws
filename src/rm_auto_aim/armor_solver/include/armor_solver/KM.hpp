#include <vector>
#include <Eigen/Dense>

class KMAlgorithm {
public:
    KMAlgorithm(int maxN);
    std::vector<std::pair<int, int>> compute(std::vector<Eigen::Vector3d>& setA, std::vector<Eigen::Vector3d>& setB, double *totalDist);

private:
    int n;
    int maxN;
    std::vector<std::vector<double>> w;
    std::vector<double> lx, ly, slack;
    std::vector<int> match;
    std::vector<bool> visx, visy;

    double distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b);
    bool dfs(int x);
};
