#ifndef MODELS_H
#define MODELS_H

#include <Eigen/Dense>
#include <functional>
#include <vector>

Eigen::MatrixXd kroneckerProduct(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);

/**
 * @class Models
 * @brief 抽象基类，定义卡尔曼滤波器的接口。
 */
class Models {
public:
  /**
     * @brief 构造函数。
     */
  Models();

  /**
     * @brief 析构函数。
     */
  virtual ~Models() = default;

  /**
     * @brief 初始化卡尔曼滤波器。
     * @param X_0 初始状态向量。
     */
  virtual void KalmanFilterInit(const Eigen::VectorXd &X_0) = 0;

  /**
     * @brief 执行卡尔曼滤波器的整个过程。
     * @param measurements 测量值的向量。
     * @return 滤波后的状态矩阵。
     */
  virtual Eigen::MatrixXd KalmanFilterWholeProcess(
    const std::vector<Eigen::VectorXd> &measurements) = 0;

  /**
     * @brief 执行卡尔曼滤波器的单次迭代。
     * @param Z 当前测量值。
     * @return 滤波后的状态向量。
     */
  virtual Eigen::VectorXd KalmanFilterIterator(const Eigen::VectorXd &Z) = 0;

  /**
     * @brief 预测下一时刻的状态。
     * @return 预测的状态矩阵。
     */
  Eigen::MatrixXd getCurrentPridection() const { return H.transpose() * X_prior; }

  /**
     * @brief 预测下一时刻的状态。
     * @return 预测的状态矩阵。
     */
  virtual Eigen::MatrixXd predict() const;

  /**
     * @brief 预测未来 N 时刻的状态。
     * @param N 未来时刻数。
     * @return 预测的状态矩阵。
     */
  virtual Eigen::MatrixXd predict(int N) const;

  /**
     * @brief 计算 Lambda 值。
     * @param Z 当前测量值。
     * @return Lambda 值。
     */
  double getLambda(const Eigen::VectorXd &Z);

protected:
  double T;                 ///< 采样时间。
  int Dim;                  ///< 状态维度。
  Eigen::MatrixXd Q;        ///< 过程噪声协方差矩阵。
  Eigen::VectorXd w_means;  ///< 过程噪声均值向量。
  Eigen::MatrixXd F;        ///< 状态转移矩阵。
  Eigen::MatrixXd H;        ///< 观测矩阵。
  Eigen::MatrixXd R;        ///< 观测噪声协方差矩阵。
  Eigen::VectorXd v_means;  ///< 观测噪声均值向量。
  Eigen::VectorXd X_prior;  ///< 滤波前的状态向量。
  Eigen::MatrixXd P_prior;  ///< 滤波前的协方差矩阵。
  Eigen::VectorXd X_after;  ///< 滤波后的状态向量。
  Eigen::MatrixXd P_after;  ///< 滤波后的协方差矩阵。
  Eigen::MatrixXd I;        ///< 单位矩阵。

public:
  /**
     * @brief 设置状态维度。
     * @param dim 状态维度。
     */
  void set_Dim(int dim) { Dim = dim; }

  /**
     * @brief 获取状态维度。
     * @return 状态维度。
     */
  int get_Dim() const { return Dim; }

  /**
     * @brief 设置过程噪声协方差矩阵。
     * @param q 过程噪声协方差矩阵。
     */
  void set_Q(const Eigen::MatrixXd &q) { Q = q; }

  /**
     * @brief 获取过程噪声协方差矩阵。
     * @return 过程噪声协方差矩阵。
     */
  Eigen::MatrixXd get_Q() const { return Q; }

  /**
     * @brief 设置过程噪声均值向量。
     * @param wMeans 过程噪声均值向量。
     */
  void set_w_means(const Eigen::VectorXd &wMeans) { w_means = wMeans; }

  /**
     * @brief 获取过程噪声均值向量。
     * @return 过程噪声均值向量。
     */
  Eigen::VectorXd get_w_means() const { return w_means; }

  /**
     * @brief 设置状态转移矩阵。
     * @param f 状态转移矩阵。
     */
  void set_F(const Eigen::MatrixXd &f) { F = f; }

  /**
     * @brief 获取状态转移矩阵。
     * @return 状态转移矩阵。
     */
  Eigen::MatrixXd get_F() const { return F; }

  /**
     * @brief 设置观测矩阵。
     * @param h 观测矩阵。
     */
  void set_H(const Eigen::MatrixXd &h) { H = h; }

  /**
     * @brief 获取观测矩阵。
     * @return 观测矩阵。
     */
  Eigen::MatrixXd get_H() const { return H; }

  /**
     * @brief 设置观测噪声协方差矩阵。
     * @param r 观测噪声协方差矩阵。
     */
  void set_R(const Eigen::MatrixXd &r) { R = r; }

  /**
     * @brief 获取观测噪声协方差矩阵。
     * @return 观测噪声协方差矩阵。
     */
  Eigen::MatrixXd get_R() const { return R; }

  /**
     * @brief 设置观测噪声均值向量。
     * @param vMeans 观测噪声均值向量。
     */
  void set_v_means(const Eigen::VectorXd &vMeans) { v_means = vMeans; }

  /**
     * @brief 获取观测噪声均值向量。
     * @return 观测噪声均值向量。
     */
  Eigen::VectorXd get_v_means() const { return v_means; }

  /**
     * @brief 设置滤波后的状态向量。
     * @param xAfter 滤波后的状态向量。
     */
  void set_X_after(const Eigen::VectorXd &xAfter) { this->set_X_after_func(xAfter); }

  /**
     * @brief 获取滤波后的状态向量。
     * @return 滤波后的状态向量。
     */
  Eigen::VectorXd get_X_after() const { return this->get_X_after_func(); }

  /**
     * @brief 设置滤波后的协方差矩阵。
     * @param pAfter 滤波后的协方差矩阵。
     */
  void set_P_after(const Eigen::MatrixXd &pAfter) { this->set_P_after_func(pAfter); }

  /**
     * @brief 获取滤波后的协方差矩阵。
     * @return 滤波后的协方差矩阵。
     */
  Eigen::MatrixXd get_P_after() const { return this->get_P_after_func(); }

protected:
  std::function<void(const Eigen::VectorXd &)> set_X_after_func =
    [this](const Eigen::VectorXd &xAfter) { this->defaultSetXAfter(xAfter); };
  std::function<void(const Eigen::MatrixXd &)> set_P_after_func =
    [this](const Eigen::MatrixXd &pAfter) { defaultSetPAfter(pAfter); };
  std::function<Eigen::VectorXd()> get_X_after_func = [this]() { return defaultGetXAfter(); };
  std::function<Eigen::MatrixXd()> get_P_after_func = [this]() { return defaultGetPAfter(); };
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> set_Z_func =
    [this](const Eigen::VectorXd &Z) { return defaultSetZ(Z); };

public:
  /**
     * @brief 默认设置滤波后的状态。
     * @param xAfter 滤波后的状态向量。
     */
  void defaultSetXAfter(const Eigen::VectorXd &xAfter) { X_after = xAfter; }

  /**
     * @brief 默认设置滤波后的协方差矩阵。
     * @param pAfter 滤波后的协方差矩阵。
     */
  void defaultSetPAfter(const Eigen::MatrixXd &pAfter) { P_after = pAfter; }

  /**
     * @brief 设置自定义的设置滤波后状态的函数。
     * @param setXAfter 自定义的设置滤波后状态的函数。
     */
  void setSetXAfterFunction(std::function<void(const Eigen::VectorXd &)> setXAfter) {
    set_X_after_func = setXAfter;
  }

  /**
     * @brief 设置自定义的设置滤波后协方差矩阵的函数。
     * @param setPAfter 自定义的设置滤波后协方差矩阵的函数。
     */
  void setSetPAfterFunction(const std::function<void(const Eigen::MatrixXd &)> &setPAfter) {
    set_P_after_func = setPAfter;
  }

  /**
     * @brief 默认获取滤波后的状态。
     * @return 滤波后的状态向量。
     */
  Eigen::VectorXd defaultGetXAfter() const { return X_after; }

  /**
     * @brief 默认获取滤波后的协方差矩阵。
     * @return 滤波后的协方差矩阵。
     */
  Eigen::MatrixXd defaultGetPAfter() const { return P_after; }

  /**
     * @brief 设置自定义的获取滤波后状态的函数。
     * @param getXAfter 自定义的获取滤波后状态的函数。
     */
  void setGetXAfterFunction(const std::function<Eigen::VectorXd()> &getXAfter) {
    get_X_after_func = getXAfter;
  }

  /**
     * @brief 设置自定义的获取滤波后协方差矩阵的函数。
     * @param getPAfter 自定义的获取滤波后协方差矩阵的函数。
     */
  void setGetPAfterFunction(const std::function<Eigen::MatrixXd()> &getPAfter) {
    get_P_after_func = getPAfter;
  }

  /**
     * @brief 默认设置观测值。
     * @param Z 当前测量值。
     * @return 处理后的测量值。
     */
  virtual Eigen::VectorXd defaultSetZ(const Eigen::VectorXd &Z) { return Z; }

  /**
     * @brief 设置自定义的设置观测值的函数。
     * @param setZ 自定义的设置观测值的函数。
     */
  void setSetZFunction(const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> &setZ) {
    set_Z_func = setZ;
  }
};

#endif  // MODELS_H
