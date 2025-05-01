#include <Eigen/Dense>
#include <memory>
#include <vector>

// #include "armor_solver/models.h"

#include "armor_solver/KM.hpp"
#include "rm_utils/logger/log.hpp"
#include <stddef.h>

// models
#include "models/models.h"
#include "basic_models/Singer_KF.h"
#include "basic_models/CS_KF.h"
#include "basic_models/CA_KF.h"
#include "combined_models/IMM_CV_CA_CS_3Dim.h"
#include "combined_models/IMM_CV_CA_CS_Singer_3Dim.h"

class ArmorFliter {
  using ModelFliter = std::shared_ptr<Models>;
  std::vector<ModelFliter> models;

  std::string id = "NULL";
  std::size_t armors_num = 114514;

  int KM_predict_iter = 0;          // 匹配时预测下一位置
  double totalDist_threshold = 1;  // 匹配时偏差总距离超过该值，重置跟踪器

  int default_ttl = 100000;
  int ttl = default_ttl;

  double T;
  double a;
  double A_max;
  int Dim;
  Eigen::MatrixXd R;

  // void setDefaultParameters() {
  //   this->T = 0.01;
  //   this->a = 5;
  //   this->A_max = 3;
  //   this->Dim = 3;
  //   this->R = Eigen::MatrixXd(3, 3);
  //   this->R << 0.005, 0.001, 0.001, 0.001, 0.005, 0.001, 0.001, 0.001, 0.005;
  // }  
  
  void setDefaultParameters() {
    this->T = 0.016;
    this->a = 1000;
    this->A_max = 100;
    this->Dim = 3;
    this->R = Eigen::MatrixXd(3, 3);
    this->R << 0.005, 0.001, 0.001, 0.001, 0.005, 0.001, 0.001, 0.001, 0.005;
  }

  KMAlgorithm km = KMAlgorithm(4);

public:
  ArmorFliter() { setDefaultParameters(); }

  void init(std::vector<Eigen::Vector3d> initstate) {
    models.clear();
    for (std::size_t i = 0; i < armors_num; i++) {

      Eigen::Vector3d p = initstate[i];
      Eigen::MatrixXd X_0(9, 1);
      X_0 << p.x(), 0, 0, p.y(), 0, 0, p.z(), 0, 0;

      std::shared_ptr<Models> model = std::make_shared<CS_KF>(T, a, A_max, Dim, R); // CS 3Dim 的模型
      model->KalmanFilterInit(X_0);

      // double sigma = 5;
      // std::shared_ptr<Models> model = std::make_shared<Singer_KF>(T, a, sigma, Dim, R);

      // std::shared_ptr<Models> model = std::make_shared<CA_KF>(T, Dim, R);
      // model->KalmanFilterInit(X_0);

      // double T = 0.01;
      // // int Dim = 3;
      // Eigen::MatrixXd R(3, 3);
      // this->R << 0.005, 0.001, 0.001, 0.001, 0.005, 0.001, 0.001, 0.001, 0.005;

      // Eigen::MatrixXd transformRateMat(3,3);
      // transformRateMat << 0.80, 0.10, 0.10,
      //                     0.10, 0.80, 0.10,
      //                     0.10, 0.10, 0.80;
  
      // Eigen::MatrixXd H(3,9);
      // H.setZero();
      // H(0,0) = 1;
      // H(1,3) = 1;
      // H(2,6) = 1;

      // // std::shared_ptr<Models> model = std::make_shared<IMM_CV_CA_CS_3Dim>(X_0); // IMM_CV_CA_CS_3Dim 的模型
      // std::shared_ptr<Models> model = std::make_shared<IMM_CV_CA_CS_3Dim>(T, 9, H, transformRateMat, X_0, R); // IMM_CV_CA_CS_3Dim 的模型


      // Eigen::MatrixXd transformRateMat(4,4);
      // transformRateMat << 0.40, 0.20, 0.20, 0.20,
      //                     0.20, 0.40, 0.20, 0.20,
      //                     0.20, 0.20, 0.40, 0.20,
      //                     0.20, 0.20, 0.20, 0.40;

      // std::shared_ptr<Models> model = std::make_shared<IMM_CV_CA_CS_Singer_3Dim>(T, 9, H, transformRateMat, X_0, R); // IMM_CV_CA_CS_Singer_3Dim 的模型

      models.push_back(model);
      FYT_DEBUG("armor_solver", "ArmorFliter {} inited!", i);
    }
  }

  std::vector<Eigen::Vector3d> update(std::vector<Eigen::Vector3d> state,
                                      std::string id,
                                      std::size_t armors_num);

  std::vector<Eigen::Vector3d> predict(std::vector<Eigen::Vector3d> state, int iters);

  void setLost() {
    this->id = "LOST";
    this->armors_num = 114514;
  }
};