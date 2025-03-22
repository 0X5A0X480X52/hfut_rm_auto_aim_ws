#include "armor_solver/armorFliter.hpp"

// #include "armor_solver/CS_KF.h"

// #include "basic_models/CS_KF.h"

// void ArmorFliter::init(std::vector<Eigen::Vector3d> initstate) {
//   models.clear();
//   for (std::size_t i = 0; i < armors_num; i++) {
//     std::shared_ptr<Models> model = std::make_shared<CS_KF>(T, a, A_max, Dim, R);

//     Eigen::Vector3d p = initstate[i];
//     Eigen::MatrixXd X_0(9, 1);
//     X_0 << p.x(), 0, 0, p.y(), 0, 0, p.z(), 0, 0;

//     model->KalmanFilterInit(X_0);
//     models.push_back(model);
//   }
// }

std::vector<Eigen::Vector3d> ArmorFliter::update(std::vector<Eigen::Vector3d> state,
                                                 std::string id,
                                                 std::size_t armors_num) {
  if (id != this->id) {
    this->id = id;
    this->armors_num = armors_num;
    init(state);
    FYT_DEBUG("armor_solver", "init new filter, id {}, armors_num {}", id, armors_num);
  }

  if (state.size() != armors_num) {
    throw std::runtime_error("The size of state is not equal to armors_num");
  }

  std::vector<Eigen::Vector3d> former_state;
  for (auto model : models) {
    former_state.push_back(model->predict(KM_predict_iter));
  }
  FYT_DEBUG("armor_solver", "former_state settef");

  double totalDist;
  std::vector<std::pair<int, int>> pairs = km.compute(former_state, state, &totalDist);

  FYT_DEBUG("armor_solver", "totalDist {}", totalDist);

  // 如果偏差过大，重置跟踪器
  if ( ttl-- < 0 || totalDist > totalDist_threshold) {

    ttl = default_ttl;
    init(state);
    FYT_DEBUG("armor_solver", "flier reset, id {}, armors_num {}", id, armors_num);
    return state;
  }

  std::vector<Eigen::Vector3d> predicts;
  for (auto pair : pairs) {
    int model_id = pair.first;
    int state_id = pair.second;

    auto model = models[model_id];

    model->KalmanFilterIterator(state[state_id]);

    Eigen::MatrixXd predict = model->getCurrentPridection();
    predicts.push_back(predict);
  }

  return predicts;
}

std::vector<Eigen::Vector3d> ArmorFliter::predict(std::vector<Eigen::Vector3d> state, int iters) {
  std::vector<Eigen::Vector3d> former_state;
  for (auto model : models) {
    former_state.push_back(model->predict(KM_predict_iter));
  }

  double totalDist;
  std::vector<std::pair<int, int>> pairs = km.compute(former_state, state, &totalDist);

  std::vector<Eigen::Vector3d> predictions;
  for (int i = 0; i < static_cast<int>(armors_num); i++) {
    for (auto pair : pairs) {
      int model_id = pair.first;
      int state_id = pair.second;
      if (i == state_id) {
        auto model = models[model_id];
        predictions.push_back(model->predict(iters));
      }
    }
  }

  return predictions;
}