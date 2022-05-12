
#pragma once

// std lib
#include <stdlib.h>

#include <cmath>
#include <iostream>

// yaml cpp
#include <yaml-cpp/yaml.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/command.hpp"
#include "flightlib/common/logger.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/common/utils.hpp"
#include "flightlib/envs/env_base.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/objects/unity_object.hpp"
#include "flightlib/sensors/rgb_camera.hpp"

namespace flightlib {

namespace afenv {

enum af : int {
  //
  kNQuadState = 25,

  kNObstacles = 10,
  kNObstaclesState = 4,

  // observations
  kObs = 0,
  kNObs = 13 + kNObstacles * kNObstaclesState,

  // control actions
  kAct = 0,
  kNAct = 4,
};
}  // namespace afenv

class AgileFlightEnv final : public EnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AgileFlightEnv();
  AgileFlightEnv(const std::string &cfg_path, const int env_id);
  AgileFlightEnv(const YAML::Node &cfg_node, const int env_id);
  ~AgileFlightEnv();

  // - public OpenAI-gym-style functions
  bool reset(Ref<Vector<>> obs) override;
  bool reset(Ref<Vector<>> obs, bool random);
  bool step(const Ref<Vector<>> act, Ref<Vector<>> obs,
            Ref<Vector<>> reward) override;

  // - public set functions
  bool loadParam(const YAML::Node &cfg);

  // - public get functions
  bool getObs(Ref<Vector<>> obs) override;
  bool getImage(Ref<ImgVector<>> img, const bool rgb = true) override;
  bool getDepthImage(Ref<DepthImgVector<>> img) override;

  bool getObstacleState(Ref<Vector<>> obstacle_obs);
  // get quadrotor states
  bool getQuadAct(Ref<Vector<>> act) const;
  bool getQuadState(Ref<Vector<>> state) const;

  // - auxiliar functions
  bool isTerminalState(Scalar &reward) override;
  bool addQuadrotorToUnity(const std::shared_ptr<UnityBridge> bridge) override;
  void updateExtraInfo() ;

  friend std::ostream &operator<<(std::ostream &os,
                                  const AgileFlightEnv &vision_env);


  bool configCamera(const YAML::Node &cfg_node);
  bool configDynamicObjects(const std::string &yaml_file);
  bool configStaticObjects(const std::string &csv_file);

  bool simDynamicObstacles(const Scalar dt);

  // flightmare (visualization)
  bool setUnity(const bool render);
  bool connectUnity();
  void disconnectUnity();
  FrameID updateUnity(const FrameID frame_id);


  //
  int getNumDetectedObstacles(void);
  bool isCollision(void) { return is_collision_; };
  inline std::vector<std::string> getRewardNames() { return reward_names_; }
  inline void setSceneID(const SceneID id) { scene_id_ = id; }
  inline std::shared_ptr<Quadrotor> getQuadrotor() { return quad_ptr_; }
  inline std::vector<std::shared_ptr<UnityObject>> getDynamicObjects() {
    return dynamic_objects_;
  }
  inline std::vector<std::shared_ptr<UnityObject>> getStaticObjects() {
    return static_objects_;
  }

  std::unordered_map<std::string, float> extra_info_{
    {"2_success",0.0},{"4_outBox",0.0},{"3_outtime",0.0},{"1_collision",0.0},
    {"final_pose_x",0.0}, //{"final_pose_y",0.0},{"final_pose_z",0.0},
    {"success_noCollision",0.0}, //{"collision_stop_num",0.0},
    {"time_exp_factor",0.0},{"collision_count",0.0},
    };

  bool setCollisionStopNum(int num);

  bool setCoefficient(std::string st, Scalar value);
  Scalar getCoefficient(std::string st);
  int checkState();

 private:
  bool computeReward(Ref<Vector<>> reward);
  void init();
  int env_id_;
  // quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  //
  std::vector<std::shared_ptr<UnityObject>> static_objects_;
  std::vector<std::shared_ptr<UnityObject>> dynamic_objects_;

  QuadState quad_state_, quad_old_state_;
  Command cmd_;
  Logger logger_{"AgileFlightEnv"};

  // Define reward for training
  Scalar vel_coeff_, collision_coeff_, angular_vel_coeff_, su_time_exp_coeff_,
  su_coll_exp_coeff_, su_coll_coeff_, su_dist_exp_coeff_,
  collision_exp_coeff_,  dist_margin_, success_rew_, survive_rew_;
  Vector<3> goal_linear_vel_;

  Scalar goal_target_;
  float collision_count_ = 0.0;
  float success_factor_ = 1.0;

  // flag
  int terminalState_ = 0; //run:0   ;colli:1    ;succ: 2  ;outtime: 3   ;outbox: 4;
  bool is_collision_ = false, allow_rewrite_paras_ = false;
  Scalar collision_stop_num_ = -1;

  // max detection range (meter)
  Scalar max_detection_range_;
  std::vector<Scalar> relative_pos_norm_;
  std::vector<Scalar> obstacle_radius_;


  int num_detected_obstacles_;
  std::string difficulty_level_;
  std::string env_folder_;
  std::vector<Scalar> world_box_;

  // observations and actions (for RL)
  Vector<afenv::kNObs> pi_obs_;
  Vector<afenv::kNAct> pi_act_;
  Vector<afenv::kNAct> old_pi_act_;

  // action and observation normalization (for learning)
  Vector<afenv::kNAct> act_mean_;
  Vector<afenv::kNAct> act_std_;
  Vector<afenv::kNObs> obs_mean_ = Vector<afenv::kNObs>::Zero();
  Vector<afenv::kNObs> obs_std_ = Vector<afenv::kNObs>::Ones();

  // robot vision
  std::shared_ptr<RGBCamera> rgb_camera_;
  cv::Mat rgb_img_, gray_img_;
  cv::Mat depth_img_;

  // auxiliary variables
  bool use_camera_{false};
  YAML::Node cfg_;
  std::vector<std::string> reward_names_;

  // Unity Rendering
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};
  Vector<3> unity_render_offset_;

  //
  std::string static_object_csv_;
  std::string obstacle_cfg_path_;
  int num_dynamic_objects_;
  int num_static_objects_;
};

}  // namespace flightlib