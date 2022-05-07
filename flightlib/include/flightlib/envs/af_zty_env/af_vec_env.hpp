#pragma once

// yaml cpp
#include <yaml-cpp/yaml.h>

#include "flightlib/envs/vec_env_base.hpp"
#include "flightlib/envs/af_zty_env/af_env.hpp"

namespace flightlib {

template<typename EnvBaseName>
class AgileFlightVecEnv : public VecEnvBase<EnvBaseName> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AgileFlightVecEnv();
  AgileFlightVecEnv(const std::string& cfg, const bool from_file = true);
  AgileFlightVecEnv(const YAML::Node& cfg_node);
  ~AgileFlightVecEnv();

  using VecEnvBase<EnvBaseName>::configEnv;

  bool reset(Ref<MatrixRowMajor<>> obs) override;
  bool reset(Ref<MatrixRowMajor<>> obs, bool random);
  bool step(Ref<MatrixRowMajor<>> act, Ref<MatrixRowMajor<>> obs,
            Ref<MatrixRowMajor<>> reward, Ref<BoolVector<>> done,
            Ref<MatrixRowMajor<>> extra_info) override;


  bool getQuadAct(Ref<MatrixRowMajor<>> quadact);
  bool getQuadState(Ref<MatrixRowMajor<>> quadstate);
  inline std::vector<std::string> getRewardNames(void) {
    return this->envs_[0]->getRewardNames();
  };

  // code build time version
  std::string getCodeBuildTime() {return "[2022-05-06-20:02]";}

  // set collision ealy stop
  bool setCollisionStopNum(int num);

 private:
  void perAgentStep(int agent_id, Ref<MatrixRowMajor<>> act,
                    Ref<MatrixRowMajor<>> obs,
                    Ref<MatrixRowMajor<>> reward_units, Ref<BoolVector<>> done,
                    Ref<MatrixRowMajor<>> extra_info) override;
  // yaml configurations
  bool random_reset_;
  // create objects
  Logger logger_{"AgileFlightVecEnv"};
  //
  YAML::Node cfg_;
  std::vector<std::string> reward_names_;
};

}  // namespace flightlib
