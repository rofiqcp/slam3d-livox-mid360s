#include "Utils.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
int main() {
  using kiss_icp_ros::utils::NormalizeTimestamps;
  auto require=[](bool ok){if(!ok)throw std::runtime_error("timestamp regression");};
  require(NormalizeTimestamps({}).empty());
  require(NormalizeTimestamps({7.0}).empty());
  require(NormalizeTimestamps({7.0,7.0}).empty());
  require(NormalizeTimestamps({0.0,std::numeric_limits<double>::quiet_NaN()}).empty());
  require(NormalizeTimestamps({0.0,std::numeric_limits<double>::infinity()}).empty());
  const auto result=NormalizeTimestamps({30.0,10.0,20.0});
  require(result.size()==3 && result[0]==1.0 && result[1]==0.0 && result[2]==0.5);
}
