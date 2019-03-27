#ifndef AABB_H_ZCY
#define AABB_H_ZCY
#include <Eigen/Core>
#include <memory>
#include <vector>
using vec = Eigen::Vector3d;
using tri = Eigen::Matrix3d;
class aabb{
 public:
  aabb(const vec& low_bd = vec::Zero(), const vec& up_bd = vec::Zero());
  vec low_bd_;
  vec up_bd_;
  void merge(const aabb& other);
};

class tri_aabb : public aabb{
 public:
  size_t id_;
  vec normal_;
  double d_;
  vec center;
  tri p_;
  tri_aabb(const size_t& id, const vec& p1, const vec& p2, const vec& p3);
  tri_aabb(const size_t& id, const tri& plane);
  tri_aabb(){}
};




aabb merge_tri_aabbs(const std::vector<std::shared_ptr<tri_aabb>> tri_aabbs);

#endif
