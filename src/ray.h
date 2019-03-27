#ifndef RAY_H_ZCY
#define RAY_H_ZCY
#include <Eigen/Core>
#include "aabb.h"
using vec = Eigen::Vector3d;
class Ray{
 public:
  vec origin_, dire_;
  Ray(const vec& orgin = vec::Zero(), const vec& dire = vec::Zero());
  bool intersect(const std::unique_ptr<KD_tree_tris>& kd, size_t& face_id, vec& cross_point)
 private:
  bool intersect_aabb(const aabb& bd_box_);
  bool intersct_tri_aabb(const tri_aabb& tri, vec&cross_point)
};


#endif
