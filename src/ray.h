#ifndef RAY_H_ZCY
#define RAY_H_ZCY
#include <Eigen/Core>
#include "aabb.h"
#include "kd_tree.h"
using vec = Eigen::Vector3d;
class Ray{
 public:
  vec origin_, dire_;
  mutable double final_offset_;
  Ray(const vec& orgin = vec::Zero(), const vec& dire = vec::Zero());
  bool intersect_forest(const std::vector<std::unique_ptr<KD_tree_tris>>& KD_forest,
                        size_t& face_id, Ray& next)const;
  bool intersect_forest_loop(const std::vector<std::unique_ptr<KD_tree_tris>>& KD_forest,
                        size_t& face_id, Ray& next)const;  

 private:
  bool intersect(const std::unique_ptr<KD_tree_tris>& kd, size_t& face_id, Ray& next)const;
  bool intersect_aabb(const aabb& bd_box_)const;
  bool intersct_tri_aabb(const tri_aabb& tri, Ray& next)const;
};

bool get_cross_point(const vec& ori, const vec& dir, const vec& norm, const double& d, double& offset );
#endif
