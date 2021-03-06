#ifndef KD_TREE_ZCY
#define KD_TREE_ZCY
#include "aabb.h"
#include <memory>
#include <vector>
class KD_tree_tris{
 public:
  aabb bd_box_;
  std::unique_ptr<KD_tree_tris> left_tree_, right_tree_;
  std::vector<std::shared_ptr<tri_aabb>> child_;
  KD_tree_tris(std::vector<std::shared_ptr<tri_aabb>>& childrens, const size_t& height, const size_t& dim, const std::shared_ptr<aabb>& parent_aabb, const bool is_root = false);
 private:
  
};

// size_t intersect(const Ray& ray, const unique_ptr<KD_tree_tris>& kd);


#endif
