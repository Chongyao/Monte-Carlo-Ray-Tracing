#include "kd_tree.h"

using namespace std;
using namespace Eigen;
using vec = Eigen::Vector3d;

vec find_mid_point(const vector<shared_ptr<tri_aabb>>& childrens){
  vec mid_point= vec::Zero();
  for(auto& child : childrens){
    mid_point += child->center;
  }
  mid_point *= 1.0 / childrens.size();
  return std::move(mid_point);
}



KD_tree_tris::KD_tree_tris(vector<shared_ptr<tri_aabb>>& childrens, const size_t& height, const size_t& dim, const shared_ptr<aabb>& parent_aabb, bool is_root){
  if(is_root)
    bd_box_ = merge_tri_aabbs(childrens);
  else
    bd_box_ = *parent_aabb;
  
  child_ = childrens;
  
  vec mid_point =  find_mid_point(childrens);
  vector<shared_ptr<tri_aabb>> left_childrens(0), right_childrens(0);
  shared_ptr<aabb> left_bdbox_ptr = make_shared<aabb>(), right_bdbox_ptr = make_shared<aabb>();
  for(auto& child_ptr : childrens){
    if(child_ptr->center(dim) < mid_point(dim)){
      left_childrens.push_back(child_ptr);
      left_bdbox_ptr->merge(*child_ptr);
    }
    else{
      right_childrens.push_back(child_ptr);
      right_bdbox_ptr->merge(*child_ptr);
    }
  }

  if(height == 0 || left_childrens.empty() || right_childrens.empty()){
    left_tree_ = nullptr;
    right_tree_ = nullptr;
  }else{
    left_tree_ = unique_ptr<KD_tree_tris>(new KD_tree_tris(left_childrens, height - 1, (dim + 1) % 3, left_bdbox_ptr));
    right_tree_ = unique_ptr<KD_tree_tris>(new KD_tree_tris(right_childrens, height - 1, (dim + 1) % 3, right_bdbox_ptr));
  }
}



