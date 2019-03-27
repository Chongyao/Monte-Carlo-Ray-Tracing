#include "aabb.h"
#include <cmath>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;

aabb::aabb(const vec& low_bd, const vec& up_bd){
  low_bd_ = low_bd;
  up_bd_ = up_bd;
}
aabb::aabb(const aabb& other){
  low_bd_ = other.low_bd_;
  up_bd_ = other.up_bd_;
}
void aabb::merge(const aabb& other){
  for(size_t i = 0; i < 3; ++i){
    if(low_bd_(i) > other.low_bd_(i))
      low_bd_(i) = other.low_bd_(i);
    if(up_bd_(i) < other.up_bd_(i))
      up_bd_(i) = other.up_bd_(i);
  }

  {//check
    for(size_t i = 0; i < 3; ++i){
      assert(low_bd_(i) <= other.low_bd_(i));
      assert(up_bd_(i) >= other.up_bd_(i));
    }
  }
}

tri_aabb::tri_aabb(const size_t& id, const vec& p1, const vec& p2, const vec& p3):id_(id){
  p_.col(0) = p1;
  p_.col(1) = p2;
  p_.col(2) = p3;
  id_ = id;
  low_bd_ = p1;
  up_bd_ = p2;
  for(size_t i = 0; i < 3; ++i){
    if(low_bd_(i) > p2(i))
      low_bd_(i) = p2(i);
    else if (up_bd_(i) < p2(i))
      up_bd_(i) = p2(i);

    if(low_bd_(i) > p3(i))
      low_bd_(i) = p3(i);
    else if (up_bd_(i) < p3(i))
      up_bd_(i) = p3(i);
  }

  center = (p1 + p2 + p3) / 3.0;

  
  vec one_edge = p2 - p1, other_edge = p3 - p1;
  normal_ = one_edge.cross(other_edge);
  // normal_(0) = (p2(1) - p1(1)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(1) - p1(1));
  // normal_(1) = (p2(0) - p1(0)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(0) - p1(0));
  // normal_(2) = (p2(0) - p1(0)) * (p3(1) - p1(1)) - (p2(1) - p1(2)) * (p3(0) - p1(0));

  double norm  = normal_.norm();
  if(norm > 1e-6)
    normal_ = normal_ / norm;
  else
    normal_ = vec::Zero();
  d_ = -normal_.dot(p1) ;

}
tri_aabb::tri_aabb(const size_t& id, const tri& plane){
  id_ = id;
  p_ = plane;
  vec p1 = plane.col(0), p2 = plane.col(1), p3 = plane.col(2);
  low_bd_ = p1;
  up_bd_ = p2;
  for(size_t i = 0; i < 3; ++i){
    if(low_bd_(i) > p2(i))
      low_bd_(i) = p2(i);
    if (up_bd_(i) < p2(i))
      up_bd_(i) = p2(i);

    if(low_bd_(i) > p3(i))
      low_bd_(i) = p3(i);
    if (up_bd_(i) < p3(i))
      up_bd_(i) = p3(i);
  }

  center = (p1 + p2 + p3) / 3.0;
  vec one_edge = p2 - p1, other_edge = p3 - p1;
  normal_ = one_edge.cross(other_edge);  
  // normal_(0) = (p2(1) - p1(1)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(1) - p1(1));
  // normal_(1) = (p2(0) - p1(0)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(0) - p1(0));
  // normal_(2) = (p2(0) - p1(0)) * (p3(1) - p1(1)) - (p2(1) - p1(2)) * (p3(0) - p1(0));

  double norm  = normal_.norm();
  if(norm > 1e-6)
    normal_ = normal_ / norm;
  else
    normal_ = vec::Zero();
  d_ = -normal_.dot(p1) ;
}


aabb merge_tri_aabbs(const std::vector<std::shared_ptr<tri_aabb>> tri_aabbs){
  if(tri_aabbs.empty())
    throw runtime_error("tri_aabbs is empty");

  bool bk =false;
  if(tri_aabbs.size() == 90)
    bk = true;
  size_t num = 0;
  aabb bd_box(tri_aabbs[0]->low_bd_, tri_aabbs[0]->up_bd_);
  for(auto& one_bdbox : tri_aabbs){
    bd_box.merge(*one_bdbox);
    ++num;
  }



  {//check
    for(size_t i = 0; i < tri_aabbs.size(); ++i){
      for(size_t j = 0; j < 3; ++j){
        assert(tri_aabbs[i]->low_bd_(j) >= bd_box.low_bd_(j));
        assert(tri_aabbs[i]->up_bd_(j) <= bd_box.up_bd_(j));

      }
    }
    
  }

  
  return bd_box;
}
