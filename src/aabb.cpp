#include "aabb.h"
#include <cmath>
using namespace Eigen;
using namespace std;

aabb::aabb(const vec& low_bd, const vec& up_bd){
  low_bd_ = low_bd;
  up_bd_ = up_bd;
}
void aabb::merge(const aabb& other){
  for(size_t i = 0; i < 3; ++i){
    if(low_bd_(i) > other.low_bd_(i))
      low_bd_(i) = other.low_bd_(i);
    else if(up_bd_(i) < other.up_bd_(i))
      up_bd_(i) = other.up_bd_(i);
  }
}

tri_aabb::tri_aabb(const size_t& id, const vec& p1, const vec& p2, const vec& p3):id_(id){
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
  
  
  a_ = (p2(1) - p1(1)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(1) - p1(1));
  b_ = (p2(0) - p1(0)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(0) - p1(0));
  c_ = (p2(0) - p1(0)) * (p3(1) - p1(1)) - (p2(1) - p1(2)) * (p3(0) - p1(0));

  double norm = sqrt(a_*a_ + b_*b_ + c_*c_);
  if(norm > 1e-6 ){
    a_ /= norm;
    b_ /= norm;
    c_ /= norm;
  }else{
    a_ = 0;b_ = 0;c_ = 0;
  }
  d_ = - (a_ * p1(0) + b_ * p1(1) + c_ * p1(2));
}
tri_aabb::tri_aabb(const size_t& id, const tri& plane){
  id_ = id;
  vec p1 = plane.col(0), p2 = plane.col(1), p3 = plane.col(2);
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
  
  
  a_ = (p2(1) - p1(1)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(1) - p1(1));
  b_ = (p2(0) - p1(0)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(0) - p1(0));
  c_ = (p2(0) - p1(0)) * (p3(1) - p1(1)) - (p2(1) - p1(2)) * (p3(0) - p1(0));

  double norm = sqrt(a_*a_ + b_*b_ + c_*c_);
  if(norm > 1e-6 ){
    a_ /= norm;
    b_ /= norm;
    c_ /= norm;
  }else{
    a_ = 0;b_ = 0;c_ = 0;
  }
  d_ = - (a_ * p1(0) + b_ * p1(1) + c_ * p1(2));
}


aabb merge_tri_aabbs(const std::vector<std::shared_ptr<tri_aabb>> tri_aabbs){
  if(tri_aabbs.empty())
    throw runtime_error("tri_aabbs is empty");

  aabb bd_box(tri_aabbs[0]->low_bd_, tri_aabbs[0]->up_bd_);
  for(auto& one_bdbox : tri_aabbs){
    bd_box.merge(*one_bdbox);
  }
  return bd_box;
}
