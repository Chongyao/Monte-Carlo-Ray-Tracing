#include "aabb.h"
#include <cmath>
using namespace Eigen;

aabb::aabb(const vec& low_bd, const vec& up_bd){
  low_bd_ = low_bd;
  up_bd_ = up_bd;
}

tri_aabb::tri_aabb(const size_t& id, const vec& p1, const vec& p2, const vec& p3):id_(id){
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
