#include "ray.h"
#include <limits>
#include <iostream>
using namespace std;
using namespace Eigen;
const double MAX_DOUBLE = std::numeric_limits<double>::max();
Ray::Ray(const vec& origin, const vec& dire):  origin_(origin), dire_(dire),final_offset_(MAX_DOUBLE){}

enum axis{x, y, z};
bool project_dim(const Eigen::Vector3d& normal, axis& dim1, axis& dim2){
  dim1 = x;
  dim2 = y;
  
  size_t count = 0;
  for(size_t i = 0; i < 3; ++i){
    if(fabs(normal(i)) < 1e-6){
      if(count == 0){
        dim1 = static_cast<axis>(i);
        dim2 = static_cast<axis>((i + 1) % 3);
        ++count;
      }
      else if(count == 1){
        dim2 = static_cast<axis>(i);
        ++count;
      }
    }
  }
  if(count > 2)
    return false;
  else
    return true;

}

bool get_cross_point(const vec& ori, const vec& dir, const vec& norm, const double& d, double& offset ){
  
  const double deno = norm(0) * dir(0) + norm(1) * dir(1) + norm(2) * dir(2);
  if(fabs(deno) < 1e-5)
    return false;
  else{
    offset = ( -d - norm(0) * ori(0) - norm(1) * ori(1) - norm(2) * ori(2) ) * (1 / deno);
    if (offset < 0)
      return false;
    else {
      // cross = ori + offset * dir;
      return true;
    }
  }
      
}


bool Ray::intersect_aabb(const aabb& bd_box_)const {
  bool is_inter = false;

  auto check_inside = [](const vec& cross_point, const aabb& bd, const size_t& dim)->bool{
    if(cross_point( (dim + 1) % 3 ) >= bd.low_bd_((dim + 1) % 3)
       && cross_point( (dim + 1) % 3 ) <= bd.up_bd_((dim + 1) % 3)
       && cross_point( (dim + 2) % 3 ) >= bd.low_bd_((dim + 2) % 3)
       && cross_point( (dim + 2) % 3 ) <= bd.up_bd_((dim + 2) % 3))
      return true;
    else
      return false;
  };


  vec cross_point, normal;
  for(size_t dim = 0; dim < 3; ++dim){
    if(fabs(dire_(dim)) < 1e-5)
      continue;
    normal = vec::Zero();{
      normal(dim) = 1;
    }
    
    double para_d = - bd_box_.low_bd_(dim);
    double offset = (- para_d - origin_(dim)) * (1 / dire_(dim));
    if(offset >0){
      cross_point = origin_ + offset * dire_;
      if(check_inside(cross_point, bd_box_, dim)){
        is_inter = true;
        break;
      }      
    }

    para_d = - bd_box_.up_bd_(dim);
    offset = (- para_d - origin_(dim)) * (1 / dire_(dim));
    if(offset >0){
      cross_point = origin_ + offset * dire_;
      if(check_inside(cross_point, bd_box_, dim)){
        is_inter = true;
        break;
      }      
    }
  }
  return is_inter;
}

bool Ray::intersct_tri_aabb(const tri_aabb& tri, Ray&next)const{
  axis dim1, dim2;
  double offset;
  if(!project_dim(tri.normal_, dim1, dim2)
     || !get_cross_point(origin_, dire_, tri.normal_, tri.d_, offset) || offset > final_offset_ || offset < 1e-4)
    return false;


  vec cross_point = origin_ + dire_ * offset;
  
  const double s = ( (cross_point(dim1) - tri.p_(dim1, 2)) * (tri.p_(dim2, 0) - tri.p_(dim2, 2))
                 - (cross_point(dim2) - tri.p_(dim2, 2)) * (tri.p_(dim1, 0) - tri.p_(dim1, 2)) ) /
             ( (tri.p_(dim1, 1) - tri.p_(dim1, 2)) * (tri.p_(dim2, 0) - tri.p_(dim2, 2))
           -(tri.p_(dim2, 1) - tri.p_(dim2, 2)) * (tri.p_(dim1, 0) - tri.p_(dim1, 2)) );
  const double t = ( (cross_point(dim1) - tri.p_(dim1, 2)) * (tri.p_(dim2, 1) - tri.p_(dim2, 2))
             - (cross_point(dim2) - tri.p_(dim2, 2)) * (tri.p_(dim1, 1) - tri.p_(dim1, 2)) ) /
      ((tri.p_(dim1, 0) - tri.p_(dim1, 2)) * (tri.p_(dim2, 1) - tri.p_(dim2, 2))
       - (tri.p_(dim2, 0) - tri.p_(dim2, 2)) * (tri.p_(dim1, 1) - tri.p_(dim1, 2)));
  


  assert(s == s);

  if ( s > 0 && s < 1 && t > 0 && t < 1 && (1 - s - t) > 0 && (1 - s - t) < 1 ){
    final_offset_ = offset;
    next.origin_ = cross_point;
    next.dire_ = tri.n_.col(0) * t + tri.n_.col(1) * s + tri.n_.col(2) * (1 - s -t);
    next.dire_ /= next.dire_.norm();
    return true;
  }
  else
    return false;
}


  
bool Ray::intersect(const unique_ptr<KD_tree_tris>& kd, size_t& face_id, Ray& next)const{

  
  if(intersect_aabb(kd->bd_box_)){

    if(kd->left_tree_ == nullptr || kd->right_tree_ == nullptr){//intersct triangles
      assert(kd->left_tree_ == nullptr);
      assert(kd->right_tree_ == nullptr);
      bool is_inter = false;

      for(size_t f_id = 0; f_id < kd->child_.size(); ++f_id){
        if(intersct_tri_aabb(*(kd->child_[f_id]), next)){
          is_inter = true;
          face_id = kd->child_[f_id]->id_;
          // break;
        }

      }
      return is_inter;
    }
    else
      return intersect(kd->left_tree_, face_id, next)
          | intersect(kd->right_tree_, face_id, next);
  }
  else{
    return false;
  }
}

bool Ray::intersect_forest(const vector<unique_ptr<KD_tree_tris>>& KD_forest, size_t& face_id, Ray& next)const{
  size_t num_model = KD_forest.size();
  bool is_inter = false;
  for(size_t m_id = 0; m_id < num_model; ++m_id){
    if(intersect(KD_forest[m_id], face_id, next)){
      is_inter = true;      
    }

  }
  return is_inter;
  

}

bool Ray::intersect_forest_loop(const std::vector<std::unique_ptr<KD_tree_tris>>& KD_forest,
                                size_t& face_id, Ray& next)const{
  bool is_inter = false;
  for(auto& kd : KD_forest){

    size_t num_tris = kd->child_.size();
    cout << "num tris is "<< num_tris << endl;
    for(int f_id = num_tris - 1; f_id >=0 ; --f_id){
      if( intersct_tri_aabb(*(kd->child_[f_id]), next)){
        is_inter = true;
        face_id = kd->child_[f_id]->id_;
        cout << "off set : " << final_offset_ << "  f id " << f_id << endl;
        cout << kd->child_[f_id]->p_ << endl;
        cout << kd->child_[f_id]->n_  << endl;
        // break;
      }

    }
    // cout << is_inter  << endl;
  }

  return is_inter;
}
