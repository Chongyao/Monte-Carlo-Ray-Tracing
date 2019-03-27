#include "ray.h"
#include "model.h"
#include <iostream>
using namespace std;
using namespace Eigen;
int main(int argc, char** argv){

  const string filename = argv[1];
  Scene scene(filename.c_str());
  size_t height = 25;
  size_t num_model = scene.models.size();
  cout << "num model is "<< num_model << endl;
  vector<unique_ptr<KD_tree_tris>> KD_forest(num_model);
  //#pragma omp parallel for
  for(size_t m_id = 0; m_id < num_model; ++m_id){
    size_t num_tris = scene.models[m_id]->tris_.size();
    vector<shared_ptr<tri_aabb>> childrens(num_tris);
    cout << "num tris is "<< num_tris << endl;
    for(size_t f_id = 0; f_id < num_tris; ++f_id){
      childrens[f_id] = std::move(make_shared<tri_aabb>(f_id, scene.models[m_id]->tris_[f_id].p_));
    }
    KD_forest[m_id] = unique_ptr<KD_tree_tris>(new KD_tree_tris(childrens, height, 0, nullptr, true));
    // cout << "bd box :" << endl << KD_forest[m_id]->bd_box_.low_bd_ << endl << KD_forest[m_id]->bd_box_.up_bd_ << endl;
  }
  cout<<"KD forest sizes" << KD_forest.size() << endl;
  //#pragma omp parallel for  
  for(size_t i = 0; i < 5000; ++i){
    vec origin;
    origin << 0., 0, 0;
    vec dire;
    dire << 0, 1, 0;
    // cout << "origin is "  << endl << origin << endl  << "dire is " << endl << dire <<endl;
    Ray one_ray(origin, dire);
    size_t face_id;
    vec cross_point = vec::Zero();
    for(size_t m_id = 0; m_id < num_model; ++m_id){
      cout << one_ray.intersect(KD_forest[m_id], face_id, cross_point) << endl;
      
    }

    cout << one_ray.final_offset_ << endl;
    
  }



  
  return 0;
}

