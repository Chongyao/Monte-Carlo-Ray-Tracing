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
  #pragma omp parallel for
  for(size_t m_id = 0; m_id < num_model; ++m_id){
    vector<shared_ptr<tri_aabb>> childrens = scene.models[m_id]->tris_;
    KD_forest[m_id] = unique_ptr<KD_tree_tris>(new KD_tree_tris(childrens, height, 0, nullptr, true));
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
    Ray next;
    one_ray.intersect_forest(KD_forest, face_id, next);
    cout << one_ray.final_offset_ << endl;
    
  }



  
  return 0;
}

