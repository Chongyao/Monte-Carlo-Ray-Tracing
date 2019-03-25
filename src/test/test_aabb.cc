#include <vector>
#include "aabb.h"
#include <iostream>
#include "model.h"
#include <Eigen/Core>
using namespace std;
using namespace Eigen;




int main(int argc, char** argv){
  const string filename = argv[1];
  Scene scene(filename.c_str());

  for(auto& model_ptr : scene.models){
    size_t num_tris = model_ptr->tris_.size();
    vector<tri_aabb> aabbs(num_tris);

    #pragma omp parallel for
    for(size_t i = 0; i < num_tris; ++i){
      aabbs[i] = tri_aabb(model_ptr->tris_[i].mtl_id_, model_ptr->tris_[i].p_.col(0),model_ptr->tris_[i].p_.col(1), model_ptr->tris_[i].p_.col(2));
    }
    #pragma omp parallel for    
    for(size_t i = 0; i < num_tris; ++i){
      for(size_t j = 0; j < 3; ++j){
        for(size_t k = 0; k < 3; ++k){
          assert(model_ptr->tris_[i].p_(k, j) <= aabbs[i].up_bd_(k) &&
                 model_ptr->tris_[i].p_(k, j) >= aabbs[i].low_bd_(k));
        }
      }
    }
  }
  cout <<"pass" << endl;

  return 0;
}
