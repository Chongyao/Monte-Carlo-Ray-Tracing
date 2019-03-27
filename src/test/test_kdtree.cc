#include "kd_tree.h"
#include "model.h"
#include <iostream>
using namespace std;
using namespace Eigen;

bool test_kd_tree(const KD_tree_tris* kd){
  if(kd == nullptr)
    return true;
  size_t num_children = kd->child_.size();
  for(size_t i = 0; i < num_children; ++i){
    for(size_t j = 0; j < 3; ++j){
      assert(kd->child_[i]->low_bd_(j) >= kd->bd_box_.low_bd_(j));
      if(kd->child_[i]->up_bd_(j) > kd->bd_box_.up_bd_(j)){
        cout << kd->child_[i]->up_bd_ << endl << kd->bd_box_.up_bd_;
      }
      assert(kd->child_[i]->up_bd_(j) <= kd->bd_box_.up_bd_(j));

    }
  }


  if(kd->left_tree_.get() != nullptr || kd->right_tree_.get() != nullptr){
    size_t num_left = kd->left_tree_->child_.size(),
          num_right = kd->right_tree_->child_.size();
  
    assert(num_children ==  num_left+  num_right);
    
    return test_kd_tree(kd->left_tree_.get()) & test_kd_tree(kd->right_tree_.get());    
  }

  else
    return true;
  
}







int main(int argc, char** argv){
  const string filename = argv[1];
  Scene scene(filename.c_str());



  size_t height = 25;
  size_t num_model = scene.models.size();
  vector<shared_ptr<KD_tree_tris>> KD_forest(num_model);
  //#pragma omp parallel for
  for(size_t m_id = 0; m_id < num_model; ++m_id){
    size_t num_tris = scene.models[m_id]->tris_.size();
    vector<shared_ptr<tri_aabb>> childrens(num_tris);
    cout << "num tris is "<< num_tris << endl;
    for(size_t f_id = 0; f_id < num_tris; ++f_id){
      childrens[f_id] = std::move(make_shared<tri_aabb>(f_id, scene.models[m_id]->tris_[f_id].p_));
    }
    KD_forest[m_id] = make_shared<KD_tree_tris>(childrens, height, 0, nullptr, true);
  }


  for(size_t m_id = 0; m_id < num_model; ++m_id){
    cout << test_kd_tree(KD_forest[m_id].get()) << endl;
  }

  // size_t num_children = KD_forest[0].child_.size();
  // cout << "children total size is " << num_children << endl;
  // do{
  //   KD_forest[0]
  // }
  
  
 
  return 0;

}


