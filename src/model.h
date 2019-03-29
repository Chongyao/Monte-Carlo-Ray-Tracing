#ifndef MODEL_H_ZCY
#define MODEL_H_ZCY
#include <memory>
#include <Eigen/Core>
#include <vector>
#include "aabb.h"


#define TINYOBJLOADER_USE_DOUBLE
#include "tiny_obj_loader.h"


using tri = Eigen::Matrix3d;
using vec = Eigen::Vector3d;
class Triangle{
 public:
  tri p_, n_;
  size_t mtl_id_;
  Triangle(const tri& p, const tri&n, const size_t& mtl_id):p_(p), n_(n), mtl_id_(mtl_id){}
  Triangle(){}
};

class Model{
 public:
  std::vector<std::shared_ptr<tri_aabb>> tris_;
  Model();
  Model(const size_t& num_tris);
};
  
class Scene{
 public:
  Scene(const std::string& filename);
  std::vector<std::shared_ptr<Model>> models;

  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

 private:
  int build_models(const std::vector<tinyobj::real_t> &pts, const std::vector<tinyobj::real_t> &vns,
                   const std::vector<tinyobj::shape_t> &shapes);

  
};



#endif
