#include "model.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <iostream>
using namespace std;
Model::Model(){
  tris_.clear();
}
Model::Model(const size_t& num_tris){
  tris_ = vector<shared_ptr<tri_aabb>>(num_tris);

}
string dirname(const string &s) {
  unsigned long find = s.find_last_of('/');
  if (find == std::string::npos)return {"."};
  return s.substr(0, find);
}
void check_material(const vector<tinyobj::shape_t> &shapes) {
  for (const auto &shape : shapes)
    for (auto id : shape.mesh.material_ids)
      if (id < 0) throw runtime_error("material index error!");
}

void check_triangle(const vector<tinyobj::shape_t> &shapes) {
  // check not triangle
  for (const auto &shape : shapes)
    for (auto n : shape.mesh.num_face_vertices)
      if (n != 3) throw runtime_error("not triangle");
}

int Scene::build_models(const vector<tinyobj::real_t> &pts, const vector<tinyobj::real_t> &vns,
                        const vector<tinyobj::shape_t> &shapes){
  
  
  size_t num_model = shapes.size();
  models.resize(num_model);

  for(size_t m_id = 0; m_id < num_model; ++m_id){
    models[m_id] = make_shared<Model>();
    size_t num_tris = shapes[m_id].mesh.indices.size() / 3;
    models[m_id]->tris_.resize(num_tris);
    
    //#pragma omp parallel for
    for(size_t face_id = 0; face_id < num_tris; ++face_id){
      vector<size_t> vert_id(3);
      tri p, n;
      
      //set position
      for(size_t dim = 0; dim < 3; ++dim){
        vert_id[dim] = shapes[m_id].mesh.indices[3 * face_id + dim].vertex_index * 3;
      }
      for(size_t v_id = 0; v_id < 3; ++v_id){
        for(size_t dim = 0; dim < 3; ++dim){
          p(dim, v_id) = pts[vert_id[v_id] + dim];

        }
      }

      
      
      //set normal
      for(size_t dim = 0; dim < 3; ++dim){
        vert_id[dim] = shapes[m_id].mesh.indices[3 * face_id + dim].normal_index * 3;
      }
      for(size_t v_id = 0; v_id < 3; ++v_id){
        for(size_t dim = 0; dim < 3; ++dim){
          n(dim, v_id) = vns[vert_id[v_id] + dim];
        }
      }

      models[m_id]->tris_[face_id] = make_shared<tri_aabb>(shapes[m_id].mesh.material_ids[face_id], p , n);
    }//loop 4 face
    
  }//loop 4 shape
  return 0;
}


Scene::Scene(const string& filename){
  models.clear();

  
   tinyobj::attrib_t attrib;
  string err;
  if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, // triangulated == true
                        filename.c_str(), (dirname(filename) + "/").c_str())) {
    throw runtime_error(err.c_str());
  }

  // TODO: check if all normals is valid
  check_material(shapes);
  check_triangle(shapes);

  build_models(attrib.vertices, attrib.normals, shapes);
}





