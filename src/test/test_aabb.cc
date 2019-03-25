#include <vector>
#include "aabb.h"
#include <iostream>
#define TINYOBJLOADER_IMPLEMENTATION
#define TINYOBJLOADER_USE_DOUBLE
#include "tiny_obj_loader.h"
#include <Eigen/Core>
using namespace std;
using namespace Eigen;

string dirname(const string &s) {
  unsigned long find = s.find_last_of('/');
  if (find == std::string::npos)return {"."};
  return s.substr(0, find);
}


int main(int argc, char** argv){
  vector<tinyobj::shape_t> shapes;
  vector<tinyobj::material_t> materials;
  tinyobj::attrib_t attrib;
  string filename = argv[1];
  string err;
  if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, // triangulated == true
                        filename.c_str(), (dirname(filename) + "/").c_str())) {
    throw runtime_error(err.c_str());
  }


  Vector3d p1, p2, p3;
  for(auto& one_shape : shapes){
    cout << one_shape.name << endl;
    size_t num_tris = one_shape.mesh.material_ids.size();
    cout << num_tris << endl;
  }

  return 0;
}
