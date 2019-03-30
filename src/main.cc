#define EIGEN_USE_BLAS
// #define NDEBUG

#include <Eigen/Dense>

#include "ray.h"
#include "model.h"
#include "radiance.h"
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
using namespace std;
using namespace Eigen;
using namespace chrono;
using vec = Eigen::Vector3d;

inline double clamp(double x) { return x < 0 ? 0 : x > 1 ? 1 : x; }

inline int toInt(double x) { return int(pow(clamp(x), 1 / 2.2) * 255 + .5); }

inline unsigned char toChar(double x) {//Gamma correction
  return static_cast<unsigned char>(toInt(x));
}

void save_ppm(const std::string &name, const std::vector<vec> &img, const int w, const int h) {
  FILE *f = fopen(name.c_str(), "w");
  fprintf(f, "P6\n%i %i 255\n", w, h);
  for (int i = 0; i < w * h; ++i) {
    for(size_t dim = 0; dim < 3; ++dim){
      fputc(toChar(img[i](dim)), f);
    }
  }
  fclose(f);
}

class Setting {
public:
  int spp, w, h;
  double fovy, aspect;
  Ray cam;
  vec up;

  Setting(const string &file) {
    ifstream fin(file);
    double fovy_angle = 0;
    fin >> spp >> w >> h >> fovy_angle;

    aspect = w * 1.0 / h;
    fovy = tan(M_PI * fovy_angle / 2 / 180.0);
    
    fin >> cam.origin_(0) >> cam.origin_(1) >> cam.origin_(2);
    vec lookat;
    fin >> lookat(0) >> lookat(1) >> lookat(2);
    cam.dire_ = (lookat - cam.origin_);
    cam.dire_ /= cam.dire_.norm();

    fin >> up(0) >> up(1) >> up(2);
    up /= up.norm();

    double scale;
    fin >> scale;
    cam.dire_ *= scale;    

    // up = up - up.dot(cam.dire_) * cam.dire_;    
    
  }
};

int main(int argc, char *argv[]) {
  Eigen::initParallel();
  if (argc < 2) return printf("usage: %s input.obj cam.txt\n", argv[0]);
  const Scene scene(argv[1]);
  const Setting st(argv[2]);
  const int samps = st.spp / 4;// using 4 subpiexel

  cout << "[INFO]>>>>>>>>>>>>>>>>>>>forest<<<<<<<<<<<<<<<<<<" << endl;  
  size_t height = 7;
  size_t num_model = scene.models.size();
  cout << "num model is "<< num_model << endl;
  vector<unique_ptr<KD_tree_tris>> KD_forest(num_model);
  #pragma omp parallel for
  for(size_t m_id = 0; m_id < num_model; ++m_id){
    KD_forest[m_id] = unique_ptr<KD_tree_tris>(new KD_tree_tris(scene.models[m_id]->tris_, height, 0, nullptr, true));
  }
  for(const auto& mtl : scene.materials)
    cout << mtl.dissolve <<endl;


  cout << "[INFO]>>>>>>>>>>>>>>>>>>>forest<<<<<<<<<<<<<<<<<<" << endl;
    
  
  // camera coordinate
  vec cx = st.cam.dire_.cross(st.up), cy = cx.cross(st.cam.dire_), r;
  cx *=  st.aspect * st.fovy * (1 / cx.norm() );
  cy *=  st.fovy * (1 / cy.norm() );
  cout << "cx is "<< endl << cx << endl << "cy is "<< endl << cy << endl;
  vector<vec> c(st.w * st.h);// color buffer
  auto start = system_clock::now();
#pragma omp parallel for schedule(dynamic, 1) private(r)
  for (int y = 0; y <  st.h; y++) {
    // cout << "y is " << y << endl;
    fprintf(stderr, "\rRendering (%d spp) %5.2f%%", st.spp, 100. * y / (st.h - 1));
    unsigned short Xi[3] = {0, 0, y * y * y};
    for (int x = 0; x < st.w; x++) {
      // cout << "x is " << x << endl;
      for (int sy = 0, i = (st.h - y - 1) * st.w + x; sy < 2; sy++) { // 2x2 subpixel
        for (int sx = 0; sx < 2; sx++, r = vec::Zero()) {
          for (int s = 1; s <= samps; s++) {
            double r1 = 2 * erand48(Xi), dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
            double r2 = 2 * erand48(Xi), dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
            vec d = cx * (((sx + .5 + dx) / 2 + x) / st.w - .5) +
                    cy * (((sy + .5 + dy) / 2 + y) / st.h - .5) + st.cam.dire_;
            d /= d.norm();
            r = r + radiance(Ray(st.cam.origin_, d), 0, Xi, scene, KD_forest) * (1. / samps);
          }
          // TODO: check whether we need lock here
          vec delt_c;
          delt_c << clamp(r(0)), clamp(r(1)), clamp(r(2));
          c[i] = c[i] + delt_c * .25;
        }
      }
    }
  }
  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  cout <<  "花费了" 
       << double(duration.count()) * microseconds::period::num / microseconds::period::den 
       << "秒" << endl;

  fprintf(stderr, "\n");
  save_ppm("image.ppm", c, st.w, st.h);
}
