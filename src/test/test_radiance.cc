#include <Eigen/Dense>
#include "ray.h"
#include "model.h"

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <fstream>
#include <iostream>
using namespace std;
using namespace Eigen;
using vec = Eigen::Vector3d;
void print_Vec(const vec& v){
  cout << v(0) << " " << v(1) << " " << v(2) << endl;
}



vec radiance(const Ray &r, int depth,  unsigned short *Xi, const Scene &scene, const vector<unique_ptr<KD_tree_tris>>& KD_forest) {
  cout << endl << endl <<endl;
  cout << "Xi" << Xi[0] <<" "<<Xi[1] << " " <<Xi[2] <<endl;
  cout << "ray origin is " << endl<< r.origin_(0) << " "<<r.origin_(1) << " " << r.origin_(2) << endl;
  cout << "dire is "<< endl << r.dire_(0) << " "<<r.dire_(1)<< " " <<r.dire_(2) << endl;
  
  size_t mtl_id;   // id of material of intersected triangle
  Ray next;     // intersection point and normal
  //TODO: replace with my intersect
  if (!r.intersect_forest(KD_forest, mtl_id, next)) return vec::Zero(); // return blaock
  
  cout << "origin is " << endl<< next.origin_(0) << " " << next.origin_(1) << " " << next.origin_(2) << endl <<"mtl id is " <<mtl_id << endl;
  cout << "dire is "<< endl << next.dire_(0) << " "<< next.dire_(1) << " " << next.dire_(2) << endl;
  const tinyobj::material_t &mtl = scene.materials[mtl_id];
  const vec &x = next.origin_;
  const vec &n = next.dire_;
  
  vec f,ff,ke, em, ambient;
  auto set_value = [](const double v_[3], vec& v){
    for(size_t i = 0; i < 3; ++i){
      v(i) = v_[i];
    }
  };
  set_value(mtl.diffuse, f);
  set_value(mtl.specular, ff);
  set_value(mtl.emission, em);
  set_value(mtl.ambient, ambient);
  // copy(f.data(), f.data() + 3, mtl.diffuse);
  // copy(ff.data(), ff.data() + 3, mtl.specular);
  // copy(em.data(), em.data() + 3, mtl.emission);
  // copy(ambient.data(), ambient.data() + 3, mtl.ambient);
  ke = em+ ambient;  // TODO: now ka is ke
  cout << "ke is " << ke(0) <<" " << ke(1) << " " << ke(2) << endl;


  double p = f.maxCoeff() ;// max refl
  double pp = ff.maxCoeff(); // max refl
  double mp = max(p, pp);
  cout<<"mp" << mp << endl;
  bool checkke = ke(0) != 0 && ke(1) != 0 && ke(2) != 0 ;

  if (ke(0) != 0 && ke(1) != 0 && ke(2) != 0){ //find light
    cout << "return ke notnull" << endl;
    print_Vec(ke);
    return ke;
  }

  // FIXME: there is a bug that causes infinite recursive loop when mp = 1.
  if (++depth > 5)
    if (erand48(Xi) < mp) // Russian roulette
      f = f * (1 / mp);
    else {
      cout << "depth > 5"<< endl;
      print_Vec(ke);
     return ke; 
    }

  const vec nl = n.dot(r.dire_) < 0 ? n : n * -1;

  if (mtl.dissolve < 1) {                                             // REFRACTION
    cout <<"mtl.dissolve " << endl;
    Ray reflRay(x, r.dire_ - n * 2 * n.dot(r.dire_));

    bool into = n.dot(nl) > 0;
    double nc = 1, nt = mtl.ior;//1.5
    double nnt = into ? nc / nt : nt / nc, ddn = r.dire_.dot(nl), cos2t;

    if ((cos2t = 1 - nnt * nnt * (1 - ddn * ddn)) < 0)                // Total internal reflection
      return ke + vec(f.array() * radiance(reflRay, depth, Xi, scene,KD_forest ).array());

    vec tdir = (r.dire_ * nnt - n * ((into ? 1 : -1) * (ddn * nnt + sqrt(cos2t))));
    tdir /= tdir.norm();
    double a = nt - nc, b = nt + nc, R0 = a * a / (b * b), c = 1 - (into ? -ddn : tdir.dot(n));
    double Re = R0 + (1 - R0) * c * c * c * c * c, Tr = 1 - Re, P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);

    vec res;
    if(depth > 2){
      if(erand48(Xi) < P)
        res = radiance(reflRay, depth, Xi, scene,KD_forest) * RP;
      else
        res = radiance(Ray(x, tdir), depth, Xi, scene, KD_forest) * TP;
    }
    else{
      res = radiance(reflRay, depth, Xi, scene, KD_forest) * Re +
          radiance(Ray(x, tdir), depth, Xi, scene, KD_forest) * Tr;
    }
    cout << "dissolve other " << endl;
    print_Vec(ke + res);
    return ke + res;

  }


  if (mtl.shininess > 1) { // REFLECTION
    cout <<"mtl.shininess " << endl;
    if (erand48(Xi) < pp / (p + pp)) {// Russian roulette
      double xx = erand48(Xi);
      double yy = erand48(Xi);
      double nn = mtl.shininess;

      vec w = r.dire_ - n * 2 * n.dot(r.dire_);
      vec u = w.cross(nl), v= w.cross(u);
      u /= u.norm(); v /= v.norm();

      double c = pow(xx, 1 / (nn + 1));
      double r1 = sqrt(1 - c * c);
      double r2 = 2 * M_PI * yy;
      double a = r1 * cos(r2);
      double b = r1 * sin(r2);

      vec dir = (w * c + u * a + v * b);
      dir /= dir.norm();
      if (dir.dot(nl) < 0)
        dir = w;

      vec res = vec(ff.array() * radiance(Ray(x, dir), depth, Xi, scene, KD_forest).array());
      cout << "shininess"<< endl;
      print_Vec(ke + res);
      return ke + res;
    }
  }


  double r1 = 2 * M_PI * erand48(Xi), r2 = erand48(Xi), r2s = sqrt(r2);
  cout << "rrr" << endl;
  cout << r1 << " " << r2 << " " << r2s;
  vec w = nl, u = vec::Zero();{
    if(fabs(w(0)) > .1)
      u(1) = 1;
    else
      u(0) = 1;
    u = u.cross(w);
    u /= u.norm();
  }

  vec v = w.cross(u);
  vec d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2));
  d /= d.norm();
  cout << "final " << endl;
  auto res = vec(f.array() * radiance(Ray(x, d), depth, Xi, scene, KD_forest).array());
  print_Vec(ke + res);
  return ke + res;


}
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
  }
};

int main(int argc, char *argv[]) {
  if (argc < 2) return printf("usage: %s input.obj cam.txt\n", argv[0]);
  const Scene scene(argv[1]);
  const Setting st(argv[2]);
  const int samps = st.spp / 4;// using 4 subpiexel

  //cout << "[INFO]>>>>>>>>>>>>>>>>>>>forest<<<<<<<<<<<<<<<<<<" << endl;
  size_t height = 25;
  size_t num_model = scene.models.size();
  //cout << "num model is "<< num_model << endl;
  vector<unique_ptr<KD_tree_tris>> KD_forest(num_model);
  #pragma omp parallel for
  for(size_t m_id = 0; m_id < num_model; ++m_id){
    KD_forest[m_id] = unique_ptr<KD_tree_tris>(new KD_tree_tris(scene.models[m_id]->tris_, height, 0, nullptr, true));
  }

  //cout << "[INFO]>>>>>>>>>>>>>>>>>>>forest<<<<<<<<<<<<<<<<<<" << endl;
    
  
  // camera coordinate
  vec cx = (st.cam.dire_.cross(st.up)) * st.aspect * st.fovy, r = vec::Zero();
  vec cy = (cx.cross(st.cam.dire_));
  cy /= cy.norm() * st.fovy;
  vector<vec> c(st.w * st.h);// color buffer

  for (int y = 0; y < 1; y++) {
    fprintf(stderr, "\rRendering (%d spp) %5.2f%%", st.spp, 100. * y / (st.h - 1));
    unsigned short Xi[3] = {0, 0, y * y * y};
    for (int x = 17; x < 18 ; x++) {
      for (int sy = 0, i = (st.h - y - 1) * st.w + x; sy < 2; sy++) { // 2x2 subpixel
        for (int sx = 0; sx < 2; sx++, r = vec::Zero()) {
          for (int s = 1; s <= samps; s++) {
            double r1 = 2 * erand48(Xi), dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
            double r2 = 2 * erand48(Xi), dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
            vec d = cx * (((sx + .5 + dx) / 2 + x) / st.w - .5) +
                    cy * (((sy + .5 + dy) / 2 + y) / st.h - .5) + st.cam.dire_;

            d /= d.norm();
            cout << "before plus " << endl;print_Vec(r);
            r = r + radiance(Ray(st.cam.origin_, d), 0, Xi, scene, KD_forest) * (1. / samps);
            cout << "this is r " << endl << r(0)<< " "<< r(1) << " " << r(2) << endl;

          }
          // TODO: check whether we need lock here
          vec delt_c;
          delt_c << clamp(r(0)), clamp(r(1)), clamp(r(2));
          c[i] = c[i] + delt_c * .25;
        }
      }
    }
  }

}
