#include "radiance.h"
using namespace std;
using namespace Eigen;


vec radiance(const Ray &r, int depth,  unsigned short *Xi, const Scene &scene, const vector<unique_ptr<KD_tree_tris>>& KD_forest) {

  
  size_t mtl_id;   // id of material of intersected triangle
  Ray next;     // intersection point and normal
  //TODO: replace with my intersect
  if (!r.intersect_forest(KD_forest, mtl_id, next)) return vec::Zero(); // return blaock
  

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
  ke = em+ ambient;  // TODO: now ka is ke



  double p = f.maxCoeff() ;// max refl
  double pp = ff.maxCoeff(); // max refl
  double mp = max(p, pp);
  bool checkke = ke(0) != 0 && ke(1) != 0 && ke(2) != 0 ;

  if (ke(0) != 0 && ke(1) != 0 && ke(2) != 0) //find light
    return vec(f.array() * ke.array());
    // return ke;

  // FIXME: there is a bug that causes infinite recursive loop when mp = 1.
  if (++depth > 5)
    if (erand48(Xi) < mp) // Russian roulette
      f = f * (1 / mp);
    else 
      return ke; 
    

  const vec nl = n.dot(r.dire_) < 0 ? n : n * -1;

  if (mtl.dissolve < 1 || mtl.transmittance[0] < 1) {           // REFRACTION
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

    return ke + res;

  }


  if (mtl.shininess > 1) { // REFLECTION

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
      return ke + res;
    }
  }


  double r1 = 2 * M_PI * erand48(Xi), r2 = erand48(Xi), r2s = sqrt(r2);


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

  auto res = vec(f.array() * radiance(Ray(x, d), depth, Xi, scene, KD_forest).array());
  return ke + res;


}

