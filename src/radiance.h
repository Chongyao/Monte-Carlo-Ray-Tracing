#ifndef RADIANCE_H
#define RADIANCE_H
#include <Eigen/Dense>
#include "ray.h"
#include "model.h"
using vec = Eigen::Vector3d;

vec radiance(const Ray &r, int depth,  unsigned short *Xi, const Scene &scene, const std::vector<std::unique_ptr<KD_tree_tris>>& KD_forest);

#endif
