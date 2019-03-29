#include "model.h"
#include <string>
#include <iostream>
using namespace std;
int main(int argc, char** argv){
  const string filename = argv[1];
  Scene scene(filename.c_str());

  for(auto& model_ptr : scene.models){
    cout << model_ptr->tris_.size() << endl;
  }
  return 0;

}

