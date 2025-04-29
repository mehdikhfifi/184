#include "CGL/CGL.h"

#include "collada.h"
#include "meshEdit.h"
#include "bezierPatch.h"
#include "bezierCurve.h"
#include "mergeVertices.h"
#include "shaderUtils.h"

#include <iostream>

using namespace std;
using namespace CGL;

#define msg(s) cerr << "[Collada Viewer] " << s << endl;

// automatically close FD when unwinding stackframe during exception/termination
struct FileHandle
{
  FILE* file = NULL;
  ~FileHandle() { if (file) fclose(file); }
};

int loadFile(MeshEdit* collada_viewer, const char* path) {

  Scene* scene = new Scene();

  std::string path_str = path;
  if (path_str.substr(path_str.length()-4, 4) == ".dae")
  {
    if (ColladaParser::load(path, scene) < 0) {
      delete scene;
      return -1;
    }
  }
  else if (path_str.substr(path_str.length()-4, 4) == ".bez")
  {
    Camera* cam = new Camera();
    cam->type = CAMERA;
    Node node;
    node.instance = cam;
    scene->nodes.push_back(node);
    Polymesh* mesh = new Polymesh();
    FileHandle file_handle;
    file_handle.file = fopen(path, "r");
    int n = 0;
    fscanf(file_handle.file, "%d", &n);
    for (int i = 0; i < n; i++)
    {
      BezierPatch patch;
      patch.loadControlPoints(file_handle.file);
      patch.add2mesh(mesh);
      mergeVertices(mesh);
    }

    mesh->type = POLYMESH;
    node.instance = mesh;
    scene->nodes.push_back(node);
  }
  else
  {
    return -1;
  }

  collada_viewer->load( scene );

  GLuint tex = makeTex("envmap/envmap.png");
  if(!tex) tex = makeTex("../envmap/envmap.png");
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, tex);
  glActiveTexture(GL_TEXTURE2);

  return 0;
}


int main( int argc, char** argv ) {

  // require exactly two scene files
  if (argc != 3) {
    msg("Usage: ./meshedit <source.dae> <target.dae>");
    exit(0);
  }

  Viewer viewer;
  MeshEdit* collada_viewer = new MeshEdit();
  viewer.set_renderer(collada_viewer);
  viewer.init();

  // load both meshes
  if (loadFile(collada_viewer, argv[1]) < 0 ||
      loadFile(collada_viewer, argv[2]) < 0) {
    msg("Failed loading one of the scene files. Check your paths!");
    exit(0);
  }

  // now start the render loop
  viewer.start();
  return 0;
}
