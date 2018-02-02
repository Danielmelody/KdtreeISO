#ifndef        HAS_GLSL_PROGRAM_H_BEEN_INCLUDED
#define        HAS_GLSL_PROGRAM_H_BEEN_INCLUDED

#include <GLFW/glfw3.h>
#include <vector>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <map>

class Program {
public:

  Program();
  ~Program();

  bool init(const char *vert, const char* frag);
  void use();
  bool setMat4(const char *name, const glm::mat4 &uniform);
  bool setVec3(const char *name, const glm::vec3 &uniform);

private:

  void printShaderInfo(GLuint shader) const;
  void printProgramInfo(GLuint program) const;
  bool compile(GLenum type, const char *src);
  bool link();

  GLuint id;
  std::vector<GLuint> shaders;
  std::map<std::string, int> locations;
  const GLint getUniformLocation(const char *name);
};

#endif    //	HAS_GLSL_PROGRAM_H_BEEN_INCLUDED

