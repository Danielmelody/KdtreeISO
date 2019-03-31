#define GLM_FORCE_CTOR_INIT

#include "program.h"
#include <fstream>
#include <sstream>
#include <iostream>

// ----------------------------------------------------------------------------

Program::Program()
  : id(0) {
}

// ----------------------------------------------------------------------------

Program::~Program() {
  if (id > 0) {
    glDeleteProgram(id);
  }
}

// ----------------------------------------------------------------------------

bool Program::init(const char *vert, const char *frag) {
  id = glCreateProgram();
  return compile(GL_VERTEX_SHADER, vert) && compile(GL_FRAGMENT_SHADER, frag) && link();
}

void Program::use() {
  glUseProgram(id);
}

// ----------------------------------------------------------------------------

void Program::printShaderInfo(GLuint shader) const {
  int maxLength = 0;
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &maxLength);

  static char buffer[2048];
  int length = 0;
  glGetShaderInfoLog(shader, maxLength, &length, buffer);

  printf("%s\n", buffer);
}

// ----------------------------------------------------------------------------

void Program::printProgramInfo(GLuint program) const {
  int maxLength = 0;
  glGetProgramiv(program, GL_INFO_LOG_LENGTH, &maxLength);

  static char buffer[2048];
  int length = 0;
  glGetProgramInfoLog(program, maxLength, &length, buffer);

  printf("%s\n", buffer);
}

bool Program::compile(GLenum type, const char *src) {

  const GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &src, nullptr);
  glCompileShader(shader);

  GLint status = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
  if (status == GL_FALSE) {
    printShaderInfo(shader);
    return false;
  }

  glAttachShader(id, shader);
  shaders.push_back(shader);

  return true;
}

// ----------------------------------------------------------------------------

bool Program::link() {
  if (shaders.empty()) {
    return false;
  }

  glLinkProgram(id);
  printProgramInfo(id);
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    return false;
  }

  GLint status = 0;
  glGetProgramiv(id, GL_LINK_STATUS, &status);
  if (status == GL_FALSE) {
    return false;
  }

  for (unsigned int shader : shaders) {
    glDetachShader(id, shader);
    glDeleteShader(shader);
  }

  shaders.clear();

  return true;
}

const GLint Program::getUniformLocation(const char *name) {
  if (locations.find(name) == locations.end()) {
    const GLint location = glGetUniformLocation(id, name);
    locations.insert({std::string(name), (int)location});
  }

  return locations[name];
}

bool Program::setMat4(const char *name, const glm::mat4 &uniform) {
  const GLint location = getUniformLocation(name);
  if (location == -1) {
    return false;
  }

  glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(uniform));
  return true;
}

bool Program::setVec3(const char *name, const glm::fvec3 &uniform) {
  const GLint location = getUniformLocation(name);
  if (location == -1) {
    return false;
  }

  glUniform3fv(location, 1, glm::value_ptr(uniform));
  return true;
}

bool Program::setFloat(const char *name, float uniform) {
  const GLint location = getUniformLocation(name);
  if (location == -1) {
    return false;
  }

  glUniform1f(location, uniform);
  return true;
}
