//
// Created by Danielhu on 2018/1/16.
//

#include <glm/glm.hpp>
#include "Qef.h"

#define SVD_NUM_SWEEPS 5

const float Tiny_Number = 1.e-4;
// const float Tiny_Number_Erroring = 1.e-8;

glm::fvec3 diag_of_mul(const glm::fvec3& v1T, const glm::fvec3 v2) {
  return v1T * v2;
}

glm::fvec3 diag(const glm::mat3& m) {
  return glm::fvec3(m[0][0], m[1][1], m[2][2]);
}

glm::fvec3 svd_vmul_sym(const glm::mat3x3 &a, const glm::fvec3 &v) {
  return glm::fvec3(
      (a[0][0] * v.x) + (a[0][1] * v.y) + (a[0][2] * v.z),
      (a[0][1] * v.x) + (a[1][1] * v.y) + (a[1][2] * v.z),
      (a[0][2] * v.x) + (a[1][2] * v.y) + (a[2][2] * v.z)
  );
}

float qef_calc_error(const glm::mat3x3& ATA, const glm::fvec3& x, const glm::fvec3& ATb, const float btb) {
  glm::fvec3 atax = svd_vmul_sym(ATA, x);
  return glm::dot(x, atax) - 2 * glm::dot(x, ATb) + btb;
}


glm::fvec3 qef_calc_co_variance(const glm::mat3x3& ATA, const glm::fvec3& x, const glm::fvec3& diag_ATc, const glm::fvec3& diag_ctc) {
  return x * diag(ATA) * x - 2.f * (x * diag_ATc) + diag_ctc;
}

void svd_rotate_xy(float &x, float &y, float c, float s) {
  float u = x;
  float v = y;
  x = c * u - s * v;
  y = s * u + c * v;
}

void svd_rotateq_xy(float &x, float &y, float a, float c, float s) {
  float cc = c * c;
  float ss = s * s;
  float mx = 2.0f * c * s * a;
  float u = x;
  float v = y;
  x = cc * u - mx + ss * v;
  y = ss * u + mx + cc * v;
}

float svd_invdet(float x, float tol) {
  return (std::abs(x) < tol || std::abs(1.0f / x) < tol) ? 0.0f : 1.0f / x;
}

void svd_pseudoinverse(glm::mat3x3 &o, const glm::fvec3 &sigma, const glm::mat3x3 &v) {
  float d0 = svd_invdet(sigma[0], Tiny_Number);
  float d1 = svd_invdet(sigma[1], Tiny_Number);
  float d2 = svd_invdet(sigma[2], Tiny_Number);
  o = glm::mat3(v[0][0] * d0 * v[0][0] + v[0][1] * d1 * v[0][1] + v[0][2] * d2 * v[0][2],
                v[0][0] * d0 * v[1][0] + v[0][1] * d1 * v[1][1] + v[0][2] * d2 * v[1][2],
                v[0][0] * d0 * v[2][0] + v[0][1] * d1 * v[2][1] + v[0][2] * d2 * v[2][2],
                v[1][0] * d0 * v[0][0] + v[1][1] * d1 * v[0][1] + v[1][2] * d2 * v[0][2],
                v[1][0] * d0 * v[1][0] + v[1][1] * d1 * v[1][1] + v[1][2] * d2 * v[1][2],
                v[1][0] * d0 * v[2][0] + v[1][1] * d1 * v[2][1] + v[1][2] * d2 * v[2][2],
                v[2][0] * d0 * v[0][0] + v[2][1] * d1 * v[0][1] + v[2][2] * d2 * v[0][2],
                v[2][0] * d0 * v[1][0] + v[2][1] * d1 * v[1][1] + v[2][2] * d2 * v[1][2],
                v[2][0] * d0 * v[2][0] + v[2][1] * d1 * v[2][1] + v[2][2] * d2 * v[2][2]);
}

void givens_coeffs_sym(float a_pp, float a_pq, float a_qq, float &c, float &s) {
  if (a_pq == 0.0f) {
    c = 1.0f;
    s = 0.0f;
    return;
  }
  float tau = (a_qq - a_pp) / (2.0f * a_pq);
  float stt = sqrt(1.0f + tau * tau);
  float tan = 1.0f / (tau >= 0.0f ? tau + stt : tau - stt);
  c = 1.0f / sqrt(1.0f + tan * tan);
  s = tan * c;
}

void svd_rotate(glm::mat3x3 &vtav, glm::mat3x3 &v, int a, int b) {
  if (vtav[a][b] == 0.0)
    return;

  float c = 0.f, s = 0.f;
  givens_coeffs_sym(vtav[a][a], vtav[a][b], vtav[b][b], c, s);

  float x, y;
  x = vtav[a][a];
  y = vtav[b][b];
  svd_rotateq_xy(x, y, vtav[a][b], c, s);
  vtav[a][a] = x;
  vtav[b][b] = y;

  x = vtav[0][3 - b];
  y = vtav[1 - a][2];
  svd_rotate_xy(x, y, c, s);
  vtav[0][3 - b] = x;
  vtav[1 - a][2] = y;

  vtav[a][b] = 0.0f;

  x = v[0][a];
  y = v[0][b];
  svd_rotate_xy(x, y, c, s);
  v[0][a] = x;
  v[0][b] = y;

  x = v[1][a];
  y = v[1][b];
  svd_rotate_xy(x, y, c, s);
  v[1][a] = x;
  v[1][b] = y;

  x = v[2][a];
  y = v[2][b];
  svd_rotate_xy(x, y, c, s);
  v[2][a] = x;
  v[2][b] = y;
}

void svd_solve_sym(glm::mat3x3 vtav, glm::fvec3 &sigma, glm::mat3x3 &v) {
  // assuming that A is symmetric: can optimize all operations for
  // the upper right triagonal
  // assuming V is identity: you can also pass a matrix the rotations
  // should be applied to
  // U is not computed
  for (int i = 0; i < SVD_NUM_SWEEPS; ++i) {
    svd_rotate(vtav, v, 0, 1);
    svd_rotate(vtav, v, 0, 2);
    svd_rotate(vtav, v, 1, 2);
  }
  sigma = glm::fvec3(vtav[0][0], vtav[1][1], vtav[2][2]);
}

glm::fvec3 svd_solve_ATA_ATb(const glm::mat3x3 &ATA, const glm::fvec3 &ATb) {
  glm::mat3x3 V;
  glm::fvec3 sigma;
  svd_solve_sym(ATA, sigma, V);

  // A = UEV^T; U = A / (E*V^T)
  glm::mat3x3 Vinv;
  svd_pseudoinverse(Vinv, sigma, V);
  glm::fvec3 x = Vinv * ATb;
  return x;
}

void QefSolver::reset() {
  ATA = glm::mat4(0.f);
  ATb = glm::fvec3(0.f);
  btb = 0.f;
  massPointSum = glm::fvec3(0.f);
  averageNormalSum = glm::fvec3(0.f);
  pointCount = 0;
}

void QefSolver::set(const QefSolver &other) {
  ATA[0][0] = other.ATA[0][0];
  ATA[1][1] = other.ATA[1][1];
  ATA[2][2] = other.ATA[2][2];

  ATA[0][1] = other.ATA[0][1];
  ATA[0][2] = other.ATA[0][2];
  ATA[1][2] = other.ATA[1][2];

  ATb = other.ATb;
  btb = other.btb;
  massPointSum = other.massPointSum;
  averageNormalSum = other.averageNormalSum;
  pointCount = other.pointCount;
  calRoughness();
}

void QefSolver::combine(const QefSolver &other) {
  ATA[0][0] += other.ATA[0][0];
  ATA[1][1] += other.ATA[1][1];
  ATA[2][2] += other.ATA[2][2];

  ATA[0][1] += other.ATA[0][1];
  ATA[0][2] += other.ATA[0][2];
  ATA[1][2] += other.ATA[1][2];

  ATb += other.ATb;
  diag_ATc += other.diag_ATc;
  btb += other.btb;
  diag_ctc += other.diag_ctc;
  massPointSum += other.massPointSum;
  pointCount += other.pointCount;
  averageNormalSum += other.averageNormalSum;
  calRoughness();
}

void QefSolver::separate(const QefSolver &other) {
  ATA[0][0] -= other.ATA[0][0];
  ATA[1][1] -= other.ATA[1][1];
  ATA[2][2] -= other.ATA[2][2];

  ATA[0][1] -= other.ATA[0][1];
  ATA[0][2] -= other.ATA[0][2];
  ATA[1][2] -= other.ATA[1][2];

  ATb -= other.ATb;
  btb -= other.btb;
  massPointSum -= other.massPointSum;
  pointCount -= other.pointCount;
  averageNormalSum -= other.averageNormalSum;
  calRoughness();
}

void QefSolver::add(const glm::fvec3 &p, const glm::fvec3 &n) {
  ATA[0][0] += n.x * n.x;
  ATA[0][1] += n.x * n.y;
  ATA[0][2] += n.x * n.z;
  ATA[1][1] += n.y * n.y;
  ATA[1][2] += n.y * n.z;
  ATA[2][2] += n.z * n.z;
  float dotp = glm::dot(p, n);
  glm::fvec3 c = p * n;
  ATb += n * dotp;
  diag_ATc += n * c;
  btb += dotp * dotp;
  diag_ctc += c * c;
  pointCount++;
  massPointSum += p;
  averageNormalSum += n;
}

void QefSolver::calRoughness() {
  roughness = 1.f - glm::length(averageNormalSum) / (float)pointCount;
}

float QefSolver::getError(const glm::fvec3 &p) {
  return qef_calc_error(ATA, p, ATb, btb);
}

float QefSolver::getError() {
  return qef_calc_error(ATA, ATb, ATb, btb);
}

glm::fvec3 QefSolver::getVariance(const glm::fvec3& p) {
  auto v = qef_calc_co_variance(ATA, p, diag_ATc, diag_ctc);
  return v * v;
}

void QefSolver::solve(glm::fvec3 &hermiteP, float &error) {
  if(pointCount > 0) {
    calRoughness();
    glm::fvec3 massPoint = massPointSum / (float) pointCount;
    glm::fvec3 _ATb = ATb - svd_vmul_sym(ATA, massPoint);
    hermiteP = svd_solve_ATA_ATb(ATA, _ATb);
    hermiteP += massPoint;
    error = qef_calc_error(ATA, hermiteP, ATb, btb);
    assert(!isnan(error));
  }
}