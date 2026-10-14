#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace flight {
constexpr double pi = 3.14159265358979323846, gravity = 9.80665;
inline double rad(double d) { return d * pi / 180.; }
inline double clamp(double x, double lo, double hi) {
  return std::clamp(x, lo, hi);
}
struct Vec {
  double x = 0, y = 0, z = 0;
  double &operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
  double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
  Vec operator+(Vec b) const { return {x + b.x, y + b.y, z + b.z}; }
  Vec operator-(Vec b) const { return {x - b.x, y - b.y, z - b.z}; }
  Vec operator-() const { return {-x, -y, -z}; }
  Vec operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec operator/(double s) const { return *this * (1 / s); }
  Vec &operator+=(Vec b) {
    *this = *this + b;
    return *this;
  }
};
inline double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec cross(Vec a, Vec b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(Vec a) { return std::sqrt(dot(a, a)); }
inline Vec unit(Vec a) { return a / std::max(norm(a), 1e-12); }
struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
  Quat operator*(Quat b) const {
    return {w * b.w - x * b.x - y * b.y - z * b.z,
            w * b.x + x * b.w + y * b.z - z * b.y,
            w * b.y - x * b.z + y * b.w + z * b.x,
            w * b.z + x * b.y - y * b.x + z * b.w};
  }
  Quat operator+(Quat b) const { return {w + b.w, x + b.x, y + b.y, z + b.z}; }
  Quat operator*(double s) const { return {w * s, x * s, y * s, z * s}; }
  Quat conjugate() const { return {w, -x, -y, -z}; }
  Quat normalized() const {
    double n = std::sqrt(w * w + x * x + y * y + z * z);
    return {w / n, x / n, y / n, z / n};
  }
  Vec rotate(Vec v) const {
    auto q = (*this) * Quat{0, v.x, v.y, v.z} * conjugate();
    return {q.x, q.y, q.z};
  }
  static Quat axis(Vec a, double angle) {
    a = unit(a);
    double s = std::sin(angle / 2);
    return {std::cos(angle / 2), a.x * s, a.y * s, a.z * s};
  }
};
struct Mat {
  std::array<std::array<double, 3>, 3> a{};
  static Mat diagonal(Vec v) {
    Mat m;
    for (int i = 0; i < 3; ++i)
      m.a[i][i] = v[i];
    return m;
  }
  Vec operator*(Vec v) const {
    Vec r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        r[i] += a[i][j] * v[j];
    return r;
  }
  Mat &operator+=(const Mat &b) {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        a[i][j] += b.a[i][j];
    return *this;
  }
  Vec solve(Vec v) const {
    auto m = a;
    for (int i = 0; i < 3; ++i) {
      int p = i;
      for (int j = i + 1; j < 3; ++j)
        if (std::abs(m[j][i]) > std::abs(m[p][i]))
          p = j;
      if (std::abs(m[p][i]) < 1e-10)
        throw std::runtime_error("singular inertia");
      std::swap(m[p], m[i]);
      std::swap(v[p], v[i]);
      double d = m[i][i];
      for (int k = i; k < 3; ++k)
        m[i][k] /= d;
      v[i] /= d;
      for (int j = 0; j < 3; ++j)
        if (j != i) {
          double f = m[j][i];
          for (int k = i; k < 3; ++k)
            m[j][k] -= f * m[i][k];
          v[j] -= f * v[i];
        }
    }
    return v;
  }
};
inline Mat parallel_axis(double mass, Vec r) {
  Mat m;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      m.a[i][j] = mass * ((i == j ? dot(r, r) : 0) - r[i] * r[j]);
  return m;
}
struct Wrench {
  Vec force{}, moment{};
  void add(Vec f, Vec r, Vec m = {}) {
    force += f;
    moment += cross(r, f) + m;
  }
};
} // namespace flight
