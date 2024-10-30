#pragma once
#include <cassert>
#include <mathematics.h>
#include <math.h>
#include <array>
#include <variant>

static constexpr size_t size_t_max = std::numeric_limits<size_t>::max();

inline float bivector_area(Vector v1, Vector v2){
  return Norm(v1 / v2);
}

inline float angle_between(Vector v1, Vector v2){
  return acos(Normalized(v1) * Normalized(v2));
}

enum class TriOrient : char {
  CW = -1,
  Flat = 0,
  CCW = 1
};

inline TriOrient orientation_2d(const std::array<Vector, 3>& tri){
  const Vector u = tri[1] - tri[0]; 
  const Vector v = tri[2] - tri[0]; 

  // Ensure a good estimation of the zero value
  // const double max_error = std::max(std::max(u[0], u[1]), std::max(v[0], v[1])); 
  // const double epsilon = std::numeric_limits<double>::epsilon() * max_error * 64; 

  const double orientation = u[0] * v[1] - u[1] * v[0];
  const double epsilon = std::numeric_limits<double>::epsilon() * std::abs(orientation) * 16; 

  if (orientation > -epsilon && orientation < epsilon)
    return TriOrient::Flat; 

  return orientation > 0 ? TriOrient::CCW : TriOrient::CW;
}

template <unsigned int k>
struct LocalId {
  LocalId(): index(0) {}
  template <typename T> LocalId(T id) : index(id) {}
  static LocalId make_invalid() { auto a = LocalId(0); a.index = -1; return a; }

  bool is_valid() const { return index >= 0; }
  LocalId operator-(const int n) const { return LocalId(get_id(index, -n)); }
  LocalId operator+(const int n) const { return LocalId(get_id(index, n)); }
  LocalId &operator--(const int n) {
    index = get_id(index, -n);
    return *this;
  }
  LocalId &operator++(const int n) {
    index = get_id(index, n);
    return *this;
  }
  operator int() const { return index; }

private:
  int index;
  int static get_id(int index, int offset) {
    if (offset < 0){
      int a = std::abs(offset);
      return (index + k - (a % k)) % k;
    }
    return (index + offset) % k;
  }
};
