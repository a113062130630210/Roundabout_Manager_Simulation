#include <cmath>
#include <iostream>
#include <stdexcept>

template <typename T>
class modular;

template <typename T>
modular<T> operator+(modular<T>, const modular<T>&);
template <typename T>
modular<T> operator-(modular<T>, const modular<T>&);
template <typename T>
modular<T> operator+(modular<T>, const T&);
template <typename T>
modular<T> operator-(modular<T>, const T&);
template <typename T>
bool operator<(const modular<T>&, const modular<T>&);
template <typename T>
bool operator>(const modular<T>&, const modular<T>&);
template <typename T>
bool operator<=(const modular<T>&, const modular<T>&);
template <typename T>
bool operator>=(const modular<T>&, const modular<T>&);
template <typename T>
bool operator==(const modular<T>&, const modular<T>&);
template <typename T>
bool operator!=(const modular<T>&, const modular<T>&);
template <typename T>
std::ostream& operator<<(std::ostream&, const modular<T>&);
template<typename T>
void divisor_error(T, T, const std::string&);

template <typename T>
class modular {
public:
  modular(T, T);

  modular(const modular<T>&);
  modular(modular<T>&&);

  modular<T>& operator=(const modular<T>&);
  modular<T>& operator=(modular<T>&&);

  modular<T>& operator+=(const modular<T>&);
  modular<T>& operator-=(const modular<T>&);
  modular<T>& operator+=(const T&);
  modular<T>& operator-=(const T&);
  
  T operator*() const;

  friend modular operator+<T>(modular<T>, const modular<T>&);
  friend modular operator-<T>(modular<T>, const modular<T>&);
  friend modular operator+<T>(modular<T>, const T&);
  friend modular operator-<T>(modular<T>, const T&);

  friend bool operator<  <T>(const modular<T>&, const modular<T>&);

  friend bool operator>  <T>(const modular<T>&, const modular<T>&);
  friend bool operator<= <T>(const modular<T>&, const modular<T>&);
  friend bool operator>= <T>(const modular<T>&, const modular<T>&);
  friend bool operator== <T>(const modular<T>&, const modular<T>&);
  friend bool operator!= <T>(const modular<T>&, const modular<T>&);

  friend std::ostream& operator<< <T>(std::ostream&, const modular<T>&);

private:
  double _n;
  double _i;
};


template <typename T>
modular<T>::modular(T n, T i): _n(n), _i(i) {
  if (n <= 0) throw std::range_error("negative divisor");
}

template <typename T>
modular<T>::modular(const modular<T>& m) {
  if (m._n <= 0) throw std::range_error("negative divisor");
  _n = m._n;
  _i = m._i;
}

template <typename T>
modular<T>::modular(modular<T>&& m) {
  if (m._n <= 0) throw std::range_error("negative divisor");
  _n = m._n;
  _i = m._i;
}

template<typename T>
modular<T>& modular<T>::operator=(const modular<T>& rhs) {
  if (this == &rhs) return *this;
  if (_n != rhs._n) divisor_error(_n, rhs._n, "copy assigment");
  _i = rhs._i;
  T t = floor(_i / _n);
  _i -= t * _n;
  return *this;
}

template<typename T>
modular<T>& modular<T>::operator=(modular<T>&& rhs) {
  if (this == &rhs) return *this;
  if (_n != rhs._n) divisor_error(_n, rhs._n, "move assigment");
  _i = rhs._i;
  T t = floor(_i / _n);
  _i -= t * _n;
  return *this;
}

template<typename T>
modular<T>& modular<T>::operator+=(const modular<T>& rhs) {
  if (_n != rhs._n) divisor_error(_n, rhs._n, "operator+=");
  _i += rhs._i;
  if (_i >= _n) _i -= _n;
  return *this;
}

template<typename T>
modular<T>& modular<T>::operator-=(const modular<T>& rhs) {
  if (_n != rhs._n) divisor_error(_n, rhs._n, "operator-=");
  _i -= rhs._i;
  if (_i < 0) _i += _n;
  return *this;
}

template<typename T>
modular<T>& modular<T>::operator+=(const T& rhs) {
  _i += rhs;
  T t = floor(_i / _n);
  _i -= t * _n;
  return *this;
}

template<typename T>
modular<T>& modular<T>::operator-=(const T& rhs) {
  _i -= rhs;
  T t = floor(_i / _n);
  _i -= t * _n;
  return *this;
}

template<typename T>
T modular<T>::operator*() const {
  return _i;
}

template<typename T>
inline modular<T> operator+(modular<T> lhs, const modular<T>& rhs) {
  lhs += rhs;
  return lhs;
}

template<typename T>
inline modular<T> operator-(modular<T> lhs, const modular<T>& rhs) {
  if (lhs._n != rhs._n) divisor_error(lhs._n, rhs._n, "operator-");
  lhs -= rhs;
  return lhs;
}

template<typename T>
inline modular<T> operator+(modular<T> lhs, const T& rhs) {
  lhs += rhs;
  return lhs;
}

template<typename T>
inline modular<T> operator-(modular<T> lhs, const T& rhs) {
  lhs -= rhs;
  return lhs;
}

template<typename T>
inline bool operator<(const modular<T>& lhs, const modular<T>& rhs) {
  if (lhs._n != rhs._n) divisor_error(lhs._n, rhs._n, "operator<");
  return lhs._i < rhs._i;
}

template<typename T>
inline bool operator>(const modular<T>& lhs, const modular<T>& rhs) {
  return rhs < lhs;
}

template<typename T>
inline bool operator<=(const modular<T>& lhs, const modular<T>& rhs) {
  return !(rhs < lhs);
}

template<typename T>
inline bool operator>=(const modular<T>& lhs, const modular<T>& rhs) {
  return !(lhs < rhs);
}

template<typename T>
inline bool operator==(const modular<T>& lhs, const modular<T>& rhs) {
  if (lhs._n != rhs._n) divisor_error(lhs._n, rhs._n, "operator==");
  return lhs._i == rhs._i;
}

template<typename T>
inline bool operator!=(const modular<T>& lhs, const modular<T>& rhs) {
  return !(lhs == rhs);
}

template<typename T>
std::ostream& operator<<(std::ostream& os, const modular<T>& rhs) {
  os << rhs._i;
  return os;
}

template<typename T>
void divisor_error(T l, T r, const std::string& cause) {
  std::cout << "divisors: " << l << " " << r << ", from " << cause << "\n";
  throw std::logic_error("different divisor");
}