#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {
    //TODO
	  t_ = Cvec3(t[0], t[1], t[2]);
	  r_ = Quat(r[0], r[1], r[2], r[3]);
  }

  explicit RigTForm(const Cvec3& t) {
    // TODO
	  t_ = Cvec3(t[0], t[1], t[2]);
	  r_ = Quat(1, 0, 0, 0);
  }

  explicit RigTForm(const Quat& r) {
    // TODO
	  t_ = Cvec3(0, 0, 0);
	  r_ = Quat(r[0], r[1], r[2], r[3]);
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = Cvec3(t[0], t[1], t[2]);
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = Quat(r[0], r[1], r[2], r[3]);
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
    // TODO
	  return r_*a + Cvec4(t_[0], t_[1], t_[2], 0);
  }

  RigTForm operator * (const RigTForm& a) const {
    // TODO
	  Quat temp;

	  temp = Quat(0, t_[0], t_[1], t_[2]) + r_ * Quat(0, a.t_[0], a.t_[1], a.t_[2]) * inv(r_);
	  return RigTForm(Cvec3(temp[1], temp[2], temp[3]), r_ * a.r_);
  }
};

inline RigTForm inv(const RigTForm& tform) {
  // TODO
	Quat q;
	Cvec3 temp;
	Quat _t;

	q = inv(tform.getRotation());
	temp = tform.getTranslation();
	_t = q * Quat(0, -1*temp[0], -1*temp[1], -1*temp[2]) * inv(q);

	return RigTForm(Cvec3(_t[1], _t[2], _t[3]), q);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline RigTForm interpolate(const RigTForm& a, const RigTForm& b, const RigTForm& c, const RigTForm& d, double t) {
	Quat d_ = power(cn(c.getRotation() * inv(a.getRotation())), 1 / 6.0) * b.getRotation();
	Quat e_ = power(cn(d.getRotation() * inv(b.getRotation())), -1 / 6.0) * c.getRotation();

	return RigTForm(lerp(a.getTranslation(), b.getTranslation(), c.getTranslation(), d.getTranslation(), t),
		slerp(
			slerp(
				slerp(b.getRotation(), d_, t),
				slerp(d_, e_, t),
				t)
			, slerp(
				slerp(d_, e_, t),
				slerp(e_, c.getRotation(), t),
				t)
			, t));
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
  // TODO
	Matrix4 m, t, r;

	//t = Matrix4();
	t = Matrix4::makeTranslation(tform.getTranslation());

	r = quatToMatrix(tform.getRotation());

	m = t * r;
  return m;
}

#endif
