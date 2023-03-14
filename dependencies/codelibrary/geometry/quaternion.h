//
// Copyright 2013-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin.x_mu@qq.com (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_QUATERNION_H_
#define CODELIBRARY_GEOMETRY_QUATERNION_H_

#include <limits>

#include "codelibrary/base/log.h"
#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/vector_3d.h"
#include "codelibrary/geometry/vector_4d.h"
#include "codelibrary/math/matrix/matrix3.h"

namespace cl {

/**
 * Quaternions provide a convenient mathematical notation for representing
 * orientations and rotations of objects in three dimensions.
 *
 * Note that, Quaternion q and -q represent the same rotation.
 */
template <typename T>
class Quaternion : public Vector4D<T> {
    static_assert(std::is_floating_point<T>::value, "");

    using Vector4 = Vector4D<T>;

public:
    /**
     * The default quaternion is all zero (invalid).
     */
    Quaternion() = default;

    /**
     * We make sure every quaternion is an unit vector.
     */
    Quaternion(T x, T y, T z, T w)
        : Vector4D<T>(x, y, z, w) {}

    /**
     * Construct quaternion that rotate the specified angle (in radians) around
     * the rotation_axis.
     */
    Quaternion(const Vector3D<T>& rotation_axis, T rotation_angle) {
        T norm = rotation_axis.norm();
        if (norm < std::numeric_limits<T>::epsilon()) {
            *this = identity();
            return;
        }

        T c = std::cos(T(0.5) * rotation_angle);
        T s = std::sin(T(0.5) * rotation_angle) / norm;
        this->x = rotation_axis.x * s;
        this->y = rotation_axis.y * s;
        this->z = rotation_axis.z * s;
        this->w = c;
    }

    /**
     * Construct quaternion from two vectors.
     * Represent rotation from v1 to v2.
     */
    Quaternion(const Vector3D<T>& v1, const Vector3D<T>& v2) {
        Vector3D<T> rotation_axis = CrossProduct(v1, v2);
        T phi = Radian(v1, v2);
        *this = Quaternion(rotation_axis, phi);
    }

    /**
     * Construct quaternion from 3 x 3 rotation matrix.
     */
    explicit Quaternion(const Matrix3<T>& rotation) {
        T trace = rotation(0, 0) + rotation(1, 1) + rotation(2, 2);
        if (trace > 0) {
            T t = T(1) + trace;
            T s = T(0.5) / std::sqrt(t);
            this->w = s * t;
            this->x = (rotation(2, 1) - rotation(1, 2)) * s;
            this->y = (rotation(0, 2) - rotation(2, 0)) * s;
            this->z = (rotation(1, 0) - rotation(0, 1)) * s;
        } else {
            int next[3] = { 1, 2, 0 };

            int i = 0;
            if (rotation(1, 1) > rotation(0, 0)) {
                i = 1;
            }
            if (rotation(2, 2) > rotation(i, i)) {
                i = 2;
            }
            int j = next[i];
            int k = next[j];
            T t = (rotation(i, i) - rotation(j, j) - rotation(k, k)) + 1;
            T s = T(0.5) / std::sqrt(t);
            T q[4];
            q[i] = s * t;
            q[3] = (rotation(k, j) - rotation(j, k)) * s;
            q[j] = (rotation(j, i) + rotation(i, j)) * s;
            q[k] = (rotation(k, i) + rotation(i, k)) * s;
            this->x = q[0];
            this->y = q[1];
            this->z = q[2];
            this->w = q[3];
        }

        this->Normalize();
    }

    /**
     * Construct Quaternion from Euler angles.
     */
    Quaternion(T pitch, T yaw, T roll) {
        T c1 = std::cos(T(0.5) * pitch);
        T c2 = std::cos(T(0.5) * yaw);
        T c3 = std::cos(T(0.5) * roll);
        T s1 = std::sin(T(0.5) * pitch);
        T s2 = std::sin(T(0.5) * yaw);
        T s3 = std::sin(T(0.5) * roll);

        this->w = c1 * c2 * c3 - s1 * s2 * s3;
        this->x = s1 * s2 * c3 + c1 * c2 * s3;
        this->y = s1 * c2 * c3 + c1 * s2 * s3;
        this->z = c1 * s2 * c3 - s1 * c2 * s3;
    }

    /**
     * Return an identity quaternion.
     */
    static Quaternion identity() {
        return Quaternion(0, 0, 0, 1);
    }

    /**
     * Return the reverse of this quaternion.
     */
    Quaternion inverse() const {
        return Quaternion(-this->x, -this->y, -this->z, this->w);
    }

    /**
     * Convert quaternion to a rotation angle(in radians) and rotation axis.
     */
    void ToRotation(Vector3D<T>* rotation_axis, T* rotation_angle) const {
        CHECK(rotation_angle);
        CHECK(rotation_axis);

        T norm = this->norm();
        if (norm < std::numeric_limits<T>::epsilon()) {
            *rotation_angle = 0;
            *rotation_axis = Vector3D<T>(0, 0, 1);
            return;
        }

        *rotation_angle = T(2) * std::acos(this->w / norm);
        rotation_axis->x = this->x;
        rotation_axis->y = this->y;
        rotation_axis->z = this->z;
        rotation_axis->Normalize();
    }

    /**
     * Convert quaternion to rotation matrix.
     */
    void ToRotationMatrix(Matrix3<T>* rotation) const {
        CHECK(rotation);

        Quaternion q = *this;
        q.Normalize();

        T sqw = q.w * q.w;
        T sqx = q.x * q.x;
        T sqy = q.y * q.y;
        T sqz = q.z * q.z;

        (*rotation)[0] =  sqx - sqy - sqz + sqw;
        (*rotation)[4] = -sqx + sqy - sqz + sqw;
        (*rotation)[8] = -sqx - sqy + sqz + sqw;

        T tmp1 = q.x * q.y;
        T tmp2 = q.z * q.w;
        (*rotation)[3] = T(2) * (tmp1 + tmp2);
        (*rotation)[1] = T(2) * (tmp1 - tmp2);

        tmp1 = q.x * q.z;
        tmp2 = q.y * q.w;
        (*rotation)[6] = T(2) * (tmp1 - tmp2);
        (*rotation)[2] = T(2) * (tmp1 + tmp2);

        tmp1 = q.y * q.z;
        tmp2 = q.x * q.w;
        (*rotation)[7] = T(2) * (tmp1 + tmp2);
        (*rotation)[5] = T(2) * (tmp1 - tmp2);
    }

    /**
     * Convert quaternion to Euler angles (in radian form).
     */
    void ToEulerAngles(T* pitch, T* yaw, T* roll) {
        CHECK(pitch && yaw && roll);

        T x = this->x, y = this->y, z = this->z, w = this->w;
        T sw = w * w;
        T sx = x * x;
        T sy = y * y;
        T sz = z * z;

        T unit = sw + sx + sy + sz;
        T test = x * y + z * w;
        if (test > T(0.499) * unit) {
            // Singularity at north pole.
            *pitch = T(2) * std::atan2(x, w);
            *yaw   = T(0.5) * M_PI;
            *roll  = 0;
            return;
        }

        if (test < T(-0.499) * unit) {
            // Singularity at south pole.
            *pitch = T(-2) * std::atan2(x, w);
            *yaw   = T(-0.5) * M_PI;
            *roll  = 0;
            return;
        }

        *pitch = std::atan2(T(2) * y * w - T(2) * x * z, sx - sy - sz + sw);
        *yaw   = std::asin(T(2) * test / unit);
        *roll  = std::atan2(T(2) * x * w - T(2) * y * z, -sx + sy - sz + sw);
    }

    /**
     * Set this quaternion to the product of two quaternions, and return the
     * product.
     * Two rotation quaternions can be combined into one equivalent quaternion
     * by multiplication.
     */
    Quaternion& Multiply(const Quaternion& a, const Quaternion& b) {
        T x1 = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        T y1 = a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z;
        T z1 = a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x;
        T w1 = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        this->x = x1;
        this->y = y1;
        this->z = z1;
        this->w = w1;
        return *this;
    }

    /**
     * Rotate the given vector by this quaternion.
     */
    Vector3D<T> Rotate(const Vector3D<T>& v) const {
        Quaternion q(v.x, v.y, v.z, 0);
        q = *this * q * this->inverse();

        return Vector3D<T>(q.x, q.y, q.z);
    }

    Quaternion& operator *=(const Quaternion& rhs) {
        return Multiply(*this, rhs);
    }

    friend Quaternion operator *(const Quaternion& lhs, const Quaternion& rhs) {
        return Quaternion().Multiply(lhs, rhs);
    }

    friend Vector3D<T> operator *(const Quaternion& a, const Vector3D<T>& b) {
        return a.Rotate(b);
    }
};

using FQuaternion = Quaternion<float>;
using RQuaternion = Quaternion<double>;

/**
 * Compute the spherical linear interpolation between two quaternions.
 * The parameter 't' argument has to be such that 0 <= t <= 1.
 */
template <typename T>
Quaternion<T> Interpolation(const Quaternion<T>& q1, const Quaternion<T>& q2,
                            T t) {
    CHECK(t >= 0 && t <= 1);

    T invert = 1;

    // Compute cos(theta) using the quaternion scalar product.
    T cosine_theta = DotProduct(q1, q2);

    // Take care of the sign of cosine_theta.
    if (cosine_theta < 0) {
        cosine_theta = -cosine_theta;
        invert = -1;
    }

    // theta will close to 0 when cos(theta) close to 1. We can rewrite
    // sin((1-t)*theta) as (1-t) and sin(t*theta) as t
    const T epsilon = T(0.00001);
    if (1 - cosine_theta < epsilon) {
        return q1 * (1 - t) + q2 * (t * invert);
    }

    // Compute the theta angle.
    T theta = std::acos(cosine_theta);

    // Compute sin(theta).
    T sine_theta = std::sin(theta);

    // Compute the two coefficients that are in the spherical linear
    // interpolation formula
    T coeff1 = std::sin((1 - t) * theta) / sine_theta;
    T coeff2 = std::sin(t * theta) / sine_theta * invert;

    // Compute and return the interpolated quaternion.
    return q1 * coeff1 + q2 * coeff2;
}

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_QUATERNION_H_
