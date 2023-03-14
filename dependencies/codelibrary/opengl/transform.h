//
// Copyright 2020-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_TRANSFORM_H_
#define CODELIBRARY_OPENGL_TRANSFORM_H_

#include <cmath>

#include "codelibrary/base/array.h"
#include "codelibrary/base/array_nd.h"
#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/quaternion.h"
#include "codelibrary/geometry/vector_3d.h"
#include "codelibrary/math/matrix/matrix3.h"
#include "codelibrary/math/matrix/matrix4.h"

namespace cl {
namespace gl {

/**
 * 4x4 transform matrix for OpenGL.
 *
 * Note that since opengl's matrix is column-major, thus we always store the
 * transpose of matrix in 'Transform'.
 */
class Transform : public FMatrix4 {
public:
    /**
     * The default transform is a unit matrix.
     */
    Transform() {
        this->SetIdentity();
    }

    Transform(const FMatrix4& mat)
        : FMatrix4(mat) {}

    Transform(float m1,  float m2,  float m3,  float m4,
              float m5,  float m6,  float m7,  float m8,
              float m9,  float m10, float m11, float m12,
              float m13, float m14, float m15, float m16)
        : FMatrix4(m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13, m14,
                   m15, m16) {}

    explicit Transform(const FQuaternion& rot) {
        this->SetIdentity();
        FMatrix3 m;
        rot.ToRotationMatrix(&m);
        this->SetRotation(m);
    }

    /**
     *                                   [1 0 0 x]
     * Multiply by a translation matrix: [0 1 0 y]
     *                                   [0 0 1 z]
     *                                   [0 0 0 1]
     */
    void Translate(float x, float y, float z) {
        data_[12] += x;
        data_[13] += y;
        data_[14] += z;
    }
    void Translate(const FVector3D& v) {
        Translate(v.x, v.y, v.z);
    }

    /**
     * Set the translation vector.
     */
    void SetTranslation(float x, float y, float z) {
        data_[12] = x;
        data_[13] = y;
        data_[14] = z;
    }
    void SetTranslation(const FVector3D& v) {
        this->SetTranslation(v.x, v.y, v.z);
    }

    /**
     * Return the translation vector.
     */
    FVector3D GetTranslation() const {
        return {data_[12], data_[13], data_[14]};
    }

    /**
     * Multiply by a rotation matrix defined by two vectors: v1 and v2
     * (rotation from v1 to v2).
     */
    void Rotate(const FVector3D& v1, const FVector3D& v2) {
        FQuaternion rot(v1, v2);
        this->Rotate(rot);
    }

    /**
     * Multiply by a rotation matrix defined by a rotation axis and rotation
     * angle.
     */
    void Rotate(const FVector3D& rotation_axis, float rotation_angle) {
        FQuaternion rot(rotation_axis, rotation_angle);
        this->Rotate(rot);
    }

    /**
     * Multiply by a quernion.
     */
    void Rotate(const FQuaternion& rot) {
        FMatrix3 m;
        rot.ToRotationMatrix(&m);
        this->Rotate(m);
    }

    /**
     * Multiply by a rotation matrix.
     */
    void Rotate(const FMatrix3& rot) {
        Transform t;
        t.SetRotation(rot);
        *this = *this * t;
    }

    /**
     * Set rotation matrix.
     */
    void SetRotation(const FMatrix3& rot) {
        data_[0] = rot[0]; data_[4] = rot[1]; data_[8]  = rot[2];
        data_[1] = rot[3]; data_[5] = rot[4]; data_[9]  = rot[5];
        data_[2] = rot[6]; data_[6] = rot[7]; data_[10] = rot[8];
    }

    /**
     * Get rotation matrix.
     */
    void GetRotation(FMatrix3* rot) const {
        CHECK(rot);

        *rot = FMatrix3(data_[0], data_[3], data_[6],
                        data_[1], data_[4], data_[7],
                        data_[2], data_[5], data_[8]);
    }

    /**
     * Get quaternion.
     */
    void GetRotation(FQuaternion* rot) const {
        CHECK(rot);

        FMatrix3 r(data_[0], data_[3], data_[6],
                   data_[1], data_[4], data_[7],
                   data_[2], data_[5], data_[8]);
        *rot = FQuaternion(r);
    }

    /**
     * Scale the transform.
     */
    void Scale(float x, float y, float z) {
        data_[0]  *= x;
        data_[1]  *= x;
        data_[2]  *= x;

        data_[4]  *= y;
        data_[5]  *= y;
        data_[6]  *= y;

        data_[8]  *= z;
        data_[9]  *= z;
        data_[10] *= z;
    }

    /**
     * Counterclockwise rotation about the local X-axis.
     */
    void Pitch(float degree) {
        // [a00 a01 a02]   [ 1 0  0]
        // [a10 a11 a12] * [ 0 c -s]
        // [a20 a21 a22]   [ 0 s  c]
        float radian = DegreeToRadian(degree);
        const float c = std::cos(radian), s = std::sin(radian);
        const float a01 = data_[4], a02 = data_[8];
        const float a11 = data_[5], a12 = data_[9];
        const float a21 = data_[6], a22 = data_[10];

        data_[4] =  a01 * c + a02 * s;
        data_[8] = -a01 * s + a02 * c;

        data_[5] =  a11 * c + a12 * s;
        data_[9] = -a11 * s + a12 * c;

        data_[6]  =  a21 * c + a22 * s;
        data_[10] = -a21 * s + a22 * c;
    }

    /**
     * Counterclockwise rotation about the local Y-axis.
     */
    void Yaw(float degree) {
        // [a00 a01 a02]   [ c  0 s]
        // [a10 a11 a12] * [ 0  1 0]
        // [a20 a21 a22]   [ -s 0 c]
        float radian = DegreeToRadian(degree);
        const float c = std::cos(radian), s = std::sin(radian);
        const float a00 = data_[0], a02 = data_[8];
        const float a10 = data_[1], a12 = data_[9];
        const float a20 = data_[2], a22 = data_[10];

        data_[0] = a00 * c - a02 * s;
        data_[8] = a00 * s + a02 * c;

        data_[1] = a10 * c - a12 * s;
        data_[9] = a10 * s + a12 * c;

        data_[2]  = a20 * c - a22 * s;
        data_[10] = a20 * s + a22 * c;
    }

    /**
     * Counterclockwise rotation about the local Z-axis.
     */
    void Roll(float degree) {
        // [a00 a01 a02]   [ c -s 0]
        // [a10 a11 a12] * [ s  c 0]
        // [a20 a21 a22]   [ 0  0 1]
        float radian = DegreeToRadian(degree);
        const float c = std::cos(radian), s = std::sin(radian);
        const float a00 = data_[0], a01 = data_[4];
        const float a10 = data_[1], a11 = data_[5];
        const float a20 = data_[2], a21 = data_[6];

        data_[0] =  a00 * c + a01 * s;
        data_[4] = -a00 * s + a01 * c;

        data_[1] =  a10 * c + a11 * s;
        data_[5] = -a10 * s + a11 * c;

        data_[2] =  a20 * c + a21 * s;
        data_[6] = -a20 * s + a21 * c;
    }

    /**
     * Transform the given point.
     */
    FPoint3D operator()(const FPoint3D& p) const {
        float x = data_[0] * p.x + data_[4] * p.y + data_[8]  * p.z + data_[12];
        float y = data_[1] * p.x + data_[5] * p.y + data_[9]  * p.z + data_[13];
        float z = data_[2] * p.x + data_[6] * p.y + data_[10] * p.z + data_[14];
        float w = data_[3] * p.x + data_[7] * p.y + data_[11] * p.z + data_[15];
        return {x / w, y / w, z / w};
    }
    FPoint3D operator *(const FPoint3D& p) const {
        return this->operator()(p);
    }

    /**
     * Transform the given vector.
     */
    FVector3D operator()(const FVector3D& v) const {
        float x = data_[0] * v.x + data_[4] * v.y + data_[8]  * v.z;
        float y = data_[1] * v.x + data_[5] * v.y + data_[9]  * v.z;
        float z = data_[2] * v.x + data_[6] * v.y + data_[10] * v.z;
        return {x, y, z};
    }
    FVector3D operator *(const FVector3D& v) const {
        return this->operator()(v);
    }

    friend Transform operator*(const Transform& lhs, const Transform& rhs) {
        Transform res;
        float* c = res.data();

        // A^-1 * B^-1 = (BA)^-1
        const float* b = lhs.data();
        const float* a = rhs.data();

        c[0] = a[0] * b[0] + a[1] * b[4] + a[2] * b[8]  + a[3] * b[12];
        c[1] = a[0] * b[1] + a[1] * b[5] + a[2] * b[9]  + a[3] * b[13];
        c[2] = a[0] * b[2] + a[1] * b[6] + a[2] * b[10] + a[3] * b[14];
        c[3] = a[0] * b[3] + a[1] * b[7] + a[2] * b[11] + a[3] * b[15];

        c[4] = a[4] * b[0] + a[5] * b[4] + a[6] * b[8]  + a[7] * b[12];
        c[5] = a[4] * b[1] + a[5] * b[5] + a[6] * b[9]  + a[7] * b[13];
        c[6] = a[4] * b[2] + a[5] * b[6] + a[6] * b[10] + a[7] * b[14];
        c[7] = a[4] * b[3] + a[5] * b[7] + a[6] * b[11] + a[7] * b[15];

        c[8]  = a[8] * b[0] + a[9] * b[4] + a[10] * b[8]  + a[11] * b[12];
        c[9]  = a[8] * b[1] + a[9] * b[5] + a[10] * b[9]  + a[11] * b[13];
        c[10] = a[8] * b[2] + a[9] * b[6] + a[10] * b[10] + a[11] * b[14];
        c[11] = a[8] * b[3] + a[9] * b[7] + a[10] * b[11] + a[11] * b[15];

        c[12] = a[12] * b[0] + a[13] * b[4] + a[14] * b[8]  + a[15] * b[12];
        c[13] = a[12] * b[1] + a[13] * b[5] + a[14] * b[9]  + a[15] * b[13];
        c[14] = a[12] * b[2] + a[13] * b[6] + a[14] * b[10] + a[15] * b[14];
        c[15] = a[12] * b[3] + a[13] * b[7] + a[14] * b[11] + a[15] * b[15];

        return res;
    }

    /**
     * Output the matrix, just for debug.
     */
    friend Message& operator <<(Message& msg, const Transform& rhs) {
        return msg.Append(rhs.data_, rhs.data_ + 16, {4, 4});
    }
};

} // namespace gl
} // namespace cl

#endif // CODELIBRARY_OPENGL_TRANSFORM_H_
