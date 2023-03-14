//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_PRINCIPAL_COMPONENT_ANALYSIS_3D_H_
#define CODELIBRARY_POINT_CLOUD_PRINCIPAL_COMPONENT_ANALYSIS_3D_H_

#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/math/matrix/matrix3.h"

namespace cl {
namespace point_cloud {

/**
 * Principal Component Analysis for 3D data.
 *
 * Principal Component Analysis (PCA) is an orthogonal linear transformation
 * that transforms the data to a new coordinate system such that the greatest
 * variance by some projection of the data comes to lie on the first coordinate
 * (called the first principal component), the second greatest variance on the
 * second coordinate, and so on.
 */
template <typename T>
class PrincipalComponentAnalysis3D {
    static_assert(std::is_floating_point<T>::value, "");

    using Vector3 = Vector3D<T>;

public:
    /**
     * Compute PCA by covariance method.
     */
    template <typename Point>
    PrincipalComponentAnalysis3D(const Array<Point>& points) {
        CHECK(!points.empty());

        // Compute centorid.
        for (const Point& p : points) {
            centroid_.x += p.x;
            centroid_.y += p.y;
            centroid_.z += p.z;
        }
        centroid_ *= T(1) / points.size();

        // Get the covariance matrix.
        T a00 = 0, a01 = 0, a02 = 0, a11 = 0, a12 = 0, a22 = 0;
        for (const Point& p : points) {
            T x = p.x - centroid_.x;
            T y = p.y - centroid_.y;
            T z = p.z - centroid_.z;

            a00 += x * x;
            a01 += x * y;
            a02 += x * z;
            a11 += y * y;
            a12 += y * z;
            a22 += z * z;
        }

        covariance_matrix_[0] = a00;
        covariance_matrix_[1] = covariance_matrix_[3] = a01;
        covariance_matrix_[2] = covariance_matrix_[6] = a02;
        covariance_matrix_[4] = a11;
        covariance_matrix_[5] = covariance_matrix_[7] = a12;
        covariance_matrix_[8] = a22;
        ComputeEigen();
    }

    /**
     * Compute weighted PCA by covariance method.
     */
    template <typename Point>
    PrincipalComponentAnalysis3D(const Array<Point>& points,
                                 const Array<T>& weights) {
        CHECK(!points.empty());
        CHECK(points.size() == weights.size());

        // Get the covariance matrix.
        centroid_ = Center(points, weights);
        T a00 = 0, a01 = 0, a02 = 0, a11 = 0, a12 = 0, a22 = 0;
        int i = 0;
        for (const Point& p : points) {
            T x = p.x - centroid_.x;
            T y = p.y - centroid_.y;
            T z = p.z - centroid_.z;
            T w = weights[i++];

            a00 += w * x * x;
            a01 += w * x * y;
            a02 += w * x * z;
            a11 += w * y * y;
            a12 += w * y * z;
            a22 += w * z * z;
        }

        covariance_matrix_[0] = a00;
        covariance_matrix_[1] = covariance_matrix_[3] = a01;
        covariance_matrix_[2] = covariance_matrix_[6] = a02;
        covariance_matrix_[4] = a11;
        covariance_matrix_[5] = covariance_matrix_[7] = a12;
        covariance_matrix_[8] = a22;
        ComputeEigen();
    }

    PrincipalComponentAnalysis3D(const PrincipalComponentAnalysis3D&) = delete;

    PrincipalComponentAnalysis3D&
    operator=(const PrincipalComponentAnalysis3D&) = delete;

    const Matrix3<T>& covariance_matrix() const {
        return covariance_matrix_;
    }

    const Array<T>& eigenvalues() const {
        return eigenvalues_;
    }

    const Array<Vector3>& eigenvectors() const {
        return eigenvectors_;
    }

    const Point3D<T>& centroid() const {
        return centroid_;
    }

private:
    /**
     * Computes all eigenvalues and, eigenvectors of a 3 * 3 real symmetric
     * matrix.
     *
     * Reference:
     *  Kopp J. Efficient numerical diagonalization of hermitian 3x3
     *  matrices[J]. International Journal of Modern Physics C, 2008, 19(03):
     *  523-548.
     *
     * Parameters:
     *  mat          - the input matrix using only the lower triangular part.
     *  eigenvalues  - the output eigenvalues will in ascending order.
     *  eigenvectors - the output eigenvectors are normalized so that
     *                 ||z_i||_2 = 1.
     */
    void ComputeEigen() {
        eigenvalues_.resize(3);
        eigenvectors_.resize(3);

        // Compute the eigenvalues.
        Array<T>& w = eigenvalues_;
        const Matrix3<T>& a = covariance_matrix_;

        T a00 = a(0, 0), a01 = a(1, 0), a02 = a(2, 0);
        T a11 = a(1, 1), a12 = a(2, 1), a22 = a(2, 2);

        T de = a01 * a12;
        T dd = a01 * a01;
        T ee = a12 * a12;
        T ff = a02 * a02;

        T m  = a00 + a11 + a22;
        T c1 = a00 * a11 + a00 * a22 + a11 * a22 - (dd + ee + ff);
        T c0 = dd * a22 + ee * a00 + ff * a11 - a00 * a11 * a22 - a02 * de *
               T(2);

        T p = m * m - c1 * T(3);
        T q = m * (p - c1 * T(1.5)) - c0 * T(13.5);
        T sqrt_p = std::sqrt(std::fabs(p));

        T phi = (c1 * c1 * (p - c1) * T(0.25) + c0 * (q + c0 * T(6.75))) *
                T(27);
        phi = std::atan2(std::sqrt(std::fabs(phi)), q) / T(3);

        T c = sqrt_p * std::cos(phi);
        T s = sqrt_p * std::sin(phi) / std::sqrt(T(3));

        w[1]  = (m - c) / T(3);
        w[2]  = w[1] + s;
        w[0]  = w[1] + c;
        w[1] -= s;

        // Compute eigenvectors.
        Array<Vector3>& v = eigenvectors_;

        T max_eigenvalue = std::max(std::fabs(w[0]), std::fabs(w[1]));
        max_eigenvalue = std::max(max_eigenvalue, std::fabs(w[2]));
        T epsilon = std::numeric_limits<T>::epsilon();
        T thresh = epsilon * max_eigenvalue * 8;
        thresh *= thresh;

        // Prepare calculation of eigenvectors.
        T n0tmp = a01 * a01 + a02 * a02;
        T n1tmp = a01 * a01 + a12 * a12;
        v[1].x = a01 * a12 - a02 * a11;
        v[1].y = a02 * a01 - a12 * a00;
        v[1].z = a01 * a01;

        // Calculate first eigenvector by the formula:
        //   v[0] = (A - w[0]).e1 x (A - w[0]).e2.
        a00 -= w[0];
        a11 -= w[0];
        v[0].x = v[1].x + a02 * w[0];
        v[0].y = v[1].y + a12 * w[0];
        v[0].z = a00 * a11 - v[1].z;

        T norm  = v[0].squared_norm();
        T n0    = n0tmp + a00 * a00;
        T n1    = n1tmp + a11 * a11;
        T error = n0 * n1;

        if (n0 <= thresh) {
            v[0] = Vector3(1, 0, 0);
        } else if (n1 <= thresh) {
            v[0] = Vector3(0, 1, 0);
        } else if (norm < T(4096) * epsilon * epsilon * error) {
            // If angle between mat_t[0] and mat_t[1] is too small, don't use
            // cross product, but calculate v = (1, -A0/A1, 0).
            T t = a01 * a01;
            T f = -a00 / a01;
            if (a11 * a11 > t) {
                t =  a11 * a11;
                f = -a01 / a11;
            }
            if (a12 * a12 > t)
                f = -a02 / a12;

            norm = 1 / std::sqrt(f * f + T(1));
            v[0] = Vector3(norm, f * norm, 0);
        } else {
            // This is the standard branch.
            v[0] *= std::sqrt(T(1) / norm);
        }

        // Prepare calculation of second eigenvector.
        T t = w[0] - w[1];
        if (std::fabs(t) > epsilon * max_eigenvalue * T(8)) {
            // For non-degenerate eigenvalue, calculate second eigenvector by
            // the formula
            //   v[1] = (A - w[1]).e1 x (A - w[1]).e2.
            a00 += t;
            a11 += t;
            v[1].x += a02 * w[1];
            v[1].y += a12 * w[1];
            v[1].z = a00 * a11 - v[1].z;

            T norm  = v[1].squared_norm();
            T n0    = n0tmp + a00 * a00;
            T n1    = n1tmp + a11 * a11;
            T error = n0 * n1;

            if (n0 <= thresh) {
                v[1] = Vector3(1, 0, 0);
            } else if (n1 <= thresh) {
                v[1] = Vector3(0, 1, 0);
            } else if (norm < 4096 * epsilon * epsilon * error) {
                // If angle between mat_t[0] and mat_t[1] is too small,
                // don't use cross product, but calculate v ~ (1, -A0/A1, 0).
                T t = a01 * a01;
                T f = -a00 / a01;
                if (a11 * a11 > t) {
                    t =  a11 * a11;
                    f = -a01 / a11;
                }
                if (a12 * a12 > t) f = -a02 / a12;

                norm = T(1) / std::sqrt(f * f + T(1));
                v[1] = Vector3(norm, f * norm, 0);
            } else {
                // This is the standard branch.
                norm = std::sqrt(T(1) / norm);
                v[1] *= norm;
            }
        } else {
            // For degenerate eigenvalue, calculate second eigenvector according
            // to:
            //   v[1] = v[0] x (A - w[1]).e[i].

            // Reset the mat_t to mat.
            a00 += w[0];
            a11 += w[0];
            Matrix3<T> mat_t = {
                a00, a01, a02,
                a01, a11, a12,
                a02, a12, a22
            };

            int i = 0;
            for (i = 0; i < 3; ++i) {
                mat_t(i, i) -= w[1];

                T n0 = mat_t(0, i) * mat_t(0, i) +
                       mat_t(1, i) * mat_t(1, i) +
                       mat_t(2, i) * mat_t(2, i);
                if (n0 > thresh) {
                    v[1].x = v[0].y * mat_t(2, i) - v[0].z * mat_t(1, i);
                    v[1].y = v[0].z * mat_t(0, i) - v[0].x * mat_t(2, i);
                    v[1].z = v[0].x * mat_t(1, i) - v[0].y * mat_t(0, i);

                    T norm = v[1].squared_norm();
                    if (norm > epsilon * epsilon * T(65536) * n0) {
                        // Accept cross product only if the angle between the
                        // two vectors was not too small.
                        v[1] *= std::sqrt(T(1) / norm);
                        break;
                    }
                }
            }

            if (i == 3) {
                // This means that any vector orthogonal to v[0] is an EV.
                for (int j = 0; j < 3; ++j) {
                    if (v[0][j] != 0) {
                        // Find nonzero element of v[0] and swap it with the
                        // next one.
                        int k = (j + 1) % 3;
                        T t = T(1) / std::sqrt(v[0][j] * v[0][j] +
                                               v[0][k] * v[0][k]);
                        v[1][j] =  v[0][k] * t;
                        v[1][k] = -v[0][j] * t;
                        v[1][(j + 2) % 3] = 0;
                        break;
                    }
                }
            }
        }

        // Calculate third eigenvector according to v[2] = v[0] x v[1].
        v[2] = CrossProduct(v[0], v[1]);

        // Sort in ascending order.
        if (w[1] < w[0]) {
            std::swap(w[0], w[1]);
            std::swap(v[0], v[1]);
        }
        if (w[2] < w[1]) {
            std::swap(w[1], w[2]);
            std::swap(v[1], v[2]);
            if (w[1] < w[0]) {
                std::swap(w[0], w[1]);
                std::swap(v[0], v[1]);
            }
        }
    }

    // The centroid point of the input data.
    Point3D<T> centroid_;

    // Covariance matrix of the input data.
    Matrix3<T> covariance_matrix_;

    // Eigenvalues of the covariance matrix, sorted in ascending order.
    Array<T> eigenvalues_;

    // Corresponding eigenvectors.
    Array<Vector3> eigenvectors_;
};

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_PRINCIPAL_COMPONENT_ANALYSIS_3D_H_
