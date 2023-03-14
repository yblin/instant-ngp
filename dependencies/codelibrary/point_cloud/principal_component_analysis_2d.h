//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_PRINCIPAL_COMPONENT_ANALYSIS_2D_H_
#define CODELIBRARY_POINT_CLOUD_PRINCIPAL_COMPONENT_ANALYSIS_2D_H_

#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/math/matrix/matrix2.h"

namespace cl {
namespace point_cloud {

/**
 * Principal component analysis for 2D points.
 *
 * Principal Component Analysis (PCA) is an orthogonal linear transformation
 * that transforms the data to a new coordinate system such that the greatest
 * variance by some projection of the data comes to lie on the first coordinate
 * (called the first principal component), the second greatest variance on the
 * second coordinate, and so on.
 */
template <typename T>
class PrincipalComponentAnalysis2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    /**
     * Compute PCA by covariance method.
     */
    template <typename Point>
    PrincipalComponentAnalysis2D(const Array<Point>& points) {
        CHECK(!points.empty());

        // Compute centorid.
        for (const Point& p : points) {
            centroid_.x += p.x;
            centroid_.y += p.y;
        }
        centroid_ *= T(1) / points.size();

        // Get the covariance matrix.
        T a00 = 0, a01 = 0, a11 = 0;
        for (const Point& p : points) {
            T x = p.x - centroid_.x;
            T y = p.y - centroid_.y;

            a00 += x * x;
            a01 += x * y;
            a11 += y * y;
        }

        covariance_matrix_[0] = a00;
        covariance_matrix_[1] = covariance_matrix_[2] = a01;
        covariance_matrix_[3] = a11;
        ComputeEigen();
    }

    /**
     * Compute weighted PCA by covariance method.
     */
    template <typename Point>
    PrincipalComponentAnalysis2D(const Array<Point>& points,
                                 const Array<T>& weights) {
        CHECK(!points.empty());
        CHECK(points.size() == weights.size());

        // Get the covariance matrix.
        centroid_ = Center(points, weights);
        T a00 = 0, a01 = 0, a11 = 0;
        int i = 0;
        for (const Point& p : points) {
            T x = p.x - centroid_.x;
            T y = p.y - centroid_.y;
            T w = weights[i++];

            a00 += w * x * x;
            a01 += w * x * y;
            a11 += w * y * y;
        }

        covariance_matrix_[0] = a00;
        covariance_matrix_[1] = covariance_matrix_[2] = a01;
        covariance_matrix_[3] = a11;

        ComputeEigen();
    }

    PrincipalComponentAnalysis2D(const PrincipalComponentAnalysis2D&) = delete;

    void operator=(const PrincipalComponentAnalysis2D&) = delete;

    const Matrix2<T>& covariance_matrix() const {
        return covariance_matrix_;
    }

    const Array<T>& eigenvalues() const {
        return eigenvalues_;
    }

    const Array<Vector2D<T>>& eigenvectors() const {
        return eigenvectors_;
    }

    const Point2D<T>& centroid() const {
        return centroid_;
    }

private:
    /**
     * Compute all eigenvalues and eigenvectors of a 2 * 2 real symmetric
     * matrix.
     *
     * Reference:
     *   Kopp J. Efficient numerical diagonalization of hermitian 3x3
     *   matrices[J]. International Journal of Modern Physics C, 2008, 19(03):
     *   523-548.
     *
     * Parameters:
     *  mat          - the input matrix using only the lower triangular part.
     *  eigenvalues  - the output eigenvalues will in ascending order.
     *  eigenvectors - the output eigenvectors are normalized so that
     *                 ||z_i||_2 = 1.
     */
    void ComputeEigen() {
        eigenvalues_.resize(2);
        eigenvectors_.resize(2);

        const Matrix2<T>& mat = covariance_matrix_;
        Array<T>& w = eigenvalues_;

        // Compute the eigenvalues.
        // [ A  B ]  =  [ cs  -sn ] [ rt1   0  ] [  cs  sn ]
        // [ B  C ]     [ sn   cs ] [  0   rt2 ] [ -sn  cs ]
        T a = mat(0, 0), b = mat(1, 0), c = mat(1, 1);
        T sm = a + c;
        T df = a - c;
        T rt = std::sqrt(df * df + b * b * 4);

        if (sm > 0) {
            w[1] = T(0.5) * (sm + rt);
            T t = T(1) / w[1];
            w[0] = (a * t) * c - (b * t) * b;
        } else if (sm < 0) {
            w[0] = T(0.5) * (sm - rt);
            T t  = T(1) / w[1];
            w[1] = (a * t) * c - (b * t) * b;
        } else {
            // This case needs to be treated separately to avoid div by 0.
            w[1] = T(0.5)  * rt;
            w[0] = T(-0.5) * rt;
        }

        // Compute eigenvectors.
        T cs = df > 0 ? df + rt : df - rt;
        T t = 0, sn = 0;
        if (std::fabs(cs) > T(2) * std::fabs(b)) {
            t = -T(2) * b / cs;
            sn = T(1) / std::sqrt(t * t + T(1));
            cs = t * sn;
        } else if (std::fabs(b) == 0) {
            cs = 1;
            sn = 0;
        } else {
            t = -T(0.5) * cs / b;
            cs = T(1) / std::sqrt(t * t + T(1));
            sn = t * cs;
        }

        if (df > 0) {
            t = cs;
            cs = -sn;
            sn = t;
        }

        eigenvectors_[0] = Vector2D<T>(-sn, cs);
        eigenvectors_[1] = Vector2D<T>(cs, sn);
    }

    // The centroid point of the input data.
    Point2D<T> centroid_;

    // Covariance matrix of the input data.
    Matrix2<T> covariance_matrix_;

    // Eigenvalues of the covariance matrix, sorted in ascending order.
    Array<T> eigenvalues_;

    // Corresponding vectors.
    Array<Vector2D<T>> eigenvectors_;
};

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_PRINCIPAL_COMPONENT_ANALYSIS_2D_H_
