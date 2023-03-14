//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_PCA_NORMALS_3D_H_
#define CODELIBRARY_POINT_CLOUD_PCA_NORMALS_3D_H_

#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/point_cloud/principal_component_analysis_3d.h"
#include "codelibrary/util/tree/kd_tree.h"

namespace cl {
namespace point_cloud {

/**
 * Estimate normal vector by weighted PCA method.
 *
 * The output normal is normalize to the unit length. The sign of orientation is
 * randomly assigned.
 */
template <typename Point, typename T>
void PCANormal(const Array<Point>& points, const Array<T>& weights,
               Vector3D<T>* normal) {
  static_assert(std::is_floating_point<T>::value, "");

  PrincipalComponentAnalysis3D<T> pca(points, weights);
  *normal = pca.eigenvectors()[0];

  CHECK(!points.empty());
  CHECK(points.size() == weights.size());
  CHECK(normal);

  Point centroid = Center(points, weights);
  T a = 0, b = 0, c = 0;
  T sum = 0;
  for (int i = 0; i < points.size(); ++i) {
      T x = points[i].x - centroid.x;
      T y = points[i].y - centroid.y;
      T w = weights[i];

      a += w * x * x;
      b += w * x * y;
      c += w * y * y;
      sum += w;
  }

  if (sum == 0) {
      *normal = Vector2D<T>(0, 1);
      return;
  }

  T t = T(1) / sum;
  a *= t;
  b *= t;
  c *= t;

  // Compute the least eigenvector of the covariance matrix.
  // [ A  B ]  =  [ cs  -sn ] [ rt1   0 ] [  cs  sn ]
  // [ B  C ]     [ sn   cs ] [  0   rT ] [ -sn  cs ]
  T df = a - c;
  T rt = std::sqrt(df * df + b * b * T(4));
  T cs = df > 0 ? df + rt : df - rt;
  T sn = 0;
  if (std::fabs(cs) > std::fabs(b) * T(2)) {
      t = -b * T(2) / cs;
      sn = T(1) / std::sqrt(t * t + T(1));
      cs = t * sn;
  } else if (std::fabs(b) == 0) {
      cs = 1;
      sn = 0;
  } else {
      t = - cs / b / 2;
      cs = T(1) / std::sqrt(t * t + T(1));
      sn = t * cs;
  }

  if (df > 0) {
      t = cs;
      cs = -sn;
      sn = t;
  }

  *normal = Vector2D<T>(-sn, cs);
}

/**
 * Estimate normal vector of points by PCA method.
 *
 * The output normal is normalize to the unit length. The sign of orientation is
 * randomly assigned.
 */
template <typename Point, typename T>
void PCANormal(const Array<Point>& points, Vector2D<T>* normal) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!points.empty());
    Array<T> weights(points.size(), 1);
    PCANormal(points, normal);
}

/**
 * Estimate normal vectors of each point in kd_tree by PCA method over K nearest
 * neighbors.
 *
 * The output normals are normalize to the unit length. The sign of orientation
 * is randomly assigned.
 *
 * Parameters:
 *   kd_tree  - the input points are stored in the KD tree.
 *   k        - used to define the k-nearest neighbors.
 *   normals  - the output normals.
 */
template <typename Point, typename T>
void PCANormals(const KDTree<Point>& kd_tree, int k,
                Array<Vector2D<T>>* normals) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!kd_tree.empty());
    CHECK(k > 0);
    CHECK(normals);

    int n = kd_tree.size();
    k = std::min(k, n);

    normals->resize(n);

    const Array<Point>& points = kd_tree.points();
    Array<Point> neighbors;
    for (int i = 0; i < n; ++i) {
        kd_tree.FindKNearestNeighbors(points[i], k, &neighbors);
        PCANormal(neighbors, &(*normals)[i]);
    }
}

/**
 * Estimate normal vector of each point in points.
 */
template <typename Point, typename T>
void PCANormals(const Array<Point>& points, int k,
                Array<Vector2D<T>>* normals) {
    static_assert(std::is_floating_point<T>::value, "");

    KDTree<Point> kd_tree(points);
    PCANormals(kd_tree, k, normals);
}

/**
 * Orientation aware PCA normal estimation.
 *
 * This function will re-orient the normal vector for each point according to
 * the neighbors whose normal vector has the same orientation.
 *
 * Parameters:
 *   kd_tree - the input points are stored in the KD tree.
 *   k       - used to define the k-nearest neighbors.
 *   normals - the output normals.
 */
template <typename Point, typename T>
void OrientationAwarePCANormals(const KDTree<Point>& kd_tree,
                                int k,
                                Array<Vector2D<T>>* normals) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!kd_tree.empty());
    CHECK(k > 0);
    CHECK(normals);
    CHECK(normals->size() == kd_tree.size());

    int n = kd_tree.size();
    k = std::min(k, n);

    const Array<Point>& points = kd_tree.points();
    Array<int> neighbors;

    for (int i = 0; i < n; ++i) {
        kd_tree.FindKNearestNeighbors(points[i], k, &neighbors);

        Array<Point> neighbor_points;
        neighbor_points.reserve(k);
        for (int j = 0; j < k; ++j) {
            if ((*normals)[i] * (*normals)[neighbors[j]] >= 0) {
                neighbor_points.push_back(points[neighbors[j]]);
            }
        }

        Vector2D<T> normal;
        PCANormal(neighbor_points, &normal);
        if (normal * (*normals)[i] < 0) {
            (*normals)[i] = -normal;
        } else {
            (*normals)[i] =  normal;
        }
    }
}

/**
 * Similar to the previous one.
 */
template <typename Point, typename T>
void OrientationAwarePCANormals(const Point& points, int k,
                                Array<Vector2D<T>>* normals) {
    KDTree<Point> kd_tree(points);
    OrientationAwarePCANormals(kd_tree, k, normals);
}


/**
 * Estimate normal direction by weighted PCA method.
 *
 * The output normal is normalize to the unit length. Its orientation is
 * randomly assigned.
 *
 * Note that, we only need to compute the least eigenvector of the covariance
 * matrix.
 */
template <typename Point, typename T>
void PCANormal(const Array<Point3D<T>>& points, const Array<T>& weights,
               Vector3D<T>* normal) {
    static_assert(std::is_floating_point<T>::value, "");

    PrincipalComponentAnalysis3D<T> pca(points, weights);
    *normal = pca.eigenvectors()[0];
}

/**
 * Estimate normal vector by PCA method.
 *
 * The output normal is normalize to the unit length, and its orientation is
 * randomly assigned.
 */
template <typename T>
void PCANormal(const Array<Point3D<T>>& points, Vector3D<T>* normal) {
    static_assert(std::is_floating_point<T>::value, "");

    Array<T> weights(points.size(), 1);
    PCANormal(points, weights, normal);
}

/**
 * Estimate normal directions of the 3D points by PCA method over the k nearest
 * neighbors.
 *
 * The output normals are normalize to the unit length, and their orientation
 * are randomly assigned.
 *
 *   kd_tree  - the input points are stored in the KD tree.
 *   k        - used to define the k-nearest neighbors.
 *   normals  - the output normals.
 */
template <typename T>
void PCANormals(const KDTree<Point3D<T>>& kd_tree, int k,
                Array<Vector3D<T>>* normals) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!kd_tree.empty());
    CHECK(k > 0);
    CHECK(normals);

    int n = kd_tree.size();
    k = std::min(k, n);

    normals->resize(n);

    const Array<Point3D<T>>& points = kd_tree.points();
    Array<Point3D<T>> neighbors;

    for (int i = 0; i < n; ++i) {
        kd_tree.FindKNearestNeighbors(points[i], k, &neighbors);
        PCANormal(neighbors, &(*normals)[i]);
    }
}

/**
 * Estimate a set of normal vectors by PCA.
 */
template <typename T>
void PCANormals(const Array<Point3D<T>>& points, int k,
                Array<Vector3D<T>>* normals) {
    KDTree<Point3D<T>> kd_tree(points);
    PCANormals(kd_tree, k, normals);
}

/**
 * Orientation-aware PCA normal estimation.
 *
 * This function will re-orient the normal vector for each point according to
 * the neighbors whose normal vector has the same orientation.
 *
 * Parameters:
 *   kd_tree - the input points are stored in the KD tree.
 *   k       - used to define the k-nearest neighbors.
 *   normals - the normals that will be recomputed.
 */
template <typename T>
void OrientationAwarePCANormals(const KDTree<Point3D<T>>& kd_tree, int k,
                                Array<Vector3D<T>>* normals) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!kd_tree.empty());
    CHECK(k > 0);
    CHECK(normals);
    CHECK(normals->size() == kd_tree.size());

    int n = kd_tree.size();
    k = std::min(k, n);

    const Array<Point3D<T>>& points = kd_tree.points();
    Array<int> neighbors;

    for (int i = 0; i < n; ++i) {
        kd_tree.FindKNearestNeighbors(points[i], k, &neighbors);

        Array<Point3D<T>> neighbor_points;
        neighbor_points.reserve(k);
        for (int j = 0; j < k; ++j) {
            if ((*normals)[i] * (*normals)[neighbors[j]] >= 0.0) {
                neighbor_points.push_back(points[neighbors[j]]);
            }
        }

        Vector3D<T> normal;
        PCANormal(neighbor_points, &normal);
        if (normal * (*normals)[i] < 0) {
            (*normals)[i] = -normal;
        } else {
            (*normals)[i] =  normal;
        }
    }
}

/**
 * Similar to the previous one.
 */
template <typename T>
void OrientationAwarePCANormals(const Array<Point3D<T>>& points, int k,
                                Array<Vector3D<T>>* normals) {
    KDTree<Point3D<T>> kd_tree(points);
    OrientationAwarePCANormals(kd_tree, k, normals);
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_PCA_NORMALS_3D_H_
