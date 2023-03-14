//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_PLANE_ARRANGEMENT_3D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_PLANE_ARRANGEMENT_3D_H_

#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/transform_3d.h"

namespace cl {
namespace geometry {

/**
 * PlaneArrangement3D similars to Arragment2D. The only difference is that
 * PlaneArrangement3D parition the given 3D plane, while Arrangment2D only lies
 * on XY plane.
 *
 * More details see Arrangement2D.
 */
template <typename T>
class PlaneArrangement3D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    using Point3D   = Point3D<T>;
    using Point2D   = Point2D<T>;
    using Segment3D = Segment3D<T>;
    using Segment2D = Segment2D<T>;
    using Vertex    = typename Arrangement2D<T>::Vertex;

    /**
     * An empty arrangement.
     *
     * Users can use Reset() to initialize the arrangement latter.
     */
    explicit PlaneArrangement3D(const Plane3D<T>& plane, double threshold = 0.0)
        : plane_(plane), arrangement2d_(threshold) {
        rotation_ = Quaternion<T>(plane.normal(), Vector3D<T>(0, 0, 1));
        Point3D p = Rotate(plane.point(), rotation_);
        z_offset_ = p.z;
    }

    PlaneArrangement3D(const PlaneArrangement3D&) = delete;

    PlaneArrangement3D& operator= (const PlaneArrangement3D&) = delete;

    /**
     * Insert a set of 3D line segments.
     */
    void Insert(const Array<Segment3D>& segs) {
        for (const Segment3D& s : segs) {
            Insert(s);
        }
    }

    /**
     * Insert a 3D line segment.
     *
     * The time complexity is O(N * logN).
     */
    void Insert(const Segment3D& seg) {
        arrangement2d_.Insert(Project(seg.lower_point()),
                              Project(seg.upper_point()));
    }
    void Insert(const Point3D& p1, const Point3D& p2, int color1 = 0,
                int color2 = 0) {
        arrangement2d_.Insert(Project(p1), Project(p2), color1, color2);
    }
    void InsertWithoutSplit(const Point3D& p1, const Point3D& p2,
                            int color1 = 0, int color2 = 0) {
        arrangement2d_.InsertWithoutSplit(Project(p1), Project(p2), color1,
                                          color2);
    }

    /**
     * Insert an incident 3D point and return the inserted vertex.
     */
    Vertex* Insert(const Point3D& p) {
        return arrangement2d_.Insert(Project(p));
    }

    /**
     * Get partition regions enclosed by line segments.
     *
     * Each region consist of serval connected inner triangles. Each triangles
     * is defined by one of its halfedge.
     */
    void Arrange() {
        arrangement2d_.Arrange();
    }

    /**
     * Get the region that contains the given 3D point.
     *
     * Make sure 'Arrange()' is called before.
     *
     * It returns -1 if no region is found.
     */
    int Locate(const Point3D& p) const {
        return arrangement2d_.Locate(Project(plane_, p));
    }

    /**
     * Project a point in world space to local plane space.
     */
    Point2D Project(const Point3D& p) const {
        Point3D q = Rotate(p, rotation_);
        return Point2D(q.x, q.y);
    }

    /**
     * Unproject a point in local plane space to world space.
     */
    Point3D Unproject(const Point2D& p) const {
        Point3D q(p.x, p.y, z_offset_);
        return Rotate(q, rotation_.inverse());
    }

    void clear() {
        arrangement2d_.clear();
    }

    bool empty() const {
        return arrangement2d_.empty();
    }

    const Arrangement2D<T>& arrangement2d() const {
        return arrangement2d_;
    }

    const Plane3D<T>& plane() const {
        return plane_;
    }

protected:
    // The plane for arrangement.
    Plane3D<T> plane_;

    // Rotation from plane's normal to XY plane.
    Quaternion<T> rotation_;

    // z_offset is used to unproject local plane coordiante to world coordinate.
    T z_offset_;

    // 2D arrangement.
    Arrangement2D<T> arrangement2d_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_PLANE_ARRANGEMENT_3D_H_
