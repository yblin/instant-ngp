//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UI_RENDER_UTIL_H_
#define CODELIBRARY_UI_RENDER_UTIL_H_

#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/mesh/polygon_triangulation_2d.h"
#include "codelibrary/geometry/triangle_2d.h"
#include "codelibrary/ui/ui.h"

namespace cl {
namespace ui {

/**
 * Map 2D point to ImVec2.
 */
template <typename T>
static ImVec2 ToImVec2(const Point2D<T>& p) {
    return ImVec2(static_cast<float>(p.x), static_cast<float>(p.y));
}

/**
 * Render a circle to the current ImGui context.
 */
template <typename T>
void RenderCircle(const Circle2D<T>& circle, const RGB32Color& color,
                  int n_segments = 0, float thickness = 1.0f) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddCircle(ToImVec2(circle.center()),
                         static_cast<float>(circle.radius()),
                         color.ToUInt(),
                         n_segments,
                         thickness);
}

/**
 * Render a filled circle.
 */
template <typename T>
void RenderCircleFilled(const Circle2D<T>& circle, const RGB32Color& color,
                        int n_segments = 0) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddCircleFilled(ToImVec2(circle.center()),
                               static_cast<float>(circle.radius()),
                               color.ToUInt(),
                               n_segments);
}

/**
 * Render a line.
 */
template <typename T>
void RenderLine(const Point2D<T>& p1, const Point2D<T>& p2,
                const RGB32Color& color, float thickness = 1.0f) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddLine(ToImVec2(p1), ToImVec2(p2), color.ToUInt(),
                       thickness);
}

/**
 * Render a line.
 */
template <typename T>
void RenderLine(const Segment2D<T>& seg, const RGB32Color& color,
             float thickness = 1.0f) {
    RenderLine(seg.lower_point(), seg.upper_point(), color, thickness);
}

/**
 * Render a polyline.
 */
template <typename T>
void RenderPolyLine(const Array<Point2D<T>>& points, const RGB32Color& color,
                    float thickness = 1.0f) {
    for (int i = 0; i < points.size(); ++i) {
        int next = i + 1 == points.size() ? 0 : i + 1;
        RenderLine(points[i], points[next], color, thickness);
    }
}

/**
 * Render a box.
 */
template <typename T>
void RenderBox(const Box2D<T>& box, const RGB32Color& color,
               float thickness = 1.0f) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRect(ImVec2(static_cast<float>(box.x_min()),
                              static_cast<float>(box.y_min())),
                       ImVec2(static_cast<float>(box.x_max()),
                              static_cast<float>(box.y_max())),
                       color.ToUInt(),
                       0.0f,
                       0,
                       thickness);
}

/**
 * Render a triangle (best organized in clockwise).
 */
template <typename T>
void RenderTriangle(const Point2D<T>& p1,
                    const Point2D<T>& p2,
                    const Point2D<T>& p3,
                    const RGB32Color& color) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddTriangleFilled(ToImVec2(p1), ToImVec2(p2), ToImVec2(p3),
                                 color.ToUInt());
}
template <typename T>
void RenderTriangle(const Triangle2D<T>& triangle,
                 const RGB32Color& color) {
    RenderTriangle(triangle.vertices()[0], triangle.vertices()[1],
                   triangle.vertices()[2], color);
}

/**
 * Render a quad (best organized in clockwise).
 */
template <typename T>
void RenderQuad(const Array<Point2D<T>>& vertices, const RGB32Color& color,
                float thickness = 1.0f) {
    CHECK(vertices.size() == 4);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddQuad(ToImVec2(vertices[0]), ToImVec2(vertices[1]),
                       ToImVec2(vertices[2]), ToImVec2(vertices[3]),
                       color.ToUInt(),
                       thickness);
}

/**
 * Render a multi-polygon (in the form of triangulation).
 */
template <typename T>
void RenderPolygon(const geometry::PolygonTriangulation2D<T>& triangulation,
                   const RGB32Color& color) {
    auto is_visted = triangulation.mesh().AddHalfedgeProperty(false);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->Flags &= ~ImDrawListFlags_AntiAliasedFill;

    for (auto* e : triangulation.mesh()) {
        if (!is_visted[e] && !triangulation.is_outer(e)) {
            is_visted[e] = true;
            is_visted[e->next()] = true;
            is_visted[e->next()->next()] = true;
            RenderTriangle(e->source_point(), e->prev()->source_point(),
                           e->next()->source_point(), color);
        }
    }
    draw_list->Flags |= ImDrawListFlags_AntiAliasedFill;
}

} // namespace ui
} // namespace cl

#endif // CODELIBRARY_UI_RENDER_UTIL_H_
