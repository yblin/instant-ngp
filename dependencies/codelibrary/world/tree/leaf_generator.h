//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_TREE_LEAF_GENERATOR_H_
#define CODELIBRARY_WORLD_TREE_LEAF_GENERATOR_H_

#include "codelibrary/world/kernel/render_data.h"
#include "codelibrary/world/tree/procedural_tree.h"

namespace cl {
namespace world {

class LeafGenerator {
public:
    void OctaveLeaf(RenderData* data) const {
        CHECK(data);

        data->type = GL_TRIANGLES;
        data->vertices = {
            {0.005f, 0.0f, 0.0f},
            {0.005f, 0.0f, 0.1f},
            {0.15f, 0.0f, 0.15f},
            {0.2f, 0.0f, 0.3f},
            {0.0f, 0.0f, 1.0f},
            {-0.2f, 0.0f, 0.6f},
            {0.0f, 0.0f, 1.0f},
            {0.005f, 0.0f, 0.0f},
            {-0.2f, 0.0f, 0.6f},
            {-0.25f, 0.0f, 0.15f},
            {-0.005f, 0.0f, 0.1f},
            {-0.005f, 0.0f, 0.0f}
        };
        data->normals.assign(data->vertices.size(),
                             FVector3D(0.0f, 1.0f, 0.0f));
        data->indices = { 0, 1, 9, 0, 9, 10, 1, 2, 3, 1, 3, 4, 4, 5, 6,
                          6, 7, 8, 6, 8, 9, 4, 6, 9, 4, 9, 1 };
    }

    void LinearLeaf(RenderData* data) const {
        CHECK(data);

        data->type = GL_TRIANGLES;
        data->vertices = {
            {0.005f, 0.0f, 0.0f},
            {0.005f, 0.0f, 0.1f},
            {0.1f, 0.0f, 0.15f},
            {0.1f, 0.0f, 0.95f},
            {0.0f, 0.0f, 1.0f},
            {-0.1f, 0.0f, 0.95f},
            {-0.1f, 0.0f, 0.15f},
            {-0.005f, 0.0f, 0.1f},
            {-0.005f, 0.0f, 0.0f}
        };
        data->normals.assign(data->vertices.size(),
                             FVector3D(0.0f, 1.0f, 0.0f));
        data->indices = { 0, 1, 7, 0, 7, 8, 1, 2, 3, 3, 4, 5, 5, 6, 7,
                          1, 3, 5, 1, 5, 7 };
    }

    void CordateLeaf(RenderData* data) const {
        CHECK(data);

        data->type = GL_TRIANGLES;
        data->vertices = {
            {0.005f, 0.0f, 0.0f},
            {0.01f, 0.0f, 0.2f},
            {0.2f, 0.0f, 0.1f},
            {0.35f, 0.0f, 0.35f},
            {0.25f, 0.0f, 0.6f},
            {0.1f, 0.0f, 0.8f},
            {0.0f, 0.0f, 1.0f},
            {-0.1f, 0.0f, 0.8f},
            {-0.25f, 0.0f, 0.6f},
            {-0.35f, 0.0f, 0.35f},
            {-0.2f, 0.0f, 0.1f},
            {-0.01f, 0.0f, 0.2f},
            {-0.005f, 0.0f, 0.0f}
        };
        data->normals.assign(data->vertices.size(),
                             FVector3D(0.0f, 1.0f, 0.0f));
        data->indices = { 0, 1, 11, 0, 11, 12, 1, 2, 3, 1, 3, 4,
                          11, 10, 9, 11, 9, 8, 11, 1, 4, 11, 4, 8,
                          8, 7, 6, 8, 6, 5, 8, 5, 4 };
    }

    void RoundOakLeaf(RenderData* data) const {
        CHECK(data);

        data->type = GL_TRIANGLES;
        data->vertices = {
            {0.005f, 0.0f, 0.0f},
            {0.005f, 0.0f, 0.1f},
            {0.11f, 0.0f, 0.16f},
            {0.11f, 0.0f, 0.2f},
            {0.22f, 0.0f, 0.26f},
            {0.23f, 0.0f, 0.32f},
            {0.15f, 0.0f, 0.34f},
            {0.25f, 0.0f, 0.45f},
            {0.23f, 0.0f, 0.53f},
            {0.16f, 0.0f, 0.5f},
            {0.23f, 0.0f, 0.64f},
            {0.2f, 0.0f, 0.72f},
            {0.11f, 0.0f, 0.7f},
            {0.16f, 0.0f, 0.83f},
            {0.12f, 0.0f, 0.87f},
            {0.06f, 0.0f, 0.85f},
            {0.07f, 0.0f, 0.95f},
            {0.0f, 0.0f, 1.0f},
            {-0.07f, 0.0f, 0.85f},
            {-0.12f, 0.0f, 0.87f},
            {-0.16f, 0.0f, 0.83f},
            {-0.11f, 0.0f, 0.7f},
            {-0.2f, 0.0f, 0.72f},
            {-0.23f, 0.0f, 0.64f},
            {-0.16f, 0.0f, 0.5f},
            {-0.23f, 0.0f, 0.53f},
            {-0.25f, 0.0f, 0.45f},
            {-0.15f, 0.0f, 0.34f},
            {-0.23f, 0.0f, 0.32f},
            {-0.22f, 0.0f, 0.26f},
            {-0.11f, 0.0f, 0.2f},
            {-0.11f, 0.0f, 0.16f},
            {-0.005f, 0.0f, 0.1f},
            {-0.005f, 0.0f, 0.0f}
        };
        data->normals.assign(data->vertices.size(),
                               FVector3D(0.0f, 1.0f, 0.0f));
        data->indices = {
            0, 1, 33, 0, 33, 34,
            1, 2, 3,
            3, 4, 5, 3, 5, 6,
            6, 7, 8, 6, 8, 9,
            9, 10, 11, 9, 11, 12,
            12, 13, 14, 12, 14, 15,
            1, 3, 6, 1, 6, 9, 1, 9, 12, 1, 12, 15, 1, 15, 17, 1, 17, 19,
            1, 19, 22, 1, 22, 25, 1, 25, 28, 1, 28, 31, 1, 31, 33,
            33, 32, 31,
            31, 30, 29, 31, 29, 28,
            28, 27, 26, 28, 26, 25,
            25, 24, 23, 25, 23, 22,
            22, 21, 20, 22, 20, 19,
            19, 18, 17
        };
    }

    void PalmateLeaf(RenderData* data) const {
        CHECK(data);

        data->type = GL_TRIANGLES;
        data->vertices = {
            {0.005f, 0.0f, 0.0f},
            {0.005f, 0.0f, 0.1f},
            {0.25f, 0.0f, 0.1f},
            {0.5f, 0.0f, 0.3f},
            {0.2f, 0.0f, 0.45f},
            {0.0f, 0.0f, 0.1f},
            {-0.2f, 0.0f, 0.45f},
            {-0.5f, 0.0f, 0.3f},
            {-0.25f, 0.0f, 0.1f},
            {-0.005f, 0.0f, 0.1f},
            {-0.005f, 0.0f, 0.0f}
        };
        data->normals.assign(data->vertices.size(),
                             FVector3D(0.0f, 1.0f, 0.0f));
        data->indices = {
            0, 1, 9, 0, 9, 10,
            1, 2, 3, 1, 3, 4,
            1, 4, 5, 1, 5, 6, 1, 6, 9,
            9, 8, 7, 9, 7, 6
        };
    }

    void SpikyOakLeaf(RenderData* data) const {
        CHECK(data);

        data->type = GL_TRIANGLES;
        data->vertices = {
            {0.005f, 0.0f, 0.0f},
            {0.005f, 0.0f, 0.1f},
            {0.16f, 0.0f, 0.17f},
            {0.11f, 0.0f, 0.2f},
            {0.23f, 0.0f, 0.33f},
            {0.15f, 0.0f, 0.34f},
            {0.32f, 0.0f, 0.55f},
            {0.16f, 0.0f, 0.5f},
            {0.27f, 0.0f, 0.75f},
            {0.11f, 0.0f, 0.7f},
            {0.18f, 0.0f, 0.9f},
            {0.07f, 0.0f, 0.86f},
            {0.0f, 0.0f, 1.0f},
            {-0.07f, 0.0f, 0.86f},
            {-0.18f, 0.0f, 0.9f},
            {-0.11f, 0.0f, 0.7f},
            {-0.27f, 0.0f, 0.75f},
            {-0.16f, 0.0f, 0.5f},
            {-0.32f, 0.0f, 0.55f},
            {-0.15f, 0.0f, 0.34f},
            {-0.23f, 0.0f, 0.33f},
            {-0.11f, 0.0f, 0.2f},
            {-0.16f, 0.0f, 0.17f},
            {-0.005f, 0.0f, 0.1f},
            {-0.005f, 0.0f, 0.0f}
        };
        data->normals.assign(data->vertices.size(),
                             FVector3D(0.0f, 1.0f, 0.0f));
        data->indices = {
            0, 1, 23, 0, 23, 24,
            1, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10, 11,
            1, 3, 5, 1, 5, 7, 1, 7, 9, 1, 9, 11, 1, 11, 12, 1, 12, 13,
            1, 13, 15, 1, 15, 17, 1, 17, 19, 1, 19, 21, 1, 21, 23,
            23, 22, 21, 21, 20, 19, 19, 18, 17, 17, 16, 15, 15, 14, 13
        };
    }

    void EllipticLeaf(RenderData* data) const {
        CHECK(data);

        data->type = GL_TRIANGLES;
        data->vertices = {
            {0.005f, 0.0f, 0.0f},
            {0.005f, 0.0f, 0.1f},
            {0.15f, 0.0f, 0.2f},
            {0.25f, 0.0f, 0.45f},
            {0.2f, 0.0f, 0.75f},
            {0.0f, 0.0f, 1.0f},
            {-0.2f, 0.0f, 0.75f},
            {-0.25f, 0.0f, 0.45f},
            {-0.15f, 0.0f, 0.2f},
            {-0.005f, 0.0f, 0.1f},
            {-0.005f, 0.0f, 0.0f}
        };
        data->normals.assign(data->vertices.size(),
                             FVector3D(0.0f, 1.0f, 0.0f));
        data->indices = { 0, 1, 9, 0, 9, 10, 1, 2, 3, 1, 3, 4,
                          4, 5, 6, 6, 7, 8, 6, 8, 9,
                          4, 6, 9, 4, 9, 1
        };
    }

    void TriangleLeaf(RenderData* data) const {
        CHECK(data);

        data->type = GL_TRIANGLES;
        data->vertices = {
            {-0.5f, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
            {0.5f, 0.0f, 0.0f}
        };
        data->normals.assign(data->vertices.size(),
                             FVector3D(0.0f, 1.0f, 0.0f));
        data->indices = { 0, 1, 2 };
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_TREE_LEAF_GENERATOR_H_
