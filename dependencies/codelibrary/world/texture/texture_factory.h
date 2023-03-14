//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_TEXTURE_TEXTURE_FACTORY_H_
#define CODELIBRARY_WORLD_TEXTURE_TEXTURE_FACTORY_H_

#include <string>
#include <unordered_map>

#include "codelibrary/base/pool.h"
#include "codelibrary/opengl/texture.h"

namespace cl {
namespace world {

/**
 * Factory to create and manage the life of textures.
 *
 * Note that all created textures are stored in video memory.
 */
class TextureFactory {
    TextureFactory() = default;

    using Texture = gl::Texture;
    using Color = RGB32Color;

public:
    static TextureFactory* GetInstance() {
        static TextureFactory factory;
        return &factory;
    }

    /**
     * Create a texture from local file.
     *
     * Return nullptr if load failed.
     */
    Texture* Create(const std::string& filename, bool mipmap = false,
                    bool flip = false) {
        Texture* texture = pool_.Allocate();
        if (texture->Load(filename, mipmap, flip)) {
            return texture;
        }

        pool_.Deallocate(texture);
        return nullptr;
    }

    /**
     * Create a manual grid line texture.
     *
     * Paramters:
     *  n_rows          - number of grids of each row.
     *  n_columns       - number of grids of each column.
     *  grid_resolution - resolution of each gird, in pixels.
     *  grid_width      - width of each grid, in pixels.
     *  grid_color      - color of grid.
     *  grid_line_color - color of grid line.
     */
    Texture* CreateGridLine(int n_rows, int n_columns,
                            int grid_resolution, int grid_width,
                            const Color& grid_color = {110, 110, 110},
                            const Color& grid_line_color = {220, 220, 220}) {
        CHECK(n_rows > 0 && n_columns > 0);
        CHECK(n_rows <= 4096);
        CHECK(n_columns <= 4096);
        CHECK(grid_resolution > 0);
        CHECK(grid_resolution <= 4096);
        CHECK(grid_width >= 2);
        CHECK(grid_width < grid_resolution);
        CHECK(n_rows * grid_resolution <= 4096);
        CHECK(n_columns * grid_resolution <= 4096);

        int size_w = n_rows * grid_resolution;
        int size_h = n_columns * grid_resolution;
        int l = grid_width / 2;
        Image image(size_h, size_w, 3);
        for (int i = 0; i < size_h; ++i) {
            for (int j = 0; j < size_w; ++j) {
                if (i % grid_resolution <= l ||
                    i % grid_resolution >= grid_resolution - l ||
                    j % grid_resolution <= l ||
                    j % grid_resolution >= grid_resolution - l) {
                    image(i, j, 0) = grid_line_color.red();
                    image(i, j, 1) = grid_line_color.green();
                    image(i, j, 2) = grid_line_color.blue();
                } else {
                    image(i, j, 0) = grid_color.red();
                    image(i, j, 1) = grid_color.green();
                    image(i, j, 2) = grid_color.blue();
                }
            }
        }

        Texture* texture = pool_.Allocate();
        texture->Load(image);
        return texture;
    }

    /**
     * Create a chessboard texture.
     *
     * Paramters:
     *  n_rows          - number of grids of each row.
     *  n_columns       - number of grids of each column.
     *  grid_resolution - resolution of each gird, in pixels.
     *  grid_color1     - color of odd grids.
     *  grid_color2     - color of even grids.
     */
    Texture* CreateChessboard(int n_rows, int n_columns,
                              int grid_resolution,
                              const Color& grid_color1 = {110, 110, 110},
                              const Color& grid_color2 = {220, 220, 220}) {
        CHECK(n_rows > 0 && n_columns > 0);
        CHECK(n_rows <= 4096);
        CHECK(n_columns <= 4096);
        CHECK(grid_resolution > 0);
        CHECK(grid_resolution <= 4096);
        CHECK(n_rows * grid_resolution <= 4096);
        CHECK(n_columns * grid_resolution <= 4096);

        int size_w = n_rows * grid_resolution;
        int size_h = n_columns * grid_resolution;
        Image image(size_h, size_w, 3);
        for (int i = 0; i < size_h; ++i) {
            for (int j = 0; j < size_w; ++j) {
                if ((i / grid_resolution + j / grid_resolution) % 2 == 0) {
                    image(i, j, 0) = grid_color1.red();
                    image(i, j, 1) = grid_color1.green();
                    image(i, j, 2) = grid_color1.blue();
                } else {
                    image(i, j, 0) = grid_color2.red();
                    image(i, j, 1) = grid_color2.green();
                    image(i, j, 2) = grid_color2.blue();
                }
            }
        }

        Texture* texture = pool_.Allocate();
        texture->Load(image);
        return texture;
    }

    /**
     * Create a roughness chessboard texture.
     *
     * Paramters:
     *  n_rows          - number of grids of each row.
     *  n_columns       - number of grids of each column.
     *  grid_resolution - resolution of each gird, in pixels.
     *  roughness1      - roughness of odd grids.
     *  roughness2      - roughness of even grids.
     */
    Texture* CreateChessboardRoughness(int n_rows, int n_columns,
                                       int grid_resolution,
                                       float roughness1,
                                       float roughness2) {
        CHECK(n_rows > 0 && n_columns > 0);
        CHECK(n_rows <= 4096);
        CHECK(n_columns <= 4096);
        CHECK(grid_resolution > 0);
        CHECK(grid_resolution <= 4096);
        CHECK(n_rows * grid_resolution <= 4096);
        CHECK(n_columns * grid_resolution <= 4096);
        CHECK(roughness1 >= 0.0f && roughness1 <= 1.0f);
        CHECK(roughness2 >= 0.0f && roughness2 <= 1.0f);

        int size_w = n_rows * grid_resolution;
        int size_h = n_columns * grid_resolution;
        ImageF image(size_h, size_w);
        for (int i = 0; i < size_h; ++i) {
            for (int j = 0; j < size_w; ++j) {
                if ((i / grid_resolution + j / grid_resolution) % 2 == 0) {
                    image(i, j) = roughness1;
                } else {
                    image(i, j) = roughness2;
                }
            }
        }

        Texture* texture = pool_.Allocate();
        texture->Load(image);
        return texture;
    }

protected:
    // Texture pool.
    Pool<Texture> pool_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_TEXTURE_TEXTURE_FACTORY_H_
