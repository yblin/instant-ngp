//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_TREE_TREE_FACTORY_H_
#define CODELIBRARY_WORLD_TREE_TREE_FACTORY_H_

#include <map>

#include "codelibrary/world/tree/procedural_tree.h"

namespace cl {
namespace world {

/**
 * Factory for procedural trees.
 */
class TreeFactory {
    TreeFactory() {
        function_map_["Acer"] = &TreeFactory::Acer;
        function_map_["Apple"] = &TreeFactory::Apple;
        function_map_["Balsam fir"] = &TreeFactory::BalsamFir;
        function_map_["Bamboo"] = &TreeFactory::Bamboo;
        function_map_["Black oak"] = &TreeFactory::BlackOak;
        function_map_["Black tupelo"] = &TreeFactory::BlackTupelo;
        function_map_["Cambridge oak"] = &TreeFactory::CambridgeOak;
        function_map_["Douglas fir"] = &TreeFactory::DouglasFir;
        function_map_["European larch"] = &TreeFactory::EuropeanLarch;
        function_map_["Fan palm"] = &TreeFactory::FanPalm;
        function_map_["Hill cherry"] = &TreeFactory::HillCherry;
        function_map_["Lombardy poplar"] = &TreeFactory::LombardyPoplar;
        function_map_["Palm"] = &TreeFactory::Palm;
        function_map_["Quaking aspen"] = &TreeFactory::QuakingAspen;
        function_map_["Sassafras"] = &TreeFactory::Sassafras;
        function_map_["Silver birch"] = &TreeFactory::SilverBirch;
        function_map_["Small pine"] = &TreeFactory::SmallPine;
        function_map_["Weeping willow"] = &TreeFactory::WeepingWillow;
    }

    using Parameter = ProceduralTree::Parameter;
    typedef Parameter (TreeFactory::*Func)();

public:
    static TreeFactory* GetInstance() {
        static TreeFactory factory;
        return &factory;
    }

    Parameter GetParameter(const std::string& name) {
        auto i = function_map_.find(name);
        CHECK(i != function_map_.end() && "Wrong name and no found.");

        Func func = i->second;
        return (this->*func)();
    }

    Parameter Acer() {
        Parameter param;
        param.shape            = 4;
        param.scale            = 10.0f;
        param.scale_v          = 1.0f;
        param.levels           = 3;
        param.ratio            = 0.025f;
        param.ratio_power      = 1.5f;
        param.flare            = 0.6f;
        param.base_splits      = -2;
        param.base_size        = { 0.1f, 0.4f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 50.0f, 50.0f, 45.0f };
        param.down_angle_v     = { 0.0f, 5.0f, 5.0f, 10.0f };
        param.rotation         = { 0.0f, 140.0f, 140.0f, 77.0f };
        param.rotation_v       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.branches         = { 1, 6, 20, 5 };
        param.length           = { 1.0f, 0.7f, 0.3f, 0.0f };
        param.length_v         = { 0.0f, 0.05f, 0.05f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 1.5f, 1.5f, 0.0f, 0.0f };
        param.split_angle      = { 50.0f, 50.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 5.0f, 5.0f, 0.0f, 0.0f };
        param.curve_resolution = { 6, 5, 3, 0 };
        param.curve            = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.curve_v          = { 200.0f, 100.0f, 100.0f, 0.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 50.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 30;
        param.leaf_shape       = 5;
        param.leaf_scale       = 0.2f;
        param.leaf_bend        = 0.3f;
        param.tropism          = { 0.0f, 0.0f, 2.0f };

        return param;
    }

    Parameter Apple() {
        Parameter param;
        param.shape            = 2;
        param.scale            = 9.0f;
        param.scale_v          = 2.0f;
        param.levels           = 3;
        param.ratio            = 0.02f;
        param.ratio_power      = 1.5f;
        param.flare            = 0.9f;
        param.base_splits      = 0;
        param.base_size        = { 0.15f, 0.02f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 60.0f, 60.0f, 45.0f };
        param.down_angle_v     = { 0.0f, -30.0f, 20.0f, 30.0f };
        param.rotation         = { 0.0f, 140.0f, 140.0f, 77.0f };
        param.rotation_v       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.branches         = { 1, 35, 25, 10 };
        param.length           = { 1.0f, 0.5f, 0.4f, 0.0f };
        param.length_v         = { 0.0f, 0.0f, 0.1f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.0f, 0.6f, 0.0f, 0.0f };
        param.split_angle      = { 0.0f, 20.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 10.0f, 0.0f, 0.0f };
        param.curve_resolution = { 5, 10, 5, 0 };
        param.curve            = { 0.0f, -20.0f, 0.0f, 0.0f };
        param.curve_v          = { 70.0f, 140.0f, 100.0f, 0.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 50.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 50;
        param.leaf_shape       = 0;
        param.leaf_scale       = 0.17f;
        param.leaf_bend        = 0.5f;
        param.tropism          = { 0.0f, 0.0f, 2.0f };

        return param;
    }

    Parameter BalsamFir() {
        Parameter param;
        param.shape            = 0;
        param.scale            = 12.0f;
        param.scale_v          = 2.0f;
        param.levels           = 3;
        param.ratio            = 0.015f;
        param.ratio_power      = 1.7f;
        param.flare            = 0.2f;
        param.base_splits      = 0;
        param.base_size        = { 0.05f, 0.02f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 50.0f, 60.0f, 45.0f };
        param.down_angle_v     = { 0.0f, -45.0f, 20.0f, 30.0f };
        param.rotation         = { 0.0f, 140.0f, -125.0f, -90.0f };
        param.rotation_v       = { 0.0f, 0.0f, 20.0f, 20.0f };
        param.branches         = { 1, 100, 75, 10 };
        param.length           = { 1.0f, 0.5f, 0.25f, 0.0f };
        param.length_v         = { 0.2f, 0.0f, 0.1f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.curve_resolution = { 5, 5, 2, 0 };
        param.curve            = { 0.0f, -40.0f, 0.0f, 0.0f };
        param.curve_v          = { 20.0f, 10.0f, 40.0f, 0.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 10.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 100;
        param.leaf_shape       = 2;
        param.leaf_scale       = 0.13f;
        param.leaf_scale_x     = 0.5f;

        return param;
    }

    Parameter Bamboo() {
        Parameter param;
        param.shape            = 7;
        param.scale            = 10.0f;
        param.scale_v          = 2.0f;
        param.levels           = 2;
        param.ratio            = 0.005f;
        param.ratio_power      = 1.0f;
        param.flare            = 0.0f;
        param.base_splits      = 0;
        param.base_size        = { 0.4f, 0.4f, 0.0f, 0.0f };
        param.down_angle       = { 0.0f, 30.0f, 30.0f, 30.0f };
        param.down_angle_v     = { 0.0f, 10.0f, 10.0f, 10.0f };
        param.rotation         = { 0.0f, 77.0f, 77.0f, 77.0f };
        param.rotation_v       = { 0.0f, 30.0f, 0.0f, 0.0f };
        param.branches         = { 50, 25, 0, 10 };
        param.length           = { 1.0f, 0.2f, 0.0f, 0.0f };
        param.length_v         = { 0.0f, 0.05f, 0.0f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.curve_resolution = { 10, 5, 0, 0 };
        param.curve            = { 50.0f, 30.0f, 0.0f, 0.0f };
        param.curve_v          = { 70.0f, 0.0f, 0.0f, 0.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 20;
        param.leaf_shape       = 0;
        param.leaf_scale       = 0.3f;
        param.leaf_scale_x     = 0.3f;
        param.leaf_bend        = 0.1f;

        return param;
    }

    Parameter BlackOak() {
        Parameter param;
        param.shape            = 2;
        param.scale            = 10.0f;
        param.scale_v          = 2.0f;
        param.levels           = 3;
        param.ratio            = 0.018f;
        param.ratio_power      = 1.25f;
        param.flare            = 1.2f;
        param.base_splits      = 0;
        param.base_size        = { 0.05f, 0.02f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 30.0f, 45.0f, 45.0f };
        param.down_angle_v     = { 0.0f, -30.0f, 10.0f, 10.0f };
        param.rotation         = { 0.0f, 80.0f, 140.0f, 140.0f };
        param.rotation_v       = { 0.0f, 20.0f, 20.0f, 20.0f };
        param.branches         = { 1, 30, 120, 0 };
        param.length           = { 1.0f, 0.8f, 0.3f, 0.4f };
        param.length_v         = { 0.0f, 0.1f, 0.05f, 0.0f };
        param.taper            = { 0.95f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.1f, 0.1f, 0.1f, 0.0f };
        param.split_angle      = { 10.0f, 10.0f, 10.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 10.0f, 10.0f, 0.0f };
        param.curve_resolution = { 8, 10, 3, 0 };
        param.curve            = { 0.0f, 40.0f, 0.0f, 0.0f };
        param.curve_v          = { 90.0f, 150.0f, -30.0f, 0.0f };
        param.curve_back       = { 0.0f, -70.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 100.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 45;
        param.leaf_shape       = 6;
        param.leaf_scale       = 0.2f;
        param.leaf_scale_x     = 0.66f;
        param.leaf_bend        = 0.3f;
        param.tropism          = { 0.0f, 0.0f, 0.8f };

        return param;
    }

    Parameter BlackTupelo() {
        Parameter param;
        param.shape            = 4;
        param.scale            = 23.0f;
        param.scale_v          = 5.0f;
        param.levels           = 4;
        param.ratio            = 0.015f;
        param.ratio_power      = 1.3f;
        param.flare            = 1.0f;
        param.base_splits      = 0;
        param.base_size        = { 0.2f, 0.02f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 60.0f, 40.0f, 45.0f };
        param.down_angle_v     = { 0.0f, -40.0f, 10.0f, 10.0f };
        param.rotation         = { 0.0f, 140.0f, 140.0f, 140.0f };
        param.rotation_v       = { 0.0f, 60.0f, 50.0f, 0.0f };
        param.branches         = { 1, 75, 25, 15 };
        param.length           = { 1.0f, 0.3f, 0.6f, 0.2f };
        param.length_v         = { 0.0f, 0.05f, 0.1f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.curve_resolution = { 10, 10, 10, 1 };
        param.curve            = { 0.0f, 0.0f, -10.0f, 0.0f };
        param.curve_v          = { 40.0f, 90.0f, 150.0f, 0.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 100.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 20;
        param.leaf_shape       = 0;
        param.leaf_scale       = 0.2f;
        param.leaf_bend        = 0.3f;
        param.tropism          = { 0.0f, 0.0f, 0.5f };

        return param;
    }

    Parameter CambridgeOak() {
        Parameter param;
        param.shape            = 3;
        param.scale            = 20.0f;
        param.scale_v          = 4.0f;
        param.levels           = 4;
        param.ratio            = 0.03f;
        param.ratio_power      = 2.0f;
        param.flare            = 0.5f;
        param.base_splits      = 1;
        param.base_size        = { 0.2f, 0.0f, 0.0f, 0.0f };
        param.down_angle       = { 0.0f, 60.0f, 60.0f, 45.0f };
        param.down_angle_v     = { 0.0f, -30.0f, 30.0f, 10.0f };
        param.rotation         = { 0.0f, 110.0f, 110.0f, 110.0f };
        param.rotation_v       = { 0.0f, 50.0f, 50.0f, 0.0f };
        param.branches         = { 1, 15, 10, 50 };
        param.length           = { 1.0f, 0.4f, 0.6f, 0.2f };
        param.length_v         = { 0.0f, 0.1f, 0.1f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.5f, 0.5f, 0.2f, 0.0f };
        param.split_angle      = { 50.0f, 50.0f, 50.0f, 0.0f };
        param.split_angle_v    = { 20.0f, 10.0f, 10.0f, 0.0f };
        param.curve_resolution = { 10, 10, 10, 3 };
        param.curve            = { 0.0f, 20.0f, 0.0f, 0.0f };
        param.curve_v          = { 100.0f, 400.0f, 500.0f, 100.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 100.0f, 100.0f, 30.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 15;
        param.leaf_shape       = 7;
        param.leaf_scale       = 0.13f;
        param.tropism          = { 0.0f, 0.0f, 0.5f };

        return param;
    }

    Parameter DouglasFir() {
        Parameter param;
        param.shape            = 0;
        param.scale            = 40.0f;
        param.scale_v          = 10.0f;
        param.levels           = 3;
        param.ratio            = 0.015f;
        param.ratio_power      = 1.2f;
        param.flare            = 1.0f;
        param.base_splits      = 0;
        param.base_size        = { 0.2f, 0.1f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 100.0f, 40.0f, 45.0f };
        param.down_angle_v     = { 0.0f, -40.0f, 10.0f, 10.0f };
        param.rotation         = { 0.0f, 140.0f, 140.0f, 140.0f };
        param.rotation_v       = { 0.0f, 60.0f, 50.0f, 0.0f };
        param.branches         = { 1, 250, 30, 0 };
        param.length           = { 1.0f, 0.13f, 0.4f, 0.0f };
        param.length_v         = { 0.0f, 0.0f, 0.1f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.0f, 0.5f, 0.0f, 0.0f };
        param.split_angle      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.curve_resolution = { 2, 5, 3, 0 };
        param.curve            = { 0.0f, -35.0f, 0.0f, 0.0f };
        param.curve_v          = { 0.0f, 90.0f, 150.0f, 0.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 50.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 100;
        param.leaf_shape       = 2;
        param.leaf_scale       = 0.15f;
        param.leaf_scale_x     = 0.3f;

        return param;
    }

    Parameter EuropeanLarch() {
        Parameter param;
        param.shape            = 0;
        param.scale            = 15.0f;
        param.scale_v          = 7.0f;
        param.levels           = 3;
        param.ratio            = 0.015f;
        param.ratio_power      = 1.3f;
        param.flare            = 0.3f;
        param.base_splits      = 0;
        param.base_size        = { 0.25f, 0.02f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 60.0f, 70.0f, 30.0f };
        param.down_angle_v     = { 0.0f, -50.0f, 30.0f, 15.0f };
        param.rotation         = { 0.0f, 70.0f, 70.0f, 120.0f };
        param.rotation_v       = { 0.0f, 30.0f, 30.0f, 30.0f };
        param.branches         = { 1, 60, 50, 0 };
        param.length           = { 1.0f, 0.25f, 0.3f, 0.0f };
        param.length_v         = { 0.0f, 0.0f, 0.1f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.0f, 0.0f, 0.15f, 0.0f };
        param.split_angle      = { 0.0f, 0.0f, 40.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 0.0f, 10.0f, 0.0f };
        param.curve_resolution = { 20, 17, 7, 0 };
        param.curve            = { 0.0f, 20.0f, 0.0f, 0.0f };
        param.curve_v          = { 0.0f, 20.0f, 0.0f, 0.0f };
        param.curve_back       = { 0.0f, -100.0f, 0.0f, 0.0f };
        param.bend_v           = { 60.0f, 120.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 200;
        param.leaf_shape       = 2;
        param.leaf_scale       = 0.07f;
        param.leaf_scale_x     = 0.5f;
        param.leaf_bend        = 0.1f;
        param.tropism          = { 0.0f, 0.0f, -3.0f };

        return param;
    }

    Parameter FanPalm() {
        Parameter param;
        param.shape            = 3;
        param.scale            = 5.0f;
        param.scale_v          = 2.0f;
        param.levels           = 2;
        param.ratio            = 0.04f;
        param.ratio_power      = 1.3f;
        param.flare            = 0.0f;
        param.base_splits      = 0;
        param.base_size        = { 0.8f, 0.0f, 0.0f, 0.0f };
        param.down_angle       = { 0.0f, 20.0f, -10.0f, 0.0f };
        param.down_angle_v     = { 0.0f, -60.0f, 10.0f, 0.0f };
        param.rotation         = { 0.0f, 160.0f, 260.0f, 0.0f };
        param.rotation_v       = { 0.0f, 40.0f, 5.0f, 0.0f };
        param.branches         = { 1, 50, 0, 0 };
        param.length           = { 1.0f, 0.25f, 0.0f, 0.0f };
        param.length_v         = { 0.0f, 0.05f, 0.0f, 0.0f };
        param.taper            = { 2.1f, 1.4f, 0.0f, 0.0f };
        param.segment_splits   = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.curve_resolution = { 6, 9, 0, 0 };
        param.curve            = { 10.0f, 50.0f, 0.0f, 0.0f };
        param.curve_v          = { 40.0f, 30.0f, 0.0f, 0.0f };
        param.curve_back       = { 0.0f, -5.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = -90;
        param.leaf_shape       = 10;
        param.leaf_scale       = 0.8f;
        param.leaf_scale_x     = 0.05f;
        param.tropism          = { 0.0f, 0.0f, -1.0f };

        return param;
    }

    Parameter HillCherry() {
        Parameter param;
        param.shape            = 2;
        param.scale            = 13.0f;
        param.scale_v          = 3.0f;
        param.levels           = 3;
        param.ratio            = 0.015f;
        param.ratio_power      = 1.3f;
        param.flare            = 0.6f;
        param.base_splits      = -2;
        param.base_size        = { 0.15f, 0.02f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 70.0f, 60.0f, 45.0f };
        param.down_angle_v     = { 0.0f, 10.0f, 20.0f, 30.0f };
        param.rotation         = { 0.0f, 140.0f, 140.0f, 77.0f };
        param.rotation_v       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.branches         = { 1, 25, 18, 10 };
        param.length           = { 0.8f, 0.5f, 0.6f, 0.0f };
        param.length_v         = { 0.0f, 0.0f, 0.1f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.5f, 0.3f, 0.0f, 0.0f };
        param.split_angle      = { 40.0f, 40.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 5.0f, 5.0f, 0.0f, 0.0f };
        param.curve_resolution = { 10, 5, 8, 0 };
        param.curve            = { 30.0f, -20.0f, -40.0f, 0.0f };
        param.curve_v          = { 150.0f, 150.0f, 150.0f, 0.0f };
        param.curve_back       = { -40.0f, 40.0f, 0.0f, 0.0f };
        param.bend_v           = { 150.0f, 150.0f, 250.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.tropism          = { 0.0f, 0.0f, -1.0f };
        param.prune_width_peak = 0.5f;
        param.prune_power_low  = 0.2f;
        param.prune_power_high = 0.5f;

        return param;
    }

    Parameter LombardyPoplar() {
        Parameter param;
        param.shape            = 2;
        param.scale            = 25.0f;
        param.scale_v          = 5.0f;
        param.levels           = 3;
        param.ratio            = 0.015f;
        param.ratio_power      = 1.2f;
        param.flare            = 0.8f;
        param.base_splits      = 0;
        param.base_size        = { 0.01f, 0.1f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 30.0f, 30.0f, 30.0f };
        param.down_angle_v     = { 0.0f, 0.0f, 10.0f, 10.0f };
        param.rotation         = { 0.0f, 77.0f, 77.0f, 77.0f };
        param.rotation_v       = { 0.0f, 15.0f, 15.0f, 15.0f };
        param.branches         = { 1, 60, 35, 10 };
        param.length           = { 1.0f, 0.3f, 0.4f, 0.0f };
        param.length_v         = { 0.0f, 0.0f, 0.1f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.curve_resolution = { 1, 3, 3, 0 };
        param.curve            = { 0.0f, -20.0f, -20.0f, 0.0f };
        param.curve_v          = { 0.0f, 0.0f, 40.0f, 0.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 30;
        param.leaf_shape       = 0;
        param.leaf_scale       = 0.3f;
        param.leaf_bend        = 0.7f;
        param.tropism          = { 0.0f, 0.0f, 0.5f };

        return param;
    }

    Parameter Palm() {
        Parameter param;
        param.shape            = 3;
        param.scale            = 14.0f;
        param.scale_v          = 3.0f;
        param.levels           = 2;
        param.ratio            = 0.015f;
        param.ratio_power      = 2.0f;
        param.flare            = 0.3f;
        param.base_splits      = 0;
        param.base_size        = { 0.95f, 0.02f, 0.0f, 0.0f };
        param.down_angle       = { 0.0f, 60.0f, 50.0f, 0.0f };
        param.down_angle_v     = { 0.0f, -80.0f, -75.0f, 0.0f };
        param.rotation         = { 0.0f, 120.0f, -120.0f, 0.0f };
        param.rotation_v       = { 0.0f, 60.0f, 20.0f, 0.0f };
        param.branches         = { 1, 25, 0, 0 };
        param.length           = { 1.0f, 0.3f, 0.0f, 0.0f };
        param.length_v         = { 0.0f, 0.02f, 0.0f, 0.0f };
        param.taper            = { 2.15f, 1.0f, 0.0f, 0.0f };
        param.segment_splits   = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.curve_resolution = { 12, 9, 0, 0 };
        param.curve            = { 20.0f, 40.0f, 0.0f, 0.0f };
        param.curve_v          = { 10.0f, 20.0f, 0.0f, 0.0f };
        param.curve_back       = { -10.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 150;
        param.leaf_shape       = 10;
        param.leaf_scale       = 0.8f;
        param.leaf_scale_x     = 0.12f;
        param.tropism          = { 0.0f, 0.0f, -3.0f };

        return param;
    }

    Parameter QuakingAspen() {
        Parameter param;
        param.shape            = 7;
        param.scale            = 13.0f;
        param.scale_v          = 3.0f;
        param.levels           = 3;
        param.ratio            = 0.015f;
        param.ratio_power      = 1.2f;
        param.flare            = 0.6f;
        param.base_splits      = 0;
        param.base_size        = { 0.3f, 0.02f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 60.0f, 60.0f, 45.0f };
        param.down_angle_v     = { 0.0f, -50.0f, 20.0f, 30.0f };
        param.rotation         = { 0.0f, 140.0f, 140.0f, 77.0f };
        param.rotation_v       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.branches         = { 1, 50, 30, 1 };
        param.length           = { 1.0f, 0.3f, 0.6f, 0.0f };
        param.length_v         = { 0.0f, 0.0f, 0.1f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle      = { 40.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 5.0f, 0.0f, 0.0f, 0.0f };
        param.curve_resolution = { 5, 5, 5, 0 };
        param.curve            = { 0.0f, -40.0f, -60.0f, 0.0f };
        param.curve_v          = { 20.0f, 100.0f, 100.0f, 0.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 50.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 40;
        param.leaf_shape       = 3;
        param.leaf_scale       = 0.17f;
        param.leaf_bend        = 0.6f;

        return param;
    }

    Parameter Sassafras() {
        Parameter param;
        param.shape            = 2;
        param.scale            = 23.0f;
        param.scale_v          = 7.0f;
        param.levels           = 4;
        param.ratio            = 0.02f;
        param.ratio_power      = 1.3f;
        param.flare            = 0.5f;
        param.base_splits      = 0;
        param.base_size        = { 0.2f, 0.02f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 90.0f, 50.0f, 45.0f };
        param.down_angle_v     = { 0.0f, -10.0f, 10.0f, 10.0f };
        param.rotation         = { 0.0f, 140.0f, 140.0f, 140.0f };
        param.rotation_v       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.branches         = { 1, 20, 20, 30 };
        param.length           = { 1.0f, 0.4f, 0.7f, 0.4f };
        param.length_v         = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle      = { 20.0f, 0.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 5.0f, 0.0f, 0.0f, 0.0f };
        param.curve_resolution = { 16, 15, 8, 3 };
        param.curve            = { 0.0f, -60.0f, -40.0f, 0.0f };
        param.curve_v          = { 60.0f, 100.0f, 150.0f, 100.0f };
        param.curve_back       = { 0.0f, 30.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 15;
        param.leaf_shape       = 0;
        param.leaf_scale       = 0.25f;
        param.leaf_scale_x     = 0.7f;
        param.leaf_bend        = 0.3f;
        param.tropism          = { 0.0f, 0.0f, 0.5f };

        return param;
    }

    Parameter SilverBirch() {
        Parameter param;
        param.shape            = 3;
        param.scale            = 20.0f;
        param.scale_v          = 5.0f;
        param.levels           = 3;
        param.ratio            = 0.015f;
        param.ratio_power      = 1.5f;
        param.flare            = 0.5f;
        param.base_splits      = 0;
        param.base_size        = { 0.3f, 0.1f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 50.0f, 40.0f, 45.0f };
        param.down_angle_v     = { 0.0f, -20.0f, 10.0f, 10.0f };
        param.rotation         = { 0.0f, 140.0f, 140.0f, 140.0f };
        param.rotation_v       = { 0.0f, 60.0f, 50.0f, 0.0f };
        param.branches         = { 1, 30, 60, 0 };
        param.length           = { 1.0f, 0.3f, 0.4f, 0.0f };
        param.length_v         = { 0.0f, 0.05f, 0.2f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.0f, 0.3f, 0.0f, 0.0f };
        param.split_angle      = { 15.0f, 10.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.curve_resolution = { 10, 10, 10, 0 };
        param.curve            = { 0.0f, 0.0f, -10.0f, 0.0f };
        param.curve_v          = { 50.0f, 150.0f, 200.0f, 0.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 100.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 65;
        param.leaf_shape       = 3;
        param.leaf_scale       = 0.15f;
        param.leaf_scale_x     = 1.0f;
        param.leaf_bend        = 0.2f;
        param.tropism          = { 0.0f, 0.0f, -2.0f };

        return param;
    }

    Parameter SmallPine() {
        Parameter param;
        param.shape            = 0;
        param.scale            = 6.0f;
        param.scale_v          = 0.5f;
        param.levels           = 2;
        param.ratio            = 0.02f;
        param.ratio_power      = 1.3f;
        param.flare            = 0.3f;
        param.base_splits      = 0;
        param.base_size        = { 0.05f, 0.02f, 0.02f, 0.02f };
        param.down_angle       = { 0.0f, 30.0f, 30.0f, 0.0f };
        param.down_angle_v     = { 0.0f, -60.0f, 10.0f, 0.0f };
        param.rotation         = { 0.0f, 140.0f, 140.0f, 0.0f };
        param.rotation_v       = { 0.0f, 30.0f, 20.0f, 0.0f };
        param.branches         = { 1, 70, 0, 0 };
        param.length           = { 1.0f, 0.35f, 0.0f, 0.0f };
        param.length_v         = { 0.0f, 0.05f, 0.0f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 0.0f, 0.0f };
        param.segment_splits   = { 0.0f, 2.0f,  0.0f, 0.0f };
        param.split_angle      = { 0.0f, -80.0f, 0.0f, 0.0f };
        param.split_angle_v    = { 0.0f, -30.0f, 0.0f, 0.0f };
        param.curve_resolution = { 5, 6, 0, 0 };
        param.curve            = { 0.0f, -20.0f, 0.0f, 0.0f };
        param.curve_v          = { 10.0f, 90.0f, 0.0f, 0.0f };
        param.curve_back       = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 70.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 3.5f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.n_leaves         = 400;
        param.leaf_shape       = 2;
        param.leaf_scale       = 0.17f;
        param.leaf_scale_x     = 0.3f;

        return param;
    }

    Parameter WeepingWillow() {
        Parameter param;
        param.shape            = 4;
        param.scale            = 15.0f;
        param.scale_v          = 3.0f;
        param.levels           = 4;
        param.ratio            = 0.03f;
        param.ratio_power      = 1.2f;
        param.flare            = 0.75f;
        param.base_splits      = 2;
        param.base_size        = { 0.05f, 0.3f, 0.05f, 0.05f };
        param.down_angle       = { 0.0f, 40.0f, 30.0f, 20.0f };
        param.down_angle_v     = { 0.0f, 10.0f, 10.0f, 10.0f };
        param.rotation         = { 0.0f, -120.0f, -120.0f, 140.0f };
        param.rotation_v       = { 0.0f, 30.0f, 30.0f, 0.0f };
        param.branches         = { 1, 17, 25, 75 };
        param.length           = { 1.0f, 0.35f, 2.0f, 0.1f };
        param.length_v         = { 0.0f, 0.1f, 0.0f, 0.0f };
        param.taper            = { 1.0f, 1.0f, 1.0f, 1.0f };
        param.segment_splits   = { 0.2f, 0.2f, 0.1f, 0.0f };
        param.split_angle      = { 40.0f, 30.0f, 45.0f, 0.0f };
        param.split_angle_v    = { 5.0f, 10.0f, 20.0f, 0.0f };
        param.curve_resolution = { 8, 16, 12, 2 };
        param.curve            = { 0.0f, 40.0f, 0.0f, 0.0f };
        param.curve_v          = { 90.0f, 200.0f, 0.0f, 0.0f };
        param.curve_back       = { 25.0f, 0.0f, 0.0f, 0.0f };
        param.bend_v           = { 0.0f, 160.0f, 0.0f, 0.0f };
        param.branch_dist      = { 0.0f, 0.0f, 0.0f, 0.0f };
        param.radius_modify    = { 1.0f, 1.0f, 0.1f, 1.0f };
        param.prune_width_peak = 0.6f;
        param.prune_power_low  = 0.001f;
        param.prune_power_high = 0.5f;
        param.n_leaves         = 5;
        param.leaf_shape       = 0;
        param.leaf_scale       = 0.13f;
        param.leaf_scale_x     = 0.2f;
        param.tropism          = { 0.0f, 0.0f, -3.0f };

        return param;
    }

    /**
     * Get all names of tree model.
     */
    void GetAllTreeNames(Array<std::string>* tree_names) {
        CHECK(tree_names);

        tree_names->clear();
        for (auto i : function_map_) {
            tree_names->push_back(i.first);
        }
    }

private:
    // Map from name to the function.
    std::map<std::string, Func> function_map_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_TREE_TREE_FACTORY_H_
