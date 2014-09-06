#pragma once

#include "gfx/vec3.h"
#include <vector>

enum KDAxis
{
    X = 0,
    Y = 1,
    Z = 2,
    N = 3
};

struct KDNode
{
    // intermediate node
    KDAxis axis;
    float split;

    KDNode* left;
    KDNode* right;

    // leaf node
    bool isLeaf;
    std::vector<Vec3> data;

    KDNode() : axis(KDAxis::X), left(nullptr), right(nullptr), isLeaf(false) {}
};

class KDTree
{
    KDNode* root;

public:
    void build(int maxDepth, const std::vector<Vec3>& points);

    Vec3 findNearestNeighbour(const Vec3& p) const;
    std::vector<Vec3> findPointsInAABB(const Vec3& minDim, const Vec3& maxDim) const;

    void dump(const Vec3& ref) const;
};

