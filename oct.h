#pragma once

#include "gfx/vec3.h"
#include <vector>

struct OctNode
{
    Vec3 center;
    Vec3 minDim;
    Vec3 maxDim;

    OctNode* children[8];

    bool isLeaf;
    std::vector<Vec3> data;

    OctNode() : isLeaf(false) {}

    float minDist2ToPoint(const Vec3& p)
    {
        Vec3 minv;
        for(int i=0; i<3; i++)
        {
            // clamp to box edge then compute distance
            // returns 0 if inside rect
            minv[i] = p[i] - std::max(std::min(p[i], maxDim[i]), minDim[i]);
        }
        return norm2(minv);
    }

    bool intersectAABB(const Vec3& min, const Vec3& max) const
    {
        for(int i=0; i<3; i++)
        {
            if(max[i] < minDim[i] || min[i] > maxDim[i])
                return false;
        }
        return true;
    }
};

class Octree
{
    OctNode* root;

public:
    void build(int maxDepth, const Vec3& minDim, const Vec3& maxDim);
    void insert(const Vec3& p); 

    Vec3 findNearestNeighbour(const Vec3& p) const;
    std::vector<Vec3> findPointsInAABB(const Vec3& minDim, const Vec3& maxDim) const;

    void dump(const Vec3& ref) const;
};


