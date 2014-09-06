#include "oct.h"

//------------------------------------
// build octree to a certain depth
// child index is consider a 3-bit value 
// where each bit decides the min/max of 
// x/y/z axis respectively
static OctNode* _build(int depth, const Vec3& minDim, const Vec3& maxDim)
{
    OctNode* node = new OctNode;
    node->minDim = minDim;
    node->maxDim = maxDim;
    node->center = (minDim + maxDim) * 0.5;

    if(depth == 0)
    {
        node->isLeaf = true;
    }
    else
    {
        for(int i=0; i<8; i++)
        {
            Vec3 corner(
                (i & (1 << 0)) ? maxDim[0] : minDim[0],
                (i & (1 << 1)) ? maxDim[1] : minDim[1],
                (i & (1 << 2)) ? maxDim[2] : minDim[2]
            );

            Vec3 childMin, childMax;
            for(int k=0; k<3; k++)
            {
                childMin[k] = std::min(node->center[k], corner[k]);
                childMax[k] = std::max(node->center[k], corner[k]);
            }

            node->children[i] = _build(depth-1, childMin, childMax);
        }
    }
    return node;
}

void Octree::build(int maxDepth, const Vec3& minDim, const Vec3& maxDim)
{
    root = _build(maxDepth, minDim, maxDim);
}

//-----------------------------------
// insert a point to tree
// indexing is same as in tree build
void Octree::insert(const Vec3& p)
{
    OctNode* n = root;
    while(!n->isLeaf)
    {
        Vec3 v = p - n->center;
        int index = ((v[0] < 0 ? 0 : 1) << 0) |
                    ((v[1] < 0 ? 0 : 1) << 1) |
                    ((v[2] < 0 ? 0 : 1) << 2);

        n = n->children[index];
    }

    n->data.push_back(p);
}

//----------------------------------
// nearest neighbour search
// instead of sorting the children distance to point,
// the order can be hard coded as a table.
static Vec3 _findNearest(OctNode* node, const Vec3& p, Vec3 best)
{
    if(node->isLeaf)
    {
        for(const Vec3& v : node->data)
        {
            if(norm2(v - p) < norm2(best - p))
                best = v;
        }
    }
    else
    {
        // this can be replaced by a table lookup
        std::vector<int> order = {0, 1, 2, 3, 4, 5, 6, 7};
        std::sort(order.begin(), order.end(), [=](int l, int r){
            return node->children[l]->minDist2ToPoint(p) < node->children[r]->minDist2ToPoint(p);
        });
        
        for(int i : order)
        {
            if(node->children[i]->minDist2ToPoint(p) > norm2(best-p))
                break;
            best = _findNearest(node->children[i], p, best);
        }
    }

    return best;
}

Vec3 Octree::findNearestNeighbour(const Vec3& p) const
{
    Vec3 best(1000000, 1000000, 1000000);
    return _findNearest(root, p, best);
}

//----------------------------------
// find points in AABB
// pretty straightforward
static void _findPointsInAABB(OctNode* node, const Vec3& min, const Vec3& max, std::vector<Vec3>& out)
{
    if(node->isLeaf)
    {
        for(const Vec3& p : node->data)
        {
            if(p[0] > min[0] && p[0] < max[0] &&
               p[1] > min[1] && p[1] < max[1] &&
               p[2] > min[2] && p[2] < max[2])
            {
                out.push_back(p);
            }
        }
    }
    else
    {
        for(int i=0; i<8; i++)
        {
            if(node->children[i]->intersectAABB(min, max))
                _findPointsInAABB(node->children[i], min, max, out);
        }
    }
}

std::vector<Vec3> Octree::findPointsInAABB(const Vec3& minDim, const Vec3& maxDim) const
{
    std::vector<Vec3> out;
    _findPointsInAABB(root, minDim, maxDim, out);
    return out;
}

//-----------------------------------
// debug dump
static void _dump(OctNode* node, int depth, const Vec3& ref)
{
    if(node->isLeaf)
    {
        if(node->data.size())
        {
            printf("%s[leaf] ", std::string(depth*4, ' ').c_str());
            for(const Vec3& v : node->data)
                printf("(%.0f, %.0f, %.0f) %.1f   ", v[0], v[1], v[2], norm(v-ref));
            printf("\n");
        }
    }
    else
    {
        printf("%s[-], (%.0f, %.0f, %.0f)\n", std::string(depth*4, ' ').c_str(), 
            node->center[0], node->center[1], node->center[2]
        );

        for(int i=0; i<8; i++)
        {
            _dump(node->children[i], depth+1, ref);
        }
    }
}

void Octree::dump(const Vec3& ref) const 
{
    _dump(this->root, 0, ref); 
}


