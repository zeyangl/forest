#include "kd.h"

// recursive tree construction
static KDNode* _build(int depth, KDAxis axis, std::vector<Vec3>& p)
{
    KDNode* node = new KDNode();
    if(depth == 0 || p.size() < 3)
    {
        node->isLeaf = true;
        node->data.insert(node->data.end(), p.begin(), p.end());
    }
    else
    {
        std::sort(p.begin(), p.end(), [=](const Vec3& l, const Vec3& r){
            return l[axis] < r[axis];
        });

        int splitLen = p.size() / 2;
        std::vector<Vec3> left(p.begin(), p.begin()+splitLen);
        std::vector<Vec3> right(p.begin()+splitLen, p.end());

        node->axis = axis;
        node->split = p[splitLen][axis];
        node->left = _build(depth-1, (KDAxis)((axis+1) % KDAxis::N), left);
        node->right = _build(depth-1, (KDAxis)((axis+1) % KDAxis::N), right);
    }
    return node;
}

void KDTree::build(int maxDepth, const std::vector<Vec3>& points)
{
    std::vector<Vec3> copy = points;
    root = _build(maxDepth, KDAxis::X, copy);
}

// recursive nearest neighbour search
static Vec3 _findNearest(KDNode* node, const Vec3& p, Vec3 best)
{
    if(node->isLeaf)
    {
        if(node->data.size() == 0)
            return best;

        std::sort(node->data.begin(), node->data.end(), [=](const Vec3& l, const Vec3& r)
        {
            return norm2(l - p) < norm2(r - p);
        });
        return norm2(best - p) < norm2(node->data[0]-p) ? best : node->data[0];
    }
    else
    {
        KDNode* first = p[node->axis] < node->split ? node->left : node->right;
        best = _findNearest(first, p, best);
        if(std::abs(node->split - p[node->axis]) <= norm(best - p))
        {
            KDNode* other = node->left == first ? node->right : node->left;
            best = _findNearest(other, p, best);
        }

        return best;
    }
}

Vec3 KDTree::findNearestNeighbour(const Vec3& p) const
{
    Vec3 best(1000000, 1000000, 1000000);
    return _findNearest(root, p, best);
}


static void _findPointsInAABB(KDNode* node, const Vec3& min, const Vec3& max, std::vector<Vec3>& out)
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
        if(min[node->axis] < node->split)
            _findPointsInAABB(node->left, min, max, out);
        if(max[node->axis] > node->split)
            _findPointsInAABB(node->right, min, max, out);
    }
}

std::vector<Vec3> KDTree::findPointsInAABB(const Vec3& minDim, const Vec3& maxDim) const
{
    std::vector<Vec3> out;
    _findPointsInAABB(root, minDim, maxDim, out);
    return out;
}


// recursive dump
static void _dump(KDNode* node, int level, const Vec3& ref)
{
    if(node->isLeaf)
    {
        printf("%s[leaf] ", std::string(level*4, ' ').c_str());
        for(const Vec3& v : node->data)
            printf("(%.0f, %.0f, %.0f) %.1f   ", v[0], v[1], v[2], norm(v-ref));
        printf("\n");
    }
    else
    {
        char axismap[] = {'x', 'y', 'z'};
        printf("%s[-] %c %.0f\n", std::string(level*4, ' ').c_str(), axismap[node->axis], node->split);
        _dump(node->left, level+1, ref);
        _dump(node->right, level+1, ref);
    }
}

void KDTree::dump(const Vec3& ref) const
{
    _dump(root, 0, ref);
}

