#include <iostream>

#include "kd.h"
#include "oct.h"

#include <set>
#include <algorithm>

static float randx()
{
    return rand() % 100;
}

int main()
{
    std::cout << "kd-tree and octree tests" << std::endl;

    srand(1987);

    std::vector<Vec3> test1;
    for(int i=0; i<300; i++)
        test1.push_back(Vec3(randx(), randx(), randx()));

    std::vector<Vec3> testPoints;
    for(int i=0; i<10; i++)
        testPoints.push_back(Vec3(randx(), randx(), randx()));

    //--------------------
    // kd-tree tests
    KDTree kd;
    kd.build(4, test1);
    //kd.dump(Vec3(0, 0, 0));

    // test nearest neighbour
    std::vector<Vec3> kdBests;
    for(const Vec3& p : testPoints)
    {
        //printf("\ntest point: (%.0f, %.0f, %.0f)\n", p[0], p[1], p[2]);
        Vec3 x = kd.findNearestNeighbour(p);
        //printf("best (%.0f, %.0f, %.0f)\n", x[0], x[1], x[2]);
        kdBests.push_back(x);
    }


    //-------------------
    // octree tests
    Octree oct;
    oct.build(4, Vec3(0, 0, 0), Vec3(100, 100, 100));
    for(const Vec3& v : test1)
        oct.insert(v);
    //oct.dump(Vec3(0, 0, 0));

    // test nearest neighbour
    std::vector<Vec3> octBests;
    for(const Vec3& p : testPoints)
    {
        //printf("\ntest point: (%.0f, %.0f, %.0f)\n", p[0], p[1], p[2]);
        Vec3 x = oct.findNearestNeighbour(p);
        //oct.dump(p);
        //printf("best (%.0f, %.0f, %.0f)\n", x[0], x[1], x[2]);
        octBests.push_back(x);
    }

    //--------------------
    // compare results
    for(int i=0; i<testPoints.size(); i++)
    {
        Vec3 p = testPoints[i];
        Vec3 k = kdBests[i];
        Vec3 o = octBests[i];

        /*
        printf("ref (%.0f, %.0f, %.0f) -> kd best (%.0f, %.0f, %.0f), oct best (%.0f, %.0f, %.0f)   ", 
                p[0], p[1], p[2],
                k[0], k[1], k[2],
                o[0], o[1], o[2]);
         */
        if(k[0] == o[0] && k[1] == o[1] && k[2] == o[2])
        {
            printf("%d pass\n", i);
        }
        else
        {
            printf("%d failed\n", i);
            //printf("kd dist %.1f, oct dist %.1f\n", norm(k-p), norm(o-p));
        }
    }

    auto comp = [](const Vec3& l, const Vec3& r){ return l[0] < r[0] || l[1] < r[1] || l[2] < r[2]; };

    std::vector<Vec3> kdPs = oct.findPointsInAABB(Vec3(15, 15, 15), Vec3(40, 40, 40));
    std::sort(kdPs.begin(), kdPs.end(), comp);
    //for(const Vec3& x : kdPs)
    //    printf("(%.0f, %.0f, %.0f)\n", x[0], x[1], x[2]);

    std::vector<Vec3> octPs = oct.findPointsInAABB(Vec3(15, 15, 15), Vec3(40, 40, 40));
    std::sort(octPs.begin(), octPs.end(), comp);
    //for(const Vec3& x : octPs)
    //    printf("(%.0f, %.0f, %.0f)\n", x[0], x[1], x[2]);

    if(kdPs.size() == octPs.size())
    {
        for(int i=0; i<kdPs.size(); i++)
        {
            if(kdPs[i][0] != octPs[i][0] || 
               kdPs[i][1] != octPs[i][1] ||
               kdPs[i][2] != octPs[i][2])
            {
                printf("aabb failed\n");
                break;
            }
        }
        printf("aabb passed\n");
    }
    else
        printf("aabb failed\n");
}
