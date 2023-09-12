#pragma once
#ifndef SAMPLE_ON_BALL_
#define SAMPLE_ON_BALL_

#include <vector>
#include <cmath>

#include "../opp/helpers.h"
#include <opencv2/imgproc/imgproc.hpp>

class SAMPLE_ON_BALL
{
public:
    class cRandom
    {
    public:
        cRandom(int x, double y) :seed(x), random(y) {};
        cRandom() :seed(0), random(0) {};

        int seed;
        double random;
    };
    cRandom my_random(int z);
    void GenerateRandomPoints();
    std::vector<cv::Point3d> random_points;
};


#endif 