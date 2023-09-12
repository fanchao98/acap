#include "sample_on_ball.h"

SAMPLE_ON_BALL::cRandom SAMPLE_ON_BALL::my_random(int z)
{
    const int m = std::pow(2, 31) - 1;
    const int a = 16807;
    const int q = 127773;
    const int r = 2836;

    int temp = a * (z % q) - r * (z / q);

    if (temp < 0)
    {
        temp = m + temp;
    }
    //z is the seed number
    z = temp;
    double t = z * 1.0 / m;

    cRandom cr;
    cr.random = t;
    cr.seed = z;

    return cr;
}

void SAMPLE_ON_BALL::GenerateRandomPoints()
{
    srand((int)time(0));
    int z1 = rand() % 100;
    int z2 = rand() % 100;
    cRandom sita(z1, 0.0);
    cRandom pesi(z2, 0.0);
    for (int i = 0; i < 1000000; i++) {
        sita = my_random(pesi.seed);
        pesi = my_random(sita.seed);

        double u = 2 * sita.random - 1.0;
        double v = 2 * pesi.random - 1.0;

        double r2 = pow(u, 2) + pow(v, 2);

        if (r2 < 1)
        {
            double x = 2 * u * sqrt(1 - r2);
            double y = 2 * v * sqrt(1 - r2);
            double z = 1 - 2 * r2;
            //std::cout << x << " " << y << " " << z << std::endl;
            this->random_points.push_back(cv::Point3d(x, y, z));
        }
        if (this->random_points.size() == total_directions) break;
    }
    //std::cout << this->random_points.size() << std::endl;
}
