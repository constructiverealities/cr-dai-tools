#include <cmath>
#include "cr/dai-tools/DepthToXYZ.h"

static void initUndistortRectifyMap( const double* lp, const double * d, int width, int height, float* map12 ) {
    float u0 = lp[2],  v0 = lp[3];
    float fx = lp[0],  fy = lp[1];

    float k1 = d[0];
    float k2 = d[1];
    float p1 = d[2];
    float p2 = d[3];
    float k3 = d[4];
    float k4 = d[5];
    float k5 = d[6];
    float k6 = d[7];

    float ir[] = {
            1.f/fx,       0,       -u0/fx,
            0,       1.f/fy,       -v0/fy,
            0,           0,       1
    };

    for( int i = 0; i < height; i++ )
    {
        float _x = i*ir[1] + ir[2], _y = i*ir[4] + ir[5], _w = i*ir[7] + ir[8];

        for( int j = 0; j < width; j++, _x += ir[0], _y += ir[3], _w += ir[6] )
        {
            float w = 1.f/_w, x = _x*w, y = _y*w;
            float x2 = x*x, y2 = y*y;
            float r2 = x2 + y2, _2xy = 2*x*y;
            float kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2)/(1 + ((k6*r2 + k5)*r2 + k4)*r2);
            float u = fx*(x*kr + p1*_2xy + p2*(r2 + 2*x2)) + u0;
            float v = fy*(y*kr + p1*(r2 + 2*y2) + p2*_2xy) + v0;

            map12[(width*i + j) * 2 + 0] = (u - u0) / fx;
            map12[(width*i + j) * 2 + 1] = (v - v0) / fy;
        }
    }
}


static inline void xyz_from_depth(float depth, const float * zxy, float *  xyz) {
    float nx_over_z = zxy[0], ny_over_z = zxy[1];
    const float z = fmax(0, depth);

    xyz[0] = nx_over_z * z;
    xyz[1] = ny_over_z * z;
    xyz[2] = z;
}

cr::dai_tools::DepthToXYZ::DepthToXYZ(int width, int height, const std::vector<double> &LP,
                                      const std::vector<double> &D) : width(width), height(height) {
    pc_map.resize(width * height * 2);
    if(D.size() >= 8)
        initUndistortRectifyMap(&LP[0], &D[0],  width, height, &pc_map[0]);
}

void cr::dai_tools::DepthToXYZ::operator()(const uint16_t *depth, float *pc) {
    auto pc_map_ptr = &pc_map[0];
    for(int j = 0;j < height;j++){
        for(int i = 0;i < width;i++){
            auto d = depth[i + j * width];

            float xyz[3] = { 0, 0, -1};
            if(d != 0) {
                xyz_from_depth(d / 1000.f, pc_map_ptr + 2 * (i + j * width), xyz);
            }
            for(int k = 0;k < 3;k++) {
                pc[3 * (i + j * width) + k] = xyz[k];
            }
        }
    }
}
