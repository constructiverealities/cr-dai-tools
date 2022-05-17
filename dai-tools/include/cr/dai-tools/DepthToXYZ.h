#include <vector>
#include "stdint.h"

namespace cr {
    namespace dai_tools {
        class DepthToXYZ {
            std::vector<float> pc_map;
            int width, height;
        public:
            const std::vector<float>& Map() const {return pc_map;}
            DepthToXYZ(int width, int height, const std::vector<double>& LP, const std::vector<double>& D);
            void operator()(const uint16_t* depth, float* pc);
        };
    }
}