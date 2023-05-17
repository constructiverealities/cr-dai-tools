#include "cr/dai-tools/DeviceRunner.h"
#include "cr/dai-tools/PipelineBuilder.h"

namespace cr {
    namespace dai_tools {
        void AutoDeviceRunner::SetupPipeline() {
            pipeline = GeneratePipeline(device);
            this->OnSetupPipeline(pipeline);
        }
    }
}