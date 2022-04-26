#include "cr/dai-tools/PipelineBuilder.h"
#include "cr/dai-tools/DeviceRunner.h"

int main() {
    cr::dai_tools::AutoDeviceRunner runner;
    runner.Run();
    return 0;
}