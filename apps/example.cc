#include "dai-pipeline-tools.h"

int main() {
    cr::dai_pipeline_tools::AutoDeviceRunner runner;
    runner.Run();
    return 0;
}