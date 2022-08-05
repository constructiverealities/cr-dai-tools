#include <boost/process.hpp>
#include <iostream>

#define NOMINMAX
#include "termcolor/termcolor.hpp"

#include "depthai/depthai.hpp"

using namespace boost::process;

typedef std::basic_ostream<char>& color_type(std::basic_ostream<char>& stream);
static std::vector< color_type* > colors = {
        termcolor::grey,
        termcolor::red,
        termcolor::green,
        termcolor::yellow,
        termcolor::blue,
        termcolor::magenta,
        termcolor::cyan,
        termcolor::white,
        termcolor::bright_grey,
        termcolor::bright_red,
        termcolor::bright_green,
        termcolor::bright_yellow,
        termcolor::bright_blue,
        termcolor::bright_magenta,
        termcolor::bright_cyan,
        termcolor::bright_white,
};
static std::hash<std::string> hasher;
static inline std::ostream& output(const std::string& v) {

    auto id = hasher(v);
    auto color_f = colors[id % colors.size()];
    return std::cout << "[" << termcolor::colorize << color_f << v << termcolor::reset << "] ";
}

struct DeviceProcess {
    child process;
    std::thread log_thread;
    DeviceProcess(const std::string& process_string, const std::string& id) {
        log_thread = std::thread([this, id, process_string]() {
            auto env = boost::this_process::environment();
            env["DEPTHAI_DEVICE_MXID_LIST"] = id;

            ipstream pipe_stream;
            process = child(process_string, env, (std_out & std_err) > pipe_stream);

            output(id) << " Creating daemon process for " << id << std::endl;

            auto strip_id = "[" + id + "] ";
            std::string line;
            while (process.running()) {
                std::getline(pipe_stream, line);
                if(strncmp(line.c_str(), strip_id.c_str(), strip_id.size()) == 0) {
                    line = line.c_str() + strip_id.size();
                }
                output(id) << line << std::endl;
            }
            output(id) << "Exiting with status " << process.exit_code() << std::endl;
        });
    }
    ~DeviceProcess() {
        kill();
    }
    void kill() {
        if(log_thread.joinable()) {
            process.terminate();
            log_thread.join();
        }
    }
};

int main(int argc, char** argv)
{
    std::map<std::string, std::shared_ptr<DeviceProcess>> devices;

    std::string process_string;
    for(int i = 1;i < argc;i++) {
        if (argv[i] == "--") {
            process_string = "";
        } else {
            auto arg = std::string(argv[i]);
            bool hasSpace = arg.find(' ') >= 0;
            if(hasSpace)
                process_string += " \"" + arg + "\"";
            else
                process_string += " " + arg;
        }
    }

    while (true) {
        auto avail_device = dai::Device::getFirstAvailableDevice();

        if(!std::get<0>(avail_device))
            continue;

        output("daemon") << "Found " << std::get<1>(avail_device).getMxId() << std::endl;

        auto id = std::get<1>(avail_device).getMxId();
        if(devices[id] == 0 && id != "<error>") {
            devices[id] = std::make_shared<DeviceProcess>(process_string, id);
            std::this_thread::sleep_for(std::chrono::seconds(4));
        } else if(id != "<error>") {
                output("daemon") << "Killing process for " << id << " -- it gave up the device" << std::endl;
                devices[id].reset();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}