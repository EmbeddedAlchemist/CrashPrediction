
#include "httplib/httplib.h"
#include "ExLog/ExLog.hpp"
#include "CrashPredictor/CrashPredictor.hpp"
#include "HTTPInterface/HTTPInterface.hpp"
#include <thread>
#include <DebugInterface/DebugDraw.hpp>

CrashPredictor predictor;
HTTPInterface httpInterface(predictor);
DebugDraw debugDraw("0.0.0.0", 8081);


std::thread httpThread([&]() {
    LOG_I << "HTTP thread start!";
    httpInterface.begin("0.0.0.0", 8080);
    });


std::thread predictThread([&]() {
    while (true) {
        auto res = predictor.predict({ 0.01, 3, 4.2 });
        if (res.size() > 0) {
            LOG_I << "Crash detected!";
        }
    }
    });

int main(int argc, const char *argv[]) {
    ExLog::Logger::setLogLevel(ExLog::LogLevel::Verbose);
    LOG_I << "Begin!";
    debugDraw.bindToWorld(predictor.getRealTimeScene()._debugGetWorld());

    httpThread.join();

    LOG_I << "Exit.";
    return 0;

}
