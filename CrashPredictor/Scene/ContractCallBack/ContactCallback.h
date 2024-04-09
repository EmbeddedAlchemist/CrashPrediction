#pragma once

#include "box2d/box2d.h"
#include "../CrashInfo/CrashInfo.hpp"
#include "../PredictOption/PredictOption.hpp"

#include <vector>
#include <memory>
#include <utility>
#include <queue>



class ContactCallback : public b2ContactListener {
private:
    PredictOption option;
    std::vector<std::pair<b2Body *, b2Body *>> record;
    virtual void BeginContact(b2Contact *contact);
    void calcContactResult(std::pair < b2Body *, b2Body *> rec);

public:
    std::vector<CrashInfo> getContact();
    inline ContactCallback(const PredictOption &option) : option(option) {};
};

