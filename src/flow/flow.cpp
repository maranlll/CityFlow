#include "flow/flow.h"
#include "engine/engine.h"

namespace CityFlow {
void Flow::nextStep(double timeInterval) { // 每个 timeInterval 对 flow 进行的操作
    if (!valid)                            // route 不可达
        return;
    if (endTime != -1 && currentTime > endTime) // 未结束
        return;
    if (currentTime >= startTime) {   // 可开始
        while (nowTime >= interval) { // 距此 flow 上次进入 RoadNet 已超过 interval，可根据此 flow 再初始化一辆车放入
            Vehicle *vehicle = new Vehicle(vehicleTemplate, id + "_" + std::to_string(cnt++), engine, this);
            // priority has been set correctlly before?
            int priority = vehicle->getPriority();
            while (engine->checkPriority(priority))
                priority = engine->rnd();
            vehicle->setPriority(priority);
            engine->pushVehicle(vehicle, false);
            vehicle->getFirstRoad()->addPlanRouteVehicle(vehicle);
            nowTime -= interval;
        }
        nowTime += timeInterval; // 累积时间
    }
    currentTime += timeInterval; // 时间同步
}

std::string Flow::getId() const {
    return id;
}

void Flow::reset() {
    nowTime = interval;
    currentTime = 0;
    cnt = 0;
}
} // namespace CityFlow