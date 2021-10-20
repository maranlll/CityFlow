#ifndef CITYFLOW_FLOW_H
#define CITYFLOW_FLOW_H

#include <iostream>

#include "flow/route.h"
#include "vehicle/vehicle.h"

namespace CityFlow {
class Engine;

struct VehicleInfo;

class Flow {
    friend class Archive;

  private:
    VehicleInfo vehicleTemplate;        // Flow 对应的车辆信息
    std::shared_ptr<const Route> route; // 车辆待走路径
    double interval;                    // 每 interval 时间进入一次 RoadNet
    double nowTime = 0;                 // 自从上次进入 RoadNet 后累积的时间
    double currentTime = 0;             // 系统当前时间
    int startTime = 0;                  // startTime 后可以进入 RoadNet
    int endTime = -1;                   // endTime 后不再进入 RoadNet
    int cnt = 0;                        // 进入 RoadNet 的次数
    Engine *engine;                     // 所属引擎
    std::string id;                     // flow_i
    bool valid = true;                  // 是否有效（route是否可达）

  public:
    Flow(const VehicleInfo &vehicleTemplate, double timeInterval, Engine *engine, int startTime, int endTime, const std::string &id)
        : vehicleTemplate(vehicleTemplate), interval(timeInterval), startTime(startTime), endTime(endTime), engine(engine), id(id) {
        assert(timeInterval >= 1 || (startTime == endTime));
        nowTime = interval;
    }

    void nextStep(double timeInterval);

    std::string getId() const;

    bool isValid() const {
        return this->valid;
    }

    void setValid(const bool valid) {
        if (this->valid && !valid)
            std::cerr << "[warning] Invalid route '" << id << "'. Omitted by default." << std::endl;
        this->valid = valid;
    }

    void reset();
};
} // namespace CityFlow

#endif // CITYFLOW_FLOW_H
