#ifndef CITYFLOW_TRAFFICLIGHT_H
#define CITYFLOW_TRAFFICLIGHT_H

#include <vector>

namespace CityFlow {
class Intersection;
class RoadLink;
class RoadNet;
class TrafficLight;

class LightPhase { // 信号状态
    friend class RoadNet;
    friend class RoadLink;
    friend class TrafficLight;

  private:
    unsigned int phase = 0;
    double time = 0.0;                   // 持续时间
    std::vector<bool> roadLinkAvailable; // 哪些 roadLink 可过
};

class TrafficLight { // 信号集
    friend class RoadNet;
    friend class Archive;

  private:
    Intersection *intersection = nullptr; // 所属 intersection
    std::vector<LightPhase> phases;       // 信号集
    std::vector<int> roadLinkIndices;
    double remainDuration = 0.0; // 当前信号剩余时间
    int curPhaseIndex = 0;       // 当前所在信号阶段

  public:
    void init(int initPhaseIndex);

    int getCurrentPhaseIndex();

    LightPhase &getCurrentPhase();

    Intersection &getIntersection();

    std::vector<LightPhase> &getPhases();

    void passTime(double seconds); // 经过 seconds

    void setPhase(int phaseIndex);

    void reset();
};
} // namespace CityFlow

#endif // CITYFLOW_TRAFFICLIGHT_H