#ifndef CITYFLOW_ROUTER
#define CITYFLOW_ROUTER

#include "engine/archive.h"

#include <memory>
#include <random>
#include <vector>

namespace CityFlow {
class Road;
class Drivable;
class Route;
class Lane;
class LaneLink;
class Vehicle;

class Router {
    friend Archive;

  private:
    Vehicle *vehicle = nullptr;                   // route 对应的车辆
    std::vector<Road *> route;                    // 由 anchorPoints 生成的路径
    std::vector<Road *> anchorPoints;             // Flow 提供的必须经过的 Road
    std::vector<Road *>::const_iterator iCurRoad; // 当前所在的 road 的位置
    std::mt19937 *rnd = nullptr;                  // 随机数

    mutable std::deque<Drivable *> planned; // 未来要走的路径缓存，由 getNextDrivable() 提前计算得出

    int selectLaneIndex(const Lane *curLane, const std::vector<Lane *> &lanes) const; // 根据当前 lane 在备选区选下一个 lane 的编号

    LaneLink *selectLaneLink(const Lane *curLane, const std::vector<LaneLink *> &laneLinks) const; // 根据当前 lane 在 laneLink 备选区的 endline 内选下一个 lane

    Lane *selectLane(const Lane *curLane, const std::vector<Lane *> &lanes) const; // 根据当前 lane 在备选区选下一个 lane

    enum class RouterType {
        LENGTH,   // 权值为长度
        DURATION, // 权值为时间
        DYNAMIC   // TODO: dynamic routing
    };

    RouterType type = RouterType::LENGTH;

  public:
    Router(const Router &other);

    Router(Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd);

    Road *getFirstRoad() {
        return anchorPoints[0];
    }

    Drivable *getFirstDrivable() const; // 进入 route[0] 应选的 drivable

    Drivable *getNextDrivable(size_t i = 0) const; // 下 i + 1 步将走的 drivable

    Drivable *getNextDrivable(const Drivable *curDrivable) const; // 由当前 drivable 计算下一个 drivable（当前为 lane 则给出 laneLink，反之同理）

    void update(); // 更新 iCurRoad 与 planned

    bool isLastRoad(const Drivable *drivable) const;

    bool onLastRoad() const;

    bool onValidLane() const { // 当无下一条路且 route 未到末尾说明有误
        return !(getNextDrivable() == nullptr && !onLastRoad());
    }

    Lane *getValidLane(const Lane *curLane) const; // 选取从 curLane 走向下一个 Road 时 laneIndex 差距最小的 lane

    void setVehicle(Vehicle *vehicle) { // 修改 route 对应车辆
        this->vehicle = vehicle;
    }

    bool dijkstra(Road *start, Road *end, std::vector<Road *> &buffer); // 最短路

    bool updateShortestPath(); // 更新 route

    bool setRoute(const std::vector<Road *> &anchor); // 修改后续需走的路径为 anchor

    std::vector<Road *> getFollowingRoads() const; // 获取未来将走的所有 Road
};
} // namespace CityFlow

#endif