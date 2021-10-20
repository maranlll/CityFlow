#ifndef CITYFLOW_VEHICLE
#define CITYFLOW_VEHICLE

#include "flow/route.h"
#include "utility/utility.h"
#include "vehicle/lanechange.h"
#include "vehicle/router.h"

#include <memory>
#include <utility>

namespace CityFlow {
class Lane;
class Intersection;
class Route;
class Cross;
class Drivable;
class Engine;
class Point;
class Flow;

struct VehicleInfo {            // 车辆预设信息
    double speed = 0;           // 速度
    double len = 5;             // 车长
    double width = 2;           // 车宽
    double maxPosAcc = 4.5;     // 最大加速
    double maxNegAcc = 4.5;     // 最大减速
    double usualPosAcc = 2.5;   // 常规加速
    double usualNegAcc = 2.5;   // 常规减速
    double minGap = 2;          // 与前车最小间距
    double maxSpeed = 16.66667; // 最大速度
    double headwayTime = 1;
    double yieldDistance = 5;                     // 让步距离
    double turnSpeed = 8.3333;                    // 转弯速度
    std::shared_ptr<const Route> route = nullptr; // 路径
};

class Vehicle {
    friend class Router;
    friend class LaneChange;
    friend class SimpleLaneChange;
    friend class Archive;

  private:
    struct Buffer { // VehicleControl 后待更新到 controllerInfo 等的车辆信息缓存
        bool isDisSet = false;
        bool isSpeedSet = false;
        bool isDrivableSet = false;
        bool isNotifiedVehicles = false;
        bool isEndSet = false;
        bool isEnterLaneLinkTimeSet = false;
        bool isBlockerSet = false;
        bool isCustomSpeedSet = false;

        double dis;         // 在 drivable 上行驶的 dis
        double speed;       // 在 drivable 上行驶的速度
        Drivable *drivable; // 新的 drivable
        std::vector<Vehicle *> notifiedVehicles;
        bool end;                   // 是否完成当前操作
        size_t enterLaneLinkTime;   // 进入 LaneLink 的时间
        Vehicle *blocker = nullptr; // 被 blocker block
        double customSpeed;
        double deltaDis; // ？
    };
    struct LaneChangeInfo {    // 换路控制
        short partnerType = 0; // 0 for no partner; 1 for real vehicle; 2 for shadow vehicle;
        Vehicle *partner = nullptr;
        double offset = 0;
        size_t segmentIndex = 0;
    };
    struct ControllerInfo {               // 车辆行驶信息？
        double dis = 0;                   // 当前 drivable 上形式的距离
        Drivable *drivable = nullptr;     // 当前所在的 drivable
        Drivable *prevDrivable = nullptr; // 上一个 drivable
        double approachingIntersectionDistance;
        double gap;                 // 与 leader 间的距离
        size_t enterLaneLinkTime;   // 如果当前所在 drivable 为 laneLink, 则为进入的时间；否则为 理论最值
        Vehicle *leader = nullptr;  // 前车
        Vehicle *blocker = nullptr; // 在 cross 处受 blocker 阻碍
        bool end = false;           // 是否已完成 router
        bool running = false;       // 正在运行
        Router router;              // 总体路径信息控制与记录
        ControllerInfo(Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd);
        ControllerInfo(Vehicle *vehicle, const ControllerInfo &other);
    };

    VehicleInfo vehicleInfo;
    Buffer buffer;
    LaneChangeInfo laneChangeInfo;
    ControllerInfo controllerInfo;

    int priority;     // 独有，用于 VehiclePool
    std::string id;   // flow_i_j，用于 VehicleMap
    double enterTime; // 由 Flow 创建的时间，即进入 RoadNet 的时间

    Engine *engine;

    std::shared_ptr<LaneChange> laneChange; // 虚函数动态绑定，记录 laneChange 相关信息

    bool routeValid = false; // route 是否可达，值由 updateRoute() 获取
    Flow *flow;

  public:
    bool isStraightHold = false;

    Vehicle(const Vehicle &vehicle, Flow *flow = nullptr);

    Vehicle(const Vehicle &vehicle, const std::string &id, Engine *engine, Flow *flow = nullptr);

    Vehicle(const VehicleInfo &init, const std::string &id, Engine *engine, Flow *flow = nullptr);

    void setDeltaDistance(double dis);

    void setSpeed(double speed);

    void setCustomSpeed(double speed) {
        buffer.customSpeed = speed;
        buffer.isCustomSpeedSet = true;
    }

    void setDis(double dis) { // 存入 buffer 距待更新 drivable 起点距离
        buffer.dis = dis;
        buffer.isDisSet = true;
    }

    void setDrivable(Drivable *drivable) { // 存入 buffer 待更新 drivable
        buffer.drivable = drivable;
        buffer.isDrivableSet = true;
    }

    bool hasSetDrivable() const {
        return buffer.isDrivableSet;
    }

    bool hasSetSpeed() const {
        return buffer.isSpeedSet;
    }

    bool hasSetCustomSpeed() const {
        return buffer.isCustomSpeedSet;
    }

    double getBufferSpeed() const {
        return buffer.speed;
    };

    bool hasSetEnd() const {
        return buffer.isEndSet;
    }

    void setEnd(bool end) {
        buffer.end = end;
        buffer.isEndSet = true;
    }

    void unSetEnd() {
        buffer.isEndSet = false;
    }

    void unSetDrivable() {
        buffer.isDrivableSet = false;
    }

    void setEnterLaneLinkTime(size_t enterLaneLinkTime) {
        buffer.enterLaneLinkTime = enterLaneLinkTime;
        buffer.isEnterLaneLinkTimeSet = true;
    }

    void setBlocker(Vehicle *blocker) {
        buffer.blocker = blocker;
        buffer.isBlockerSet = true;
    }

    double getBufferDis() const {
        return buffer.dis;
    }

    void update();

    void setPriority(int priority) {
        this->priority = priority;
    }

    inline std::string getId() const {
        return id;
    }

    inline double getSpeed() const {
        return vehicleInfo.speed;
    }

    inline double getLen() const {
        return vehicleInfo.len;
    }

    inline double getWidth() const {
        return vehicleInfo.width;
    }

    inline double getDistance() const {
        return controllerInfo.dis;
    }

    Point getPoint() const;

    inline double getMaxPosAcc() const {
        return vehicleInfo.maxPosAcc;
    }

    inline double getMaxNegAcc() const {
        return vehicleInfo.maxNegAcc;
    }

    inline double getUsualPosAcc() const {
        return vehicleInfo.usualPosAcc;
    }

    inline double getUsualNegAcc() const {
        return vehicleInfo.usualNegAcc;
    }

    inline double getMinGap() const {
        return vehicleInfo.minGap;
    }

    inline double getYieldDistance() const {
        return vehicleInfo.yieldDistance;
    }

    inline double getTurnSpeed() const {
        return vehicleInfo.turnSpeed;
    }

    inline Vehicle *getBlocker() const {
        return controllerInfo.blocker;
    }

    inline Vehicle *getBufferBlocker() {
        return buffer.blocker;
    }

    Drivable *getCurDrivable() const;

    Lane *getCurLane() const {
        if (getCurDrivable()->isLane())
            return (Lane *)getCurDrivable();
        else
            return nullptr;
    }

    inline Drivable *getNextDrivable(int i = 0) {
        return controllerInfo.router.getNextDrivable(i);
    }

    inline Drivable *getPrevDrivable(int i = 1) const {
        return controllerInfo.prevDrivable;
    }

    inline int getPriority() const {
        return priority;
    }

    std::pair<Point, Point> getCurPos() const;

    ControlInfo getNextSpeed(double interval);

    Drivable *getChangedDrivable() const;

    double getEnterTime() const {
        return enterTime;
    }

    bool isEnd() const {
        return controllerInfo.end;
    }

    bool isIntersectionRelated();

    double getBrakeDistanceAfterAccel(double acc, double dec, double interval) const;

    inline double getMinBrakeDistance() const { // 减速到 0 最短距离
        return 0.5 * vehicleInfo.speed * vehicleInfo.speed / vehicleInfo.maxNegAcc;
    }

    inline double getUsualBrakeDistance() const {
        return 0.5 * vehicleInfo.speed * vehicleInfo.speed / vehicleInfo.usualNegAcc;
    }

    double getNoCollisionSpeed(double vL, double dL, double vF, double dF, double gap, double interval, double targetGap) const;

    double getCarFollowSpeed(double interval);

    double getStopBeforeSpeed(double distance, double interval) const;

    int getReachSteps(double distance, double targetSpeed, double acc) const;

    int getReachStepsOnLaneLink(double distance, LaneLink *laneLink) const;

    double getDistanceUntilSpeed(double speed, double acc) const;

    bool canYield(double dist) const;

    void updateLeaderAndGap(Vehicle *leader);

    Vehicle *getLeader() const {
        return controllerInfo.leader;
    }

    inline double getEnterLaneLinkTime() const {
        return controllerInfo.enterLaneLinkTime;
    }

    inline double getHeadwayTime() const {
        return vehicleInfo.headwayTime;
    }

    inline double getMaxSpeed() const {
        return vehicleInfo.maxSpeed;
    }

    inline double getApproachingIntersectionDistance() const {
        return 0.0;
    }

    double getIntersectionRelatedSpeed(double interval);

    inline bool isRunning() const {
        return controllerInfo.running;
    }

    inline void setRunning(bool isRunning = true) { // 状态开跑
        controllerInfo.running = isRunning;
    }

    inline bool hasPartner() const {
        return laneChangeInfo.partnerType > 0;
    }

    inline bool isReal() const { // 非 shadow
        return laneChangeInfo.partnerType != 2;
    }

    inline size_t getSegmentIndex() const {
        return laneChangeInfo.segmentIndex;
    }

    inline void setSegmentIndex(int segmentIndex) {
        laneChangeInfo.segmentIndex = segmentIndex;
    }

    inline void setShadow(Vehicle *veh) { // 自己是原 vehicle
        laneChangeInfo.partnerType = 1, laneChangeInfo.partner = veh;
    }

    inline void setParent(Vehicle *veh) { // 自己是 shadow
        laneChangeInfo.partnerType = 2, laneChangeInfo.partner = veh;
    }

    void setLane(Lane *nextLane);

    void finishChanging();

    inline void setOffset(double offset) {
        laneChangeInfo.offset = offset;
    }

    inline double getOffset() const {
        return laneChangeInfo.offset;
    }

    inline Vehicle *getPartner() const {
        return laneChangeInfo.partner;
    }

    inline void setId(const std::string &id) {
        this->id = id;
    }

    // for lane change
    inline void makeLaneChangeSignal(double interval) {
        laneChange->makeSignal(interval);
    }

    inline bool planLaneChange() {
        return laneChange->planChange();
    }

    void receiveSignal(Vehicle *sender);

    void sendSignal() {
        laneChange->sendSignal();
    }

    void clearSignal() {
        laneChange->clearSignal();
    }

    void updateLaneChangeNeighbor() {
        laneChange->updateLeaderAndFollower();
    }

    std::shared_ptr<LaneChange> getLaneChange() {
        return laneChange;
    }

    std::list<Vehicle *>::iterator getListIterator();

    void insertShadow(Vehicle *shadow) {
        laneChange->insertShadow(shadow);
    }

    bool onValidLane() const {
        return controllerInfo.router.onValidLane();
    }

    Lane *getValidLane() const {
        assert(getCurDrivable()->isLane());
        return controllerInfo.router.getValidLane(dynamic_cast<Lane *>(getCurDrivable()));
    }

    bool canChange() const {
        return laneChange->canChange();
    }

    double getGap() const {
        return controllerInfo.gap;
    }

    int laneChangeUrgency() const {
        return laneChange->signalSend->urgency;
    }

    Vehicle *getTargetLeader() const {
        return laneChange->targetLeader;
    }

    int lastLaneChangeDirection() const {
        return laneChange->lastDir;
    }

    int getLaneChangeDirection() const {
        if (!laneChange->signalSend) // laneChange = false
            return 0;
        return laneChange->signalSend->direction;
    }

    bool isChanging() const {
        return laneChange->changing;
    }

    double getMaxOffset() const {
        auto target = laneChange->signalSend->target;
        return (target->getWidth() + getCurLane()->getWidth()) / 2;
    }

    void abortLaneChange();

    void updateRoute();

    Road *getFirstRoad();

    void setFirstDrivable();

    bool isRouteValid() const {
        return this->routeValid;
    }

    Flow *getFlow() {
        return flow;
    }

    bool setRoute(const std::vector<Road *> &anchor);

    std::map<std::string, std::string> getInfo() const;
};

} // namespace CityFlow

#endif