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
    double yieldDistance = 5;                     // 让步距离（如果减速让别人过则与交叉点应有的距离）
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
        bool end;                   // 是否到达终点 / finishChange / abortChange
        size_t enterLaneLinkTime;   // 进入 LaneLink 的时间
        Vehicle *blocker = nullptr; // 被 blocker block
        double customSpeed;         // api 设置
        double deltaDis;            // 上次前进距离
    };
    struct LaneChangeInfo {    // laneChange 中自身相关信息记录
        short partnerType = 0; // 0 for no partner; 1 for real vehicle; 2 for shadow vehicle;
        Vehicle *partner = nullptr;
        double offset = 0;       // 偏移量，大于 maxOffset 说明已开到下一 lane 完成 laneChange
        size_t segmentIndex = 0; // 所在 segmentIndex
    };
    struct ControllerInfo {                     // 当前车辆行驶信息
        double dis = 0;                         // 当前 drivable 上形式的距离
        Drivable *drivable = nullptr;           // 当前所在的 drivable
        Drivable *prevDrivable = nullptr;       // 上一个 drivable
        double approachingIntersectionDistance; // 与 intersection 距离小于此值使判定为接近 intersection
        double gap;                             // 与 leader 间的距离
        size_t enterLaneLinkTime;               // 如果当前所在 drivable 为 laneLink, 则为进入的时间；否则为 理论最值
        Vehicle *leader = nullptr;              // 前车
        Vehicle *blocker = nullptr;             // 在 cross 处受 blocker 阻碍
        bool end = false;                       // 是否到达终点 / finishChange / abortChange
        bool running = false;                   // 正在行驶
        Router router;                          // 总体路径信息控制与记录
        ControllerInfo(Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd);
        ControllerInfo(Vehicle *vehicle, const ControllerInfo &other);
    };

    VehicleInfo vehicleInfo;
    Buffer buffer;
    LaneChangeInfo laneChangeInfo;
    ControllerInfo controllerInfo;

    int priority;     // 独有优先级，用于 VehiclePool
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

    void update(); // buffer 信息传入 controllerInfo

    // set / get
    void setDeltaDistance(double dis); // 由 dis 算出当前在哪条 drivable 上并更新 buffer

    void setSpeed(double speed);

    void setCustomSpeed(double speed) { // 设置 CustomSpeed，仅由 RL api 调用
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

    Point getPoint() const; // 获取 vehicle 当前坐标

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

    std::pair<Point, Point> getCurPos() const; // 获取 vehicle 头尾坐标

    ControlInfo getNextSpeed(double interval); //求解 interval 后的速度

    Drivable *getChangedDrivable() const; // 如 drivable 改变则返回新的 drivable

    double getEnterTime() const {
        return enterTime;
    }

    bool isEnd() const {
        return controllerInfo.end;
    }

    bool isIntersectionRelated(); // 是否已在 intersection 或将进入 intersection

    double getBrakeDistanceAfterAccel(double acc, double dec, double interval) const; // 在加速 interval 时间后减速到 0 需要的距离

    inline double getMinBrakeDistance() const { // 减速到 0 最短距离
        return 0.5 * vehicleInfo.speed * vehicleInfo.speed / vehicleInfo.maxNegAcc;
    }

    inline double getUsualBrakeDistance() const { // 常规减速到 0 所需距离
        return 0.5 * vehicleInfo.speed * vehicleInfo.speed / vehicleInfo.usualNegAcc;
    }

    double getNoCollisionSpeed(double vL, double dL, double vF, double dF, double gap, double interval, double targetGap) const; // 在给定数据下减速使最终距离为 targetGap，减速 interval 时间后的速度

    double getCarFollowSpeed(double interval); // 跟随速度

    double getStopBeforeSpeed(double distance, double interval) const; // 能在 distance 内停下时经过 interval 时间后的速度

    int getReachSteps(double distance, double targetSpeed, double acc) const; // 在 distance 内加速到 targetSpeed 所用时间段

    int getReachStepsOnLaneLink(double distance, LaneLink *laneLink) const; // 在 laneLink 上以最大可能行驶 distance 所用时间段

    double getDistanceUntilSpeed(double speed, double acc) const; // 以 acc 加速度加速到 speed 所需距离

    bool canYield(double dist) const; // 未到 cross 且能在 yield 范围前停住或已过 cross 且不覆盖 cross

    void updateLeaderAndGap(Vehicle *leader); // 更新 leader 与 gap

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

    double getIntersectionRelatedSpeed(double interval); // 将进入或已在 intersection 时的速度计算

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

    void setLane(Lane *nextLane); // 设置 controllerInfo.drivable

    void finishChanging(); // laneChange 完成，修改自身 laneChange 与 shadow 的 laneChangeInfo

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
    inline void makeLaneChangeSignal(double interval) { // 交由 laneChange 创建 signalSend 并设置 signal 内各值并寻找目标 lane
        laneChange->makeSignal(interval);
    }

    inline bool planLaneChange() { // 交由 laneChange 判断是否满足 laneChange 条件
        return laneChange->planChange();
    }

    void receiveSignal(Vehicle *sender); // laneChange signal 接收，按 priority 判断

    void sendSignal() { // 交由 laneChange 向 targetLeader 和 targetFollower 传递信号
        laneChange->sendSignal();
    }

    void clearSignal() {
        laneChange->clearSignal();
    }

    void updateLaneChangeNeighbor() { // 交由 laneChange 寻找 laneChange 后的 leader 与 follower
        laneChange->updateLeaderAndFollower();
    }

    std::shared_ptr<LaneChange> getLaneChange() {
        return laneChange;
    }

    std::list<Vehicle *>::iterator getListIterator();

    void insertShadow(Vehicle *shadow) { // 交由 laneChange 将 shadow 插入 targetLane
        laneChange->insertShadow(shadow);
    }

    bool onValidLane() const { // 交由 router，当无下一条路且 route 未到末尾说明有误
        return controllerInfo.router.onValidLane();
    }

    Lane *getValidLane() const { // nextLane
        assert(getCurDrivable()->isLane());
        return controllerInfo.router.getValidLane(dynamic_cast<Lane *>(getCurDrivable()));
    }

    bool canChange() const { // 交由 laneChange，自己发送了 signal 且未 receive 信号，如receive 说明 receive 信号优先级更高
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

    double getMaxOffset() const { // 最大偏移量，大于此量表示完成 laneChange
        auto target = laneChange->signalSend->target;
        return (target->getWidth() + getCurLane()->getWidth()) / 2; // 当前 lane 中心到 targetLane 中心的距离
    }

    void abortLaneChange(); // 由 shadow 调用，终止 laneChange

    void updateRoute(); // 用 updateShortestPath 更新 route

    Road *getFirstRoad();

    void setFirstDrivable(); // controllerInfo.drivable 初次设置

    bool isRouteValid() const {
        return this->routeValid;
    }

    Flow *getFlow() {
        return flow;
    }

    bool setRoute(const std::vector<Road *> &anchor); // 修改 controllerInfo.route

    std::map<std::string, std::string> getInfo() const;
};

} // namespace CityFlow

#endif