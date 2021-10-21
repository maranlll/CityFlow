#include "vehicle/vehicle.h"
#include "engine/engine.h"

#include <iostream>
#include <limits>
#include <random>

namespace CityFlow {

Vehicle::ControllerInfo::ControllerInfo(Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd) : router(vehicle, route, rnd) {
    enterLaneLinkTime = std::numeric_limits<int>::max();
}

Vehicle::ControllerInfo::ControllerInfo(Vehicle *vehicle, const Vehicle::ControllerInfo &other) : ControllerInfo(other) {
    router.setVehicle(vehicle);
}

Vehicle::Vehicle(const Vehicle &vehicle, Flow *flow)
    : vehicleInfo(vehicle.vehicleInfo), controllerInfo(this, vehicle.controllerInfo), laneChangeInfo(vehicle.laneChangeInfo), buffer(vehicle.buffer), priority(vehicle.priority), id(vehicle.id),
      engine(vehicle.engine), laneChange(std::make_shared<SimpleLaneChange>(this, *vehicle.laneChange)), flow(flow) {
    enterTime = vehicle.enterTime;
}

Vehicle::Vehicle(const Vehicle &vehicle, const std::string &id, Engine *engine, Flow *flow) // shadow 创建，除 laneChange（新建）和 flow（nullptr）与 router.vehicle 外全部一致
    : vehicleInfo(vehicle.vehicleInfo), controllerInfo(this, vehicle.controllerInfo), laneChangeInfo(vehicle.laneChangeInfo), buffer(vehicle.buffer), id(id), engine(engine),
      laneChange(std::make_shared<SimpleLaneChange>(this)), flow(flow) {
    while (engine->checkPriority(priority = engine->rnd()))
        ;
    controllerInfo.router.setVehicle(this); // 修改 route 的对应 vehicle
    enterTime = vehicle.enterTime;
}

Vehicle::Vehicle(const VehicleInfo &vehicleInfo, const std::string &id, Engine *engine, Flow *flow)
    : vehicleInfo(vehicleInfo), controllerInfo(this, vehicleInfo.route, &(engine->rnd)), id(id), engine(engine), laneChange(std::make_shared<SimpleLaneChange>(this)), flow(flow) {
    controllerInfo.approachingIntersectionDistance = vehicleInfo.maxSpeed * vehicleInfo.maxSpeed / vehicleInfo.usualNegAcc / 2 + vehicleInfo.maxSpeed * engine->getInterval() * 2;
    while (engine->checkPriority(priority = engine->rnd()))
        ;
    enterTime = engine->getCurrentTime();
}

void Vehicle::setDeltaDistance(double dis) {         // 由 dis 算出当前在哪条 drivable 上并更新 buffer
    if (!buffer.isDisSet || dis < buffer.deltaDis) { // 条件二为何会出现？
        unSetEnd();                                  // 用于条件二
        unSetDrivable();                             // 用于条件二
        buffer.deltaDis = dis;
        dis = dis + controllerInfo.dis; // 到目前所在 drivable 起点的总距离
        Drivable *drivable = getCurDrivable();
        for (int i = 0; drivable && dis > drivable->getLength(); ++i) {
            dis -= drivable->getLength();
            Drivable *nextDrivable = controllerInfo.router.getNextDrivable(i);
            if (nextDrivable == nullptr) { // 没有下一 drivable 且 dis 大于当前 drivable 长度，即此时已到达末尾
                assert(controllerInfo.router.isLastRoad(drivable));
                setEnd(true);
            }
            drivable = nextDrivable;
            setDrivable(drivable); // 新 drivable 存入 buffer
        }
        setDis(dis);
    }
}

void Vehicle::setSpeed(double speed) { // 速度设置
    buffer.speed = speed;
    buffer.isSpeedSet = true;
}

Drivable *Vehicle::getChangedDrivable() const { // 如 drivable 改变则返回新的 drivable
    if (!buffer.isDrivableSet)
        return nullptr;
    return buffer.drivable;
}

Point Vehicle::getPoint() const {                                                  // 获取 vehicle 当前坐标
    if (fabs(laneChangeInfo.offset) < eps || !controllerInfo.drivable->isLane()) { // 偏移量很小或者在 laneLink 上
        return controllerInfo.drivable->getPointByDistance(controllerInfo.dis);    // 直接由距离计算
    } else {                                                                       // 在 lane 上且 laneChange 进行中
        assert(controllerInfo.drivable->isLane());
        const Lane *lane = static_cast<const Lane *>(controllerInfo.drivable);
        Point origin = lane->getPointByDistance(controllerInfo.dis); // 未 laneChange 时位置
        Point next;
        double percentage;
        std::vector<Lane> &lans = lane->getBelongRoad()->getLanes();
        if (laneChangeInfo.offset > 0) {                                                                             // 向外侧便宜
            next = lans[lane->getLaneIndex() + 1].getPointByDistance(controllerInfo.dis);                            // 外侧同距离位置
            percentage = 2 * laneChangeInfo.offset / (lane->getWidth() + lans[lane->getLaneIndex() + 1].getWidth()); // 横向所占比例
        } else {
            next = lans[lane->getLaneIndex() - 1].getPointByDistance(controllerInfo.dis);
            percentage = -2 * laneChangeInfo.offset / (lane->getWidth() + lans[lane->getLaneIndex() - 1].getWidth());
        }
        Point cur;
        cur.x = next.x * percentage + origin.x * (1 - percentage); // 位置计算
        cur.y = next.y * percentage + origin.y * (1 - percentage);
        return cur;
    }
}

void Vehicle::update() { // TODO: use something like reflection?    buffer 信息导入 controllerInfo
    if (buffer.isEndSet) {
        controllerInfo.end = buffer.end;
        buffer.isEndSet = false;
    }
    if (buffer.isDisSet) {
        controllerInfo.dis = buffer.dis;
        buffer.isDisSet = false;
    }
    if (buffer.isSpeedSet) {
        vehicleInfo.speed = buffer.speed;
        buffer.isSpeedSet = false;
    }
    if (buffer.isCustomSpeedSet) {
        buffer.isCustomSpeedSet = false;
    }
    if (buffer.isDrivableSet) {
        controllerInfo.prevDrivable = controllerInfo.drivable;
        controllerInfo.drivable = buffer.drivable;
        buffer.isDrivableSet = false;
        controllerInfo.router.update();
    }
    if (buffer.isEnterLaneLinkTimeSet) {
        controllerInfo.enterLaneLinkTime = buffer.enterLaneLinkTime;
        buffer.isEnterLaneLinkTimeSet = false;
    }
    if (buffer.isBlockerSet) {
        controllerInfo.blocker = buffer.blocker;
        buffer.isBlockerSet = false;
    } else {
        controllerInfo.blocker = nullptr;
    }
    if (buffer.isNotifiedVehicles) {
        buffer.notifiedVehicles.clear();
        buffer.isNotifiedVehicles = false;
    }
}

std::pair<Point, Point> Vehicle::getCurPos() const {                                       // 获取 vehicle 头尾坐标
    std::pair<Point, Point> ret;                                                           // 头尾坐标
    ret.first = controllerInfo.drivable->getPointByDistance(controllerInfo.dis);           // 车头坐标
    Point direction = controllerInfo.drivable->getDirectionByDistance(controllerInfo.dis); // 行驶方向
    Point tail(ret.first);
    tail.x -= direction.x * vehicleInfo.len;
    tail.y -= direction.y * vehicleInfo.len;
    ret.second = tail; // 车尾坐标
    return ret;
}

void Vehicle::updateLeaderAndGap(Vehicle *leader) {                          // 更新 leader 与 gap
    if (leader != nullptr && leader->getCurDrivable() == getCurDrivable()) { // 传入 leader 且和当前车在同一 lane
        controllerInfo.leader = leader;
        controllerInfo.gap = leader->getDistance() - leader->getLen() - controllerInfo.dis;
    } else { // 当前车为 lane 首
        controllerInfo.leader = nullptr;
        Drivable *drivable = nullptr;
        Vehicle *candidateLeader = nullptr;
        double candidateGap = 0;
        double dis = controllerInfo.drivable->getLength() - controllerInfo.dis; // 距 lane 首距离
        for (int i = 0;; ++i) {                                                 // 在未来将驶向的 drivable 内搜寻 leader
            drivable = getNextDrivable(i);
            if (drivable == nullptr) // 已到 route 末尾，则无 leader
                return;
            if (drivable->isLaneLink()) { // if laneLink, check all laneLink start from previous lane, because lanelinks may overlap
                for (auto laneLink : static_cast<LaneLink *>(drivable)->getStartLane()->getLaneLinks()) {
                    if ((candidateLeader = laneLink->getLastVehicle()) != nullptr) {
                        candidateGap = dis + candidateLeader->getDistance() - candidateLeader->getLen();
                        if (controllerInfo.leader == nullptr || candidateGap < controllerInfo.gap) {
                            controllerInfo.leader = candidateLeader;
                            controllerInfo.gap = candidateGap;
                        }
                    }
                }
                if (controllerInfo.leader)
                    return;
            } else {
                if ((controllerInfo.leader = drivable->getLastVehicle()) != nullptr) { // drivable 中的 lastVehicle 为 leader
                    controllerInfo.gap = dis + controllerInfo.leader->getDistance() - controllerInfo.leader->getLen();
                    return;
                }
            }
            // 当前 drivable 中无车，再向前找
            dis += drivable->getLength();
            if (dis > vehicleInfo.maxSpeed * vehicleInfo.maxSpeed / vehicleInfo.usualNegAcc / 2 + vehicleInfo.maxSpeed * engine->getInterval() * 2) // ？多次寻找后 dis 距离过大，停止寻找
                return;
        }
        return;
    }
}

double Vehicle::getNoCollisionSpeed(double vL, double dL, double vF, double dF, double gap, double interval,
                                    double targetGap) const { // 在给定数据下减速使最终距离为 targetGap，减速 interval 时间后的速度
    double c = vF * interval / 2 + targetGap - 0.5 * vL * vL / dL - gap;
    double a = 0.5 / dF;
    double b = 0.5 * interval;
    if (b * b < 4 * a * c)
        return -100;
    double v1 = 0.5 / a * (sqrt(b * b - 4 * a * c) - b);
    double v2 = 2 * vL - dL * interval + 2 * (gap - targetGap) / interval;
    return min2double(v1, v2);
}

// should be move to seperate CarFollowing (Controller?) class later?
double Vehicle::getCarFollowSpeed(double interval) { // 跟随速度
    Vehicle *leader = getLeader();
    if (leader == nullptr)                                                      // 没有前车
        return hasSetCustomSpeed() ? buffer.customSpeed : vehicleInfo.maxSpeed; // 习惯速度/上限速度

    // collision free
    double v = getNoCollisionSpeed(leader->getSpeed(), leader->getMaxNegAcc(), vehicleInfo.speed, vehicleInfo.maxNegAcc, controllerInfo.gap, interval,
                                   0); // 极端情况下制动无碰撞

    if (hasSetCustomSpeed())
        return min2double(buffer.customSpeed, v); // 有习惯速度则以习惯速度

    // safe distance
    // get relative decel (mimic real scenario)
    double assumeDecel = 0, leaderSpeed = leader->getSpeed(); // 速度差
    if (vehicleInfo.speed > leaderSpeed) {
        assumeDecel = vehicleInfo.speed - leaderSpeed;
    }
    v = min2double(v, getNoCollisionSpeed(leader->getSpeed(), leader->getUsualNegAcc(), vehicleInfo.speed, vehicleInfo.usualNegAcc, controllerInfo.gap, interval,
                                          vehicleInfo.minGap)); // 常规情况下制动保持 minGap
    v = min2double(v, (controllerInfo.gap + (leaderSpeed + assumeDecel / 2) * interval - vehicleInfo.speed * interval / 2) / (vehicleInfo.headwayTime + interval / 2)); // ?

    return v;
}

double Vehicle::getStopBeforeSpeed(double distance, double interval) const { // 能在 distance 内停下时经过 interval 时间后的速度
    assert(distance >= 0);
    if (getBrakeDistanceAfterAccel(vehicleInfo.usualPosAcc, vehicleInfo.usualNegAcc, interval) < distance) // 如果加速 interval 时间后再减速距离依旧满足，那就加速
        return vehicleInfo.speed + vehicleInfo.usualPosAcc * interval;
    double takeInterval = 2 * distance / (vehicleInfo.speed + eps) / interval; // 在 distance 距离内减速到 0 需要几个 interval
    if (takeInterval >= 1) {
        return vehicleInfo.speed - vehicleInfo.speed / (int)takeInterval;
    } else {
        return vehicleInfo.speed - vehicleInfo.speed / takeInterval; // <0 ? (后续会判是否小于0)
    }
}

int Vehicle::getReachSteps(double distance, double targetSpeed, double acc) const { // 在 distance 内加速到 targetSpeed 所用时间段
    if (distance <= 0) {                                                            // 已到
        return 0;
    }
    if (vehicleInfo.speed > targetSpeed) {              // 当前速度大于目标速度
        return std::ceil(distance / vehicleInfo.speed); // 时间？？ 少除了 interval？
    }
    double distanceUntilTargetSpeed = getDistanceUntilSpeed(targetSpeed, acc); // 加速到 targetSpeed 距离
    double interval = engine->getInterval();
    if (distanceUntilTargetSpeed > distance) {                                                                                          // distance 内加速不到 targetSpeed
        return std::ceil((std::sqrt(vehicleInfo.speed * vehicleInfo.speed + 2 * acc * distance) - vehicleInfo.speed) / acc / interval); // 在 distance 内加到最终速所用时间段
    } else {
        return std::ceil((targetSpeed - vehicleInfo.speed) / acc / interval) + std::ceil((distance - distanceUntilTargetSpeed) / targetSpeed / interval); // distance 内加速并匀速所用时间段
    }
}

int Vehicle::getReachStepsOnLaneLink(double distance, LaneLink *laneLink) const { // 在 laneLink 上以最大可能行驶 distance 所用时间段
    return getReachSteps(distance, laneLink->isTurn() ? vehicleInfo.turnSpeed : vehicleInfo.maxSpeed, vehicleInfo.usualPosAcc);
}

double Vehicle::getDistanceUntilSpeed(double speed, double acc) const { // 以 acc 加速度加速到 speed 所需距离
    if (speed <= vehicleInfo.speed)                                     // 已到
        return 0;
    double interval = engine->getInterval();
    int stage1steps = std::floor((speed - vehicleInfo.speed) / acc / interval);            // 加速到 speed 需要的 step 数
    double stage1speed = vehicleInfo.speed + stage1steps * acc / interval;                 // 最终速度
    double stage1dis = (vehicleInfo.speed + stage1speed) * (stage1steps * interval) / 2;   // 初加速阶段行驶距离
    return stage1dis + (stage1speed < speed ? ((stage1speed + speed) * interval / 2) : 0); // 总距离
}

bool Vehicle::canYield(double dist) const { // 未到 cross 且能在 yield 范围前停住或已过 cross 且不覆盖 cross
    return (dist > 0 && getMinBrakeDistance() < dist - vehicleInfo.yieldDistance) || (dist < 0 && dist + vehicleInfo.len < 0);
}

bool Vehicle::isIntersectionRelated() { // 是否已在 intersection 或将进入 intersection
    if (controllerInfo.drivable->isLaneLink())
        return true;
    if (controllerInfo.drivable->isLane()) {
        Drivable *drivable = getNextDrivable();
        if (drivable && drivable->isLaneLink() && controllerInfo.drivable->getLength() - controllerInfo.dis <= controllerInfo.approachingIntersectionDistance) {
            return true;
        }
    }
    return false;
}

double Vehicle::getBrakeDistanceAfterAccel(double acc, double dec, double interval) const { // 在加速 interval 时间后减速到 0 需要的距离
    double currentSpeed = vehicleInfo.speed;
    double nextSpeed = currentSpeed + acc * interval;
    return (currentSpeed + nextSpeed) * interval / 2 + (nextSpeed * nextSpeed / dec / 2);
}

ControlInfo Vehicle::getNextSpeed(double interval) { // TODO: pass as parameter or not? 求解 interval 后的速度
    ControlInfo controlInfo;
    Drivable *drivable = controllerInfo.drivable;
    double v = vehicleInfo.maxSpeed;                                         // 上限速度
    v = min2double(v, vehicleInfo.speed + vehicleInfo.maxPosAcc * interval); // TODO: random??? 当前速度能加到的最快速度

    v = min2double(v, drivable->getMaxSpeed()); // 道路限速

    // car follow
    v = min2double(v, getCarFollowSpeed(interval)); // 跟随速度

    if (isIntersectionRelated()) {
        v = min2double(v, getIntersectionRelatedSpeed(interval)); // 过 intersection 速度
    }

    if (laneChange) {
        v = min2double(v, laneChange->yieldSpeed(interval));                                                                                         // 让步速度
        if (!controllerInfo.router.onValidLane()) {                                                                                                  // 上错路了 ？
            double vn = getNoCollisionSpeed(0, 1, getSpeed(), getMaxNegAcc(), getCurDrivable()->getLength() - getDistance(), interval, getMinGap()); // 刹车
            v = min2double(v, vn);
        }
    }

    v = max2double(v, vehicleInfo.speed - vehicleInfo.maxNegAcc * interval); // 能减到的最小速度
    controlInfo.speed = v;

    return controlInfo;
}

double Vehicle::getIntersectionRelatedSpeed(double interval) { // 将进入或已在 intersection 时的速度计算
    double v = vehicleInfo.maxSpeed;                           // 最大速度
    Drivable *nextDrivable = getNextDrivable();
    const LaneLink *laneLink = nullptr;
    if (nextDrivable && nextDrivable->isLaneLink()) { // 即将进入 intersection
        laneLink = (LaneLink *)nextDrivable;
        if (!laneLink->isAvailable() || !laneLink->getEndLane()->canEnter(this)) { // not only the first vehicle should follow intersection logic  由于红灯或 endLane 车辆过多而不可通行
            if (getMinBrakeDistance() > controllerInfo.drivable->getLength() - controllerInfo.dis) { // 无法在线前刹车
                // TODO: what if it cannot brake before red light?
            } else {
                v = min2double(v, getStopBeforeSpeed(controllerInfo.drivable->getLength() - controllerInfo.dis, interval)); // 能停下的话经过 interval 时间的速度
                return v;
            }
        }
        if (laneLink->isTurn()) {                     // 绿灯转弯限速
            v = min2double(v, vehicleInfo.turnSpeed); // TODO: define turn speed
        }
    }
    if (laneLink == nullptr && controllerInfo.drivable->isLaneLink())      // 已在 intersection
        laneLink = static_cast<const LaneLink *>(controllerInfo.drivable); // 获取当前 laneLink
    double distanceToLaneLinkStart = controllerInfo.drivable->isLane() ? -(controllerInfo.drivable->getLength() - controllerInfo.dis)
                                                                       : controllerInfo.dis; // vehicle 距离 laneLink start 的 距离 <0 表示在 laneLink 前，>0 在 laneLink 后
    double distanceOnLaneLink;
    for (auto &cross : laneLink->getCrosses()) {                 // 对当前 laneLink 上每个 cross
        distanceOnLaneLink = cross->getDistanceByLane(laneLink); // cross 距 laneLink 起点距离
        if (distanceOnLaneLink < distanceToLaneLinkStart)        // 车头已过此 cross，说明先前已对当前 cross 进行了 canPass 判断，无需再考虑
            continue;
        if (!cross->canPass(this, laneLink, distanceToLaneLinkStart)) { // 当前不可通过
            v = min2double(v, getStopBeforeSpeed(distanceOnLaneLink - distanceToLaneLinkStart - vehicleInfo.yieldDistance,
                                                 interval)); // TODO: headway distance  能停下的话经过 interval 时间的速度
            setBlocker(cross->getFoeVehicle(laneLink));      // 被 block
            break;
        }
    }
    return v;
}

void Vehicle::finishChanging() { // laneChange 完成，修改自身 laneChange 与 shadow 的 laneChangeInfo
    laneChange->finishChanging();
    setEnd(true);
}

void Vehicle::setLane(Lane *nextLane) { // 设置 controllerInfo.drivable
    controllerInfo.drivable = nextLane;
}

Drivable *Vehicle::getCurDrivable() const {
    return controllerInfo.drivable;
}

void Vehicle::receiveSignal(Vehicle *sender) { // laneChange signal 接收，按 priority 判断
    if (laneChange->changing)                  // 当前车正在 langChange，无视
        return;
    auto signal_recv = laneChange->signalRecv;                               // 之前接收的 signal
    auto signal_send = laneChange->signalSend;                               // 自己发送的 signal
    int curPriority = signal_recv ? signal_recv->source->getPriority() : -1; // 获取之前 receiveSignal 来源 vehicle 的 priority
    int newPriority = sender->getPriority();                                 // 当前 receiveSignal 来源 vehicle 的 priority

    if ((!signal_recv || curPriority < newPriority) && (!signal_send || priority < newPriority)) // （尚未接收 || sender 的优先级更高更高） && (自己未发 || sender 优先级更高)
        laneChange->signalRecv = sender->laneChange->signalSend;                                 // 更换 signalRecv
}

std::list<Vehicle *>::iterator Vehicle::getListIterator() {
    assert(getCurDrivable()->isLane());
    Segment *seg = ((Lane *)getCurDrivable())->getSegment(getSegmentIndex());

    auto result = seg->findVehicle(this);
    return result;
}

void Vehicle::abortLaneChange() { // 由 shadow 调用，终止 laneChange
    assert(laneChangeInfo.partner);
    this->setEnd(true);
    laneChange->abortChanging();
}

Road *Vehicle::getFirstRoad() {
    return controllerInfo.router.getFirstRoad();
}

void Vehicle::setFirstDrivable() { // controllerInfo.drivable 初次设置
    controllerInfo.drivable = controllerInfo.router.getFirstDrivable();
}

void Vehicle::updateRoute() { // 用 updateShortestPath 更新 route
    routeValid = controllerInfo.router.updateShortestPath();
}

bool Vehicle::setRoute(const std::vector<Road *> &anchor) {
    return controllerInfo.router.setRoute(anchor);
}

std::map<std::string, std::string> Vehicle::getInfo() const { //// 对应 id 车辆信息获取 <title, info>
    git std::map<std::string, std::string> info;
    info["running"] = std::to_string(isRunning());
    if (!isRunning())
        return info;

    info["distance"] = std::to_string(getDistance());
    info["speed"] = std::to_string(getSpeed());
    const auto &drivable = getCurDrivable();
    info["drivable"] = drivable->getId();
    const auto &road = drivable->isLane() ? getCurLane()->getBelongRoad() : nullptr;
    if (road) {
        info["road"] = road->getId();
        info["intersection"] = road->getEndIntersection().getId();
    }
    // add routing info
    std::string route;
    for (const auto &r : controllerInfo.router.getFollowingRoads()) {
        route += r->getId() + " ";
    }
    info["route"] = route;

    return info;
}
} // namespace CityFlow
