#include "vehicle/lanechange.h"
#include "engine/engine.h"

#include <iostream>

namespace CityFlow {
LaneChange::LaneChange(Vehicle *vehicle, const LaneChange &other)
    : lastDir(other.lastDir), signalRecv(other.signalRecv), vehicle(vehicle), targetLeader(other.targetLeader),
      targetFollower(other.targetFollower), // useless in archive
      leaderGap(other.leaderGap), followerGap(other.followerGap), waitingTime(other.waitingTime), changing(other.changing),
      lastChangeTime(other.lastChangeTime) {
    if (other.signalSend) {
        signalSend = std::make_shared<Signal>(*other.signalSend);
        signalSend->source = vehicle;
    }
}

Lane *LaneChange::getTarget() const {
    assert(vehicle->getCurDrivable()->isLane());
    return signalSend ? signalSend->target : (Lane *)vehicle->getCurDrivable();
}

bool LaneChange::planChange() const { // Engine::laneChange = true && 找到目标 lane && target 设定后还未 laneChange || 正在 change
    return (signalSend && signalSend->target && signalSend->target != vehicle->getCurDrivable()) || changing;
}

void LaneChange::updateLeaderAndFollower() {
    targetLeader = targetFollower = nullptr;
    Lane *target = signalSend->target;
    targetLeader = target->getVehicleAfterDistance(vehicle->getDistance(), vehicle->getSegmentIndex()); // 换行后前面的车
    Lane *curLane = dynamic_cast<Lane *>(vehicle->getCurDrivable());
    leaderGap = followerGap = std::numeric_limits<double>::max();
    if (!targetLeader) { // ?
        // Find target leader in following lanelinks
        double rest = curLane->getLength() - vehicle->getDistance();
        leaderGap = rest;
        double gap = std::numeric_limits<double>::max();
        for (auto lanelink : signalSend->target->getLaneLinks()) {
            Vehicle *leader = lanelink->getLastVehicle();
            if (leader && leader->getDistance() + rest < gap) {
                gap = leader->getDistance() + rest;
                if (gap < leader->getLen()) {
                    targetLeader = leader;
                    leaderGap = rest - (leader->getLen() - gap);
                }
            }
        }
    } else {
        leaderGap = targetLeader->getDistance() - vehicle->getDistance() - targetLeader->getLen();
    }

    targetFollower = target->getVehicleBeforeDistance(vehicle->getDistance(), vehicle->getSegmentIndex());

    // TODO : potential bug here: a vehicle entering the lane is too close.

    if (targetFollower)
        followerGap = vehicle->getDistance() - targetFollower->getDistance() - vehicle->getLen();
    else
        followerGap = std::numeric_limits<double>::max();
}

double LaneChange::gapBefore() const { //?
    return followerGap;
}

double LaneChange::gapAfter() const { // ?
    return leaderGap;
}

void LaneChange::insertShadow(Vehicle *shadow) { // 对 shadow controllerInfor 信息进行补足，并在 targetLane 中更新 shadow 的信息
    assert(!changing);
    assert(vehicle->getOffset() == 0);
    changing = true; // 当前车在 changing
    waitingTime = 0; // 开始等待

    assert(vehicle->getCurDrivable()->isLane());
    Lane *targetLane = signalSend->target;
    int segId = vehicle->getSegmentIndex();
    auto targetSeg = targetLane->getSegment(segId);
    auto followerItr = (vehicle->getListIterator()); // ?
    followerItr++;

    shadow->setParent(vehicle); // 相互绑定
    vehicle->setShadow(shadow); // 相互绑定
    shadow->controllerInfo.blocker = nullptr;
    shadow->controllerInfo.drivable = targetLane; // 修改 shadow drivable
    shadow->controllerInfo.router.update();       // 更新 shadow iCurRoad 与 清除 planned

    auto targetFollowerItr = targetFollower ? targetFollower->getListIterator() : targetLane->getVehicles().end();

    auto newItr = targetLane->getVehicles().insert(targetFollowerItr, shadow); // targetLane 插入 vehicle

    targetSeg->insertVehicle(newItr); // 更新 targetLane 对应 segment

    shadow->updateLeaderAndGap(targetLeader); // 更新 shadow 与前车距离
    if (targetFollower)                       // 更新后车 与 shadow 距离
        targetFollower->updateLeaderAndGap(shadow);
}

int LaneChange::getDirection() {              // directoin 确定
    if (!vehicle->getCurDrivable()->isLane()) // 当前为 lane，直行
        return 0;
    Lane *curLane = dynamic_cast<Lane *>(vehicle->getCurDrivable());
    if (!signalSend) // 未初始化 signal 则直行（Engine::laneChange = false）
        return 0;
    if (!signalSend->target) // 未定目标 lane 则直行
        return 0;
    if (signalSend->target == curLane->getOuterLane()) // 转向外侧
        return 1;
    if (signalSend->target == curLane->getInnerLane()) // 转向内侧
        return -1;
    return 0;
}

void LaneChange::finishChanging() {
    changing = false;
    finished = true;
    lastChangeTime = vehicle->engine->getCurrentTime();
    Vehicle *partner = vehicle->getPartner();
    if (!partner->isReal())
        partner->setId(vehicle->getId());
    partner->laneChangeInfo.partnerType = 0;
    partner->laneChangeInfo.offset = 0;
    partner->laneChangeInfo.partner = nullptr;
    vehicle->laneChangeInfo.partner = nullptr;
    clearSignal();
}

void LaneChange::clearSignal() {
    targetLeader = nullptr;
    targetFollower = nullptr;
    if (signalSend)
        lastDir = signalSend->direction;
    else
        lastDir = 0;
    if (changing)
        return;
    signalSend = nullptr;
    signalRecv = nullptr;
}

void LaneChange::abortChanging() {
    Vehicle *partner = vehicle->getPartner();
    partner->laneChange->changing = false;
    partner->laneChangeInfo.partnerType = 0;
    partner->laneChangeInfo.offset = 0;
    partner->laneChangeInfo.partner = nullptr;
    clearSignal();
}

void SimpleLaneChange::makeSignal(double interval) { // 设置 signal 内各值并寻找目标 lane
    if (changing)                                    // change 中
        return;
    if (vehicle->engine->getCurrentTime() - lastChangeTime < coolingTime) // laneChange 冷却未完成
        return;
    signalSend = std::make_shared<Signal>();
    signalSend->source = vehicle;              // 来源是此车
    if (vehicle->getCurDrivable()->isLane()) { // 当前是 lane，处于 intersection 的 laneLink 时不应 laneChange
        Lane *curLane = (Lane *)vehicle->getCurDrivable();

        if (curLane->getLength() - vehicle->getDistance() < 30) // 将离开此 lane 则不 laneChange
            return;
        double curEst = vehicle->getGap(); // 与前车距离
        double outerEst = 0;
        double expectedGap = 2 * vehicle->getLen() + 4 * interval * vehicle->getMaxSpeed(); // 与前车的期望距离
        if (vehicle->getGap() > expectedGap || vehicle->getGap() < 1.5 * vehicle->getLen()) // 与前车距离过大或过小暂不 laneChange
            return;

        Router &router = vehicle->controllerInfo.router;
        if (curLane->getLaneIndex() <
            curLane->getBelongRoad()->getLanes().size() - 1) { // 当前不在 road 的最外侧，则测试外侧 lane 是否满足 laneChange 需求
            if (router.onLastRoad() || router.getNextDrivable(curLane->getOuterLane())) { // 已到 route 末尾或者外侧路满足通行要求
                outerEst = estimateGap(curLane->getOuterLane());                          // 与外侧前车间距
                if (outerEst > curEst + vehicle->getLen()) // 转向外侧要在一个车位内完成？所以要求外侧间距 > 当前间距 + 车长
                    signalSend->target = curLane->getOuterLane(); // signal 传向外侧
            }
        }

        if (curLane->getLaneIndex() > 0) { // 当前不在最内侧，则测试内侧 lane 是否满足 laneChange 需求
            if (router.onLastRoad() || router.getNextDrivable(curLane->getInnerLane())) {
                double innerEst = estimateGap(curLane->getInnerLane());
                if (innerEst > curEst + vehicle->getLen() && innerEst > outerEst) // 转向内侧间距更大
                    signalSend->target = curLane->getInnerLane();
            }
        }
        signalSend->urgency = 1;
    }
    LaneChange::makeSignal(interval); // 修改 direction
}

double SimpleLaneChange::yieldSpeed(double interval) {
    if (planChange())
        waitingTime += interval;
    if (signalRecv) {
        if (vehicle == signalRecv->source->getTargetLeader()) {
            return 100;
        } else {
            Vehicle *source = signalRecv->source;
            double srcSpeed = source->getSpeed();
            double gap = source->laneChange->gapBefore() - source->laneChange->safeGapBefore();

            double v = vehicle->getNoCollisionSpeed(srcSpeed, source->getMaxNegAcc(), vehicle->getSpeed(), vehicle->getMaxNegAcc(), gap, interval, 0);

            if (v < 0)
                v = 100;
            // If the follower is too fast, let it go.

            return v;
        }
    }
    return 100;
}

void SimpleLaneChange::sendSignal() { // targetLeader 与 targetFollower 接收 signal
    if (targetLeader)
        targetLeader->receiveSignal(vehicle);
    if (targetFollower)
        targetFollower->receiveSignal(vehicle);
}

double SimpleLaneChange::safeGapBefore() const {
    return targetFollower ? targetFollower->getMinBrakeDistance() : 0;
}

double SimpleLaneChange::safeGapAfter() const {
    return vehicle->getMinBrakeDistance();
}

double SimpleLaneChange::estimateGap(const Lane *lane) const { // 当前 vehicle 在 lane 上对应前车的间距
    int curSegIndex = vehicle->getSegmentIndex();
    Vehicle *leader = lane->getVehicleAfterDistance(vehicle->getDistance(), curSegIndex);
    if (!leader)
        return lane->getLength() - vehicle->getDistance();
    else
        return leader->getDistance() - vehicle->getDistance() - leader->getLen();
}

} // namespace CityFlow