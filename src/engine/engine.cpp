#include "engine/engine.h"
#include "utility/utility.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>

#include <ctime>
namespace CityFlow {

Engine::Engine(const std::string &configFile, int threadNum) : threadNum(threadNum), startBarrier(threadNum + 1), endBarrier(threadNum + 1) {
    for (int i = 0; i < threadNum; i++) { // 各线程池建立
        threadVehiclePool.emplace_back();
        threadRoadPool.emplace_back();
        threadIntersectionPool.emplace_back();
        threadDrivablePool.emplace_back();
    }
    bool success = loadConfig(configFile); // 读取，初始化 Engine 并向线程池填入信息
    if (!success) {
        std::cerr << "load config failed!" << std::endl;
    }
    for (int i = 0; i < threadNum; i++) { // 线程创建
        threadPool.emplace_back(&Engine::threadController, this, std::ref(threadVehiclePool[i]), std::ref(threadRoadPool[i]),
                                std::ref(threadIntersectionPool[i]), std::ref(threadDrivablePool[i]));
    }
}

bool Engine::loadConfig(const std::string &configFile) { // 主 load， load Engine
    rapidjson::Document document;
    if (!readJsonFromFile(configFile, document)) {
        std::cerr << "cannot open config file!" << std::endl;
        return false;
    }

    if (!document.IsObject()) {
        std::cerr << "wrong format of config file" << std::endl;
        return false;
    }

    try {
        // Engine 信息读取
        interval = getJsonMember<double>("interval", document);
        warnings = false;
        rlTrafficLight = getJsonMember<bool>("rlTrafficLight", document);
        laneChange = getJsonMember<bool>("laneChange", document, false);
        seed = getJsonMember<int>("seed", document);
        rnd.seed(seed);
        dir = getJsonMember<const char *>("dir", document);
        std::string roadnetFile = getJsonMember<const char *>("roadnetFile", document);
        std::string flowFile = getJsonMember<const char *>("flowFile", document);

        // load RoadNet
        if (!loadRoadNet(dir + roadnetFile)) {
            std::cerr << "loading roadnet file error!" << std::endl;
            return false;
        }

        // load Flow
        if (!loadFlow(dir + flowFile)) {
            std::cerr << "loading flow file error!" << std::endl;
            return false;
        }

        if (warnings)
            checkWarning();

        saveReplayInConfig = saveReplay = getJsonMember<bool>("saveReplay", document);

        if (saveReplay) {
            std::string roadnetLogFile = getJsonMember<const char *>("roadnetLogFile", document);
            std::string replayLogFile = getJsonMember<const char *>("replayLogFile", document);
            setLogFile(dir + roadnetLogFile, dir + replayLogFile);
        }
    } catch (const JsonFormatError &e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    stepLog = "";
    return true;
}

bool Engine::loadRoadNet(const std::string &jsonFile) { // load RoadNet，并向线程池填入信息
    bool ans = roadnet.loadFromJson(jsonFile);
    int cnt = 0;
    for (Road &road : roadnet.getRoads()) {
        threadRoadPool[cnt].push_back(&road);
        cnt = (cnt + 1) % threadNum;
    }
    for (Intersection &intersection : roadnet.getIntersections()) {
        threadIntersectionPool[cnt].push_back(&intersection);
        cnt = (cnt + 1) % threadNum;
    }
    for (Drivable *drivable : roadnet.getDrivables()) {
        threadDrivablePool[cnt].push_back(drivable);
        cnt = (cnt + 1) % threadNum;
    }
    jsonRoot.SetObject();
    jsonRoot.AddMember("static", roadnet.convertToJson(jsonRoot.GetAllocator()), jsonRoot.GetAllocator());
    return ans;
}

bool Engine::loadFlow(const std::string &jsonFilename) { // load Flow
    rapidjson::Document root;
    if (!readJsonFromFile(jsonFilename, root)) {
        std::cerr << "cannot open flow file!" << std::endl;
        return false;
    }
    std::list<std::string> path;
    try {
        if (!root.IsArray())
            throw JsonTypeError("flow file", "array");
        for (rapidjson::SizeType i = 0; i < root.Size(); i++) {
            path.emplace_back("flow[" + std::to_string(i) + "]");
            rapidjson::Value &flow = root[i];
            std::vector<Road *> roads;
            const auto &routes = getJsonMemberArray("route", flow);
            roads.reserve(routes.Size());
            for (auto &route : routes.GetArray()) {
                path.emplace_back("route[" + std::to_string(roads.size()) + "]");
                if (!route.IsString())
                    throw JsonTypeError("route", "string");
                std::string roadName = route.GetString();
                auto road = roadnet.getRoadById(roadName);
                if (!road)
                    throw JsonFormatError("No such road: " + roadName);
                roads.push_back(road);
                path.pop_back();
            }
            auto route = std::make_shared<const Route>(roads);

            VehicleInfo vehicleInfo;
            const auto &vehicle = getJsonMemberObject("vehicle", flow);
            vehicleInfo.len = getJsonMember<double>("length", vehicle);
            vehicleInfo.width = getJsonMember<double>("width", vehicle);
            vehicleInfo.maxPosAcc = getJsonMember<double>("maxPosAcc", vehicle);
            vehicleInfo.maxNegAcc = getJsonMember<double>("maxNegAcc", vehicle);
            vehicleInfo.usualPosAcc = getJsonMember<double>("usualPosAcc", vehicle);
            vehicleInfo.usualNegAcc = getJsonMember<double>("usualNegAcc", vehicle);
            vehicleInfo.minGap = getJsonMember<double>("minGap", vehicle);
            vehicleInfo.maxSpeed = getJsonMember<double>("maxSpeed", vehicle);
            vehicleInfo.headwayTime = getJsonMember<double>("headwayTime", vehicle);
            vehicleInfo.route = route;
            int startTime = getJsonMember<int>("startTime", flow, 0);
            int endTime = getJsonMember<int>("endTime", flow, -1);
            Flow newFlow(vehicleInfo, getJsonMember<double>("interval", flow), this, startTime, endTime, "flow_" + std::to_string(i));
            flows.push_back(newFlow);
            path.pop_back();
        }
        assert(path.empty());
    } catch (const JsonFormatError &e) {
        std::cerr << "Error occurred when reading flow file" << std::endl;
        for (const auto &node : path) {
            std::cerr << "/" << node;
        }
        std::cerr << " " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool Engine::checkWarning() { // check data
    bool result = true;

    if (interval < 0.2 || interval > 1.5) {
        std::cerr << "Deprecated time interval, recommended interval between 0.2 and 1.5" << std::endl;
        result = false;
    }

    for (Lane *lane : roadnet.getLanes()) {
        if (lane->getLength() < 50) {
            std::cerr << "Deprecated road length, recommended road length at least "
                         "50 meters"
                      << std::endl;
            result = false;
        }
        if (lane->getMaxSpeed() > 30) {
            std::cerr << "Deprecated road max speed, recommended max speed at most "
                         "30 meters/s"
                      << std::endl;
            result = false;
        }
    }

    return result;
}

// 线程函数
void Engine::threadController(std::set<Vehicle *> &vehicles, std::vector<Road *> &roads, std::vector<Intersection *> &intersections,
                              std::vector<Drivable *> &drivables) { // 线程函数
    while (!finished) {
        threadPlanRoute(roads); // 对 planBuffer 中的每个 vehicle 求最短路 route 与 route 是否有效
        if (laneChange) {
            threadInitSegments(roads); // 更新 lane 内各 segment 所含的 vehicle 信息
            threadPlanLaneChange(vehicles);
            threadUpdateLeaderAndGap(drivables);
        }
        threadNotifyCross(intersections);
        threadGetAction(vehicles);
        threadUpdateLocation(drivables);
        threadUpdateAction(vehicles);
        threadUpdateLeaderAndGap(drivables);
    }
}

void Engine::threadPlanRoute(const std::vector<Road *> &roads) { // 对 planBuffer 中的每个 vehicle 求最短路 route与 route 是否有效
    startBarrier.wait();
    for (auto &road : roads) {
        for (auto &vehicle : road->getPlanRouteBuffer()) {
            vehicle->updateRoute(); // 对每个 routeBuffer 中的 buffer 根据 anchorpoint 生成最短路 route，求 vehicle 是否 controllerInfor.routeValid
        }
    }
    endBarrier.wait();
}

void Engine::threadInitSegments(const std::vector<Road *> &roads) { // 更新 lane 内各 segment 所含的 vehicle 信息
    startBarrier.wait();
    for (Road *road : roads)
        for (Lane &lane : road->getLanes()) {
            lane.initSegments();
        }
    endBarrier.wait();
}

void Engine::threadPlanLaneChange(const std::set<CityFlow::Vehicle *> &vehicles) { // 对 vehicles 判断是否可 laneChange
    startBarrier.wait();
    std::vector<CityFlow::Vehicle *> buffer;

    for (auto vehicle : vehicles)
        if (vehicle->isRunning() && vehicle->isReal()) { // 车辆行驶中且非 shadow
            vehicle->makeLaneChangeSignal(interval);     // laneChange 判断并寻找 target
            if (vehicle->planLaneChange()) {             // 待转
                buffer.emplace_back(vehicle);
            }
        }
    {
        std::lock_guard<std::mutex> guard(lock);                                                   // 线程互斥锁
        laneChangeNotifyBuffer.insert(laneChangeNotifyBuffer.end(), buffer.begin(), buffer.end()); // 塞入 buffer
    }
    endBarrier.wait();
}

void Engine::threadUpdateLeaderAndGap(const std::vector<Drivable *> &drivables) { // 更新 drivable 上每辆车与前车的距离
    startBarrier.wait();
    for (Drivable *drivable : drivables) {
        Vehicle *leader = nullptr;
        for (Vehicle *vehicle : drivable->getVehicles()) {
            vehicle->updateLeaderAndGap(leader); // 每辆车与前车距离更新
            leader = vehicle;
        }
        if (drivable->isLane()) { // 更新 lane 的 historyRecord
            static_cast<Lane *>(drivable)->updateHistory();
        }
    }
    endBarrier.wait();
}

void Engine::threadNotifyCross(
    const std::vector<Intersection *> &intersections) { // 为每一个 cross 填入两个 notifyVehicle (为啥不填入最近的 vehicle ？)
    // TODO: iterator for laneLink
    startBarrier.wait();
    for (Intersection *intersection : intersections)
        for (Cross &cross : intersection->getCrosses())
            cross.clearNotify();

    for (Intersection *intersection : intersections)
        for (LaneLink *laneLink : intersection->getLaneLinks()) {
            // XXX: no cross in laneLink?
            const auto &crosses = laneLink->getCrosses();
            auto rIter = crosses.rbegin();

            // first check the vehicle on the end lane
            Vehicle *vehicle = laneLink->getEndLane()->getLastVehicle();
            if (vehicle && static_cast<LaneLink *>(vehicle->getPrevDrivable()) == laneLink) { // 存在且是从此 laneLink 来的
                double vehDistance = vehicle->getDistance() - vehicle->getLen();              // vehicle 距离 endLane 起点距离
                while (rIter != crosses.rend()) {
                    double crossDistance = laneLink->getLength() - (*rIter)->getDistanceByLane(laneLink); // cross 距 laneLink 终点
                    if (crossDistance + vehDistance < (*rIter)->getLeaveDistance()) { // vehicle 距 cross 的距离小于 leaveDistance
                        (*rIter)->notify(laneLink, vehicle, -(vehicle->getDistance() + crossDistance)); // 信息填入此 cross
                        ++rIter;
                    } else
                        break;
                }
            }

            // check each vehicle on laneLink
            for (Vehicle *linkVehicle : laneLink->getVehicles()) {
                double vehDistance = linkVehicle->getDistance();

                while (rIter != crosses.rend()) {
                    double crossDistance = (*rIter)->getDistanceByLane(laneLink);
                    if (vehDistance > crossDistance) {                                                             // vehicle 已过 cross
                        if (vehDistance - crossDistance - linkVehicle->getLen() <= (*rIter)->getLeaveDistance()) { // 在 leaveDistance 范围内
                            (*rIter)->notify(laneLink, linkVehicle, crossDistance - vehDistance);                  // notifyCross
                        } else                                                                                     // 用更近的 vehicle
                            break;
                    } else { // vehicle 未过 cross
                        (*rIter)->notify(laneLink, linkVehicle, crossDistance - vehDistance);
                    }
                    ++rIter;
                }
            }

            // check vehicle on the incoming lane（laneLink 上车已经检查完成但仍有 cross 未 notify）
            vehicle = laneLink->getStartLane()->getFirstVehicle();
            if (vehicle && static_cast<LaneLink *>(vehicle->getNextDrivable()) == laneLink && laneLink->isAvailable()) { // 将走向此 laneLink
                double vehDistance = laneLink->getStartLane()->getLength() - vehicle->getDistance();
                while (rIter != crosses.rend()) {
                    (*rIter)->notify(laneLink, vehicle, vehDistance + (*rIter)->getDistanceByLane(laneLink));
                    ++rIter;
                }
            }
        }
    endBarrier.wait();
}

void Engine::threadGetAction(std::set<Vehicle *> &vehicles) { // 各类数据计算
    startBarrier.wait();
    std::vector<std::pair<Vehicle *, double>> buffer;
    for (auto vehicle : vehicles)
        if (vehicle->isRunning())
            vehicleControl(*vehicle, buffer); // speed、dis 计算，offset 计算并完成 laneChange
    {
        std::lock_guard<std::mutex> guard(lock);
        pushBuffer.insert(pushBuffer.end(), buffer.begin(), buffer.end());
    }
    endBarrier.wait();
}

void Engine::threadUpdateLocation(const std::vector<Drivable *> &drivables) { // 从各 drivable 去除离开的或被 abort 的 vehicle
    startBarrier.wait();
    for (Drivable *drivable : drivables) {
        auto &vehicles = drivable->getVehicles();
        auto vehicleItr = vehicles.begin();
        while (vehicleItr != vehicles.end()) {
            Vehicle *vehicle = *vehicleItr;

            if ((vehicle->getChangedDrivable()) != nullptr ||
                vehicle->hasSetEnd()) {                  // 该车已移动到下一个 drivable 或 vehicle.finishChange 或 shadow.abortChange
                vehicleItr = vehicles.erase(vehicleItr); // 从此 drivable 清除
            } else {
                vehicleItr++;
            }

            if (vehicle->hasSetEnd()) { // vehicle.finishChange 或 shadow.abortChange
                std::lock_guard<std::mutex> guard(lock);
                vehicleRemoveBuffer.insert(vehicle);
                if (!vehicle->getLaneChange()->hasFinished()) { // shadow.abortChange
                    vehicleMap.erase(vehicle->getId());
                    finishedVehicleCnt += 1; // ? 为啥 shadow 要被记录
                    cumulativeTravelTime += getCurrentTime() - vehicle->getEnterTime();
                }
                auto iter = vehiclePool.find(vehicle->getPriority());
                threadVehiclePool[iter->second.second].erase(vehicle);
                //                    assert(vehicle->getPartner() == nullptr);
                delete vehicle;
                vehiclePool.erase(iter);
                activeVehicleCount--;
            }
        }
    }
    endBarrier.wait();
}

void Engine::threadUpdateAction(std::set<Vehicle *> &vehicles) { // vehicle 信息更新，并将 buffer 信息移入 vehicle.controllerInfo
    startBarrier.wait();
    for (auto vehicle : vehicles)
        if (vehicle->isRunning()) {
            if (vehicleRemoveBuffer.count(vehicle->getBufferBlocker())) { // blocker 被移除
                vehicle->setBlocker(nullptr);
            }

            vehicle->update();      // vehicle.buffer 信息移入 vehicle.controllerInfo
            vehicle->clearSignal(); // 清空信号
        }
    endBarrier.wait();
}

// 主进程函数

void Engine::planRoute() { // 待线程处理完成后将有效 vehicle 转入 lane 的 waitingBuffer
    startBarrier.wait();
    endBarrier.wait();
    for (auto &road : roadnet.getRoads()) {
        for (auto &vehicle : road.getPlanRouteBuffer())
            if (vehicle->isRouteValid()) {                          // vehicle.routeValid = true
                vehicle->setFirstDrivable();                        // vehicle controllerInfor.drivable 设置
                vehicle->getCurLane()->pushWaitingVehicle(vehicle); // 放入 lane 的 buffer
            } else {
                Flow *flow = vehicle->getFlow();
                if (flow)
                    flow->setValid(false); // flow.valid = false, 以后 skip 此 flow

                // remove this vehicle
                auto iter = vehiclePool.find(vehicle->getPriority());
                threadVehiclePool[iter->second.second].erase(vehicle);
                delete vehicle;
                vehiclePool.erase(iter);
            }
        road.clearPlanRouteBuffer();
    }
}

void Engine::handleWaiting() { // 对每个 lane 的 waitingBuffer，判断 buffer 首是否可入 lane；如不可，则等下一个 interval阶段
    for (Lane *lane : roadnet.getLanes()) {
        auto &buffer = lane->getWaitingBuffer();
        if (buffer.empty())
            continue;
        auto &vehicle = buffer.front();
        if (lane->available(vehicle)) { // 可进入
            vehicle->setRunning(true);  // 车开跑
            activeVehicleCount += 1;    // 运行车辆 +1
            Vehicle *tail = lane->getLastVehicle();
            lane->pushVehicle(vehicle);        // 入 lane
            vehicle->updateLeaderAndGap(tail); // 更新这个新入 lane 的 vehicle 与前车的距离
            buffer.pop_front();
        }
    }
}

void Engine::initSegments() { // segment 内车辆信息更新
    startBarrier.wait();
    endBarrier.wait();
}

void Engine::planLaneChange() { // 线程判断是否可 laneChange
    startBarrier.wait();
    endBarrier.wait();
    scheduleLaneChange(); // 对满足 laneChange 要求的 insert shadow
}

void Engine::scheduleLaneChange() { // 对 notifyBuffer 内的 vehicle 进行 insertShadow 操作
    std::sort(laneChangeNotifyBuffer.begin(), laneChangeNotifyBuffer.end(),
              [](Vehicle *a, Vehicle *b) { return a->laneChangeUrgency() > b->laneChangeUrgency(); }); // 按 urgency 排序，似乎全是 1？
    for (auto v : laneChangeNotifyBuffer) {
        v->updateLaneChangeNeighbor(); // 找当前车的 targetLeader 与 targetFollower
        v->sendSignal();               // targetLeader 和 targetFollower 接收 signal 并根据 vehicle priority 判断是否接受或覆盖
        // Lane Change
        // Insert a shadow vehicle
        if (v->planLaneChange() && v->canChange() && !v->isChanging()) { // 可以 laneChange 且优先级最高
            std::shared_ptr<LaneChange> lc = v->getLaneChange();
            if (lc->isGapValid() && v->getCurDrivable()->isLane()) { // 满足安全距离可 laneChange 且 当前在 lane 上
                // std::cerr << getCurrentTime() << " " << v->getId() << " dis: " << v->getDistance() << " Can Change from"
                //<< ((Lane *)v->getCurDrivable())->getId() << " to " << lc->getTarget()->getId() << std::endl;
                insertShadow(v); // 目标 lane 上设置 shadow
            }
        }
    }
    laneChangeNotifyBuffer.clear();
}

void Engine::updateLeaderAndGap() { // 更新每个 drivable 上车辆的 gap，交由线程完成
    startBarrier.wait();
    endBarrier.wait();
}

void Engine::notifyCross() { // 更新每个 cross 的 notifyVehicile，交由线程完成
    startBarrier.wait();
    endBarrier.wait();
}

void Engine::getAction() { // 线程负责数据计算计算
    startBarrier.wait();
    endBarrier.wait();
}

void Engine::vehicleControl(Vehicle &vehicle, std::vector<std::pair<Vehicle *, double>> &buffer) { // speed、dis 计算，offset 计算并完成 laneChange
    double nextSpeed;
    if (vehicle.hasSetSpeed()) //已作为 partner 被设定过速度
        nextSpeed = vehicle.getBufferSpeed();
    else
        nextSpeed = vehicle.getNextSpeed(interval).speed;

    if (laneChange) {
        Vehicle *partner = vehicle.getPartner();
        if (partner != nullptr && !partner->hasSetSpeed()) {             // partner 尚未进行 vehicleControl，同步速度
            double partnerSpeed = partner->getNextSpeed(interval).speed; // partner 角度同步运行
            nextSpeed = min2double(nextSpeed, partnerSpeed);
            partner->setSpeed(nextSpeed);

            if (partner->hasSetEnd()) // ？
                vehicle.setEnd(true);
        }
    }

    if (vehicle.getPartner()) {
        assert(vehicle.getDistance() == vehicle.getPartner()->getDistance());
    }

    double deltaDis, speed = vehicle.getSpeed();

    if (nextSpeed < 0) {                                         // 存在先前计算导致速度为负时实际情况为停车后倒车
        deltaDis = 0.5 * speed * speed / vehicle.getMaxNegAcc(); // 到停车为止
        nextSpeed = 0;
    } else {
        deltaDis = (speed + nextSpeed) * interval / 2;
    }
    vehicle.setSpeed(nextSpeed);        // speed 设置
    vehicle.setDeltaDistance(deltaDis); // drivable、dis 设置

    if (laneChange) {
        if (!vehicle.isReal() && vehicle.getChangedDrivable() != nullptr) { // 当前为 shadow 在此阶段行驶 deltaDis 后发生 drivable 变化
            vehicle.abortLaneChange();                                      // 放弃此次 laneChange 并清除 partner laneChange 状态
        }

        if (vehicle.isChanging()) {
            assert(vehicle.isReal());

            int dir = vehicle.getLaneChangeDirection();                                                     // 0:直行，1:outerLane，-1:innerLane
            double newOffset = fabs(vehicle.getOffset() + max2double(0.2 * nextSpeed, 1) * interval * dir); // 横向偏移量计算
            newOffset = min2double(newOffset, vehicle.getMaxOffset());
            vehicle.setOffset(newOffset * dir); // 更新偏移量

            if (newOffset >= vehicle.getMaxOffset()) {              // laneChange 完成
                std::lock_guard<std::mutex> guard(lock);            // 互斥锁
                vehicleMap.erase(vehicle.getPartner()->getId());    // 清除 shadow
                vehicleMap[vehicle.getId()] = vehicle.getPartner(); // 完成 laneChange
                vehicle.finishChanging();                           // change = false
            }
        }
    }

    if (!vehicle.hasSetEnd() && vehicle.hasSetDrivable()) {    // 发生 drivable 变动且此时尚未到达 end
        buffer.emplace_back(&vehicle, vehicle.getBufferDis()); // 发生位置移动的进 buffer
    }
}

void Engine::updateLocation() { // 线程将 changed Vehicle 从 原 drivable 删去，主进程加入新 drivable
    startBarrier.wait();
    endBarrier.wait();
    std::sort(pushBuffer.begin(), pushBuffer.end(), vehicleCmp);
    for (auto &vehiclePair : pushBuffer) {
        Vehicle *vehicle = vehiclePair.first;
        Drivable *drivable = vehicle->getChangedDrivable();
        if (drivable != nullptr) {
            drivable->pushVehicle(vehicle);
            if (drivable->isLaneLink()) {
                vehicle->setEnterLaneLinkTime(step);
            } else {
                vehicle->setEnterLaneLinkTime(std::numeric_limits<int>::max());
            }
        }
    }
    pushBuffer.clear();
}

void Engine::updateAction() { // 线程对每个 vehicle 信息进行更新
    startBarrier.wait();
    endBarrier.wait();
    vehicleRemoveBuffer.clear();
}

void Engine::updateLog() {
    std::string result;
    for (const Vehicle *vehicle : getRunningVehicles()) {
        Point pos = vehicle->getPoint();
        Point dir = vehicle->getCurDrivable()->getDirectionByDistance(vehicle->getDistance());

        int lc = vehicle->lastLaneChangeDirection();
        result.append(double2string(pos.x) + " " + double2string(pos.y) + " " + double2string(atan2(dir.y, dir.x)) + " " + vehicle->getId() + " " +
                      std::to_string(lc) + " " + double2string(vehicle->getLen()) + " " + double2string(vehicle->getWidth()) + ",");
    }
    result.append(";");

    for (const Road &road : roadnet.getRoads()) {
        if (road.getEndIntersection().isVirtualIntersection())
            continue;
        result.append(road.getId());
        for (const Lane &lane : road.getLanes()) {
            if (lane.getEndIntersection()->isImplicitIntersection()) {
                result.append(" i");
                continue;
            }

            bool can_go = true;
            for (LaneLink *laneLink : lane.getLaneLinks()) {
                if (!laneLink->isAvailable()) {
                    can_go = false;
                    break;
                }
            }
            result.append(can_go ? " g" : " r");
        }
        result.append(",");
    }
    logOut << result << std::endl;
}

bool Engine::checkPriority(int priority) { // vehiclePool 中是否存在优先级为 priority 的车子
    return vehiclePool.find(priority) != vehiclePool.end();
}

void Engine::insertShadow(Vehicle *vehicle) {                                    // 创建 vehicle 的 shadow 并插入
    size_t threadIndex = vehiclePool.at(vehicle->getPriority()).second;          // 当前车所在线程编号
    Vehicle *shadow = new Vehicle(*vehicle, vehicle->getId() + "_shadow", this); // 创建 shadow
    vehicleMap.emplace(shadow->getId(), shadow);
    vehiclePool.emplace(shadow->getPriority(), std::make_pair(shadow, threadIndex));
    threadVehiclePool[threadIndex].insert(shadow);
    vehicle->insertShadow(shadow); // 对当前 vehicle、shadow与 targetLane 信息进行更新
    activeVehicleCount++;          // 行驶车辆 +1
}

// 执行步骤
void Engine::nextStep() { // 执行过程
    for (auto &flow : flows)
        flow.nextStep(interval); // 向 road 添加此时会进入的车辆
    planRoute();        // 多线程，由 anchorpoint 得出 vehicle 所要走的最短路 route，并由 Road 的 buffer，并转入 lane 的 buffer
    handleWaiting();    // 由 buffer 进入 lane，计算新进入 vehicle 的 leader 与 gap
    if (laneChange) {   // 允许 laneChange
        initSegments(); // 多线程, 对 lane 中的每一 vehicle 更新 segmentIndex
        planLaneChange(); // 多线程, 找出所有想要 laneChange 的 Vehicle 并 insertShadow
        updateLeaderAndGap(); // 多线程, 在 insertShadow 后对每个 drivable 中的车辆 leader 与 gap 信息进行更新，并更新 lane 的 historyRecord
    }
    notifyCross();        // 多线程, 更新每个 intersection 中的 cross 的 notify信息
    getAction();          // 多线程, 对每一个 vehicle 各类数据计算
    updateLocation();     // 多线程, 更新 drivable 内 vehicle 信息，去除已经到达终点的 vehicle
    updateAction();       // 多线程, 更新每个 vehicle 的信息
    updateLeaderAndGap(); // 多线程，在移动完成后对每个 vehicle 的 leader 和 gap 进行更新

    if (!rlTrafficLight) { // 未开启 rlTrafficLight 则按导入的 trafficLight 进行交通控制
        std::vector<Intersection> &intersections = roadnet.getIntersections();
        for (auto &intersection : intersections)
            intersection.getTrafficLight().passTime(interval);
    }

    if (saveReplay) { // 保存
        updateLog();
    }

    step += 1;
}

// RL related api
void Engine::pushVehicle(Vehicle *const vehicle, bool pushToDrivable) {
    size_t threadIndex = rnd() % threadNum;
    vehiclePool.emplace(vehicle->getPriority(), std::make_pair(vehicle, threadIndex));
    vehicleMap.emplace(vehicle->getId(), vehicle);
    threadVehiclePool[threadIndex].insert(vehicle);

    if (pushToDrivable)
        ((Lane *)vehicle->getCurDrivable())->pushWaitingVehicle(vehicle);
}

size_t Engine::getVehicleCount() const {
    return activeVehicleCount;
}

std::vector<std::string> Engine::getVehicles(bool includeWaiting) const {
    std::vector<std::string> ret;
    ret.reserve(activeVehicleCount);
    for (const Vehicle *vehicle : getRunningVehicles(includeWaiting)) {
        ret.emplace_back(vehicle->getId());
    }
    return ret;
}

std::map<std::string, int> Engine::getLaneVehicleCount() const {
    std::map<std::string, int> ret;
    for (const Lane *lane : roadnet.getLanes()) {
        ret.emplace(lane->getId(), lane->getVehicleCount());
    }
    return ret;
}

std::map<std::string, int> Engine::getLaneWaitingVehicleCount() const {
    std::map<std::string, int> ret;
    for (const Lane *lane : roadnet.getLanes()) {
        int cnt = 0;
        for (Vehicle *vehicle : lane->getVehicles()) {
            if (vehicle->getSpeed() < 0.1) { // TODO: better waiting critera
                cnt += 1;
            }
        }
        ret.emplace(lane->getId(), cnt);
    }
    return ret;
}

std::map<std::string, std::vector<std::string>> Engine::getLaneVehicles() {
    std::map<std::string, std::vector<std::string>> ret;
    for (const Lane *lane : roadnet.getLanes()) {
        std::vector<std::string> vehicles;
        for (Vehicle *vehicle : lane->getVehicles()) {
            vehicles.push_back(vehicle->getId());
        }
        ret.emplace(lane->getId(), vehicles);
    }
    return ret;
}

std::map<std::string, double> Engine::getVehicleSpeed() const {
    std::map<std::string, double> ret;
    for (const Vehicle *vehicle : getRunningVehicles()) {
        ret.emplace(vehicle->getId(), vehicle->getSpeed());
    }
    return ret;
}

std::map<std::string, double> Engine::getVehicleDistance() const {
    std::map<std::string, double> ret;
    for (const Vehicle *vehicle : getRunningVehicles()) {
        ret.emplace(vehicle->getId(), vehicle->getDistance());
    }
    return ret;
}

double Engine::getCurrentTime() const {
    return step * interval;
}

std::map<std::string, std::string> Engine::getVehicleInfo(const std::string &id) const {
    auto iter = vehicleMap.find(id);
    if (iter == vehicleMap.end()) {
        throw std::runtime_error("Vehicle '" + id + "' not found");
    } else {
        Vehicle *vehicle = iter->second;
        return vehicle->getInfo();
    }
}

double Engine::getAverageTravelTime() const {
    double tt = cumulativeTravelTime;
    int n = finishedVehicleCnt;
    for (auto &vehicle_pair : vehiclePool) {
        auto &vehicle = vehicle_pair.second.first;
        tt += getCurrentTime() - vehicle->getEnterTime();
        n++;
    }
    return n == 0 ? 0 : tt / n;
}

void Engine::pushVehicle(const std::map<std::string, double> &info, const std::vector<std::string> &roads) {
    VehicleInfo vehicleInfo;
    std::map<std::string, double>::const_iterator it;
    if ((it = info.find("speed")) != info.end())
        vehicleInfo.speed = it->second;
    if ((it = info.find("length")) != info.end())
        vehicleInfo.len = it->second;
    if ((it = info.find("width")) != info.end())
        vehicleInfo.width = it->second;
    if ((it = info.find("maxPosAcc")) != info.end())
        vehicleInfo.maxPosAcc = it->second;
    if ((it = info.find("maxNegAcc")) != info.end())
        vehicleInfo.maxNegAcc = it->second;
    if ((it = info.find("usualPosAcc")) != info.end())
        vehicleInfo.usualPosAcc = it->second;
    if ((it = info.find("usualNegAcc")) != info.end())
        vehicleInfo.usualNegAcc = it->second;
    if ((it = info.find("minGap")) != info.end())
        vehicleInfo.minGap = it->second;
    if ((it = info.find("maxSpeed")) != info.end())
        vehicleInfo.maxSpeed = it->second;
    if ((it = info.find("headwayTime")) != info.end())
        vehicleInfo.headwayTime = it->second;

    std::vector<Road *> routes;
    routes.reserve(roads.size());
    for (auto &road : roads)
        routes.emplace_back(roadnet.getRoadById(road));
    auto route = std::make_shared<const Route>(routes);
    vehicleInfo.route = route;

    Vehicle *vehicle = new Vehicle(vehicleInfo, "manually_pushed_" + std::to_string(manuallyPushCnt++), this);
    pushVehicle(vehicle, false);
    vehicle->getFirstRoad()->addPlanRouteVehicle(vehicle);
}

void Engine::setTrafficLightPhase(const std::string &id, int phaseIndex) {
    if (!rlTrafficLight) {
        std::cerr << "please set rlTrafficLight to true to enable traffic light control" << std::endl;
        return;
    }
    roadnet.getIntersectionById(id)->getTrafficLight().setPhase(phaseIndex);
}

void Engine::setReplayLogFile(const std::string &logFile) {
    if (!saveReplayInConfig) {
        std::cerr << "saveReplay is not set to true in config file!" << std::endl;
        return;
    }
    if (logOut.is_open())
        logOut.close();
    logOut.open(dir + logFile);
}

void Engine::setSaveReplay(bool open) {
    if (!saveReplayInConfig) {
        std::cerr << "saveReplay is not set to true in config file!" << std::endl;
        return;
    }
    saveReplay = open;
}

void Engine::reset(bool resetRnd) {
    for (auto &vehiclePair : vehiclePool)
        delete vehiclePair.second.first;
    for (auto &pool : threadVehiclePool)
        pool.clear();
    vehiclePool.clear();
    vehicleMap.clear();
    roadnet.reset();

    finishedVehicleCnt = 0;
    cumulativeTravelTime = 0;

    for (auto &flow : flows)
        flow.reset();
    step = 0;
    activeVehicleCount = 0;
    if (resetRnd) {
        rnd.seed(seed);
    }
}

void Engine::setLogFile(const std::string &jsonFile, const std::string &logFile) {
    if (!writeJsonToFile(jsonFile, jsonRoot)) {
        std::cerr << "write roadnet log file error" << std::endl;
    }
    logOut.open(logFile);
}

std::vector<const Vehicle *> Engine::getRunningVehicles(bool includeWaiting) const {
    std::vector<const Vehicle *> ret;
    ret.reserve(activeVehicleCount);
    for (const auto &vehiclePair : vehiclePool) {
        const Vehicle *vehicle = vehiclePair.second.first;
        if (vehicle->isReal() && (includeWaiting || vehicle->isRunning())) {
            ret.emplace_back(vehicle);
        }
    }
    return ret;
}

void Engine::setVehicleSpeed(const std::string &id, double speed) {
    auto iter = vehicleMap.find(id);
    if (iter == vehicleMap.end()) {
        throw std::runtime_error("Vehicle '" + id + "' not found");
    } else {
        iter->second->setCustomSpeed(speed);
    }
}

std::string Engine::getLeader(const std::string &vehicleId) const {
    auto iter = vehicleMap.find(vehicleId);
    if (iter == vehicleMap.end()) {
        throw std::runtime_error("Vehicle '" + vehicleId + "' not found");
    } else {
        Vehicle *vehicle = iter->second;
        if (laneChange) {
            if (!vehicle->isReal())
                vehicle = vehicle->getPartner();
        }
        Vehicle *leader = vehicle->getLeader();
        if (leader)
            return leader->getId();
        else
            return "";
    }
}

// 析构
Engine::~Engine() {
    logOut.close();
    finished = true;
    for (int i = 0; i < (laneChange ? 9 : 6); ++i) {
        startBarrier.wait();
        endBarrier.wait();
    }
    for (auto &thread : threadPool)
        thread.join();
    for (auto &vehiclePair : vehiclePool)
        delete vehiclePair.second.first;
}

// archive
void Engine::loadFromFile(const char *fileName) {
    Archive archive(*this, fileName);
    archive.resume(*this);
}

bool Engine::setRoute(const std::string &vehicle_id, const std::vector<std::string> &anchor_id) {
    auto vehicle_itr = vehicleMap.find(vehicle_id);
    if (vehicle_itr == vehicleMap.end())
        return false;
    Vehicle *vehicle = vehicle_itr->second;

    std::vector<Road *> anchors;
    for (const auto &id : anchor_id) {
        auto anchor = roadnet.getRoadById(id);
        if (!anchor)
            return false;
        anchors.emplace_back(anchor);
    }

    return vehicle->setRoute(anchors);
}

} // namespace CityFlow
