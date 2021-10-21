#include "vehicle/router.h"
#include "flow/route.h"
#include "roadnet/roadnet.h"
#include "vehicle/vehicle.h"

#include <limits>
#include <queue>
#include <set>

namespace CityFlow {
Router::Router(const Router &other) : vehicle(other.vehicle), route(other.route), anchorPoints(other.anchorPoints), rnd(other.rnd) {
    iCurRoad = this->route.begin();
}

Router::Router(Vehicle *vehicle, std::shared_ptr<const Route> route, std::mt19937 *rnd) : vehicle(vehicle), anchorPoints(route->getRoute()), rnd(rnd) {
    assert(this->anchorPoints.size() > 0);
    this->route = route->getRoute();
    iCurRoad = this->route.begin();
}

Drivable *Router::getFirstDrivable() const { // 进入 route[0] 应选的 drivable
    const std::vector<Lane *> &lanes = route[0]->getLanePointers();
    if (route.size() == 1) {               // 仅 1 road
        return selectLane(nullptr, lanes); // 随机选 lane
    } else {
        std::vector<Lane *> candidateLanes;
        for (auto lane : lanes) {
            if (lane->getLaneLinksToRoad(route[1]).size() > 0) { // 选择拥有由 route[0] 驶向 route[1] 的 laneLink 的 lane
                candidateLanes.push_back(lane);
            }
        }
        assert(candidateLanes.size() > 0);
        return selectLane(nullptr, candidateLanes); // candidateLanes 内选 lane
    }
}

Drivable *Router::getNextDrivable(size_t i) const { // 下 i + 1 步将走的 drivable
    if (i < planned.size()) {                       // 已计算
        return planned[i];
    } else {                                                                                          // 未事先计算
        Drivable *ret = getNextDrivable(planned.size() ? planned.back() : vehicle->getCurDrivable()); // 在前一个 drivable 基础上计算 nextDrivable
        planned.push_back(ret);                                                                       // 填入 planned 备用
        return ret;
    }
}

Drivable *Router::getNextDrivable(const Drivable *curDrivable) const { // 由当前 drivable 计算下一个 drivable（当前为 lane 则给出 laneLink，反之同理）
    if (curDrivable->isLaneLink()) {                                   // 当前是 laneLink 直接得出
        return static_cast<const LaneLink *>(curDrivable)->getEndLane();
    } else { // 当前是 lane
        const Lane *curLane = static_cast<const Lane *>(curDrivable);
        auto tmpCurRoad = iCurRoad;
        while ((*tmpCurRoad) != curLane->getBelongRoad() && tmpCurRoad != route.end()) { // 找到 curDrivable 对应的 CurRoad
            tmpCurRoad++;
        }
        assert(tmpCurRoad != route.end() && curLane->getBelongRoad() == (*tmpCurRoad));
        if (tmpCurRoad == route.end() - 1) { // 已到 route 末尾
            return nullptr;
        } else if (tmpCurRoad == route.end() - 2) { // route 内倒数第二 road
            std::vector<LaneLink *> laneLinks = curLane->getLaneLinksToRoad(*(tmpCurRoad + 1));
            return selectLaneLink(curLane, laneLinks);                                          // 走向可选 laneLink 的 endLane 中距离 curlane 最近的 lane
        } else {                                                                                // 选取的 laneLink 需能确保到达 route 的再下一个 road
            std::vector<LaneLink *> laneLinks = curLane->getLaneLinksToRoad(*(tmpCurRoad + 1)); // 由 route[i] 到 route[i+1] 的 laneLink
            std::vector<LaneLink *> candidateLaneLinks;
            for (auto laneLink : laneLinks) {
                Lane *nextLane = laneLink->getEndLane();
                if (nextLane->getLaneLinksToRoad(*(tmpCurRoad + 2)).size()) { // 走此 laneLink 后能从 route[i+1] 到达 route[i+2]
                    candidateLaneLinks.push_back(laneLink);
                }
            }
            return selectLaneLink(curLane, candidateLaneLinks); // 变动最少的 laneLink
        }
    }
}

void Router::update() { // 更新 iCurRoad 与 planned
    const Drivable *curDrivable = vehicle->getCurDrivable();
    if (curDrivable->isLane()) {
        while (iCurRoad < route.end() && static_cast<const Lane *>(curDrivable)->getBelongRoad() != (*iCurRoad)) {
            iCurRoad++;
        }
        assert(iCurRoad < route.end());
    }
    for (auto it = planned.begin(); it != planned.end();) { // curDrivable 发生变化则先前缓存的 planned 部分或全部不再可用
        if ((*it) != curDrivable) {
            it = planned.erase(it);
        } else {
            it = planned.erase(it);
            break;
        }
    }
}

int Router::selectLaneIndex(const Lane *curLane, const std::vector<Lane *> &lanes) const { // 根据当前 lane 在备选区选下一个 lane 的编号
    assert(lanes.size() > 0);
    if (curLane == nullptr) { // 当前未入 lane，随机选择备选 lane
        return (*rnd)() % lanes.size();
    }
    // 已在 lane 上则选择最相邻的 lane
    int laneDiff = std::numeric_limits<int>::max();
    int selected = -1;
    for (size_t i = 0; i < lanes.size(); ++i) {
        int curLaneDiff = lanes[i]->getLaneIndex() - curLane->getLaneIndex();
        if (abs(curLaneDiff) < laneDiff) {
            laneDiff = abs(curLaneDiff);
            selected = i;
        }
    }
    return selected;
}

Lane *Router::selectLane(const Lane *curLane, const std::vector<Lane *> &lanes) const { // 根据当前 lane 在备选区选下一个 lane
    if (lanes.size() == 0) {
        return nullptr;
    }
    return lanes[selectLaneIndex(curLane, lanes)];
}

LaneLink *Router::selectLaneLink(const Lane *curLane, const std::vector<LaneLink *> &laneLinks) const { // 根据当前 lane 在 laneLink 备选区的 endline 内选下一个 lane
    if (laneLinks.size() == 0) {
        return nullptr;
    }
    std::vector<Lane *> lanes;
    for (auto laneLink : laneLinks) {
        lanes.push_back(laneLink->getEndLane());
    }
    return laneLinks[selectLaneIndex(curLane, lanes)];
}

bool Router::isLastRoad(const Drivable *drivable) const {
    if (drivable->isLaneLink())
        return false;
    return static_cast<const Lane *>(drivable)->getBelongRoad() == route.back();
}

bool Router::onLastRoad() const {
    return isLastRoad(vehicle->getCurDrivable());
}

Lane *Router::getValidLane(const Lane *curLane) const { // 选取从 curLane 走向下一个 Road 时 laneIndex 差距最小的 lane
    if (isLastRoad(curLane))
        return nullptr;
    auto nextRoad = iCurRoad;
    nextRoad++;

    int min_diff = curLane->getBelongRoad()->getLanes().size();
    Lane *chosen = nullptr;
    for (auto lane : curLane->getBelongRoad()->getLanePointers()) {
        int curLaneDiff = lane->getLaneIndex() - curLane->getLaneIndex();
        if (lane->getLaneLinksToRoad(*nextRoad).size() > 0 && abs(curLaneDiff) < min_diff) {
            min_diff = abs(curLaneDiff);
            chosen = lane;
        }
    }
    assert(chosen->getBelongRoad() == curLane->getBelongRoad());
    return chosen;
}

bool Router::dijkstra(Road *start, Road *end, std::vector<Road *> &buffer) { // 最短路
    std::map<Road *, double> dis;
    std::map<Road *, Road *> from;
    std::set<Road *> visited;
    bool success = false;
    using pair = std::pair<Road *, double>;

    auto cmp = [](const pair &a, const pair &b) { return a.second > b.second; };

    std::priority_queue<pair, std::vector<pair>, decltype(cmp)> queue(cmp);

    dis[start] = 0;
    queue.push(std::make_pair(start, 0));
    while (!queue.empty()) {
        auto curRoad = queue.top().first;
        if (curRoad == end) {
            success = true;
            break;
        }
        queue.pop();
        if (visited.count(curRoad))
            continue;
        visited.insert(curRoad);
        double curDis = dis.find(curRoad)->second;
        dis[curRoad] = curDis;
        for (const auto &adjRoad : curRoad->getEndIntersection().getRoads()) {
            if (!curRoad->connectedToRoad(adjRoad))
                continue;
            auto iter = dis.find(adjRoad);
            double newDis;

            switch (type) {
            case RouterType::LENGTH:
                newDis = curDis + adjRoad->averageLength();
                break;
            case RouterType::DURATION: {
                double avgDur;
                avgDur = adjRoad->getAverageDuration();
                if (avgDur < 0) {
                    avgDur = adjRoad->getLength() / vehicle->getMaxSpeed();
                }
                newDis = curDis + avgDur;
            } break;
            default:
                assert(false); // under construction
                break;
            }

            if (iter == dis.end() || newDis < iter->second) {
                from[adjRoad] = curRoad;
                dis[adjRoad] = newDis;
                queue.emplace(std::make_pair(adjRoad, newDis));
            }
        }
    }

    std::vector<Road *> path;
    path.push_back(end);

    auto iter = from.find(end);
    while (iter != from.end() && iter->second != start) {
        path.emplace_back(iter->second);
        iter = from.find(iter->second);
    }

    buffer.insert(buffer.end(), path.rbegin(), path.rend());
    return success;
}

bool Router::updateShortestPath() { // 更新 route 为经过 anchorpoint 各路的最短路
    // Dijkstra
    planned.clear();
    route.clear();
    route.push_back(anchorPoints[0]);
    for (size_t i = 1; i < anchorPoints.size(); ++i) {
        if (anchorPoints[i - 1] == anchorPoints[i])
            continue;
        if (!dijkstra(anchorPoints[i - 1], anchorPoints[i], route))
            return false;
    }
    if (route.size() <= 1)
        return false;
    iCurRoad = this->route.begin();
    return true;
}

bool Router::setRoute(const std::vector<Road *> &anchor) { // 修改后续需走的路径为 anchor
    if (vehicle->getCurDrivable()->isLaneLink())
        return false;
    Road *cur_road = *iCurRoad;
    auto backup = std::move(anchorPoints);
    auto backup_route = std::move(route);
    anchorPoints.clear();
    anchorPoints.emplace_back(cur_road);
    anchorPoints.insert(anchorPoints.end(), anchor.begin(), anchor.end());
    bool result = updateShortestPath();
    if (result && onValidLane()) { // 有效修改
        return true;
    } else { // 恢复原数据
        anchorPoints = std::move(backup);
        route = std::move(backup_route);
        planned.clear();
        iCurRoad = route.begin();
        for (iCurRoad = route.begin(); *iCurRoad != cur_road && iCurRoad != route.end(); ++iCurRoad)
            ;
        return false;
    }
}

std::vector<Road *> Router::getFollowingRoads() const { // 获取未来将走的所有 Road
    std::vector<Road *> ret;
    ret.insert(ret.end(), iCurRoad, route.end());
    return ret;
}

} // namespace CityFlow
