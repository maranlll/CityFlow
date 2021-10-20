#ifndef CITYFLOW_ROADNET_H
#define CITYFLOW_ROADNET_H

#include "roadnet/trafficlight.h"
#include "utility/utility.h"

#include <iostream>
#include <list>
#include <map>
#include <queue>

namespace CityFlow {
class RoadNet;
class Intersection;
class Road;
class Lane;
class LaneLink;
class Vehicle;
class Cross;

// 存储于 Lane
class Segment {
    friend Lane;

  private:
    size_t index = 0;                                   // 当前 Segment 的编号
    Lane *belongLane = nullptr;                         // Segment 所属的 lane
    double startPos = 0;                                // lane 上的起点 dis
    double endPos = 0;                                  // lane 上的终点 dis
    std::list<std::list<Vehicle *>::iterator> vehicles; // Segment 上的车辆，指向 Lane 的父类 Dirvable 的 vehicles 中的一段
    std::list<Vehicle *>::iterator prev_vehicle_iter;   // 指向车辆的迭代器

  public:
    // Vehicle * tryChange = nullptr;

    Segment() = default;

    Segment(size_t index, Lane *belongLane, double startPos, double endPos)
        : index(index), belongLane(belongLane), startPos(startPos), endPos(endPos) {}

    double getStartPos() const {
        return this->startPos;
    }

    double getEndPos() const {
        return this->endPos;
    }

    size_t getIndex() const {
        return this->index;
    }

    const std::list<std::list<Vehicle *>::iterator> &getVehicles() const {
        return this->vehicles;
    }

    std::list<std::list<Vehicle *>::iterator> &getVehicles() {
        return this->vehicles;
    }

    //        std::list<Vehicle *>::iterator getPrevVehicleIter() const { return this->prev_vehicle_iter; }

    std::list<Vehicle *>::iterator findVehicle(Vehicle *vehicle);

    void removeVehicle(Vehicle *vehicle);

    void insertVehicle(std::list<Vehicle *>::iterator &vehicle);
};

// 存储于 RoadNet，含有 RoadLink 与 Cross
class Intersection {
    friend class RoadNet;
    friend class RoadLink;
    friend class Road;
    friend class TrafficLight;

  private:
    std::string id; // intersection id
    bool isVirtual; // ??
    double width = 0.0;
    Point point; // position
    TrafficLight trafficLight;
    std::vector<Road *> roads;         // 连接的 road（指针版），原对象存储于 RoadNet
    std::vector<RoadLink> roadLinks;   // 可通行的 roadLinks
    std::vector<Cross> crosses;        // laneLink 产生的交叉点集
    std::vector<LaneLink *> laneLinks; // 可通行的 laneLinks， 原对象存储于 RoadLink

    void initCrosses();

  public:
    std::string getId() const {
        return this->id;
    }

    const TrafficLight &getTrafficLight() const {
        return trafficLight;
    }

    TrafficLight &getTrafficLight() {
        return trafficLight;
    }

    const std::vector<Road *> &getRoads() const {
        return this->roads;
    }

    std::vector<Road *> &getRoads() {
        return this->roads;
    }

    const std::vector<RoadLink> &getRoadLinks() const {
        return this->roadLinks;
    }

    std::vector<RoadLink> &getRoadLinks() {
        return this->roadLinks;
    }

    std::vector<Cross> &getCrosses() {
        return crosses;
    }

    bool isVirtualIntersection() const {
        return this->isVirtual;
    }

    const std::vector<LaneLink *> &getLaneLinks();

    void reset();

    std::vector<Point> getOutline();

    bool isImplicitIntersection();

    const Point &getPosition() const {
        return point;
    }
};

// Cross 由 laneLink 产生，存储于 Intersection, laneLink可获取与之相关的 cross
class Cross {
    friend class RoadNet;
    friend class Intersection;

  private:
    LaneLink *laneLinks[2];                        // 两条穿过路口的 laneLink 形成 cross
    Vehicle *notifyVehicles[2];                    // 每个 cross 中可能冲突的车
    double notifyDistances[2];                     // <0 表示在 cross 后，>0 表示在 cross 前，距离 cross 的距离
    double distanceOnLane[2];                      // 交叉点距离 laneLink 起点的距离
    double leaveDistance = 0, arriveDistance = 30; // TODO
    double ang;                                    // 夹角的弧度
    double safeDistances[2];

  public:
    double getLeaveDistance() const {
        return leaveDistance;
    }

    double getArriveDistance() const {
        return arriveDistance;
    }

    void notify(LaneLink *laneLink, Vehicle *vehicle, double notifyDistance); // notify all the crosspoint of its arrival

    bool canPass(const Vehicle *vehicle, const LaneLink *laneLink,
                 double distanceToLaneLinkStart) const; // XXX: change to LaneLink based?

    void clearNotify() {
        notifyVehicles[0] = notifyVehicles[1] = nullptr;
    }

    Vehicle *getFoeVehicle(const LaneLink *laneLink) const {
        assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
        return laneLink == laneLinks[0] ? notifyVehicles[1] : notifyVehicles[0];
    }

    double getDistanceByLane(const LaneLink *laneLink) const {
        // XXX: lanelink not in cross?
        assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
        return laneLink == laneLinks[0] ? distanceOnLane[0] : distanceOnLane[1];
    }

    double getNotifyDistanceByLane(LaneLink *laneLink) const {
        // XXX: lanelink not in cross?
        assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
        return laneLink == laneLinks[0] ? notifyDistances[0] : notifyDistances[1];
    }

    double getSafeDistanceByLane(LaneLink *laneLink) {
        assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
        return laneLink == laneLinks[0] ? safeDistances[0] : safeDistances[1];
    }

    double getAng() const {
        return ang;
    }

    LaneLink *getLaneLink(int i) const {
        return laneLinks[i];
    }

    void reset();
};

// 存储于 RoadNet, 含有 Lane
class Road {
    friend class RoadNet;
    friend class Lane;

  private:
    std::string id;
    Intersection *startIntersection = nullptr;
    Intersection *endIntersection = nullptr;
    std::vector<Lane> lanes;                // 含有的 lane
    std::vector<Lane *> lanePointers;       // 含有的 lane（指针版）
    std::vector<Point> points;              // 连续的 points 记录 road 走向
    std::vector<Vehicle *> planRouteBuffer; // 待进入此 Road 的 vehicle 的缓冲区

    void initLanesPoints();

  public:
    std::string getId() const {
        return id;
    }

    const Intersection &getStartIntersection() const {
        return *(this->startIntersection);
    }

    const Intersection &getEndIntersection() const {
        return *(this->endIntersection);
    }

    const std::vector<Lane> &getLanes() const {
        return lanes;
    }

    std::vector<Lane> &getLanes() {
        return lanes;
    }

    const std::vector<Lane *> &getLanePointers();

    void buildSegmentationByInterval(double interval); // 建立 Segment

    bool connectedToRoad(const Road *road) const;

    void reset();

    double getWidth() const;

    double getLength() const;

    double averageLength() const;

    double getAverageSpeed() const;

    double getAverageDuration() const;

    const std::vector<Vehicle *> &getPlanRouteBuffer() const {
        return planRouteBuffer;
    }

    void addPlanRouteVehicle(Vehicle *vehicle) {
        planRouteBuffer.emplace_back(vehicle);
    }

    void clearPlanRouteBuffer() {
        planRouteBuffer.clear();
    }
};

// 可通行路的基类, Lane 子类存储于 Road， LaneLink 子类存储于 RoadLink
class Drivable {
    friend class RoadNet;
    friend class Archive;

  public:
    enum DrivableType { LANE = 0, LANELINK = 1 };

  protected:
    double length;
    double width;
    double maxSpeed;
    std::list<Vehicle *> vehicles; // 在此通行的车
    std::vector<Point> points;     // 连续的 points 记录走向
    DrivableType drivableType;     // lane / laneLink

  public:
    virtual ~Drivable() = default;

    const std::list<Vehicle *> &getVehicles() const {
        return vehicles;
    }

    std::list<Vehicle *> &getVehicles() {
        return vehicles;
    }

    double getLength() const {
        return length;
    }

    double getWidth() const {
        return width;
    }

    double getMaxSpeed() const {
        return maxSpeed;
    }

    size_t getVehicleCount() const {
        return vehicles.size();
    }

    DrivableType getDrivableType() const {
        return drivableType;
    }

    bool isLane() const {
        return drivableType == LANE;
    }

    bool isLaneLink() const {
        return drivableType == LANELINK;
    }

    Vehicle *getFirstVehicle() const {
        if (!vehicles.empty())
            return vehicles.front();
        return nullptr;
    }

    Vehicle *getLastVehicle() const {
        if (!vehicles.empty())
            return vehicles.back();
        return nullptr;
    }

    Point getPointByDistance(double dis) const;

    Point getDirectionByDistance(double dis) const;

    void pushVehicle(Vehicle *vehicle) {
        vehicles.push_back(vehicle);
    }

    void popVehicle() {
        vehicles.pop_front();
    }

    virtual std::string getId() const = 0;
};

// 存储于 Road，含有 segment
class Lane : public Drivable {
    friend class RoadNet;
    friend class Road;
    friend class Archive;

  private:
    int laneIndex;                       // 所属 Road 中的编号
    std::vector<Segment> segments;       // 所含 segments
    std::vector<LaneLink *> laneLinks;   // 以此 lane 作为 startLane 的 laneLinks（指针版），总 laneLinks 存储于 RoadLink
    Road *belongRoad = nullptr;          // 所属 road
    std::deque<Vehicle *> waitingBuffer; // 待进入此 lane 的车（内容由所属 road 的 buffer 处理后获得）

    struct HistoryRecord { // lane 车流信息记录
        int vehicleNum;
        double averageSpeed;
        HistoryRecord(int vehicleNum, double averageSpeed) : vehicleNum(vehicleNum), averageSpeed(averageSpeed) {}
    };
    std::list<HistoryRecord> history; // lane 车流信息记录

    int historyVehicleNum = 0;
    double historyAverageSpeed = 0;

    static constexpr int historyLen = 240; // 最大可容纳的记录

  public:
    Lane();

    Lane(double width, double maxSpeed, int laneIndex, Road *belongRoad);

    std::string getId() const override {
        return belongRoad->getId() + '_' + std::to_string(getLaneIndex());
    }

    Road *getBelongRoad() const {
        return this->belongRoad;
    }

    bool available(const Vehicle *vehicle) const; // 车是否可以从此 Lane 走?

    bool canEnter(const Vehicle *vehicle) const;

    size_t getLaneIndex() const {
        return this->laneIndex;
    }

    Lane *getInnerLane() const {
        return laneIndex > 0 ? &(belongRoad->lanes[laneIndex - 1]) : nullptr;
    }

    Lane *getOuterLane() const {
        int lane_num = belongRoad->lanes.size();
        return laneIndex < lane_num - 1 ? &(belongRoad->lanes[laneIndex + 1]) : nullptr;
    }

    const std::vector<LaneLink *> &getLaneLinks() const {
        return this->laneLinks;
    }

    std::vector<LaneLink *> &getLaneLinks() {
        return this->laneLinks;
    }

    Intersection *getStartIntersection() const {
        return belongRoad->startIntersection;
    }

    Intersection *getEndIntersection() const {
        return belongRoad->endIntersection;
    }

    std::vector<LaneLink *> getLaneLinksToRoad(const Road *road) const;

    void reset();

    /* waiting buffer */
    const std::deque<Vehicle *> &getWaitingBuffer() const {
        return waitingBuffer;
    }

    std::deque<Vehicle *> &getWaitingBuffer() {
        return waitingBuffer;
    }

    void pushWaitingVehicle(Vehicle *vehicle) {
        waitingBuffer.emplace_back(vehicle);
    }

    /* segmentation */
    void buildSegmentation(size_t numSegs);

    void initSegments();

    const Segment *getSegment(size_t index) const {
        return &segments[index];
    }

    Segment *getSegment(size_t index) {
        return &segments[index];
    }

    const std::vector<Segment> &getSegments() const {
        return segments;
    }

    std::vector<Segment> &getSegments() {
        return segments;
    }

    size_t getSegmentNum() const {
        return segments.size();
    }

    std::vector<Vehicle *> getVehiclesBeforeDistance(double dis, size_t segmentIndex, double deltaDis = 50);

    /* history */
    void updateHistory();

    int getHistoryVehicleNum() const;

    double getHistoryAverageSpeed() const;

    Vehicle *getVehicleBeforeDistance(double dis, size_t segmentIndex) const; // TODO: set a limit, not too far way

    Vehicle *getVehicleAfterDistance(double dis, size_t segmentIndex) const;
};

enum RoadLinkType { go_straight = 3, turn_left = 2, turn_right = 1 };

// 存储于 Intersection， 含有 LaneLink
class RoadLink {
    friend class RoadNet;
    friend class LaneLink;

  private:
    Intersection *intersection = nullptr; // 所属 intersection
    Road *startRoad = nullptr;
    Road *endRoad = nullptr;
    RoadLinkType type;
    std::vector<LaneLink> laneLinks;          // 所含的 laneLinks
    std::vector<LaneLink *> laneLinkPointers; // 所含的 laneLinks（指针版）
    int index;                                // 编号

  public:
    const std::vector<LaneLink> &getLaneLinks() const {
        return this->laneLinks;
    }

    std::vector<LaneLink> &getLaneLinks() {
        return this->laneLinks;
    }

    const std::vector<LaneLink *> &getLaneLinkPointers();

    Road *getStartRoad() const {
        return this->startRoad;
    }

    Road *getEndRoad() const {
        return this->endRoad;
    }

    bool isAvailable() const { // 当前信号灯可通行过路口
        return this->intersection->trafficLight.getCurrentPhase().roadLinkAvailable[this->index];
    }

    bool isTurn() const {
        return type == turn_left || type == turn_right;
    }

    void reset();
};

// 存储于 RoadLink
class LaneLink : public Drivable {
    friend class RoadNet;
    friend class Intersection;

  private:
    RoadLink *roadLink = nullptr; //所属的 roadlink
    Lane *startLane = nullptr;
    Lane *endLane = nullptr;
    std::vector<Cross *> crosses; // 与其余 laneLink 相交会产生的 crosses，总 cross 存储于 intersection，按距 startlane 起始点距离排序

  public:
    LaneLink() {
        width = 4;
        maxSpeed = 10000; // TODO
        drivableType = LANELINK;
    }

    RoadLink *getRoadLink() const {
        return this->roadLink;
    }

    RoadLinkType getRoadLinkType() const {
        return this->roadLink->type;
    }

    const std::vector<Cross *> &getCrosses() const {
        return this->crosses;
    }

    std::vector<Cross *> &getCrosses() {
        return this->crosses;
    }

    Lane *getStartLane() const {
        return startLane;
    }

    Lane *getEndLane() const {
        return endLane;
    }

    bool isAvailable() const {
        return roadLink->isAvailable();
    }

    bool isTurn() const {
        return roadLink->isTurn();
    }

    void reset();

    std::string getId() const override {
        return (startLane ? startLane->getId() : "") + "_TO_" + (endLane ? endLane->getId() : "");
    }
};

// 路网，含有 Road、Intersection
class RoadNet {
  private:
    std::vector<Road> roads;                        // 整体路网所含的 roads
    std::vector<Intersection> intersections;        // 整体路网所含的 intersections
    std::map<std::string, Road *> roadMap;          // id to Road*
    std::map<std::string, Intersection *> interMap; // id to Intersection*
    std::map<std::string, Drivable *> drivableMap;  // getId() to Drivable*

    std::vector<Lane *> lanes;         // 整体路网所含的 lanes(指针版)，原对象存储于 Road
    std::vector<LaneLink *> laneLinks; // 整体路网所含的 laneLinks(指针版)，原对象存储于 RoadLink
    std::vector<Drivable *> drivables; // 整体路网所含的 drivables(指针版)
    Point getPoint(const Point &p1, const Point &p2, double a);

  public:
    bool loadFromJson(std::string jsonFileName);

    rapidjson::Value convertToJson(rapidjson::Document::AllocatorType &allocator);

    const std::vector<Road> &getRoads() const {
        return this->roads;
    }

    std::vector<Road> &getRoads() {
        return this->roads;
    }

    const std::vector<Intersection> &getIntersections() const {
        return this->intersections;
    }

    std::vector<Intersection> &getIntersections() {
        return this->intersections;
    }

    Road *getRoadById(const std::string &id) const {
        return roadMap.count(id) > 0 ? roadMap.at(id) : nullptr;
    }

    Intersection *getIntersectionById(const std::string &id) const {
        return interMap.count(id) > 0 ? interMap.at(id) : nullptr;
    }

    Drivable *getDrivableById(const std::string &id) const {
        return drivableMap.count(id) > 0 ? drivableMap.at(id) : nullptr;
    }

    const std::vector<Lane *> &getLanes() const {
        return lanes;
    }

    const std::vector<LaneLink *> &getLaneLinks() const {
        return laneLinks;
    }

    const std::vector<Drivable *> &getDrivables() const {
        return drivables;
    }

    void reset();
};
} // namespace CityFlow

#endif // CITYFLOW_ROADNET_H
