#ifndef CITYFLOW_ENGINE_H
#define CITYFLOW_ENGINE_H

#include "engine/archive.h"
#include "flow/flow.h"
#include "roadnet/roadnet.h"
#include "utility/barrier.h"

#include <fstream>
#include <mutex>
#include <random>
#include <set>
#include <thread>

namespace CityFlow {

class Engine {
    friend class Archive;

  private:
    static bool vehicleCmp(const std::pair<Vehicle *, double> &a, const std::pair<Vehicle *, double> &b) {
        return a.second > b.second;
    }

    std::map<int, std::pair<Vehicle *, int>> vehiclePool;            // map<priority, pair<Vehicle *, threadBelongedTo>>
    std::map<std::string, Vehicle *> vehicleMap;                     // map<id, Vehicle *>  id: "flow_x_x"
    std::vector<std::set<Vehicle *>> threadVehiclePool;              // Vehicle 线程池
    std::vector<std::vector<Road *>> threadRoadPool;                 // Road 线程池
    std::vector<std::vector<Intersection *>> threadIntersectionPool; // Interserction 线程池
    std::vector<std::vector<Drivable *>> threadDrivablePool;         // Drivable 线程池
    std::vector<Flow> flows;                                         // 待进入 RoadNet 的车辆流信息
    RoadNet roadnet;                                                 // 路网
    int threadNum;                                                   // 线程数
    double interval;                                                 // step 时间间隔
    bool saveReplay;                                                 // 是否记录
    bool saveReplayInConfig;                                         // saveReplay option in config json
    bool warnings;                                                   // 是否开 waring
    std::vector<std::pair<Vehicle *, double>> pushBuffer;            // vector<pair<Vehicle *, deltadis>> 缓存 drivable 发生变化的车辆
    std::vector<Vehicle *> laneChangeNotifyBuffer;                   // 待 laneChange 的车辆
    std::set<Vehicle *> vehicleRemoveBuffer;                         // 到达 end 的车辆
    rapidjson::Document jsonRoot;
    std::string stepLog;

    size_t step = 0;                     // next_step 执行步数
    size_t activeVehicleCount = 0;       // 运行中车辆数
    int seed;                            // 随机数种子
    std::mutex lock;                     // 互斥锁
    Barrier startBarrier, endBarrier;    // 线程 start 与 end 控制对象
    std::vector<std::thread> threadPool; // 主程序线程池
    bool finished = false;               // 是否程序执行结束
    std::string dir;                     // 文件夹路径
    std::ofstream logOut;

    bool rlTrafficLight; // 是否使用强化学习控制 TrafficLight，如非则使用固定 TrafficLight phase
    bool laneChange;     // 是否允许变道
    int manuallyPushCnt = 0;

    int finishedVehicleCnt = 0;      // 累计穿行时间
    double cumulativeTravelTime = 0; // 累计穿行时间

  private:
    void vehicleControl(Vehicle &vehicle, std::vector<std::pair<Vehicle *, double>> &buffer);

    void planRoute();

    void getAction();

    void updateAction();

    void updateLocation();

    void updateLeaderAndGap();

    void planLaneChange();

    void threadController(std::set<Vehicle *> &vehicles, std::vector<Road *> &roads, std::vector<Intersection *> &intersections,
                          std::vector<Drivable *> &drivables);

    void threadPlanRoute(const std::vector<Road *> &roads);

    void threadGetAction(std::set<Vehicle *> &vehicles);

    void threadUpdateAction(std::set<Vehicle *> &vehicles);

    void threadUpdateLeaderAndGap(const std::vector<Drivable *> &drivables);

    void threadUpdateLocation(const std::vector<Drivable *> &drivables);

    void threadNotifyCross(const std::vector<Intersection *> &intersections);

    void threadInitSegments(const std::vector<Road *> &roads);

    void threadPlanLaneChange(const std::set<Vehicle *> &vehicles);

    void handleWaiting();

    void updateLog();

    bool checkWarning();

    bool loadRoadNet(const std::string &jsonFile);

    bool loadFlow(const std::string &jsonFilename);

    std::vector<const Vehicle *> getRunningVehicles(bool includeWaiting = false) const;

    void scheduleLaneChange();

    void insertShadow(Vehicle *vehicle);

  public:
    std::mt19937 rnd;

    Engine(const std::string &configFile, int threadNum);

    double getInterval() const {
        return interval;
    }

    bool hasLaneChange() const {
        return laneChange;
    }

    bool loadConfig(const std::string &configFile);

    void notifyCross();

    void nextStep();

    bool checkPriority(int priority);

    void pushVehicle(Vehicle *const vehicle, bool pushToDrivable = true);

    void setLogFile(const std::string &jsonFile, const std::string &logFile);

    void initSegments();

    ~Engine();

    // RL related api

    void pushVehicle(const std::map<std::string, double> &info, const std::vector<std::string> &roads);

    size_t getVehicleCount() const;

    std::vector<std::string> getVehicles(bool includeWaiting = false) const;

    std::map<std::string, int> getLaneVehicleCount() const;

    std::map<std::string, int> getLaneWaitingVehicleCount() const;

    std::map<std::string, std::vector<std::string>> getLaneVehicles();

    std::map<std::string, double> getVehicleSpeed() const;

    std::map<std::string, double> getVehicleDistance() const;

    std::map<std::string, std::string> getVehicleInfo(const std::string &id) const;

    std::string getLeader(const std::string &vehicleId) const;

    double getCurrentTime() const;

    double getAverageTravelTime() const;

    void setTrafficLightPhase(const std::string &id, int phaseIndex);

    void setReplayLogFile(const std::string &logFile);

    void setSaveReplay(bool open);

    void setVehicleSpeed(const std::string &id, double speed);

    void setRandomSeed(int seed) {
        rnd.seed(seed);
    }

    void reset(bool resetRnd = false);

    // archive
    void load(const Archive &archive) {
        archive.resume(*this);
    }

    Archive snapshot() {
        return Archive(*this);
    }

    void loadFromFile(const char *fileName);

    bool setRoute(const std::string &vehicle_id, const std::vector<std::string> &anchor_id);
};

} // namespace CityFlow

#endif // CITYFLOW_ENGINE_H
