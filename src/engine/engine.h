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
    static bool vehicleCmp(const std::pair<Vehicle *, double> &a, const std::pair<Vehicle *, double> &b) { // 按 dis 从大到小
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
    std::set<Vehicle *> vehicleRemoveBuffer;                         // 将从 RoadNet 移除的车辆
    rapidjson::Document jsonRoot;
    std::string stepLog;

    size_t step = 0;                     // next_step 执行步数
    size_t activeVehicleCount = 0;       // 运行中车辆数
    int seed;                            // 随机数种子
    std::mutex lock;                     // 互斥锁
    Barrier startBarrier, endBarrier;    // 线程控制对象
    std::vector<std::thread> threadPool; // 主程序线程池
    bool finished = false;               // 是否程序执行结束
    std::string dir;                     // 文件夹路径
    std::ofstream logOut;                // 输出流

    bool rlTrafficLight;     // 是否使用强化学习控制 TrafficLight，如非则使用固定 TrafficLight phase
    bool laneChange;         // 是否允许变道
    int manuallyPushCnt = 0; // 通过接口手动 push 入的车辆

    int finishedVehicleCnt = 0;      // 累计穿行时间
    double cumulativeTravelTime = 0; // 累计穿行时间

  private:
    void vehicleControl(Vehicle &vehicle, std::vector<std::pair<Vehicle *, double>> &buffer); // speed、dis 计算，offset 计算并判断是否完成 laneChange

    void planRoute(); //主线程，待子线程处理完 route 后将有效 vehicle 转入 lane 的 waitingBuffer

    void getAction(); // 主线程，交由子线程负责车辆数据计算

    void updateAction(); // 主线程，交由子线程对每个 vehicle 信息进行更新

    void updateLocation(); // 主线程，交由子线程将离开的 vehicle 从原 drivable 删去并记录跑完 route 的数据，主线程将其加入新 drivable

    void updateLeaderAndGap(); // 主线程，交由子线程更新每个 drivable 上车辆的 leader 与 gap，并更新 lane 的 historyRecord

    void planLaneChange(); // 主线程，子线程判断是否可 laneChange，主线程进行 insertShadow

    void threadController(std::set<Vehicle *> &vehicles, std::vector<Road *> &roads, std::vector<Intersection *> &intersections,
                          std::vector<Drivable *> &drivables); // 子线程创建

    void threadPlanRoute(const std::vector<Road *> &roads); // 对各 vehicles 判断是否可 laneChange

    void threadGetAction(std::set<Vehicle *> &vehicles); // vehicle 各数据计算

    void threadUpdateAction(std::set<Vehicle *> &vehicles); // vehicle 信息更新

    void threadUpdateLeaderAndGap(const std::vector<Drivable *> &drivables); // 更新 drivable 上每辆车与前车的距离

    void threadUpdateLocation(const std::vector<Drivable *> &drivables); // 从各 drivable 去除离开的 vehicle，记录完成 route 车辆，delete 不需要的

    void threadNotifyCross(const std::vector<Intersection *> &intersections); // 更新每个 cross 的信息

    void threadInitSegments(const std::vector<Road *> &roads); // 更新 lane 内各 segment 所含的 vehicle 信息

    void threadPlanLaneChange(const std::set<Vehicle *> &vehicles); // 对各 vehicles 判断是否可 laneChange

    void handleWaiting(); // 对每个 lane 的 waitingBuffer 的首车，判断其是否可入 lane。如可则进入并更新 leader 与 gap；如不可，则等下一个

    void updateLog(); // log 信息输出

    bool checkWarning(); // check data

    bool loadRoadNet(const std::string &jsonFile); // load RoadNet，并向线程池填入信息

    bool loadFlow(const std::string &jsonFilename); // load Flow

    std::vector<const Vehicle *> getRunningVehicles(bool includeWaiting = false) const;

    void scheduleLaneChange(); // 对 notifyBuffer 内满足要求的 vehicle 进行 insertShadow 操作

    void insertShadow(Vehicle *vehicle); // 创建 vehicle 的 shadow 并插入

  public:
    std::mt19937 rnd;

    Engine(const std::string &configFile, int threadNum);

    double getInterval() const {
        return interval;
    }

    bool hasLaneChange() const { // 是否允许 laneChange
        return laneChange;
    }

    bool loadConfig(const std::string &configFile); // 主 load，load Engine

    void notifyCross(); // 主线程，交由子线程更新每个 cross 的信息

    void nextStep(); // 执行过程

    bool checkPriority(int priority); // vehiclePool 中是否存在优先级为 priority 的车子

    void pushVehicle(Vehicle *const vehicle, bool pushToDrivable = true); // 手动添加车辆

    void setLogFile(const std::string &jsonFile, const std::string &logFile);

    void initSegments(); // 主线程，交由子线程完成 Segment 内车辆的更新

    ~Engine();

    // RL related api

    void pushVehicle(const std::map<std::string, double> &info, const std::vector<std::string> &roads); // 手动添加车辆

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

    bool setRoute(const std::string &vehicle_id, const std::vector<std::string> &anchor_id);

    void reset(bool resetRnd = false);

    // archive
    void load(const Archive &archive) {
        archive.resume(*this);
    }

    Archive snapshot() {
        return Archive(*this);
    }

    void loadFromFile(const char *fileName);
};

} // namespace CityFlow

#endif // CITYFLOW_ENGINE_H
