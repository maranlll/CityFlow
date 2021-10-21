#ifndef CITYFLOW_LANECHANGE_H
#define CITYFLOW_LANECHANGE_H

#include "roadnet/roadnet.h"

#include <memory>

namespace CityFlow {

class Vehicle;
class Lane;

class LaneChange {
    friend class Vehicle;
    friend class Archive;
    // The interface of lane changing
  protected:
    struct Signal {
        int urgency;     // 优先级，似乎都是 1
        int direction;   // -1 for left , 1 for right, 0 for unchanged
        Lane *target;    // signal 去向的 lane
        Vehicle *source; // signal 来源
        int response = 0;
        double extraSpace = 0;
    };

    int lastDir; // // -1 for left , 1 for right, 0 for unchanged

    std::shared_ptr<Signal> signalRecv; // 接受的信号
    std::shared_ptr<Signal> signalSend; // 发出的信号

    Vehicle *vehicle;                  // laneChange 对应的当前车辆
    Vehicle *targetLeader = nullptr;   // laneChange 后预计的前车
    Vehicle *targetFollower = nullptr; // laneChange 后预计的后车

    double leaderGap;                        // 与前车间隔
    double followerGap;                      // 与后车间隔
    double waitingTime = 0;                  // 自从 insertShadow 后等待时间
    bool changing = false;                   // true 则已 insertShadow
    bool finished = false;                   // change 完成，将被 remove
    double lastChangeTime = 0;               // 上一次完成 laneChagne 的时间记录
    static constexpr double coolingTime = 3; // lanechange 冷却时间

  public:
    LaneChange(Vehicle *vehicle, const LaneChange &other);

    explicit LaneChange(Vehicle *vehicle) : vehicle(vehicle){};

    virtual ~LaneChange() = default;

    void updateLeaderAndFollower(); // 更新 targetLeader 与 targetFollower 与 gap

    Lane *getTarget() const; // 将驶向的目标 lane

    Vehicle *getTargetLeader() const {
        return targetLeader;
    }

    Vehicle *getTargetFollower() const {
        return targetFollower;
    }

    double gapBefore() const; // targetLane 上与后车的距离

    double gapAfter() const; // targetLane 上与前车的距离

    void insertShadow(Vehicle *shadow); // 对 shadow controllerInfor 信息进行补足，并在 targetLane 中更新 shadow 的信息

    virtual double safeGapBefore() const = 0; // 后侧所需安全距离
    virtual double safeGapAfter() const = 0;  // 前侧所需安全距离

    virtual void makeSignal(double interval) { // 调用虚函数设置 direction
        if (signalSend)
            signalSend->direction = getDirection();
    };

    bool planChange() const; // 是否准备 laneChange

    bool canChange() const {              // 自己发送了 signal 且未 receive 信号，如receive 说明 receive 信号优先级更高
        return signalSend && !signalRecv; // 如 receive 则说明接收的 signal 的 sender 车辆优先级更高，所以暂不 change
    }

    bool isGapValid() const { // 是否都满足安全距离
        return gapAfter() >= safeGapAfter() && gapBefore() >= safeGapBefore();
    }

    void finishChanging(); // 由原 vehicle 调用，完成 laneChange，将 shadow 设置为新 vehicle

    void abortChanging(); // 由 shadow 调用，并将原 vehicle 的 laneChange 信息重置

    virtual double yieldSpeed(double interval) = 0; // 有车 laneChange 到自己前或后方时的让步速度

    virtual void sendSignal() = 0;

    int getDirection(); // directoin 确定  0：直行；-1: 内测；1：外侧

    void clearSignal();

    bool hasFinished() const {
        return this->finished;
    }
};

class SimpleLaneChange : public LaneChange {
  private:
    double estimateGap(const Lane *lane) const;

  public:
    explicit SimpleLaneChange(Vehicle *vehicle) : LaneChange(vehicle){};
    explicit SimpleLaneChange(Vehicle *vehicle, const LaneChange &other) : LaneChange(vehicle, other){};

    void makeSignal(double interval) override; // 创建 signalSend 并设置 signal 内各值并寻找目标 lane
    void sendSignal() override;                // targetLeader 与 targetFollower 接收 signal

    double yieldSpeed(double interval) override; // 有车 laneChange 到自己前或后方时的让步速度

    double safeGapBefore() const override;

    double safeGapAfter() const override;
};
} // namespace CityFlow

#endif // CITYFLOW_LANECHANGE_H
