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
        int urgency;     // 优先级
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

    double leaderGap;   // 与前车间隔
    double followerGap; // 与后车间隔
    double waitingTime = 0;

    bool changing = false;                   // true 则已 insertShadow
    bool finished = false;                   // change 完成，将被 remove
    double lastChangeTime = 0;               // 上一次 laneChagne 的时间记录
    static constexpr double coolingTime = 3; // lanechange 冷却时间

  public:
    LaneChange(Vehicle *vehicle, const LaneChange &other);

    explicit LaneChange(Vehicle *vehicle) : vehicle(vehicle){};

    virtual ~LaneChange() = default;

    void updateLeaderAndFollower();

    Lane *getTarget() const;

    Vehicle *getTargetLeader() const {
        return targetLeader;
    }

    Vehicle *getTargetFollower() const {
        return targetFollower;
    }

    double gapBefore() const;

    double gapAfter() const;

    void insertShadow(Vehicle *shadow);

    virtual double safeGapBefore() const = 0;
    virtual double safeGapAfter() const = 0;

    virtual void makeSignal(double interval) { // 调用虚函数设置 direction
        if (signalSend)
            signalSend->direction = getDirection();
    };

    bool planChange() const;

    bool canChange() const {              // 自己发送了 signal 且未收到信号
        return signalSend && !signalRecv; // 如 receive 则说明接收的 signal 的 sender 车辆优先级更高，所以暂不 change
    }

    bool isGapValid() const { // 是否都满足安全距离
        return gapAfter() >= safeGapAfter() && gapBefore() >= safeGapBefore();
    }

    void finishChanging();

    void abortChanging();

    virtual double yieldSpeed(double interval) = 0;

    virtual void sendSignal() = 0;

    int getDirection();

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

    void makeSignal(double interval) override;
    void sendSignal() override;

    double yieldSpeed(double interval) override;

    double safeGapBefore() const override;

    double safeGapAfter() const override;
};
} // namespace CityFlow

#endif // CITYFLOW_LANECHANGE_H
