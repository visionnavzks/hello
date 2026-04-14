#include "state_machine.hpp"
#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <chrono>

// ==================== 机器人上下文数据 ====================
struct RobotContext {
    double x = 0.0;           // X坐标
    double y = 0.0;           // Y坐标
    double angle = 0.0;       // 朝向角度
    double battery = 100.0;   // 电量百分比
    bool obstacleDetected = false;  // 是否检测到障碍物
    std::string targetLocation = ""; // 目标位置
    int moveCount = 0;        // 移动次数
    
    void printStatus() const {
        std::cout << "Robot Status: Position(" << x << ", " << y 
                  << "), Angle: " << angle << ", Battery: " << battery 
                  << "%, Obstacle: " << (obstacleDetected ? "Yes" : "No")
                  << ", Target: " << targetLocation << std::endl;
    }
};

// ==================== 事件定义 ====================
struct StartEvent {
    std::string target;
    StartEvent(const std::string& t) : target(t) {}
};

struct ObstacleDetectedEvent {
    double distance;
    ObstacleDetectedEvent(double d) : distance(d) {}
};

struct ObstacleClearedEvent {};

struct BatteryLowEvent {
    double level;
    BatteryLowEvent(double l) : level(l) {}
};

struct ArrivedEvent {};

struct StopEvent {};

struct ContinueEvent {};

// 定时器超时事件（用于演示）
struct TimeoutEvent {
    std::string reason;
    TimeoutEvent(const std::string& r) : reason(r) {}
};

// ==================== 状态实现 ====================

// 空闲状态
class IdleState : public fsm::State<RobotContext> {
public:
    std::string name() const override { return "Idle"; }
    
    void onEnter(RobotContext& context) override {
        std::cout << "[Idle] Robot is idle. Waiting for commands." << std::endl;
    }
};

// 移动状态
class MovingState : public fsm::State<RobotContext> {
public:
    std::string name() const override { return "Moving"; }
    
    void onEnter(RobotContext& context) override {
        std::cout << "[Moving] Started moving to: " << context.targetLocation << std::endl;
        context.moveCount++;
    }
    
    void onExit(RobotContext& context) override {
        std::cout << "[Moving] Stopped moving. Total moves: " << context.moveCount << std::endl;
    }
};

// 避障状态
class AvoidingObstacleState : public fsm::State<RobotContext> {
public:
    std::string name() const override { return "AvoidingObstacle"; }
    
    void onEnter(RobotContext& context) override {
        std::cout << "[AvoidingObstacle] Obstacle detected! Initiating avoidance maneuver." << std::endl;
        context.obstacleDetected = true;
    }
    
    void onExit(RobotContext& context) override {
        std::cout << "[AvoidingObstacle] Obstacle cleared. Resuming normal operation." << std::endl;
        context.obstacleDetected = false;
    }
};

// 充电状态
class ChargingState : public fsm::State<RobotContext> {
public:
    std::string name() const override { return "Charging"; }
    
    void onEnter(RobotContext& context) override {
        std::cout << "[Charging] Battery low! Starting charging process." << std::endl;
    }
    
    void onExit(RobotContext& context) override {
        std::cout << "[Charging] Charging complete. Battery: " << context.battery << "%" << std::endl;
    }
};

// 到达状态
class ArrivedState : public fsm::State<RobotContext> {
public:
    std::string name() const override { return "Arrived"; }
    
    void onEnter(RobotContext& context) override {
        std::cout << "[Arrived] Successfully reached destination: " << context.targetLocation << "!" << std::endl;
    }
};

// 错误状态
class ErrorState : public fsm::State<RobotContext> {
public:
    std::string name() const override { return "Error"; }
    
    void onEnter(RobotContext& context) override {
        std::cout << "[Error] System error occurred. Requires manual intervention." << std::endl;
    }
    
    void onExit(RobotContext& context) override {
        std::cout << "[Error] Recovered from error state." << std::endl;
    }
};

// 等待状态（演示定时器用）
class WaitingState : public fsm::State<RobotContext> {
public:
    std::string name() const override { return "Waiting"; }
    
    void onEnter(RobotContext& context) override {
        std::cout << "[Waiting] Entered waiting state. Timer will trigger timeout." << std::endl;
    }
    
    void onExit(RobotContext& context) override {
        std::cout << "[Waiting] Exiting waiting state." << std::endl;
    }
};

// ==================== 辅助函数 ====================
void simulateMovement(RobotContext& context) {
    // 模拟向目标移动
    if (!context.targetLocation.empty()) {
        context.x += 1.0;
        context.y += 1.0;
        std::cout << "  -> Moved to (" << context.x << ", " << context.y << ")" << std::endl;
    }
}

int main() {
    std::cout << "=== Robot State Machine Demo ===" << std::endl << std::endl;
    
    try {
        // 创建机器人上下文
        RobotContext robot;
        
        // 创建状态机
        fsm::StateMachine<RobotContext> sm(robot);
        
        // 创建状态实例
        auto idleState = std::make_shared<IdleState>();
        auto movingState = std::make_shared<MovingState>();
        auto avoidingState = std::make_shared<AvoidingObstacleState>();
        auto chargingState = std::make_shared<ChargingState>();
        auto arrivedState = std::make_shared<ArrivedState>();
        auto errorState = std::make_shared<ErrorState>();
        auto waitingState = std::make_shared<WaitingState>();
        
        // 注册所有状态
        sm.addState(idleState);
        sm.addState(movingState);
        sm.addState(avoidingState);
        sm.addState(chargingState);
        sm.addState(arrivedState);
        sm.addState(errorState);
        sm.addState(waitingState);
        
        // 定义状态转换
        
        // 从 Idle 开始移动
        sm.addTransition("Idle", "Moving", typeid(StartEvent),
            [](const RobotContext& ctx, const std::any& event) {
                auto e = std::any_cast<const StartEvent&>(event);
                return !e.target.empty();
            },
            [](RobotContext& ctx, const std::any& event) {
                auto e = std::any_cast<const StartEvent&>(event);
                ctx.targetLocation = e.target;
                std::cout << "  -> Set target to: " << e.target << std::endl;
            });
        
        // 移动中检测到障碍物
        sm.addTransition("Moving", "AvoidingObstacle", typeid(ObstacleDetectedEvent),
            [](const RobotContext& ctx, const std::any& event) {
                auto e = std::any_cast<const ObstacleDetectedEvent&>(event);
                return e.distance < 2.0;  // 距离小于2米时触发
            },
            [](RobotContext& ctx, const std::any& event) {
                auto e = std::any_cast<const ObstacleDetectedEvent&>(event);
                std::cout << "  -> Obstacle at distance: " << e.distance << "m" << std::endl;
            });
        
        // 避障完成，继续移动
        sm.addTransition("AvoidingObstacle", "Moving", typeid(ObstacleClearedEvent));
        
        // 移动中电量低
        sm.addTransition("Moving", "Charging", typeid(BatteryLowEvent),
            [](const RobotContext& ctx, const std::any& event) {
                return ctx.battery < 20.0;
            });
        
        // 空闲时电量低也可以去充电
        sm.addTransition("Idle", "Charging", typeid(BatteryLowEvent),
            [](const RobotContext& ctx, const std::any& event) {
                return ctx.battery < 20.0;
            });
        
        // 充电完成回到空闲
        sm.addTransition("Charging", "Idle", typeid(ContinueEvent),
            [](const RobotContext& ctx, const std::any& event) {
                return ctx.battery >= 80.0;
            },
            [](RobotContext& ctx, const std::any& event) {
                ctx.battery = 100.0;
                std::cout << "  -> Battery charged to 100%" << std::endl;
            });
        
        // 到达目的地
        sm.addTransition("Moving", "Arrived", typeid(ArrivedEvent));
        
        // 到达后回到空闲
        sm.addTransition("Arrived", "Idle", typeid(ContinueEvent));
        
        // 任何状态都可以因为严重错误进入错误状态（这里简化为从 Moving）
        sm.addTransition("Moving", "Error", typeid(StopEvent));
        
        // 错误恢复
        sm.addTransition("Error", "Idle", typeid(ContinueEvent));
        
        // 停止到空闲
        sm.addTransition("Moving", "Idle", typeid(StopEvent));
        sm.addTransition("AvoidingObstacle", "Idle", typeid(StopEvent));
        
        // 定时器演示：从 Idle 进入 Waiting 状态
        sm.addTransition("Idle", "Waiting", typeid(StartEvent),
            [](const RobotContext& ctx, const std::any& event) {
                auto e = std::any_cast<const StartEvent&>(event);
                return e.target == "WAIT";
            });
        
        // Waiting 状态超时后回到 Idle（使用 TimerEvent）
        sm.addTransition("Waiting", "Idle", typeid(fsm::TimerEvent),
            [](const RobotContext& ctx, const std::any& event) {
                auto e = std::any_cast<const fsm::TimerEvent&>(event);
                return e.timerId == "wait_timeout";
            },
            [](RobotContext& ctx, const std::any& event) {
                auto e = std::any_cast<const fsm::TimerEvent&>(event);
                std::cout << "  -> Timer expired: " << e.timerId << std::endl;
            });
        
        // 也可以使用 TimeoutEvent
        sm.addTransition("Waiting", "Idle", typeid(TimeoutEvent));
        
        // ==================== 订阅消息 ====================
        std::cout << "Setting up message subscribers..." << std::endl;
        
        // 订阅位置更新消息
        sm.getSubscriber().subscribe("position_update", 
            [](RobotContext& ctx, const std::any& msg) {
                std::cout << "[Subscriber] Position update received!" << std::endl;
            });
        
        // 订阅电池状态消息
        sm.getSubscriber().subscribe("battery_status",
            [](RobotContext& ctx, const std::any& msg) {
                std::cout << "[Subscriber] Battery status check!" << std::endl;
            });
        
        // ==================== 运行状态机 ====================
        std::cout << "\n--- Starting State Machine ---" << std::endl;
        sm.start("Idle");
        std::cout << "Current state: " << sm.getCurrentStateName() << std::endl << std::endl;
        
        // 场景 1: 开始移动到目标
        std::cout << "--- Scenario 1: Start Moving ---" << std::endl;
        StartEvent startEvt("Warehouse-A");
        sm.handleEvent(startEvt);
        std::cout << "Current state: " << sm.getCurrentStateName() << std::endl;
        simulateMovement(robot);
        sm.publishMessage("position_update", robot.x);
        std::cout << std::endl;
        
        // 场景 2: 检测到障碍物
        std::cout << "--- Scenario 2: Obstacle Detected ---" << std::endl;
        ObstacleDetectedEvent obstacleEvt(1.5);
        sm.handleEvent(obstacleEvt);
        std::cout << "Current state: " << sm.getCurrentStateName() << std::endl;
        std::cout << std::endl;
        
        // 场景 3: 障碍物清除
        std::cout << "--- Scenario 3: Obstacle Cleared ---" << std::endl;
        ObstacleClearedEvent clearedEvt;
        sm.handleEvent(clearedEvt);
        std::cout << "Current state: " << sm.getCurrentStateName() << std::endl;
        simulateMovement(robot);
        std::cout << std::endl;
        
        // 场景 4: 继续移动并到达
        std::cout << "--- Scenario 4: Arrived at Destination ---" << std::endl;
        ArrivedEvent arrivedEvt;
        sm.handleEvent(arrivedEvt);
        std::cout << "Current state: " << sm.getCurrentStateName() << std::endl;
        std::cout << std::endl;
        
        // 场景 5: 回到空闲
        std::cout << "--- Scenario 5: Return to Idle ---" << std::endl;
        ContinueEvent contEvt;
        sm.handleEvent(contEvt);
        std::cout << "Current state: " << sm.getCurrentStateName() << std::endl;
        std::cout << std::endl;
        
        // 场景 6: 电量低测试
        std::cout << "--- Scenario 6: Low Battery ---" << std::endl;
        robot.battery = 15.0;
        BatteryLowEvent batteryEvt(robot.battery);
        sm.handleEvent(batteryEvt);
        std::cout << "Current state: " << sm.getCurrentStateName() << std::endl;
        sm.publishMessage("battery_status", robot.battery);
        std::cout << std::endl;
        
        // 场景 7: 充电完成（先模拟充电）
        std::cout << "--- Scenario 7: Charging Complete ---" << std::endl;
        robot.battery = 85.0;  // 模拟充电到 85%
        ContinueEvent contEvt2;
        sm.handleEvent(contEvt2);
        std::cout << "Current state: " << sm.getCurrentStateName() << std::endl;
        std::cout << std::endl;
        
        // 场景 8: 再次移动并测试停止
        std::cout << "--- Scenario 8: Move and Stop ---" << std::endl;
        
        // 从 Idle 进入 Waiting 状态来演示定时器
        StartEvent waitEvt("WAIT");
        sm.handleEvent(waitEvt);
        std::cout << "Current state: " << sm.getCurrentStateName() << std::endl;
        
        // 启动一次性定时器，1 秒后触发 TimerEvent
        std::cout << "\n--- Scenario 9: Timer Event Demo ---" << std::endl;
        std::cout << "Starting timer 'wait_timeout' for 1 second..." << std::endl;
        sm.startTimer("wait_timeout", std::chrono::milliseconds(1000), false);
        std::cout << "Timer started, waiting for timeout..." << std::endl;
        
        // 等待定时器触发（实际应用中不需要 sleep，事件会自动处理）
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        std::cout << "Current state after timer: " << sm.getCurrentStateName() << std::endl;
        std::cout << std::endl;
        
        // 场景 10: 重复定时器演示
        std::cout << "--- Scenario 10: Repeating Timer Demo ---" << std::endl;
        int tickCount = 0;
        
        // 订阅定时器回调
        sm.getSubscriber().subscribe("timer_tick",
            [&tickCount](RobotContext& ctx, const std::any& msg) {
                tickCount++;
                std::cout << "[Subscriber] Timer tick #" << tickCount << std::endl;
            });
        
        // 启动重复定时器，每 500ms 触发一次
        sm.startTimer("repeat_timer", std::chrono::milliseconds(500), true,
            [&sm, &tickCount](const std::string& timerId) {
                sm.publishMessage("timer_tick", tickCount);
                if (tickCount >= 3) {
                    sm.stopTimer(timerId);
                    std::cout << "Stopped repeating timer after 3 ticks." << std::endl;
                }
            });
        
        std::cout << "Repeating timer started..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1800));
        std::cout << "Total ticks received: " << tickCount << std::endl;
        std::cout << std::endl;
        
        // 打印最终状态
        std::cout << "--- Final Status ---" << std::endl;
        robot.printStatus();
        
        // 停止所有定时器和状态机
        sm.stopAllTimers();
        sm.stop();
        std::cout << "\nState machine stopped." << std::endl;
        
    } catch (const fsm::StateMachineException& e) {
        std::cerr << "State Machine Exception: " << e.what() << std::endl;
        return 1;
    } catch (const std::bad_any_cast& e) {
        std::cerr << "Type Cast Exception: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "\n=== Demo Completed Successfully ===" << std::endl;
    return 0;
}
