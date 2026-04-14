# C++ 状态机库 (Finite State Machine Library)

一个简洁、优雅的 C++17 状态机库，支持状态转换、守卫条件、动作回调、消息订阅和异常处理。

## 特性

- ✅ **简洁的 API** - 直观的接口设计，易于使用
- ✅ **类型安全** - 基于模板和 `std::any` 的类型安全事件处理
- ✅ **守卫条件** - 支持条件判断的状态转换
- ✅ **动作回调** - 进入/离开状态和执行转换时的回调函数
- ✅ **消息订阅** - 发布/订阅模式的消息系统
- ✅ **异常处理** - 完善的异常处理机制
- ✅ **零外部依赖** - 仅需 C++17 标准库

## 文件结构

```
├── state_machine.hpp    # 状态机核心库（单头文件）
├── robot_example.cpp    # 机器人移动示例程序
└── README.md           # 说明文档
```

## 快速开始

### 1. 定义上下文数据

```cpp
struct RobotContext {
    double x = 0.0;
    double y = 0.0;
    double battery = 100.0;
    bool obstacleDetected = false;
};
```

### 2. 定义事件

```cpp
struct StartEvent {
    std::string target;
    StartEvent(const std::string& t) : target(t) {}
};

struct ObstacleDetectedEvent {
    double distance;
    ObstacleDetectedEvent(double d) : distance(d) {}
};

struct StopEvent {};
```

### 3. 创建状态类

```cpp
class IdleState : public fsm::State<RobotContext> {
public:
    std::string name() const override { return "Idle"; }
    
    void onEnter(RobotContext& context) override {
        std::cout << "[Idle] Robot is idle." << std::endl;
    }
    
    void onExit(RobotContext& context) override {
        std::cout << "[Idle] Leaving idle state." << std::endl;
    }
};

class MovingState : public fsm::State<RobotContext> {
public:
    std::string name() const override { return "Moving"; }
    
    void onEnter(RobotContext& context) override {
        std::cout << "[Moving] Started moving." << std::endl;
    }
};
```

### 4. 配置状态机

```cpp
// 创建上下文和状态机
RobotContext robot;
fsm::StateMachine<RobotContext> sm(robot);

// 创建状态
auto idleState = std::make_shared<IdleState>();
auto movingState = std::make_shared<MovingState>();

// 注册状态
sm.addState(idleState);
sm.addState(movingState);

// 添加状态转换（带守卫条件和动作）
sm.addTransition("Idle", "Moving", typeid(StartEvent),
    // 守卫条件：目标不能为空
    [](const RobotContext& ctx, const std::any& event) {
        auto e = std::any_cast<const StartEvent&>(event);
        return !e.target.empty();
    },
    // 转换动作：设置目标位置
    [](RobotContext& ctx, const std::any& event) {
        auto e = std::any_cast<const StartEvent&>(event);
        ctx.targetLocation = e.target;
    });

// 添加简单转换
sm.addTransition("Moving", "Idle", typeid(StopEvent));

// 启动状态机
sm.start("Idle");
```

### 5. 处理事件

```cpp
// 发送事件
StartEvent startEvt("Warehouse-A");
sm.handleEvent(startEvt);

// 检查当前状态
std::cout << "Current state: " << sm.getCurrentStateName() << std::endl;

// 停止状态机
sm.stop();
```

## 消息订阅

```cpp
// 订阅消息主题
sm.getSubscriber().subscribe("position_update", 
    [](RobotContext& ctx, const std::any& msg) {
        std::cout << "Position updated!" << std::endl;
    });

// 发布消息
sm.publishMessage("position_update", robot.x);
```

## 异常处理

库提供以下异常类：

- `StateMachineException` - 基础异常类
- `InvalidTransitionException` - 无效的状态转换
- `EventHandlingException` - 事件处理错误

```cpp
try {
    sm.handleEvent(event);
} catch (const fsm::StateMachineException& e) {
    std::cerr << "State Machine Error: " << e.what() << std::endl;
} catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
}
```

## 编译示例

```bash
# 编译示例程序
g++ -std=c++17 -o robot_example robot_example.cpp

# 运行示例
./robot_example
```

## API 参考

### StateMachine<ContextType>

| 方法 | 描述 |
|------|------|
| `addState(state)` | 注册状态 |
| `addTransition(from, to, eventType, guard, action)` | 添加状态转换 |
| `start(initialState)` | 启动状态机 |
| `handleEvent(event)` | 处理事件 |
| `stop()` | 停止状态机 |
| `getCurrentStateName()` | 获取当前状态名 |
| `getSubscriber()` | 获取消息订阅器 |
| `publishMessage(topic, message)` | 发布消息 |

### State<ContextType>

| 方法 | 描述 |
|------|------|
| `name()` | 返回状态名称（纯虚函数） |
| `onEnter(context)` | 进入状态时调用 |
| `onExit(context)` | 离开状态时调用 |

### Transition 参数

- `from` / `to`: 源状态和目标状态
- `eventType`: 使用 `typeid(EventType)` 指定触发事件类型
- `guard`: 可选，守卫条件函数，返回 `bool`
- `action`: 可选，转换动作函数

## 完整示例

查看 `robot_example.cpp` 了解完整的机器人移动示例，包括：
- 6 种状态（空闲、移动、避障、充电、到达、错误）
- 多种事件类型
- 守卫条件判断
- 消息订阅系统
- 异常处理

## 许可证

MIT License
