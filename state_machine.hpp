#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <iostream>
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <functional>
#include <stdexcept>
#include <typeindex>
#include <any>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

namespace fsm {

// 前向声明
template<typename ContextType>
class StateMachine;

// 状态基类
template<typename ContextType>
class State {
public:
    virtual ~State() = default;
    
    // 进入状态时调用
    virtual void onEnter(ContextType& context) {}
    
    // 离开状态时调用
    virtual void onExit(ContextType& context) {}
    
    // 获取状态名称
    virtual std::string name() const = 0;
};

// 状态机上下文（包含共享数据）
template<typename ContextType>
class StateContext {
public:
    explicit StateContext(ContextType& data) : data_(data) {}
    
    ContextType& getData() { return data_; }
    const ContextType& getData() const { return data_; }
    
private:
    ContextType& data_;
};

// 事件基类
struct Event {
    virtual ~Event() = default;
    virtual std::string name() const = 0;
};

// 转换定义
template<typename ContextType>
struct Transition {
    using StatePtr = std::shared_ptr<State<ContextType>>;
    using GuardFunc = std::function<bool(const ContextType&, const std::any&)>;
    using ActionFunc = std::function<void(ContextType&, const std::any&)>;
    
    StatePtr from;
    StatePtr to;
    std::type_index eventType;
    GuardFunc guard;      // 守卫条件，返回true才允许转换
    ActionFunc action;    // 转换动作
    
    Transition(StatePtr f, StatePtr t, std::type_index et, 
               GuardFunc g = nullptr, ActionFunc a = nullptr)
        : from(f), to(t), eventType(et), guard(g), action(a) {}
    
    bool canExecute(const ContextType& context, const std::any& event) const {
        if (!guard) return true;
        try {
            return guard(context, event);
        } catch (...) {
            return false;
        }
    }
    
    void execute(ContextType& context, const std::any& event) const {
        if (action) {
            action(context, event);
        }
    }
};

// 异常类
class StateMachineException : public std::runtime_error {
public:
    explicit StateMachineException(const std::string& msg) 
        : std::runtime_error(msg) {}
};

class InvalidTransitionException : public StateMachineException {
public:
    explicit InvalidTransitionException(const std::string& from, 
                                       const std::string& to,
                                       const std::string& event)
        : StateMachineException("Invalid transition from '" + from + 
                               "' to '" + to + "' on event '" + event + "'") {}
};

class EventHandlingException : public StateMachineException {
public:
    explicit EventHandlingException(const std::string& msg)
        : StateMachineException("Event handling error: " + msg) {}
};

// 定时器事件
struct TimerEvent : public Event {
    std::string timerId;
    std::chrono::milliseconds elapsed;
    
    TimerEvent(const std::string& id, std::chrono::milliseconds ms)
        : timerId(id), elapsed(ms) {}
    
    std::string name() const override {
        return "TimerEvent[" + timerId + "]";
    }
};

// 定时器配置
struct TimerConfig {
    std::string id;
    std::chrono::milliseconds duration;
    bool repeat;
    std::function<void(const std::string&)> callback;
    
    TimerConfig() : duration(0), repeat(false) {}
    
    TimerConfig(const std::string& timerId, 
                std::chrono::milliseconds dur,
                bool rep = false,
                std::function<void(const std::string&)> cb = nullptr)
        : id(timerId), duration(dur), repeat(rep), callback(cb) {}
};

// 定时器管理器
class TimerManager {
public:
    void startTimer(const TimerConfig& config) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        TimerInfo info;
        info.config = config;
        info.startTime = std::chrono::steady_clock::now();
        info.running = true;
        
        timers_[config.id] = info;
        
        // 启动定时器线程
        std::thread(&TimerManager::timerThread, this, config.id).detach();
    }
    
    void stopTimer(const std::string& timerId) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = timers_.find(timerId);
        if (it != timers_.end()) {
            it->second.running = false;
        }
    }
    
    void stopAllTimers() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& pair : timers_) {
            pair.second.running = false;
        }
    }
    
    bool isTimerRunning(const std::string& timerId) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = timers_.find(timerId);
        return it != timers_.end() && it->second.running;
    }
    
    void setCallback(std::function<void(const std::string&, const TimerEvent&)> cb) {
        callback_ = cb;
    }
    
private:
    struct TimerInfo {
        TimerConfig config;
        std::chrono::steady_clock::time_point startTime;
        bool running;
        
        TimerInfo() : running(false) {}
    };
    
    mutable std::mutex mutex_;
    std::map<std::string, TimerInfo> timers_;
    std::function<void(const std::string&, const TimerEvent&)> callback_;
    
    void timerThread(const std::string& timerId) {
        TimerInfo* info = nullptr;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto it = timers_.find(timerId);
            if (it == timers_.end()) return;
            info = &it->second;
        }
        
        while (info->running) {
            std::this_thread::sleep_for(info->config.duration);
            
            if (!info->running) break;
            
            // 触发定时器事件
            TimerEvent evt(timerId, info->config.duration);
            if (callback_) {
                callback_(timerId, evt);
            }
            
            // 执行定时器配置的回调
            if (info->config.callback) {
                info->config.callback(timerId);
            }
            
            // 如果不重复，停止定时器
            if (!info->config.repeat) {
                std::lock_guard<std::mutex> lock(mutex_);
                info->running = false;
            }
        }
    }
};

// 消息订阅器
template<typename ContextType>
class MessageSubscriber {
public:
    using CallbackFunc = std::function<void(ContextType&, const std::any&)>;
    
    void subscribe(const std::string& topic, CallbackFunc callback) {
        subscribers_[topic].push_back(callback);
    }
    
    void publish(const std::string& topic, ContextType& context, const std::any& message) {
        auto it = subscribers_.find(topic);
        if (it != subscribers_.end()) {
            for (auto& callback : it->second) {
                try {
                    callback(context, message);
                } catch (...) {
                    // 单个订阅者异常不影响其他订阅者
                    std::cerr << "Warning: Exception in subscriber for topic: " << topic << std::endl;
                }
            }
        }
    }
    
private:
    std::map<std::string, std::vector<CallbackFunc>> subscribers_;
};

// 状态机主类
template<typename ContextType>
class StateMachine {
public:
    using StatePtr = std::shared_ptr<State<ContextType>>;
    using TransitionPtr = std::shared_ptr<Transition<ContextType>>;
    
    explicit StateMachine(ContextType& context) 
        : context_(context), currentState_(nullptr), running_(false) {
        // 设置定时器回调，自动触发 TimerEvent
        timerManager_.setCallback([this](const std::string& timerId, const TimerEvent& evt) {
            this->handleEvent(evt);
        });
    }
    
    ~StateMachine() {
        stop();
    }
    
    // 注册状态
    void addState(StatePtr state) {
        if (!state) {
            throw StateMachineException("Cannot add null state");
        }
        states_[state->name()] = state;
    }
    
    // 添加转换
    void addTransition(StatePtr from, StatePtr to, 
                      std::type_index eventType,
                      typename Transition<ContextType>::GuardFunc guard = nullptr,
                      typename Transition<ContextType>::ActionFunc action = nullptr) {
        auto transition = std::make_shared<Transition<ContextType>>(
            from, to, eventType, guard, action);
        transitions_.push_back(transition);
    }
    
    // 便捷方法：通过状态名添加转换
    void addTransition(const std::string& fromName, const std::string& toName,
                      std::type_index eventType,
                      typename Transition<ContextType>::GuardFunc guard = nullptr,
                      typename Transition<ContextType>::ActionFunc action = nullptr) {
        auto fromIt = states_.find(fromName);
        auto toIt = states_.find(toName);
        
        if (fromIt == states_.end()) {
            throw StateMachineException("State not found: " + fromName);
        }
        if (toIt == states_.end()) {
            throw StateMachineException("State not found: " + toName);
        }
        
        addTransition(fromIt->second, toIt->second, eventType, guard, action);
    }
    
    // 启动状态机
    void start(StatePtr initialState) {
        if (!initialState) {
            throw StateMachineException("Initial state cannot be null");
        }
        
        if (running_) {
            throw StateMachineException("State machine is already running");
        }
        
        currentState_ = initialState;
        running_ = true;
        
        try {
            currentState_->onEnter(context_);
        } catch (...) {
            running_ = false;
            throw;
        }
    }
    
    // 通过状态名启动
    void start(const std::string& stateName) {
        auto it = states_.find(stateName);
        if (it == states_.end()) {
            throw StateMachineException("Initial state not found: " + stateName);
        }
        start(it->second);
    }
    
    // 处理事件
    bool handleEvent(const std::any& event) {
        if (!running_ || !currentState_) {
            throw StateMachineException("State machine is not running");
        }
        
        auto eventType = std::type_index(event.type());
        
        // 查找匹配的转换
        for (const auto& transition : transitions_) {
            if (transition->from == currentState_ && 
                transition->eventType == eventType &&
                transition->canExecute(context_, event)) {
                
                try {
                    // 离开当前状态
                    currentState_->onExit(context_);
                    
                    // 执行转换动作
                    transition->execute(context_, event);
                    
                    // 进入新状态
                    currentState_ = transition->to;
                    currentState_->onEnter(context_);
                    
                    return true;
                } catch (...) {
                    // 状态转换异常，尝试恢复
                    throw EventHandlingException("Failed to process event during transition");
                }
            }
        }
        
        // 没有匹配的转换
        return false;
    }
    
    // 类型安全的事件处理
    template<typename EventType>
    bool handleEvent(const EventType& event) {
        std::any eventAny = event;
        return handleEvent(eventAny);
    }
    
    // 停止状态机
    void stop() {
        if (running_ && currentState_) {
            try {
                currentState_->onExit(context_);
            } catch (...) {
                // 忽略退出时的异常
            }
            currentState_ = nullptr;
            running_ = false;
        }
    }
    
    // 获取当前状态
    StatePtr getCurrentState() const {
        return currentState_;
    }
    
    std::string getCurrentStateName() const {
        return currentState_ ? currentState_->name() : "None";
    }
    
    bool isRunning() const {
        return running_;
    }
    
    // 消息订阅功能
    MessageSubscriber<ContextType>& getSubscriber() {
        return subscriber_;
    }
    
    void publishMessage(const std::string& topic, const std::any& message) {
        subscriber_.publish(topic, context_, message);
    }
    
    template<typename MessageType>
    void publishMessage(const std::string& topic, const MessageType& message) {
        std::any messageAny = message;
        publishMessage(topic, messageAny);
    }
    
    // 定时器功能
    void startTimer(const std::string& timerId, 
                    std::chrono::milliseconds duration,
                    bool repeat = false) {
        TimerConfig config(timerId, duration, repeat);
        timerManager_.startTimer(config);
    }
    
    void startTimer(const std::string& timerId,
                    std::chrono::milliseconds duration,
                    bool repeat,
                    std::function<void(const std::string&)> callback) {
        TimerConfig config(timerId, duration, repeat, callback);
        timerManager_.startTimer(config);
    }
    
    void stopTimer(const std::string& timerId) {
        timerManager_.stopTimer(timerId);
    }
    
    void stopAllTimers() {
        timerManager_.stopAllTimers();
    }
    
    bool isTimerRunning(const std::string& timerId) const {
        return timerManager_.isTimerRunning(timerId);
    }
    
private:
    ContextType& context_;
    StatePtr currentState_;
    bool running_;
    std::map<std::string, StatePtr> states_;
    std::vector<TransitionPtr> transitions_;
    MessageSubscriber<ContextType> subscriber_;
    TimerManager timerManager_;
};

} // namespace fsm

#endif // STATE_MACHINE_HPP
