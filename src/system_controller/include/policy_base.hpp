#ifndef POLICY_BASE_HPP
#define POLICY_BASE_HPP

#include <string>
#include <memory>

namespace system_controller {

class PolicyBase {
public:
    PolicyBase(const std::string& name) : name_(name) {}
    virtual ~PolicyBase() = default;
    
    virtual std::string getCommand() = 0;
    virtual void update() {}
    virtual bool isActive() const { return active_; }
    virtual void setActive(bool active) { active_ = active; }
    
    std::string getName() const { return name_; }

protected:
    std::string name_;
    bool active_ = true;
};

class Policy1 : public PolicyBase {
public:
    Policy1() : PolicyBase("Policy1") {}
    std::string getCommand() override;
};

class Policy2 : public PolicyBase {
public:
    Policy2() : PolicyBase("Policy2") {}
    std::string getCommand() override;
};

class Policy3 : public PolicyBase {
public:
    Policy3() : PolicyBase("Policy3") {}
    std::string getCommand() override;
};

} // namespace system_controller

#endif // POLICY_BASE_HPP 