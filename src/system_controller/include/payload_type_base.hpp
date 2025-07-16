#ifndef PAYLOAD_TYPE_BASE_HPP
#define PAYLOAD_TYPE_BASE_HPP

#include <string>
#include <memory>

namespace system_controller {

class PayloadTypeBase {
public:
    PayloadTypeBase(const std::string& name) : name_(name) {}
    virtual ~PayloadTypeBase() = default;
    
    virtual void activate() = 0;
    virtual void deactivate() = 0;
    virtual std::string getStatus() = 0;
    virtual void processCommand(const std::string& command) {}
    virtual void configure(const std::string& config) {}
    
    std::string getName() const { return name_; }
    bool isActive() const { return active_; }

protected:
    std::string name_;
    bool active_ = false;
};

class PayloadType1 : public PayloadTypeBase {
public:
    PayloadType1() : PayloadTypeBase("PayloadType1") {}
    void activate() override;
    void deactivate() override;
    std::string getStatus() override;
    void processCommand(const std::string& command) override;
};

class PayloadType2 : public PayloadTypeBase {
public:
    PayloadType2() : PayloadTypeBase("PayloadType2") {}
    void activate() override;
    void deactivate() override;
    std::string getStatus() override;
    void processCommand(const std::string& command) override;
};

} // namespace system_controller

#endif // PAYLOAD_TYPE_BASE_HPP 