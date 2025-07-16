#ifndef VEHICLE_TYPE_BASE_HPP
#define VEHICLE_TYPE_BASE_HPP

#include <string>
#include <memory>

namespace system_controller {

class VehicleTypeBase {
public:
    VehicleTypeBase(const std::string& name) : name_(name) {}
    virtual ~VehicleTypeBase() = default;
    
    virtual void processCommand(const std::string& command) = 0;
    virtual std::string getStatus() = 0;
    virtual void initialize() {}
    virtual void shutdown() {}
    
    std::string getName() const { return name_; }

protected:
    std::string name_;
    bool initialized_ = false;
};

class VehicleType1 : public VehicleTypeBase {
public:
    VehicleType1() : VehicleTypeBase("VehicleType1") {}
    void processCommand(const std::string& command) override;
    std::string getStatus() override;
    void initialize() override;
};

class VehicleType2 : public VehicleTypeBase {
public:
    VehicleType2() : VehicleTypeBase("VehicleType2") {}
    void processCommand(const std::string& command) override;
    std::string getStatus() override;
    void initialize() override;
};

} // namespace system_controller

#endif // VEHICLE_TYPE_BASE_HPP 