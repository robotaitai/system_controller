#ifndef MISSION_TYPE_BASE_HPP
#define MISSION_TYPE_BASE_HPP

#include <string>
#include <memory>
#include <vector>

namespace system_controller {

class MissionTypeBase {
public:
    MissionTypeBase(const std::string& name) : name_(name) {}
    virtual ~MissionTypeBase() = default;
    
    virtual void startMission() = 0;
    virtual void stopMission() = 0;
    virtual std::string getStatus() = 0;
    virtual float getProgress() = 0;
    virtual void updateMission() {}
    
    std::string getName() const { return name_; }
    bool isActive() const { return active_; }

protected:
    std::string name_;
    bool active_ = false;
    float progress_ = 0.0f;
};

class MissionType1 : public MissionTypeBase {
public:
    MissionType1() : MissionTypeBase("MissionType1") {}
    void startMission() override;
    void stopMission() override;
    std::string getStatus() override;
    float getProgress() override;
    void updateMission() override;
};

class MissionType2 : public MissionTypeBase {
public:
    MissionType2() : MissionTypeBase("MissionType2") {}
    void startMission() override;
    void stopMission() override;
    std::string getStatus() override;
    float getProgress() override;
    void updateMission() override;
};

} // namespace system_controller

#endif // MISSION_TYPE_BASE_HPP 