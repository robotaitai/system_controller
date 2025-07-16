#ifndef MISSION_BASE_HPP
#define MISSION_BASE_HPP

#include <string>
#include <memory>
#include <vector>
#include <map>
#include <functional>
#include "../adapters/implement_adapter_base.hpp"

namespace system_controller {

/**
 * @brief Mission configuration parameters
 */
struct MissionConfig {
    std::string mission_type;        // "sprayer", "mower", "seeder", etc.
    std::string mission_id;          // Unique mission instance ID
    std::string field_id;            // Field or area identifier
    double field_area = 0.0;         // Field area in hectares
    double working_width = 12.0;     // Working width in meters
    double overlap = 0.1;            // Overlap percentage (0.0-1.0)
    double target_speed = 15.0;      // Target working speed (km/h)
    std::string start_location;      // Starting location/waypoint
    std::string end_location;        // Ending location/waypoint
    std::map<std::string, std::string> custom_params; // Mission-specific parameters
};

/**
 * @brief Mission status information
 */
struct MissionStatus {
    enum State {
        IDLE = 0,
        STARTING = 1,
        WORKING = 2,
        END_OF_ROW = 3,
        TURNING = 4,
        PAUSED = 5,
        STOPPING = 6,
        COMPLETED = 7,
        ERROR = 8
    };
    
    State current_state = IDLE;
    double progress_percentage = 0.0; // 0.0 to 100.0
    double area_completed = 0.0;      // Hectares completed
    double area_remaining = 0.0;      // Hectares remaining
    uint32_t current_row = 0;         // Current row number
    uint32_t total_rows = 0;          // Total number of rows
    std::string current_implement;    // Currently active implement
    bool implement_active = false;    // Implement is actively working
    std::string last_error;           // Last error message
    uint64_t mission_start_time = 0;  // Mission start timestamp
    uint64_t estimated_completion_time = 0; // Estimated completion timestamp
};

/**
 * @brief Base class for all mission types
 * 
 * This class provides the standard interface that all mission-specific
 * classes must implement. It contains the business logic for each mission
 * type and manages the associated implement adapters.
 */
class MissionBase {
public:
    MissionBase(const std::string& mission_name);
    virtual ~MissionBase() = default;

    // Core mission interface
    virtual bool initialize(const MissionConfig& config) = 0;
    virtual bool start() = 0;
    virtual bool pause() = 0;
    virtual bool resume() = 0;
    virtual bool stop() = 0;
    virtual bool abort() = 0;
    
    // Mission execution
    virtual void update() = 0;  // Called periodically to update mission state
    virtual MissionStatus getStatus() const = 0;
    virtual double getProgress() const = 0;
    
    // Row management (for field operations)
    virtual bool startNewRow() = 0;
    virtual bool endCurrentRow() = 0;
    virtual bool performTurn() = 0;
    
    // Implement management
    virtual bool selectImplement(const std::string& implement_id) = 0;
    virtual bool activateImplement() = 0;
    virtual bool deactivateImplement() = 0;
    virtual std::vector<std::string> getRequiredImplements() const = 0;
    
    // Configuration management
    virtual bool configure(const std::string& param_name, const std::string& param_value) = 0;
    virtual std::string getParameter(const std::string& param_name) const = 0;
    virtual std::map<std::string, std::string> getAllParameters() const = 0;
    
    // Safety and validation
    virtual bool validateMissionConfig(const MissionConfig& config) const = 0;
    virtual bool isReadyToStart() const = 0;
    virtual bool performSafetyCheck() = 0;
    
    // Business logic hooks (implemented by derived classes)
    virtual void onMissionStart() {}
    virtual void onMissionComplete() {}
    virtual void onRowStart() {}
    virtual void onRowEnd() {}
    virtual void onTurnStart() {}
    virtual void onTurnEnd() {}
    virtual void onError(const std::string& error) {}
    
    // Getters
    std::string getMissionName() const { return mission_name_; }
    std::string getMissionType() const { return config_.mission_type; }
    std::string getMissionId() const { return config_.mission_id; }

protected:
    // Helper methods for derived classes
    void setError(const std::string& error);
    void clearError();
    void updateProgress(double percentage);
    void setState(MissionStatus::State state);
    bool isImplementConnected(const std::string& implement_id) const;
    
    // Member variables
    std::string mission_name_;
    MissionConfig config_;
    MissionStatus status_;
    std::string last_error_;
    
    // Implement adapter management
    std::map<std::string, std::shared_ptr<ImplementAdapterBase>> implement_adapters_;
    std::string active_implement_id_;
    
    // Callbacks
    std::function<void(const MissionStatus&)> status_callback_;
    std::function<void(const std::string&)> error_callback_;
    std::function<void(double)> progress_callback_;

public:
    // Implement adapter management
    void addImplementAdapter(const std::string& implement_id, 
                           std::shared_ptr<ImplementAdapterBase> adapter);
    void removeImplementAdapter(const std::string& implement_id);
    std::shared_ptr<ImplementAdapterBase> getImplementAdapter(const std::string& implement_id);
    
    // Callback setters
    void setStatusCallback(std::function<void(const MissionStatus&)> callback) {
        status_callback_ = callback;
    }
    
    void setErrorCallback(std::function<void(const std::string&)> callback) {
        error_callback_ = callback;
    }
    
    void setProgressCallback(std::function<void(double)> callback) {
        progress_callback_ = callback;
    }
};

} // namespace system_controller

#endif // MISSION_BASE_HPP 