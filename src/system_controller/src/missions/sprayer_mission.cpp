#include "missions/sprayer_mission.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace system_controller {

SprayerMission::SprayerMission() 
    : MissionBase("sprayer_mission", "Precision Spraying Mission") 
{
    mission_type_ = "spraying";
    implement_type_ = "sprayer";
}

bool SprayerMission::initialize(const MissionConfig& config) 
{
    if (!MissionBase::initialize(config)) {
        return false;
    }
    
    // Load sprayer-specific configuration
    if (config.mission_params.count("chemical_type")) {
        sprayer_config_.chemical_type = config.mission_params.at("chemical_type");
    }
    if (config.mission_params.count("application_rate")) {
        sprayer_config_.application_rate = std::stod(config.mission_params.at("application_rate"));
    }
    if (config.mission_params.count("spray_pressure")) {
        sprayer_config_.spray_pressure = std::stod(config.mission_params.at("spray_pressure"));
    }
    if (config.mission_params.count("boom_height")) {
        sprayer_config_.boom_height = std::stod(config.mission_params.at("boom_height"));
    }
    
    // Initialize weather monitoring
    weather_data_.wind_speed = 5.0;  // km/h - simulated
    weather_data_.temperature = 20.0; // °C - simulated
    weather_data_.humidity = 65.0;    // % - simulated
    
    // Initialize GPS monitoring
    gps_data_.accuracy = 0.05;  // meters - simulated high accuracy
    gps_data_.latitude = 45.123456;  // simulated coordinates
    gps_data_.longitude = -93.654321;
    
    // Initialize operational state
    operational_state_.spray_active = false;
    operational_state_.boom_lowered = false;
    operational_state_.current_section = 0;
    operational_state_.area_sprayed = 0.0;
    operational_state_.chemical_used = 0.0;
    operational_state_.turn_in_progress = false;
    
    setStatus(MissionStatus::READY);
    return true;
}

bool SprayerMission::start() 
{
    if (status_ != MissionStatus::READY && status_ != MissionStatus::PAUSED) {
        return false;
    }
    
    // Pre-flight checks
    if (!performPreSprayChecks()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    // Lower boom to working position
    if (!lowerBoom()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    // Enable spray system
    if (!enableSpraySystem()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    operational_state_.spray_active = true;
    setStatus(MissionStatus::ACTIVE);
    
    RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), 
                "Sprayer mission started - %s application at %.1f L/ha",
                sprayer_config_.chemical_type.c_str(),
                sprayer_config_.application_rate);
    
    return true;
}

bool SprayerMission::pause() 
{
    if (status_ != MissionStatus::ACTIVE) {
        return false;
    }
    
    // Lift boom and stop spraying for pause
    operational_state_.spray_active = false;
    disableSpraySystem();
    raiseBoom();
    
    setStatus(MissionStatus::PAUSED);
    
    RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "Sprayer mission paused");
    return true;
}

bool SprayerMission::resume() 
{
    if (status_ != MissionStatus::PAUSED) {
        return false;
    }
    
    // Pre-resumption checks
    if (!performPreSprayChecks()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    // Resume spraying
    lowerBoom();
    enableSpraySystem();
    operational_state_.spray_active = true;
    
    setStatus(MissionStatus::ACTIVE);
    
    RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "Sprayer mission resumed");
    return true;
}

bool SprayerMission::stop() 
{
    // Graceful shutdown
    operational_state_.spray_active = false;
    disableSpraySystem();
    raiseBoom();
    
    setStatus(MissionStatus::COMPLETED);
    
    // Log mission summary
    RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), 
                "Sprayer mission completed - %.2f hectares sprayed, %.1f L used",
                operational_state_.area_sprayed,
                operational_state_.chemical_used);
    
    return true;
}

bool SprayerMission::abort() 
{
    // Emergency shutdown
    operational_state_.spray_active = false;
    disableSpraySystem();
    raiseBoom();
    
    setStatus(MissionStatus::ABORTED);
    
    RCLCPP_WARN(rclcpp::get_logger("sprayer_mission"), "Sprayer mission aborted");
    return true;
}

void SprayerMission::update() 
{
    if (status_ != MissionStatus::ACTIVE) {
        return;
    }
    
    // Update sensor data (in real implementation, this would come from sensors)
    updateWeatherData();
    updateGPSData();
    updateOperationalData();
    
    // Check safety conditions
    if (!checkSafetyConditions()) {
        RCLCPP_WARN(rclcpp::get_logger("sprayer_mission"), "Safety conditions violated - pausing spray");
        pause();
        return;
    }
    
    // Execute spray pattern logic
    executeSprayPattern();
    
    // Handle turn sequences
    handleTurnSequence();
    
    // Monitor spray quality
    monitorSprayQuality();
    
    // Update progress
    updateProgress();
}

MissionStatus SprayerMission::getStatus() const 
{
    return status_;
}

bool SprayerMission::performPreSprayChecks() 
{
    // Check tank level
    if (getTankLevel() < 10.0) {
        RCLCPP_ERROR(rclcpp::get_logger("sprayer_mission"), "Tank level too low: %.1f%%", getTankLevel());
        return false;
    }
    
    // Check weather conditions
    if (!checkWeatherConditions()) {
        return false;
    }
    
    // Check GPS accuracy
    if (!checkGPSAccuracy()) {
        return false;
    }
    
    // Check system status
    if (!checkSystemStatus()) {
        return false;
    }
    
    return true;
}

bool SprayerMission::checkWeatherConditions() 
{
    // Check wind speed
    if (weather_data_.wind_speed > sprayer_config_.max_wind_speed) {
        RCLCPP_WARN(rclcpp::get_logger("sprayer_mission"), 
                    "Wind speed too high: %.1f km/h (max: %.1f km/h)",
                    weather_data_.wind_speed, sprayer_config_.max_wind_speed);
        return false;
    }
    
    // Check temperature range
    if (weather_data_.temperature < sprayer_config_.min_temperature || 
        weather_data_.temperature > sprayer_config_.max_temperature) {
        RCLCPP_WARN(rclcpp::get_logger("sprayer_mission"), 
                    "Temperature out of range: %.1f°C (range: %.1f-%.1f°C)",
                    weather_data_.temperature, 
                    sprayer_config_.min_temperature,
                    sprayer_config_.max_temperature);
        return false;
    }
    
    // Check for precipitation
    if (weather_data_.precipitation > 0.1) {  // > 0.1 mm/hr
        RCLCPP_WARN(rclcpp::get_logger("sprayer_mission"), "Precipitation detected - not suitable for spraying");
        return false;
    }
    
    return true;
}

bool SprayerMission::checkGPSAccuracy() 
{
    if (sprayer_config_.gps_required && gps_data_.accuracy > sprayer_config_.gps_accuracy_threshold) {
        RCLCPP_WARN(rclcpp::get_logger("sprayer_mission"), 
                    "GPS accuracy insufficient: %.2fm (required: %.2fm)",
                    gps_data_.accuracy, sprayer_config_.gps_accuracy_threshold);
        return false;
    }
    
    return true;
}

bool SprayerMission::checkSystemStatus() 
{
    // Simulate system health checks
    if (getTankLevel() < 5.0) {
        RCLCPP_ERROR(rclcpp::get_logger("sprayer_mission"), "Critical tank level");
        return false;
    }
    
    // Check pump pressure
    if (getPumpPressure() < 1.0) {
        RCLCPP_ERROR(rclcpp::get_logger("sprayer_mission"), "Pump pressure too low");
        return false;
    }
    
    return true;
}

bool SprayerMission::lowerBoom() 
{
    RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "Lowering boom to %.2fm height", sprayer_config_.boom_height);
    operational_state_.boom_lowered = true;
    return true;
}

bool SprayerMission::raiseBoom() 
{
    RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "Raising boom to transport position");
    operational_state_.boom_lowered = false;
    return true;
}

bool SprayerMission::enableSpraySystem() 
{
    RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "Enabling spray system - %s at %.1f L/ha", 
                sprayer_config_.chemical_type.c_str(),
                sprayer_config_.application_rate);
    return true;
}

bool SprayerMission::disableSpraySystem() 
{
    RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "Disabling spray system");
    return true;
}

void SprayerMission::executeSprayPattern() 
{
    if (!operational_state_.spray_active || !operational_state_.boom_lowered) {
        return;
    }
    
    // Business logic for spray pattern execution
    // In real implementation, this would control section valves based on GPS position
    
    // Check for tree detection (example business logic)
    if (detectTrees()) {
        RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "Trees detected - pausing spray");
        pauseSprayTemporarily();
    }
    
    // Check buffer zones
    if (inBufferZone()) {
        RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "In buffer zone - adjusting spray pattern");
        adjustSprayForBufferZone();
    }
    
    // Calculate application rate based on ground speed
    adjustApplicationRate();
}

void SprayerMission::handleTurnSequence() 
{
    // End-of-row turn logic - key business requirement
    if (approachingEndOfRow() && !operational_state_.turn_in_progress) {
        RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "End of row detected - initiating turn sequence");
        operational_state_.turn_in_progress = true;
        
        // Step 1: Disable spray sections
        disableSpraySystem();
        
        // Step 2: Raise boom for turn
        raiseBoom();
        
        RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "Turn sequence: Boom raised, spray disabled");
    }
    
    if (operational_state_.turn_in_progress && startOfNewRow()) {
        RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "Start of new row - completing turn sequence");
        
        // Step 3: Lower boom for next row
        lowerBoom();
        
        // Step 4: Re-enable spray system
        enableSpraySystem();
        
        operational_state_.turn_in_progress = false;
        operational_state_.current_section++;
        
        RCLCPP_INFO(rclcpp::get_logger("sprayer_mission"), "Turn sequence complete - starting section %d", 
                    operational_state_.current_section);
    }
}

void SprayerMission::monitorSprayQuality() 
{
    // Monitor for nozzle blockages
    if (detectNozzleBlockage()) {
        RCLCPP_WARN(rclcpp::get_logger("sprayer_mission"), "Nozzle blockage detected");
    }
    
    // Monitor boom position
    if (detectBoomDrift()) {
        RCLCPP_WARN(rclcpp::get_logger("sprayer_mission"), "Boom drift detected - adjusting");
    }
    
    // Monitor application uniformity
    if (!checkApplicationUniformity()) {
        RCLCPP_WARN(rclcpp::get_logger("sprayer_mission"), "Application uniformity issue detected");
    }
}

void SprayerMission::updateProgress() 
{
    // Calculate area covered
    double current_speed = 8.0;  // km/h - simulated
    double boom_width = 12.0;    // meters - simulated
    double time_delta = 1.0;     // seconds
    
    double area_increment = (current_speed * 1000.0 / 3600.0) * boom_width * time_delta / 10000.0; // hectares
    operational_state_.area_sprayed += area_increment;
    
    // Calculate chemical used
    double chemical_increment = area_increment * sprayer_config_.application_rate;
    operational_state_.chemical_used += chemical_increment;
    
    // Update completion percentage
    double total_planned_area = 100.0;  // hectares - would come from mission plan
    completion_percentage_ = (operational_state_.area_sprayed / total_planned_area) * 100.0;
}

void SprayerMission::updateWeatherData() 
{
    // Simulate weather data updates
    weather_data_.wind_speed += (rand() % 21 - 10) * 0.1;  // +/- 1 km/h variation
    weather_data_.wind_speed = std::max(0.0, std::min(25.0, weather_data_.wind_speed));
    
    weather_data_.temperature += (rand() % 21 - 10) * 0.05;  // +/- 0.5°C variation
    weather_data_.temperature = std::max(-10.0, std::min(45.0, weather_data_.temperature));
}

void SprayerMission::updateGPSData() 
{
    // Simulate GPS data updates
    gps_data_.accuracy = 0.02 + (rand() % 10) * 0.01;  // 2-12cm accuracy
    gps_data_.latitude += (rand() % 21 - 10) * 0.00001;
    gps_data_.longitude += (rand() % 21 - 10) * 0.00001;
}

void SprayerMission::updateOperationalData() 
{
    // Update tank level (simulate consumption)
    double consumption_rate = sprayer_config_.application_rate * 0.01;  // L per update
    tank_level_ = std::max(0.0, tank_level_ - consumption_rate);
}

bool SprayerMission::checkSafetyConditions() 
{
    return checkWeatherConditions() && 
           checkGPSAccuracy() && 
           checkSystemStatus() && 
           !inRestrictedArea();
}

// Utility and simulation methods
double SprayerMission::getTankLevel() const { return tank_level_; }
double SprayerMission::getPumpPressure() const { return 3.5; }  // bar - simulated
bool SprayerMission::detectTrees() const { return false; }  // Simulated - no trees detected
bool SprayerMission::inBufferZone() const { return false; }  // Simulated - not in buffer zone
bool SprayerMission::approachingEndOfRow() const { return operational_state_.current_section > 0 && (rand() % 1000) < 5; }
bool SprayerMission::startOfNewRow() const { return operational_state_.turn_in_progress && (rand() % 100) < 10; }
bool SprayerMission::detectNozzleBlockage() const { return (rand() % 10000) < 5; }
bool SprayerMission::detectBoomDrift() const { return (rand() % 5000) < 3; }
bool SprayerMission::checkApplicationUniformity() const { return (rand() % 1000) > 50; }
bool SprayerMission::inRestrictedArea() const { return false; }

void SprayerMission::pauseSprayTemporarily() {
    // Temporarily pause spray for obstacles
    RCLCPP_DEBUG(rclcpp::get_logger("sprayer_mission"), "Temporarily pausing spray");
}

void SprayerMission::adjustSprayForBufferZone() {
    // Reduce application rate in buffer zones
    RCLCPP_DEBUG(rclcpp::get_logger("sprayer_mission"), "Adjusting spray for buffer zone");
}

void SprayerMission::adjustApplicationRate() {
    // Adjust based on ground speed and conditions
    RCLCPP_DEBUG(rclcpp::get_logger("sprayer_mission"), "Adjusting application rate");
}

} // namespace system_controller 