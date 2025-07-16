#include "missions/mower_mission.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace system_controller {

MowerMission::MowerMission() 
    : MissionBase("mower_mission", "Precision Mowing Mission") 
{
    mission_type_ = "mowing";
    implement_type_ = "mower";
}

bool MowerMission::initialize(const MissionConfig& config) 
{
    if (!MissionBase::initialize(config)) {
        return false;
    }
    
    // Load mower-specific configuration
    if (config.mission_params.count("cutting_height")) {
        mower_config_.cutting_height = std::stod(config.mission_params.at("cutting_height"));
    }
    if (config.mission_params.count("cutting_speed")) {
        mower_config_.cutting_speed = std::stod(config.mission_params.at("cutting_speed"));
    }
    if (config.mission_params.count("cutting_pattern")) {
        mower_config_.cutting_pattern = std::stoi(config.mission_params.at("cutting_pattern"));
    }
    if (config.mission_params.count("overlap_percentage")) {
        mower_config_.overlap_percentage = std::stod(config.mission_params.at("overlap_percentage"));
    }
    if (config.mission_params.count("mulching_mode")) {
        mower_config_.mulching_mode = (config.mission_params.at("mulching_mode") == "true");
    }
    
    // Initialize weather monitoring
    weather_data_.temperature = 22.0;    // °C - simulated
    weather_data_.humidity = 55.0;       // % - simulated
    weather_data_.precipitation = 0.0;   // mm/hr - simulated
    weather_data_.ground_wet = false;
    
    // Initialize terrain monitoring
    terrain_data_.current_slope = 2.0;   // degrees - simulated gentle slope
    terrain_data_.surface_roughness = 0.3; // relative scale
    terrain_data_.obstacles_detected = false;
    
    // Initialize grass assessment
    grass_data_.height = 0.08;           // 8cm grass height
    grass_data_.density = 85.0;          // good coverage
    grass_data_.moisture = 25.0;         // acceptable moisture
    grass_data_.growth_stage_ok = true;
    
    // Initialize operational state
    operational_state_.blades_engaged = false;
    operational_state_.deck_lowered = false;
    operational_state_.current_pattern = mower_config_.cutting_pattern;
    operational_state_.current_pass = 0;
    operational_state_.area_mowed = 0.0;
    operational_state_.cutting_time = 0.0;
    operational_state_.turn_in_progress = false;
    operational_state_.blade_wear = 100.0;
    
    setStatus(MissionStatus::READY);
    return true;
}

bool MowerMission::start() 
{
    if (status_ != MissionStatus::READY && status_ != MissionStatus::PAUSED) {
        return false;
    }
    
    // Pre-mow checks
    if (!performPreMowChecks()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    // Lower deck to cutting position
    if (!lowerDeck()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    // Engage cutting blades
    if (!engageBlades()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    operational_state_.blades_engaged = true;
    setStatus(MissionStatus::ACTIVE);
    
    RCLCPP_INFO(rclcpp::get_logger("mower_mission"), 
                "Mower mission started - cutting at %.1fcm height, pattern: %s",
                mower_config_.cutting_height * 100,
                mower_config_.cutting_pattern == 0 ? "stripe" : 
                mower_config_.cutting_pattern == 1 ? "spiral" : "random");
    
    return true;
}

bool MowerMission::pause() 
{
    if (status_ != MissionStatus::ACTIVE) {
        return false;
    }
    
    // Raise deck and disengage blades for pause
    operational_state_.blades_engaged = false;
    disengageBlades();
    raiseDeck();
    
    setStatus(MissionStatus::PAUSED);
    
    RCLCPP_INFO(rclcpp::get_logger("mower_mission"), "Mower mission paused");
    return true;
}

bool MowerMission::resume() 
{
    if (status_ != MissionStatus::PAUSED) {
        return false;
    }
    
    // Pre-resumption checks
    if (!performPreMowChecks()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    // Resume mowing
    lowerDeck();
    engageBlades();
    operational_state_.blades_engaged = true;
    
    setStatus(MissionStatus::ACTIVE);
    
    RCLCPP_INFO(rclcpp::get_logger("mower_mission"), "Mower mission resumed");
    return true;
}

bool MowerMission::stop() 
{
    // Graceful shutdown
    operational_state_.blades_engaged = false;
    disengageBlades();
    raiseDeck();
    
    setStatus(MissionStatus::COMPLETED);
    
    // Log mission summary
    RCLCPP_INFO(rclcpp::get_logger("mower_mission"), 
                "Mower mission completed - %.2f hectares mowed in %.1f hours",
                operational_state_.area_mowed,
                operational_state_.cutting_time);
    
    return true;
}

bool MowerMission::abort() 
{
    // Emergency shutdown
    operational_state_.blades_engaged = false;
    disengageBlades();
    raiseDeck();
    
    setStatus(MissionStatus::ABORTED);
    
    RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Mower mission aborted");
    return true;
}

void MowerMission::update() 
{
    if (status_ != MissionStatus::ACTIVE) {
        return;
    }
    
    // Update sensor data
    updateWeatherData();
    updateTerrainData();
    updateGrassData();
    updateOperationalData();
    
    // Check safety conditions
    if (!checkSafetyConditions()) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Safety conditions violated - pausing mower");
        pause();
        return;
    }
    
    // Execute cutting pattern logic
    switch (operational_state_.current_pattern) {
        case 0: executeStripeMowing(); break;
        case 1: executeSpiralMowing(); break;
        case 2: executeRandomMowing(); break;
        default: executeStripeMowing(); break;
    }
    
    // Handle turn sequences
    handleTurnSequence();
    
    // Monitor cutting quality
    monitorCuttingQuality();
    
    // Adjust cutting parameters
    adjustCuttingParameters();
    
    // Update progress
    updateProgress();
}

MissionStatus MowerMission::getStatus() const 
{
    return status_;
}

bool MowerMission::performPreMowChecks() 
{
    // Check weather conditions
    if (!checkMowingConditions()) {
        return false;
    }
    
    // Check grass conditions
    if (!checkGrassConditions()) {
        return false;
    }
    
    // Check terrain safety
    if (!checkTerrainSafety()) {
        return false;
    }
    
    // Check blade condition
    if (getBladeSharpness() < 30.0) {
        RCLCPP_ERROR(rclcpp::get_logger("mower_mission"), "Blade sharpness too low: %.1f%%", getBladeSharpness());
        return false;
    }
    
    return true;
}

bool MowerMission::checkMowingConditions() 
{
    // Check for wet ground conditions
    if (weather_data_.ground_wet || weather_data_.precipitation > 0.1) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Ground too wet for mowing");
        return false;
    }
    
    // Check temperature (avoid extremely hot conditions)
    if (weather_data_.temperature > 35.0) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Temperature too high for safe mowing: %.1f°C", weather_data_.temperature);
        return false;
    }
    
    // Check humidity (high humidity can cause clumping)
    if (weather_data_.humidity > 85.0) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Humidity too high: %.1f%%", weather_data_.humidity);
        return false;
    }
    
    return true;
}

bool MowerMission::checkGrassConditions() 
{
    // Check grass height
    if (grass_data_.height > mower_config_.grass_height_threshold) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), 
                    "Grass too tall: %.1fcm (max: %.1fcm)",
                    grass_data_.height * 100, 
                    mower_config_.grass_height_threshold * 100);
        return false;
    }
    
    // Check grass moisture content
    if (grass_data_.moisture > 70.0) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Grass too wet: %.1f%% moisture", grass_data_.moisture);
        return false;
    }
    
    // Check if grass is ready for cutting
    if (!grass_data_.growth_stage_ok) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Grass not ready for cutting");
        return false;
    }
    
    return true;
}

bool MowerMission::checkTerrainSafety() 
{
    // Check slope angle
    if (terrain_data_.current_slope > mower_config_.max_slope_angle) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), 
                    "Slope too steep: %.1f° (max: %.1f°)",
                    terrain_data_.current_slope, 
                    mower_config_.max_slope_angle);
        return false;
    }
    
    // Check for obstacles
    if (terrain_data_.obstacles_detected) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Obstacles detected in path");
        return false;
    }
    
    return true;
}

bool MowerMission::lowerDeck() 
{
    RCLCPP_INFO(rclcpp::get_logger("mower_mission"), "Lowering deck to %.1fcm cutting height", mower_config_.cutting_height * 100);
    operational_state_.deck_lowered = true;
    return true;
}

bool MowerMission::raiseDeck() 
{
    RCLCPP_INFO(rclcpp::get_logger("mower_mission"), "Raising deck to transport position");
    operational_state_.deck_lowered = false;
    return true;
}

bool MowerMission::engageBlades() 
{
    RCLCPP_INFO(rclcpp::get_logger("mower_mission"), "Engaging cutting blades - %s mode", 
                mower_config_.mulching_mode ? "mulching" : 
                mower_config_.side_discharge ? "side discharge" : "collection");
    return true;
}

bool MowerMission::disengageBlades() 
{
    RCLCPP_INFO(rclcpp::get_logger("mower_mission"), "Disengaging cutting blades");
    return true;
}

void MowerMission::executeStripeMowing() 
{
    // Stripe pattern: parallel passes with overlap
    if (!operational_state_.blades_engaged || !operational_state_.deck_lowered) {
        return;
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("mower_mission"), "Executing stripe mowing pattern - pass %d", operational_state_.current_pass);
    
    // Simulate cutting operation
    simulateCutting();
}

void MowerMission::executeSpiralMowing() 
{
    // Spiral pattern: start from outside and work inward
    if (!operational_state_.blades_engaged || !operational_state_.deck_lowered) {
        return;
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("mower_mission"), "Executing spiral mowing pattern");
    
    // Simulate cutting operation
    simulateCutting();
}

void MowerMission::executeRandomMowing() 
{
    // Random pattern: for robotic mowers
    if (!operational_state_.blades_engaged || !operational_state_.deck_lowered) {
        return;
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("mower_mission"), "Executing random mowing pattern");
    
    // Simulate cutting operation
    simulateCutting();
}

void MowerMission::handleTurnSequence() 
{
    // End-of-row turn logic for mowing
    if (approachingEndOfRow() && !operational_state_.turn_in_progress) {
        RCLCPP_INFO(rclcpp::get_logger("mower_mission"), "End of pass detected - initiating turn sequence");
        operational_state_.turn_in_progress = true;
        
        // Step 1: Raise deck for turn
        raiseDeck();
        
        RCLCPP_INFO(rclcpp::get_logger("mower_mission"), "Turn sequence: Deck raised for turn");
    }
    
    if (operational_state_.turn_in_progress && startOfNewPass()) {
        RCLCPP_INFO(rclcpp::get_logger("mower_mission"), "Start of new pass - completing turn sequence");
        
        // Step 2: Lower deck for next pass
        lowerDeck();
        
        operational_state_.turn_in_progress = false;
        operational_state_.current_pass++;
        
        RCLCPP_INFO(rclcpp::get_logger("mower_mission"), "Turn sequence complete - starting pass %d", 
                    operational_state_.current_pass);
    }
}

void MowerMission::monitorCuttingQuality() 
{
    // Monitor blade wear
    if (operational_state_.blade_wear < 40.0) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Blade wear critical: %.1f%%", operational_state_.blade_wear);
    }
    
    // Check for uneven cutting
    if (checkBladeDamage()) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Blade damage detected");
    }
    
    // Monitor cutting height consistency
    double current_height = getDeckHeight();
    if (std::abs(current_height - mower_config_.cutting_height) > 0.01) {
        RCLCPP_WARN(rclcpp::get_logger("mower_mission"), "Cutting height deviation: %.1fcm", 
                    (current_height - mower_config_.cutting_height) * 100);
    }
}

void MowerMission::adjustCuttingParameters() 
{
    // Adjust cutting speed based on grass conditions
    if (grass_data_.density > 90.0) {
        // Slow down for thick grass
        double adjusted_speed = mower_config_.cutting_speed * 0.8;
        RCLCPP_DEBUG(rclcpp::get_logger("mower_mission"), "Adjusting speed for thick grass: %.1f m/s", adjusted_speed);
    }
    
    // Adjust cutting height for terrain
    if (terrain_data_.surface_roughness > 0.5) {
        // Raise deck slightly for rough terrain
        double adjusted_height = mower_config_.cutting_height + 0.01;
        RCLCPP_DEBUG(rclcpp::get_logger("mower_mission"), "Adjusting height for rough terrain: %.1fcm", adjusted_height * 100);
    }
}

void MowerMission::updateProgress() 
{
    // Calculate area mowed
    double current_speed = mower_config_.cutting_speed;  // m/s
    double deck_width = 1.8;  // meters - simulated deck width
    double time_delta = 1.0;  // seconds
    
    if (operational_state_.blades_engaged && operational_state_.deck_lowered) {
        double area_increment = current_speed * deck_width * time_delta / 10000.0; // hectares
        operational_state_.area_mowed += area_increment;
        operational_state_.cutting_time += time_delta / 3600.0; // hours
    }
    
    // Update completion percentage
    double total_planned_area = 50.0;  // hectares - would come from mission plan
    completion_percentage_ = (operational_state_.area_mowed / total_planned_area) * 100.0;
}

void MowerMission::updateWeatherData() 
{
    // Simulate weather data updates
    weather_data_.temperature += (rand() % 11 - 5) * 0.1;  // +/- 0.5°C variation
    weather_data_.temperature = std::max(0.0, std::min(40.0, weather_data_.temperature));
    
    weather_data_.humidity += (rand() % 11 - 5) * 0.5;  // +/- 2.5% variation
    weather_data_.humidity = std::max(0.0, std::min(100.0, weather_data_.humidity));
    
    // Update ground wetness based on precipitation
    if (weather_data_.precipitation > 0.5) {
        weather_data_.ground_wet = true;
    } else if (weather_data_.precipitation == 0.0 && weather_data_.temperature > 20.0) {
        weather_data_.ground_wet = false; // Drying out
    }
}

void MowerMission::updateTerrainData() 
{
    // Simulate terrain data updates
    terrain_data_.current_slope += (rand() % 11 - 5) * 0.1;  // +/- 0.5° variation
    terrain_data_.current_slope = std::max(0.0, std::min(25.0, terrain_data_.current_slope));
    
    // Random obstacle detection
    terrain_data_.obstacles_detected = (rand() % 10000) < 10;  // Very rare
}

void MowerMission::updateGrassData() 
{
    // Simulate grass growth (very slow)
    grass_data_.height += 0.00001;  // 0.01mm per update
    
    // Grass moisture varies with weather
    if (weather_data_.precipitation > 0.1) {
        grass_data_.moisture = std::min(100.0, grass_data_.moisture + 5.0);
    } else {
        grass_data_.moisture = std::max(10.0, grass_data_.moisture - 0.1);
    }
}

void MowerMission::updateOperationalData() 
{
    // Update blade wear
    if (operational_state_.blades_engaged) {
        operational_state_.blade_wear = std::max(0.0, operational_state_.blade_wear - 0.01);
    }
}

bool MowerMission::checkSafetyConditions() 
{
    return checkMowingConditions() && 
           checkTerrainSafety() && 
           !inNoMowZone() &&
           !detectObstacles();
}

// Utility and simulation methods
bool MowerMission::approachingEndOfRow() const { return operational_state_.current_pass > 0 && (rand() % 2000) < 5; }
bool MowerMission::startOfNewPass() const { return operational_state_.turn_in_progress && (rand() % 200) < 10; }
bool MowerMission::detectObstacles() const { return terrain_data_.obstacles_detected; }
bool MowerMission::inNoMowZone() const { return false; }  // Simulated - not in no-mow zone
double MowerMission::getBladeSharpness() const { return operational_state_.blade_wear; }
double MowerMission::getDeckHeight() const { return mower_config_.cutting_height + (rand() % 21 - 10) * 0.001; }
bool MowerMission::checkBladeDamage() const { return operational_state_.blade_wear < 20.0; }

void MowerMission::simulateCutting() {
    // Simulate the cutting operation
    RCLCPP_DEBUG(rclcpp::get_logger("mower_mission"), "Cutting grass at %.1fcm height", mower_config_.cutting_height * 100);
}

} // namespace system_controller 