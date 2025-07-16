#include "missions/seeder_mission.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

namespace system_controller {

SeederMission::SeederMission() 
    : MissionBase("seeder_mission", "Precision Seeding Mission") 
{
    mission_type_ = "seeding";
    implement_type_ = "seeder";
}

bool SeederMission::initialize(const MissionConfig& config) 
{
    if (!MissionBase::initialize(config)) {
        return false;
    }
    
    // Load seeder-specific configuration
    if (config.mission_params.count("seed_type")) {
        seeder_config_.seed_type = config.mission_params.at("seed_type");
    }
    if (config.mission_params.count("seed_rate")) {
        seeder_config_.seed_rate = std::stod(config.mission_params.at("seed_rate"));
    }
    if (config.mission_params.count("planting_depth")) {
        seeder_config_.planting_depth = std::stod(config.mission_params.at("planting_depth"));
    }
    if (config.mission_params.count("row_spacing")) {
        seeder_config_.row_spacing = std::stod(config.mission_params.at("row_spacing"));
    }
    if (config.mission_params.count("planting_speed")) {
        seeder_config_.planting_speed = std::stod(config.mission_params.at("planting_speed"));
    }
    if (config.mission_params.count("variable_rate_seeding")) {
        seeder_config_.variable_rate_seeding = (config.mission_params.at("variable_rate_seeding") == "true");
    }
    if (config.mission_params.count("fertilizer_application")) {
        seeder_config_.fertilizer_application = (config.mission_params.at("fertilizer_application") == "true");
    }
    
    // Initialize soil monitoring
    soil_data_.moisture = 25.0;          // % - good planting moisture
    soil_data_.temperature = 12.0;       // °C - suitable for corn
    soil_data_.ph = 6.8;                 // good pH for most crops
    soil_data_.organic_matter = 3.5;     // % - healthy soil
    soil_data_.compaction_detected = false;
    
    // Initialize field zone
    current_zone_.zone_id = "zone_1";
    current_zone_.productivity_index = 85.0;
    current_zone_.recommended_population = seeder_config_.seed_rate;
    current_zone_.soil_quality = 8.0;
    current_zone_.requires_tillage = false;
    
    // Initialize operational state
    operational_state_.metering_active = false;
    operational_state_.seeder_lowered = false;
    operational_state_.fertilizer_active = false;
    operational_state_.current_row = 0;
    operational_state_.area_seeded = 0.0;
    operational_state_.seeds_planted = 0.0;
    operational_state_.fertilizer_applied = 0.0;
    operational_state_.turn_in_progress = false;
    operational_state_.seed_level = 100.0;
    operational_state_.fertilizer_level = 100.0;
    
    // Initialize quality metrics
    quality_metrics_.spacing_accuracy = 95.0;
    quality_metrics_.depth_accuracy = 90.0;
    quality_metrics_.emergence_rate = 0.0;
    quality_metrics_.population_achieved = 0.0;
    quality_metrics_.singulation_rate = 98.0;
    
    // Initialize equipment status
    equipment_status_.seed_meter_ok = true;
    equipment_status_.fertilizer_meter_ok = true;
    equipment_status_.depth_control_ok = true;
    equipment_status_.down_pressure_ok = true;
    equipment_status_.meter_wear = 5.0;
    
    setStatus(MissionStatus::READY);
    return true;
}

bool SeederMission::start() 
{
    if (status_ != MissionStatus::READY && status_ != MissionStatus::PAUSED) {
        return false;
    }
    
    // Pre-seeding checks
    if (!performPreSeedChecks()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    // Lower seeder to planting position
    if (!lowerSeeder()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    // Enable seed metering
    if (!enableMetering()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    // Start fertilizer application if enabled
    if (seeder_config_.fertilizer_application && !startFertilizerApplication()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    operational_state_.metering_active = true;
    setStatus(MissionStatus::ACTIVE);
    
    RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), 
                "Seeder mission started - planting %s at %.0f seeds/ha, depth %.1fcm",
                seeder_config_.seed_type.c_str(),
                seeder_config_.seed_rate,
                seeder_config_.planting_depth * 100);
    
    return true;
}

bool SeederMission::pause() 
{
    if (status_ != MissionStatus::ACTIVE) {
        return false;
    }
    
    // Lift seeder and stop metering for pause
    operational_state_.metering_active = false;
    disableMetering();
    stopFertilizerApplication();
    raiseSeeder();
    
    setStatus(MissionStatus::PAUSED);
    
    RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Seeder mission paused");
    return true;
}

bool SeederMission::resume() 
{
    if (status_ != MissionStatus::PAUSED) {
        return false;
    }
    
    // Pre-resumption checks
    if (!performPreSeedChecks()) {
        setStatus(MissionStatus::ERROR);
        return false;
    }
    
    // Resume seeding
    lowerSeeder();
    enableMetering();
    if (seeder_config_.fertilizer_application) {
        startFertilizerApplication();
    }
    operational_state_.metering_active = true;
    
    setStatus(MissionStatus::ACTIVE);
    
    RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Seeder mission resumed");
    return true;
}

bool SeederMission::stop() 
{
    // Graceful shutdown
    operational_state_.metering_active = false;
    disableMetering();
    stopFertilizerApplication();
    raiseSeeder();
    
    setStatus(MissionStatus::COMPLETED);
    
    // Log mission summary
    RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), 
                "Seeder mission completed - %.2f hectares seeded, %.0f seeds planted, %.1f kg fertilizer applied",
                operational_state_.area_seeded,
                operational_state_.seeds_planted,
                operational_state_.fertilizer_applied);
    
    return true;
}

bool SeederMission::abort() 
{
    // Emergency shutdown
    operational_state_.metering_active = false;
    disableMetering();
    stopFertilizerApplication();
    raiseSeeder();
    
    setStatus(MissionStatus::ABORTED);
    
    RCLCPP_WARN(rclcpp::get_logger("seeder_mission"), "Seeder mission aborted");
    return true;
}

void SeederMission::update() 
{
    if (status_ != MissionStatus::ACTIVE) {
        return;
    }
    
    // Update sensor data
    updateSoilData();
    updateFieldZone();
    updateOperationalData();
    updateQualityMetrics();
    
    // Check safety conditions
    if (!checkSafetyConditions()) {
        RCLCPP_WARN(rclcpp::get_logger("seeder_mission"), "Safety conditions violated - pausing seeder");
        pause();
        return;
    }
    
    // Calculate optimal seeding rate for current conditions
    calculateSeedingRate();
    
    // Adjust seeding parameters
    adjustSeedingParameters();
    
    // Handle turn sequences
    handleTurnSequence();
    
    // Monitor seeding quality
    monitorSeedingQuality();
    
    // Update progress
    updateProgress();
}

MissionStatus SeederMission::getStatus() const 
{
    return status_;
}

bool SeederMission::performPreSeedChecks() 
{
    // Check soil conditions
    if (!checkSoilConditions()) {
        return false;
    }
    
    // Check seed conditions
    if (!checkSeedConditions()) {
        return false;
    }
    
    // Check equipment status
    if (!checkEquipmentStatus()) {
        return false;
    }
    
    return true;
}

bool SeederMission::checkSoilConditions() 
{
    // Check soil moisture
    if (soil_data_.moisture < seeder_config_.soil_moisture_min) {
        RCLCPP_WARN(rclcpp::get_logger("seeder_mission"), 
                    "Soil moisture too low: %.1f%% (min: %.1f%%)",
                    soil_data_.moisture, seeder_config_.soil_moisture_min);
        return false;
    }
    
    // Check soil temperature
    if (soil_data_.temperature < seeder_config_.soil_temperature_min) {
        RCLCPP_WARN(rclcpp::get_logger("seeder_mission"), 
                    "Soil temperature too low: %.1f°C (min: %.1f°C)",
                    soil_data_.temperature, seeder_config_.soil_temperature_min);
        return false;
    }
    
    // Check for soil compaction
    if (soil_data_.compaction_detected) {
        RCLCPP_WARN(rclcpp::get_logger("seeder_mission"), "Soil compaction detected - not suitable for seeding");
        return false;
    }
    
    // Check if tillage is required
    if (current_zone_.requires_tillage) {
        RCLCPP_WARN(rclcpp::get_logger("seeder_mission"), "Field requires tillage before seeding");
        return false;
    }
    
    return true;
}

bool SeederMission::checkSeedConditions() 
{
    // Check seed level
    if (getSeedLevel() < 10.0) {
        RCLCPP_ERROR(rclcpp::get_logger("seeder_mission"), "Seed level too low: %.1f%%", getSeedLevel());
        return false;
    }
    
    // Check seed flow
    if (!checkSeedFlow()) {
        RCLCPP_ERROR(rclcpp::get_logger("seeder_mission"), "Seed flow problem detected");
        return false;
    }
    
    return true;
}

bool SeederMission::checkEquipmentStatus() 
{
    // Check seed meter condition
    if (!equipment_status_.seed_meter_ok) {
        RCLCPP_ERROR(rclcpp::get_logger("seeder_mission"), "Seed meter malfunction");
        return false;
    }
    
    // Check fertilizer meter if using fertilizer
    if (seeder_config_.fertilizer_application && !equipment_status_.fertilizer_meter_ok) {
        RCLCPP_ERROR(rclcpp::get_logger("seeder_mission"), "Fertilizer meter malfunction");
        return false;
    }
    
    // Check depth control
    if (!equipment_status_.depth_control_ok) {
        RCLCPP_ERROR(rclcpp::get_logger("seeder_mission"), "Depth control system malfunction");
        return false;
    }
    
    // Check meter wear
    if (equipment_status_.meter_wear > 50.0) {
        RCLCPP_WARN(rclcpp::get_logger("seeder_mission"), "High meter wear: %.1f%%", equipment_status_.meter_wear);
    }
    
    return true;
}

bool SeederMission::lowerSeeder() 
{
    RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Lowering seeder to %.1fcm planting depth", seeder_config_.planting_depth * 100);
    operational_state_.seeder_lowered = true;
    return true;
}

bool SeederMission::raiseSeeder() 
{
    RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Raising seeder to transport position");
    operational_state_.seeder_lowered = false;
    return true;
}

bool SeederMission::enableMetering() 
{
    RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Enabling seed metering - %s at %.0f seeds/ha", 
                seeder_config_.seed_type.c_str(),
                seeder_config_.seed_rate);
    return true;
}

bool SeederMission::disableMetering() 
{
    RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Disabling seed metering");
    return true;
}

bool SeederMission::startFertilizerApplication() 
{
    RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Starting fertilizer application - %s at %.1f kg/ha", 
                seeder_config_.fertilizer_type.c_str(),
                seeder_config_.fertilizer_rate);
    operational_state_.fertilizer_active = true;
    return true;
}

bool SeederMission::stopFertilizerApplication() 
{
    RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Stopping fertilizer application");
    operational_state_.fertilizer_active = false;
    return true;
}

void SeederMission::calculateSeedingRate() 
{
    if (!seeder_config_.variable_rate_seeding) {
        return; // Use fixed rate
    }
    
    // Start with base rate
    double optimal_rate = seeder_config_.seed_rate;
    
    // Adjust for soil conditions
    optimal_rate = adjustForSoilConditions();
    
    // Adjust for field history
    optimal_rate = adjustForFieldHistory();
    
    // Apply productivity index
    optimal_rate *= (current_zone_.productivity_index / 100.0);
    
    RCLCPP_DEBUG(rclcpp::get_logger("seeder_mission"), 
                 "Calculated optimal seeding rate: %.0f seeds/ha for zone %s",
                 optimal_rate, current_zone_.zone_id.c_str());
    
    // Update current zone recommendation
    current_zone_.recommended_population = optimal_rate;
}

void SeederMission::adjustSeedingParameters() 
{
    // Adjust planting depth based on soil conditions
    double adjusted_depth = seeder_config_.planting_depth;
    
    if (soil_data_.moisture < 25.0) {
        // Plant deeper in dry conditions
        adjusted_depth += 0.005; // +5mm
        RCLCPP_DEBUG(rclcpp::get_logger("seeder_mission"), "Adjusting depth for dry soil: +5mm");
    }
    
    if (soil_data_.temperature < 12.0) {
        // Plant shallower in cold soil
        adjusted_depth -= 0.003; // -3mm
        RCLCPP_DEBUG(rclcpp::get_logger("seeder_mission"), "Adjusting depth for cold soil: -3mm");
    }
    
    // Ensure depth stays within reasonable bounds
    adjusted_depth = std::max(0.015, std::min(0.08, adjusted_depth)); // 15mm to 80mm
}

void SeederMission::handleTurnSequence() 
{
    // End-of-row turn logic for seeding
    if (approachingEndOfRow() && !operational_state_.turn_in_progress) {
        RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "End of row detected - initiating turn sequence");
        operational_state_.turn_in_progress = true;
        
        // Step 1: Disable metering
        disableMetering();
        
        // Step 2: Stop fertilizer application
        stopFertilizerApplication();
        
        // Step 3: Raise seeder for turn
        raiseSeeder();
        
        RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Turn sequence: Seeder raised, metering disabled");
    }
    
    if (operational_state_.turn_in_progress && startOfNewRow()) {
        RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Start of new row - completing turn sequence");
        
        // Step 4: Lower seeder for next row
        lowerSeeder();
        
        // Step 5: Re-enable metering
        enableMetering();
        
        // Step 6: Restart fertilizer if enabled
        if (seeder_config_.fertilizer_application) {
            startFertilizerApplication();
        }
        
        operational_state_.turn_in_progress = false;
        operational_state_.current_row++;
        
        RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), "Turn sequence complete - starting row %d", 
                    operational_state_.current_row);
    }
}

void SeederMission::monitorSeedingQuality() 
{
    // Monitor spacing accuracy
    if (quality_metrics_.spacing_accuracy < 90.0) {
        RCLCPP_WARN(rclcpp::get_logger("seeder_mission"), "Poor spacing accuracy: %.1f%%", quality_metrics_.spacing_accuracy);
    }
    
    // Monitor depth accuracy
    if (quality_metrics_.depth_accuracy < 85.0) {
        RCLCPP_WARN(rclcpp::get_logger("seeder_mission"), "Poor depth accuracy: %.1f%%", quality_metrics_.depth_accuracy);
    }
    
    // Monitor singulation rate
    if (quality_metrics_.singulation_rate < 95.0) {
        RCLCPP_WARN(rclcpp::get_logger("seeder_mission"), "Poor singulation rate: %.1f%%", quality_metrics_.singulation_rate);
    }
    
    // Check for seed blockages
    if (detectSeedBlockage()) {
        RCLCPP_ERROR(rclcpp::get_logger("seeder_mission"), "Seed blockage detected");
    }
}

void SeederMission::updateProgress() 
{
    // Calculate area seeded
    double current_speed = seeder_config_.planting_speed;  // m/s
    double implement_width = seeder_config_.row_spacing * 12;  // 12 rows - simulated
    double time_delta = 1.0;  // seconds
    
    if (operational_state_.metering_active && operational_state_.seeder_lowered) {
        double area_increment = current_speed * implement_width * time_delta / 10000.0; // hectares
        operational_state_.area_seeded += area_increment;
        
        // Calculate seeds planted
        double seeds_increment = area_increment * current_zone_.recommended_population;
        operational_state_.seeds_planted += seeds_increment;
        
        // Calculate fertilizer applied
        if (operational_state_.fertilizer_active) {
            double fertilizer_increment = area_increment * seeder_config_.fertilizer_rate;
            operational_state_.fertilizer_applied += fertilizer_increment;
        }
    }
    
    // Update completion percentage
    double total_planned_area = 200.0;  // hectares - would come from mission plan
    completion_percentage_ = (operational_state_.area_seeded / total_planned_area) * 100.0;
}

void SeederMission::updateSoilData() 
{
    // Simulate soil data updates
    soil_data_.moisture += (rand() % 11 - 5) * 0.2;  // +/- 1% variation
    soil_data_.moisture = std::max(10.0, std::min(40.0, soil_data_.moisture));
    
    soil_data_.temperature += (rand() % 11 - 5) * 0.1;  // +/- 0.5°C variation
    soil_data_.temperature = std::max(5.0, std::min(25.0, soil_data_.temperature));
    
    // Random compaction detection (very rare)
    soil_data_.compaction_detected = (rand() % 50000) < 5;
}

void SeederMission::updateFieldZone() 
{
    // Simulate field zone changes
    if ((rand() % 1000) < 10) {  // 1% chance to enter new zone
        current_zone_.zone_id = "zone_" + std::to_string((rand() % 5) + 1);
        current_zone_.productivity_index = 70.0 + (rand() % 31);  // 70-100%
        current_zone_.soil_quality = 6.0 + (rand() % 41) * 0.1;  // 6.0-10.0
        
        RCLCPP_INFO(rclcpp::get_logger("seeder_mission"), 
                    "Entered new field zone: %s (productivity: %.0f%%)",
                    current_zone_.zone_id.c_str(),
                    current_zone_.productivity_index);
    }
}

void SeederMission::updateOperationalData() 
{
    // Update tank levels (simulate consumption)
    if (operational_state_.metering_active) {
        operational_state_.seed_level = std::max(0.0, operational_state_.seed_level - 0.02);
    }
    
    if (operational_state_.fertilizer_active) {
        operational_state_.fertilizer_level = std::max(0.0, operational_state_.fertilizer_level - 0.03);
    }
    
    // Update equipment wear
    if (operational_state_.metering_active) {
        equipment_status_.meter_wear = std::min(100.0, equipment_status_.meter_wear + 0.001);
    }
}

void SeederMission::updateQualityMetrics() 
{
    // Simulate quality metric updates
    quality_metrics_.spacing_accuracy += (rand() % 11 - 5) * 0.2;  // +/- 1% variation
    quality_metrics_.spacing_accuracy = std::max(80.0, std::min(100.0, quality_metrics_.spacing_accuracy));
    
    quality_metrics_.depth_accuracy += (rand() % 11 - 5) * 0.3;  // +/- 1.5% variation
    quality_metrics_.depth_accuracy = std::max(75.0, std::min(100.0, quality_metrics_.depth_accuracy));
    
    quality_metrics_.singulation_rate += (rand() % 11 - 5) * 0.1;  // +/- 0.5% variation
    quality_metrics_.singulation_rate = std::max(90.0, std::min(100.0, quality_metrics_.singulation_rate));
}

bool SeederMission::checkSafetyConditions() 
{
    return checkSoilConditions() && 
           checkSeedConditions() && 
           !inNoSeedZone();
}

double SeederMission::adjustForSoilConditions() 
{
    double adjusted_rate = seeder_config_.seed_rate;
    
    // Adjust for soil quality
    if (current_zone_.soil_quality > 8.0) {
        adjusted_rate *= 1.1; // Increase rate in high-quality soil
    } else if (current_zone_.soil_quality < 6.0) {
        adjusted_rate *= 0.9; // Decrease rate in poor soil
    }
    
    // Adjust for organic matter
    if (soil_data_.organic_matter > 4.0) {
        adjusted_rate *= 1.05; // Slight increase in high-OM soil
    }
    
    return adjusted_rate;
}

double SeederMission::adjustForFieldHistory() 
{
    // In real implementation, this would use historical yield data
    double adjusted_rate = current_zone_.recommended_population;
    
    // Simulate adjustment based on previous year's performance
    if (current_zone_.productivity_index > 90.0) {
        adjusted_rate *= 1.05; // Increase in high-performing areas
    } else if (current_zone_.productivity_index < 75.0) {
        adjusted_rate *= 0.95; // Decrease in low-performing areas
    }
    
    return adjusted_rate;
}

// Utility and simulation methods
bool SeederMission::approachingEndOfRow() const { return operational_state_.current_row > 0 && (rand() % 1500) < 5; }
bool SeederMission::startOfNewRow() const { return operational_state_.turn_in_progress && (rand() % 150) < 10; }
bool SeederMission::inNoSeedZone() const { return false; }  // Simulated - not in no-seed zone
bool SeederMission::checkSeedFlow() const { return operational_state_.seed_level > 1.0; }
double SeederMission::getSeedLevel() const { return operational_state_.seed_level; }
double SeederMission::getFertilizerLevel() const { return operational_state_.fertilizer_level; }
bool SeederMission::detectSeedBlockage() const { return (rand() % 20000) < 5; }

void SeederMission::simulatePlanting() {
    // Simulate the planting operation
    RCLCPP_DEBUG(rclcpp::get_logger("seeder_mission"), "Planting %s at %.1fcm depth", 
                 seeder_config_.seed_type.c_str(), seeder_config_.planting_depth * 100);
}

void SeederMission::checkEmergence() {
    // Post-planting emergence monitoring (would be used days after planting)
    RCLCPP_DEBUG(rclcpp::get_logger("seeder_mission"), "Monitoring emergence rate: %.1f%%", quality_metrics_.emergence_rate);
}

void SeederMission::validatePopulation() {
    // Population validation (would be used after emergence)
    RCLCPP_DEBUG(rclcpp::get_logger("seeder_mission"), "Validating population: %.0f plants/ha achieved", quality_metrics_.population_achieved);
}

} // namespace system_controller 