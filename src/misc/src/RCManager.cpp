#include <RCManager.hpp>

void RCManager::init() {
    this->rcInput = RCInput{0, 0, 0, 0, 0, FlightMode::MANUAL, WPTMode::OFF};
}