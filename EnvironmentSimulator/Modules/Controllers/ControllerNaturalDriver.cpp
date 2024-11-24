/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

/*
 * This controller simulates a simple Natural Driver
 */

#include "ControllerNaturalDriver.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "playerbase.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateNaturalDriver(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerNaturalDriver(initArgs);
}

ControllerNaturalDriver::ControllerNaturalDriver(InitArgs* args)
    : Controller(args),
      active_(false),
      desired_distance_(20.0),
      actual_distance_(-1.0),
      distance_adjustment_t_(3.0),
      desired_speed_(15.0), // TODO: Take from road speed
      current_speed_(desired_speed_),
      speed_tolerance_(8.0),
      lane_change_duration_(3.0),
      adj_rear_dist_(5.0),
      adj_lead_dist_(20.0),
      lookahead_dist_(115.0),
      max_deceleration_(-3.0),
      max_acceleration_(3.0),
      lane_ids_available_({0, 0}), // Left, Right side available 
      vehicles_of_interest_({}),
      distance_to_adjacent_lead_(-1.0),
      lane_change_injected(false),
      state_(State::DRIVE),
      lane_change_delay_(1.0),
      lane_change_cooldown_(lane_change_duration_ + lane_change_delay_),
      target_lane_(0),
      desired_thw_(2.0),
      max_imposed_braking_(3.0),
      politeness_(0.5),
      lane_change_acc_gain_(0.2),
      route_(-1)
{
    operating_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_LONG);

    if (args && args->properties && args->properties->ValueExists("desiredDistance"))
    {
        desired_distance_ = strtod(args->properties->GetValueStr("desiredDistance"));
    }
    if (args && args->properties && args->properties->ValueExists("adjRearDistance"))
    {
        adj_rear_dist_ = strtod(args->properties->GetValueStr("adjRearDistance"));
    }
    if (args && args->properties && args->properties->ValueExists("adjLeadDistance"))
    {
        adj_lead_dist_ = strtod(args->properties->GetValueStr("adjLeadDistance"));
    }
    if (args && args->properties && args->properties->ValueExists("desiredSpeed"))
    {
        desired_speed_    = strtod(args->properties->GetValueStr("desiredSpeed"));
    }
    if (args && args->properties && args->properties->ValueExists("speedTolerance"))
    {
        speed_tolerance_    = strtod(args->properties->GetValueStr("speedTolerance"));
    }
    if (args && args->properties && args->properties->ValueExists("laneChangeDuration"))
    {
        lane_change_duration_    = static_cast<float>(strtod(args->properties->GetValueStr("laneChangeDuration")));
    }
    if (args && args->properties && args->properties->ValueExists("maxDec"))
    {
        max_deceleration_ = strtod(args->properties->GetValueStr("maxDec"));
    }
    if (args && args->properties && args->properties->ValueExists("maxAcc"))
    {
        max_acceleration_ = strtod(args->properties->GetValueStr("maxAcc"));
    }
    if (args && args->properties && args->properties->ValueExists("laneChangeDelay"))
    {
        lane_change_delay_ = strtod(args->properties->GetValueStr("laneChangeDelay"));
    }
    if (args && args->properties && args->properties->ValueExists("THW"))
    {
        desired_thw_ = strtod(args->properties->GetValueStr("THW"));
    }
    if (args && args->properties && args->properties->ValueExists("maxImposedBraking"))
    {
        max_imposed_braking_ = strtod(args->properties->GetValueStr("maxImposedBraking"));
    }
    if (args && args->properties && args->properties->ValueExists("route"))
    {
        route_ = strtoi(args->properties->GetValueStr("route"));
    }
    if (args && args->properties && args->properties->ValueExists("politeness"))
    {
        politeness_ = strtod(args->properties->GetValueStr("politeness"));
    }
    if (args && args->properties && !args->properties->ValueExists("mode"))
    {
        // Default mode for this controller is additive
        // which will use speed set by other actions as setSpeed
        // in override mode setSpeed is set explicitly (if missing
        // the current speed when controller is activated will be
        // used as setSpeed)
        mode_ = ControlOperationMode::MODE_ADDITIVE;
    }
}

void ControllerNaturalDriver::Init()
{
    Controller::Init();
}

void ControllerNaturalDriver::InitPostPlayer()
{
    // Uncomment line below to enable example how to add sensors. Press 'r' to visualize sensor frustum.
    // player_->AddObjectSensor(object_, 4.0, 0.0, 0.5, 0.0, 1.0, 50.0, 1.2, 100);
}

void ControllerNaturalDriver::Step(double dt)
{
    switch (state_)
    {
        case State::DRIVE:
        {
            GetLeadVehicle();

            double acceleration = GetAcceleration(this->GetLinkedObject(), vehicles_of_interest_[VoIType::LEAD]);
            current_speed_ += acceleration * dt;

            if (current_speed_ >= desired_speed_)
            {
                current_speed_ = desired_speed_;
            }

            if (!lane_change_injected) // Should actually be current_speed < tolerance. We only want to change lane if we are following someone
            {
                bool adjacent_lanes_available = AdjacentLanesAvailable();
                if (!adjacent_lanes_available)
                {
                    break;
                }
                for (const auto& id : lane_ids_available_)
                {
                    if (id != 0)
                    {
                        bool lead, follow;
                        GetAdjacentLeadAndFollow(id, lead, follow);
                        bool initiate_lanechange = CheckLaneChangePossible(id);
                        if (initiate_lanechange && this->GetLinkedObject()->GetSpeed() > 0)
                        {
                            target_lane_ = id;
                            state_ = State::CHANGE_LANE;
                            break;
                        }
                    }
                }
            }
            break;
        }
        case State::CHANGE_LANE:
        {
            // Check if someone else is already changing
            // For v : vehicles -> if v.lane_change_injected, target_lane_ = current_lane, State::Drive
            if (!lane_change_injected)
            {
                auto lane_change = LaneChangeActionStruct{this->GetLinkedObject()->GetId(), 0, target_lane_, 2, 2, static_cast<float>(lane_change_duration_)};
                player_->player_server_->InjectLaneChangeAction(lane_change); // Why does it change the lane in 1 step?
                lane_change_injected = true;
            }

            break;
        }
    }

    object_->MoveAlongS(current_speed_ * dt);
    gateway_->updateObjectPos(object_->GetId(), 0.0, &object_->pos_);
    gateway_->updateObjectSpeed(object_->GetId(), 0.0, current_speed_);

    // Wait a while before initiating another lane change
    if (lane_change_injected)
    {
        lane_change_cooldown_ -= dt;
        if (state_ == State::CHANGE_LANE && lane_change_cooldown_ <= lane_change_delay_) // Keep constant long. acceleration during lane_change_duration_
        {
            state_ = State::DRIVE;
        }
        if (lane_change_cooldown_ < 0.0)
        {
            lane_change_cooldown_ = lane_change_delay_ + lane_change_duration_; // Reset the cooldown
            lane_change_injected = false;
        }
    }

    Controller::Step(dt);
}

/* 
    GetAcceleration() and GetDesiredGap() based on IDM (https://en.wikipedia.org/wiki/Intelligent_driver_model)
*/
double ControllerNaturalDriver::GetAcceleration(scenarioengine::Object* follow, scenarioengine::Object* lead)
{
    if (follow == nullptr) // No car to calculate acceleration for, return 0
    {
        return 0.0;
    }

    int delta = 4;

    double follow_current_speed = follow->GetSpeed();
    double acceleration = max_acceleration_ * (1 - std::pow(follow_current_speed / desired_speed_, delta));

    if (lead != nullptr)
    {
        double lead_current_speed = lead->GetSpeed();
        double desired_gap = GetDesiredGap(max_acceleration_, max_deceleration_, follow_current_speed, lead_current_speed, desired_distance_, desired_thw_);

        roadmanager::PositionDiff diff;
        follow->pos_.Delta(&lead->pos_, diff, false, lookahead_dist_);

        acceleration -= max_acceleration_ * std::pow(desired_gap / diff.ds, 2);
    }

    return acceleration;
}

double ControllerNaturalDriver::GetDesiredGap(double max_acceleration, double max_deceleration, double follow_speed, double lead_speed, double desired_distance, double desired_thw)
{
    double ab = -max_acceleration * max_deceleration;
    double relative_speed = follow_speed - lead_speed;
    return desired_distance + std::max(0.0, follow_speed * desired_thw + follow_speed * relative_speed / (2 * std::sqrt(ab)));
}

int ControllerNaturalDriver::Activate(ControlActivationMode lat_activation_mode,
                            ControlActivationMode long_activation_mode,
                            ControlActivationMode light_activation_mode,
                            ControlActivationMode anim_activation_mode)
{
    current_speed_ = object_->GetSpeed();
    if (mode_ == ControlOperationMode::MODE_ADDITIVE)
    {
        // desired_speed_ = object_->GetSpeed();
    }


    Controller::Activate(lat_activation_mode, long_activation_mode, light_activation_mode, anim_activation_mode);

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LAT)))
    {
        // Make sure heading is aligned with road driving direction
        object_->pos_.SetHeadingRelative((object_->pos_.GetHRelative() > M_PI_2 && object_->pos_.GetHRelative() < 3 * M_PI_2) ? M_PI : 0.0);
    }

    if (player_)
    {
        player_->SteeringSensorSetVisible(object_->GetId(), true);
    }

    return 0;
}

void ControllerNaturalDriver::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}

bool ControllerNaturalDriver::CheckLaneChangePossible(const int lane_id)
{
    if (lane_change_injected)
    {
        return false;
    }

    VoIType adj_lead, adj_follow;
    GetVehicleOfInterestType(lane_id, adj_lead, adj_follow);
    
    double new_following_acceleration = GetAcceleration(vehicles_of_interest_[adj_follow], vehicles_of_interest_[adj_lead]);
    double new_following_pred_acceleration = GetAcceleration(vehicles_of_interest_[adj_follow], this->GetLinkedObject());
    if (new_following_pred_acceleration < -max_imposed_braking_)
    {
        return false;
    }

    double predicted_new_acceleration = GetAcceleration(this->GetLinkedObject(), vehicles_of_interest_[adj_lead]);
    if (lane_id == route_)
    {
        if (predicted_new_acceleration < -max_imposed_braking_)
        {
            return false;
        }
    }
    else 
    {
        GetFollowVehicle();
        double acceleration = GetAcceleration(this->GetLinkedObject(), vehicles_of_interest_[VoIType::LEAD]);
        double old_following_acceleration = GetAcceleration(vehicles_of_interest_[VoIType::FOLLOWING], this->GetLinkedObject());
        double old_following_pred_acceleration = GetAcceleration(vehicles_of_interest_[VoIType::FOLLOWING], vehicles_of_interest_[VoIType::LEAD]);
        
        /* Jerk
            If I change lane, how much...
            "predicted_new_acceleration - acceleration" ...more can ego accelerate?
            "politeness_" ...do I care about being in the way of others?
            "new_following_pred_acceleration - new_following_acceleration" ...more can adjacent behind vehicle accelerate?
            "old_following_pred_acceleration - old_following_acceleration" ...more can current following vehicle accelerate?
        */
        
        double jerk = predicted_new_acceleration - acceleration + politeness_ * (new_following_pred_acceleration - new_following_acceleration + old_following_pred_acceleration - old_following_acceleration);

        if (jerk <= lane_change_acc_gain_)
        {
            return false;
        }
    }

    return true;
}

bool ControllerNaturalDriver::GetLeadVehicle()
{
    bool have_lead = false;

    std::vector<scenarioengine::Object*> vehicles_in_lane = {};
    bool has_vehicles_in_lane = VehiclesInEgoLane(vehicles_in_lane);
    if (has_vehicles_in_lane)
    {
        have_lead = FindClosestAhead(vehicles_in_lane, VoIType::LEAD); // Lead and lead within lookahead distance
    }

    return have_lead;
}

bool ControllerNaturalDriver::GetFollowVehicle()
{
    bool have_follow = false;

    std::vector<scenarioengine::Object*> following_vehicles = {};
    bool vehicles = VehiclesInEgoLane(following_vehicles); // idx 0 is left
    if (vehicles)
    {
        have_follow = FindClosestBehind(following_vehicles, VoIType::FOLLOWING);
    }

    return have_follow;
}

void ControllerNaturalDriver::GetAdjacentLeadAndFollow(const int lane_id, bool &lead, bool& follow)
{
    VoIType adj_lane_lead, adj_lane_follow;
    GetVehicleOfInterestType(lane_id, adj_lane_lead, adj_lane_follow);

    std::vector<scenarioengine::Object*> adjacent_lane_vehicles = {};
    bool vehicles = VehiclesInAdjacentLane(adjacent_lane_vehicles, lane_id); // idx 0 is left
    if (vehicles)
    {
        lead = FindClosestAhead(adjacent_lane_vehicles, adj_lane_lead);
        follow = FindClosestBehind(adjacent_lane_vehicles, adj_lane_follow);
    }
    else
    {
        ClearVehicleOfInterest(adj_lane_lead);
        ClearVehicleOfInterest(adj_lane_follow);
    }
}

void ControllerNaturalDriver::GetVehicleOfInterestType(int lane_id, VoIType &lead, VoIType &follow)
{
    if (lane_id == lane_ids_available_[0])
    {
        lead = VoIType::LEFT_LEAD;
        follow = VoIType::LEFT_FOLLOW;
    }
    else
    {
        lead = VoIType::RIGHT_LEAD;
        follow = VoIType::RIGHT_FOLLOW;
    }
}

bool ControllerNaturalDriver::FindClosestAhead(std::vector<scenarioengine::Object*> vehicles, VoIType type)
{
    scenarioengine::Object* closest_lead = nullptr;

    double distance = lookahead_dist_;
    roadmanager::PositionDiff diff;
    for (const auto& vehicle : vehicles)
    {
        roadmanager::PositionDiff temp_diff;
        this->GetLinkedObject()->pos_.Delta(&vehicle->pos_, temp_diff, false, lookahead_dist_); // Lookahead dist false not working?

        if (temp_diff.ds > 0 && temp_diff.ds < distance)
        {
            distance = temp_diff.ds;
            diff = temp_diff;
            closest_lead = vehicle;
        }
    }

    if (!closest_lead)
    {
        ClearVehicleOfInterest(type);
        return false;
    }

    vehicles_of_interest_[type] = {closest_lead};
    
    return true;
}

bool ControllerNaturalDriver::FindClosestBehind(std::vector<scenarioengine::Object*> vehicles, VoIType type)
{
    scenarioengine::Object* follow_vehicle = nullptr;

    double distance = -lookahead_dist_;
    roadmanager::PositionDiff diff;
    for (const auto& vehicle : vehicles)
    {
        roadmanager::PositionDiff temp_diff;
        this->GetLinkedObject()->pos_.Delta(&vehicle->pos_, temp_diff, true, lookahead_dist_);

        if (temp_diff.ds < 0 && temp_diff.ds > distance)
        {
            distance = temp_diff.ds;
            diff = temp_diff;
            follow_vehicle = vehicle;
        }
    }

    if (!follow_vehicle)
    {
        ClearVehicleOfInterest(type);
        return false; // No lead
    }

    vehicles_of_interest_[type] = follow_vehicle;
    
    return true;
}

void ControllerNaturalDriver::ClearVehicleOfInterest(VoIType type)
{
    vehicles_of_interest_[type] = nullptr;
}

bool ControllerNaturalDriver::AdjacentLanesAvailable()
{
    double current_s = this->GetLinkedObject()->pos_.GetS();
    int current_lane = this->GetLinkedObject()->pos_.GetLaneId();

    id_t road_id = this->GetLinkedObject()->pos_.GetTrackId();
    auto road = this->GetLinkedObject()->pos_.GetRoadById(road_id);

    auto ls = road->GetLaneSectionByS(current_s);
    auto driving_lanes_available = ls->GetNumberOfDrivingLanesSide(current_lane);

    if (driving_lanes_available == 1)
    {
        // Only 1 lane in current direction
        lane_ids_available_[0] = 0; 
        lane_ids_available_[1] = 0;

        return false;
    }

    if (abs(current_lane) == 1) // In first lane, available lane to the right
    {
        lane_ids_available_[0] = 0;
        lane_ids_available_[1] = current_lane + SIGN(current_lane);
    }
    else if (driving_lanes_available > abs(current_lane)) // Not in first lane and there are lanes to the right, both lanes available
    {
        lane_ids_available_[0] = current_lane - SIGN(current_lane);
        lane_ids_available_[1] = current_lane + SIGN(current_lane);
    }
    else // Left lane only available
    {
        lane_ids_available_[0] = current_lane - SIGN(current_lane);
        lane_ids_available_[1] = 0;
    }

    return true;
}

bool ControllerNaturalDriver::VehiclesInEgoLane(std::vector<scenarioengine::Object*> &vehicles)
{
    // Should use Delta here instead
    const int ego_lane_id = this->GetLinkedObject()->pos_.GetLaneId();
    for (size_t i = 1; i < entities_->object_.size(); i++)
    {
        if (entities_->object_[i]->pos_.GetLaneId() == ego_lane_id)
        {
            vehicles.push_back(entities_->object_[i]); // Candidates for lead vehicle
        }
    }

    if (vehicles.empty())
    {
        ClearVehicleOfInterest(VoIType::LEAD);
        ClearVehicleOfInterest(VoIType::FOLLOWING);
        return false;
    }

    return true;
}

bool ControllerNaturalDriver::VehiclesInAdjacentLane(std::vector<scenarioengine::Object*> &vehicles, int lane_id)
{
    // TODO: Populate both left and right lanes
    for (size_t i = 1; i < entities_->object_.size(); i++)
    {
        if (entities_->object_[i]->pos_.GetLaneId() == lane_id)
        {
            vehicles.push_back(entities_->object_[i]); // Exist in adjacent lane
        }
    }

    if (vehicles.empty())
    {
        return false;
    }

    return true;
}