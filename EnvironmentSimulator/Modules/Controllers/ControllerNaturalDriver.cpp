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
      desired_distance_(1.5),
      actual_distance_(-1.0),
      distance_adjustment_t_(3.0),
      desired_speed_(15.0), // TODO: Take from road speed
      current_speed_(desired_speed_),
      speed_tolerance_(2.0),
      lane_change_duration_(3.0),
      rear_dist_(5.0),
      lookahead_dist_(115.0),
      max_deceleration_(-4.0)
{
    operating_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_LONG);

    if (args && args->properties && args->properties->ValueExists("desiredDistance"))
    {
        desired_distance_ = strtod(args->properties->GetValueStr("desiredDistance"));
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
    if (args && args->properties && args->properties->ValueExists("rearDist"))
    {
        rear_dist_ = strtod(args->properties->GetValueStr("rearDist"));
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
    bool has_lead = false;
    bool has_adj_lead = false;
    bool has_adj_follow = false;

    std::vector<scenarioengine::Object*> vehicles_in_lane;
    bool has_vehicles_in_lane = VehiclesInEgoLane(vehicles_in_lane);
    if (has_vehicles_in_lane)
    {
        has_lead = FindVehicleAhead(vehicles_in_lane, VehicleOfInterestType::LEAD);
    }

    bool adj_lanes_available = AdjacentLanesAvailable();
    if (adj_lanes_available)
    {
        std::vector<scenarioengine::Object*> adjacent_lane_vehicles;
        bool has_adjacent_lane_vehicles = VehiclesInAdjacentLane(adjacent_lane_vehicles);
        if (has_adjacent_lane_vehicles)
        {
            has_adj_lead = FindVehicleAhead(adjacent_lane_vehicles, VehicleOfInterestType::ADJACENT_LEAD);
            has_adj_follow = FindVehicleBehind(adjacent_lane_vehicles, VehicleOfInterestType::ADJACENT_FOLLOW);
        }
    }

    state_ = State::DRIVE;
    if (has_lead)
    {
        state_ = State::FOLLOW;
    }

    if (current_speed_ < desired_speed_ - abs(speed_tolerance_)) // Should actually be current_speed < tolerance
    {
        state_ = State::DRIVE;
        if (!has_adj_lead && !has_adj_follow)
        {
            state_ = State::CHANGE_LEFT;
        }

        if (has_adj_lead)
        {
            double lead_speed = vehicles_of_interest_[VehicleOfInterestType::LEAD].vehicle->GetSpeed();
            double adjacent_lead_speed = vehicles_of_interest_[VehicleOfInterestType::ADJACENT_LEAD].vehicle->GetSpeed();
            double adjacent_lead_ds = vehicles_of_interest_[VehicleOfInterestType::ADJACENT_LEAD].position_diff.ds;
            if (adjacent_lead_ds > 10 || (adjacent_lead_ds > 5 && adjacent_lead_speed > lead_speed)) // Adjacent lead far away and faster than ego, we want to change
            {
                state_ = State::CHANGE_LEFT;
            }
        }
        else // No adjacent lead, we want to change
        {
            state_ = State::CHANGE_LEFT;
        }

        if (has_adj_follow && state_ == State::CHANGE_LEFT) // Want to change, but we have an adjacent follow
        {
            double adjacent_follow_ds = vehicles_of_interest_[VehicleOfInterestType::ADJACENT_FOLLOW].position_diff.ds;
            if (adjacent_follow_ds < -abs(rear_dist_)) // Enough distance, change lane.
            {
                state_ = State::CHANGE_LEFT;
            }
            else
            {
                state_ = State::DRIVE;
            }
        }
    }

    switch (state_)
    {
        case State::DRIVE:
        {
            break;
        }
        case State::FOLLOW:
        {
            break;
        }
        case State::CHANGE_LEFT:
        {
            if (!lane_change_injected)
            {
                int target_lane;
                (state_ == State::CHANGE_LEFT) ? target_lane = lane_ids_available_[0] : target_lane = lane_ids_available_[1];
                auto lane_change = LaneChangeActionStruct{0, 0, target_lane, 2, 2, lane_change_duration_};
                player_->player_server_->InjectLaneChangeAction(lane_change);
                lane_change_injected = true;
            }
            break;
        }
    }

    // if (has_lead && adj_lanes_available) 
    //     std::cout << vehicles_of_interest_[VehicleOfInterestType::LEAD].position_diff.ds << "\n";
    // if (has_adj_lead)
    //     std::cout << vehicles_of_interest_[VehicleOfInterestType::ADJACENT_LEAD].position_diff.ds << "\n";
    // if (has_adj_follow)
    //     std::cout << vehicles_of_interest_[VehicleOfInterestType::ADJACENT_FOLLOW].position_diff.ds << "\n";

    // if (actual_distance_ > 0 && actual_distance_ < desired_distance_)
    // {
    //     current_speed_ += max_deceleration_ * dt;
    // }
    // else if (actual_distance_ > desired_distance_ && current_speed_ + speed_tolerance_ < desired_speed_)
    // {
    //     current_speed_ += 4.0 * dt;
    // }

    object_->MoveAlongS(current_speed_ * dt);
    gateway_->updateObjectPos(object_->GetId(), 0.0, &object_->pos_);
    gateway_->updateObjectSpeed(object_->GetId(), 0.0, current_speed_);


    Controller::Step(dt);
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


bool ControllerNaturalDriver::FindVehicleAhead(std::vector<scenarioengine::Object*> vehicles, VehicleOfInterestType type)
{
    scenarioengine::Object* closest_lead = nullptr;

    double distance = lookahead_dist_;
    roadmanager::PositionDiff diff;
    for (const auto& vehicle : vehicles)
    {
        roadmanager::PositionDiff temp_diff;
        entities_->object_[0]->pos_.Delta(&vehicle->pos_, temp_diff, false, lookahead_dist_); // Lookahead dist false not working?

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

    vehicles_of_interest_[type] = VehiclesOfInterest{closest_lead, diff};
    
    return true;
}

bool ControllerNaturalDriver::FindVehicleBehind(std::vector<scenarioengine::Object*> vehicles, VehicleOfInterestType type)
{
    scenarioengine::Object* follow_vehicle = nullptr;

    double distance = -lookahead_dist_;
    roadmanager::PositionDiff diff;
    for (const auto& vehicle : vehicles)
    {
        roadmanager::PositionDiff temp_diff;
        entities_->object_[0]->pos_.Delta(&vehicle->pos_, temp_diff, true, lookahead_dist_);

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

    vehicles_of_interest_[type] = VehiclesOfInterest{follow_vehicle, diff};
    
    return true;
}

void ControllerNaturalDriver::ClearVehicleOfInterest(VehicleOfInterestType type)
{
    vehicles_of_interest_[type].vehicle = nullptr;
    vehicles_of_interest_[type].position_diff = {};
}

bool ControllerNaturalDriver::AdjacentLanesAvailable()
{
    double current_s = entities_->object_[0]->pos_.GetS();
    int current_lane = entities_->object_[0]->pos_.GetLaneId();

    id_t road_id = entities_->object_[0]->pos_.GetTrackId();
    auto road = entities_->object_[0]->pos_.GetRoadById(road_id);

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
    const int ego_lane_id = entities_->object_[0]->pos_.GetLaneId();
    for (size_t i = 1; i < entities_->object_.size(); i++)
    {
        if (entities_->object_[i]->pos_.GetLaneId() == ego_lane_id)
        {
            vehicles.push_back(entities_->object_[i]); // Candidates for lead vehicle
        }
    }

    if (vehicles.empty())
    {
        return false;
    }

    return true;
}

bool ControllerNaturalDriver::VehiclesInAdjacentLane(std::vector<scenarioengine::Object*> &vehicles)
{
    // TODO: Populate both left and right lanes
    double lane_id;
    if (lane_ids_available_[0] != 0) // Prefer left lane
    {
        lane_id = lane_ids_available_[0];
    }
    else
    {
        lane_id = lane_ids_available_[1];
    }

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