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
      max_deceleration_(-3.0),
      max_acceleration_(3.0),
      lane_ids_available_({0, 0}), // Left, Right side available 
      vehicles_of_interest_({}),
      distance_to_adjacent_lead_(-1.0),
      lane_change_injected(false),
      state_(State::DRIVE),
      cooldown_period_(5.0 + lane_change_duration_),
      target_lane_(0),
      k_p_(0.5),
      k_d_(1.0)
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
    if (args && args->properties && args->properties->ValueExists("maxDec"))
    {
        max_deceleration_ = strtod(args->properties->GetValueStr("maxDec"));
    }
    if (args && args->properties && args->properties->ValueExists("maxAcc"))
    {
        max_acceleration_ = strtod(args->properties->GetValueStr("maxAcc"));
    }
    if (args && args->properties && args->properties->ValueExists("KP"))
    {
        k_p_ = strtod(args->properties->GetValueStr("KP"));
    }
    if (args && args->properties && args->properties->ValueExists("KD"))
    {
        k_d_ = strtod(args->properties->GetValueStr("KD"));
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
            bool has_lead = false;

            std::vector<scenarioengine::Object*> vehicles_in_lane = {};
            bool has_vehicles_in_lane = VehiclesInEgoLane(vehicles_in_lane);
            if (has_vehicles_in_lane)
            {
                has_lead = FindVehicleAhead(vehicles_in_lane, VoIType::LEAD);
            }
            
            if (has_lead)// && (diff_ds <= desired_distance_ || diff_ds > desired_distance_)) // Lead within ACC distance
            {
                state_ = State::FOLLOW;
            }
            else if (!has_lead && current_speed_ < desired_speed_ + speed_tolerance_)
            {
                current_speed_ += max_acceleration_ * dt;
                if (current_speed_ > desired_speed_)
                {
                    current_speed_ = desired_speed_;
                }
            }
            break;
        }
        case State::FOLLOW:
        {
            k_p_ = 2.0;
            k_d_ = 2.0;

            roadmanager::PositionDiff diff;
            entities_->object_[0]->pos_.Delta(&vehicles_of_interest_[VoIType::LEAD].vehicle->pos_, diff, false, lookahead_dist_);
            double distance_error = diff.ds - desired_distance_;
            double relative_speed = current_speed_ - vehicles_of_interest_[VoIType::LEAD].vehicle->GetSpeed();
            double distance_error_rate = -relative_speed;

            double desired_acceleration = k_p_ * distance_error + k_d_ * distance_error_rate;
            desired_acceleration = CLAMP(desired_acceleration, -max_acceleration_, max_acceleration_);

            const double distance_tolerance = 0.1; // meters
            if (abs(distance_error) < distance_tolerance && abs(relative_speed) <= 0.1) 
            {
                desired_acceleration = 0; // No need to accelerate or decelerate
            }

            current_speed_ += desired_acceleration * dt;
            if (current_speed_ > desired_speed_)
            {
                current_speed_ = desired_speed_;
            }
            if (current_speed_ < desired_speed_ - abs(speed_tolerance_)) // Should actually be current_speed < tolerance. We only want to change lane if we are following someone
            {
                state_ = State::TRY_CHANGE_LEFT; // We want to change lane
            }
            // Follow vehicle by ID, either ID of target in ego lane or ID of target in lane we change to
            break;
        }
        case State::TRY_CHANGE_LEFT:
        {
            bool adjacent_lanes_available = AdjacentLanesAvailable(lane_ids_available_);
            bool left_lead = false;
            bool left_follow = false;

            if (!adjacent_lanes_available)
            {
                state_ = State::FOLLOW;
                break;
            }


            if (lane_ids_available_[0] != 0)
            {
                target_lane_ = lane_ids_available_[0];
                std::vector<scenarioengine::Object*> adjacent_lane_vehicles = {};
                bool left_side_vehicles = VehiclesInAdjacentLane(adjacent_lane_vehicles, lane_ids_available_[0]); // idx 0 is left
                if (left_side_vehicles)
                {
                    left_lead = FindVehicleAhead(adjacent_lane_vehicles, VoIType::LEFT_LEAD);
                    left_follow = FindVehicleBehind(adjacent_lane_vehicles, VoIType::LEFT_FOLLOW);
                }
            }
            else
            {
                state_ = State::TRY_CHANGE_RIGHT;
                break;
            }

            if (!left_lead && !left_follow)
            {
                state_ = State::CHANGE_LANE;
                break;
            }

            bool left_lead_conditions_fulfilled = true;
            if (left_lead)
            {
                double lead_speed = vehicles_of_interest_[VoIType::LEAD].vehicle->GetSpeed();
                double adjacent_lead_speed = vehicles_of_interest_[VoIType::LEFT_LEAD].vehicle->GetSpeed();
                roadmanager::PositionDiff diff;
                entities_->object_[0]->pos_.Delta(&vehicles_of_interest_[VoIType::LEFT_LEAD].vehicle->pos_, diff, false, lookahead_dist_);
                if (diff.ds < 10 || (diff.ds < 5 && adjacent_lead_speed < lead_speed)) // Adjacent lead far away and faster than ego, we want to change
                {
                    left_lead_conditions_fulfilled = false;
                }
            }

            bool left_follow_conditions_fulfilled = true;
            if (left_follow) // Want to change, but we have an adjacent follow
            {
                roadmanager::PositionDiff diff;
                entities_->object_[0]->pos_.Delta(&vehicles_of_interest_[VoIType::LEFT_FOLLOW].vehicle->pos_, diff, true, lookahead_dist_);
                if (diff.ds > -abs(rear_dist_)) // Too short distance, can't change
                {
                    left_follow_conditions_fulfilled = false;
                }
            }

            if (left_lead_conditions_fulfilled && left_follow_conditions_fulfilled)
            {
                state_ = State::CHANGE_LANE;
                break;
            }
            else
            {
                state_ = State::TRY_CHANGE_RIGHT;
                break;
            }
            break;
        }
        case State::TRY_CHANGE_RIGHT:
        {
            bool right_lead = false;
            bool right_follow = false;
            
            bool right_lane_available = lane_ids_available_[1] != 0;
            if (right_lane_available)
            {
                target_lane_ = lane_ids_available_[1];
                std::vector<scenarioengine::Object*> adjacent_lane_vehicles = {};
                bool right_side_vehicles = VehiclesInAdjacentLane(adjacent_lane_vehicles, lane_ids_available_[1]); // idx 0 is left
                if (right_side_vehicles)
                {
                    right_lead = FindVehicleAhead(adjacent_lane_vehicles, VoIType::RIGHT_LEAD);
                    right_follow = FindVehicleBehind(adjacent_lane_vehicles, VoIType::RIGHT_FOLLOW);
                }
            }
            else
            {
                state_ = State::FOLLOW;
                break;
            }

            if (!right_lead && !right_follow)
            {
                state_ = State::CHANGE_LANE;
                break;
            }

            bool right_lead_conditions_fulfilled = true;
            if (right_lead)
            {
                double lead_speed = vehicles_of_interest_[VoIType::LEAD].vehicle->GetSpeed();
                double adjacent_lead_speed = vehicles_of_interest_[VoIType::RIGHT_LEAD].vehicle->GetSpeed();
                roadmanager::PositionDiff diff;
                entities_->object_[0]->pos_.Delta(&vehicles_of_interest_[VoIType::RIGHT_LEAD].vehicle->pos_, diff, false, lookahead_dist_);

                if (diff.ds < 10 || (diff.ds < 25 && adjacent_lead_speed < lead_speed)) // Adjacent lead far away and faster than ego, we want to change
                {
                    right_lead_conditions_fulfilled = false;
                }
            }

            bool right_follow_conditions_fulfilled = true;
            if (right_follow) // Want to change, but we have an adjacent follow
            {
                roadmanager::PositionDiff diff;
                entities_->object_[0]->pos_.Delta(&vehicles_of_interest_[VoIType::RIGHT_FOLLOW].vehicle->pos_, diff, true, lookahead_dist_);
                if (diff.ds > -abs(rear_dist_)) // Too short distance, can't change
                {
                    right_follow_conditions_fulfilled = false;
                }
            }

            if (right_lead_conditions_fulfilled && right_follow_conditions_fulfilled)
            {
                state_ = State::CHANGE_LANE;
                break;
            }
            else
            {
                state_ = State::FOLLOW;
                break;
            }
            break;
        }
        case State::CHANGE_LANE:
        {
            if (!lane_change_injected)
            {
                auto lane_change = LaneChangeActionStruct{0, 0, target_lane_, 2, 2, lane_change_duration_};
                player_->player_server_->InjectLaneChangeAction(lane_change);
                lane_change_injected = true;
            }
            state_ = State::DRIVE;
            break;
        }
    }

    object_->MoveAlongS(current_speed_ * dt);
    gateway_->updateObjectPos(object_->GetId(), 0.0, &object_->pos_);
    gateway_->updateObjectSpeed(object_->GetId(), 0.0, current_speed_);

    // Wait a while before initiating another lane change
    if (lane_change_injected)
    {
        cooldown_period_ -= dt;
        if (cooldown_period_ < 0.0)
        {
            cooldown_period_ = 5.0 + lane_change_duration_;
            lane_change_injected = false;
        }
    }

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


bool ControllerNaturalDriver::FindVehicleAhead(std::vector<scenarioengine::Object*> vehicles, VoIType type)
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

    vehicles_of_interest_[type] = {closest_lead};
    
    return true;
}

bool ControllerNaturalDriver::FindVehicleBehind(std::vector<scenarioengine::Object*> vehicles, VoIType type)
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

    vehicles_of_interest_[type] = VoI{follow_vehicle};
    
    return true;
}

void ControllerNaturalDriver::ClearVehicleOfInterest(VoIType type)
{
    vehicles_of_interest_[type].vehicle = nullptr;
}

bool ControllerNaturalDriver::AdjacentLanesAvailable(std::array<int, 2> &lane_ids_available)
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