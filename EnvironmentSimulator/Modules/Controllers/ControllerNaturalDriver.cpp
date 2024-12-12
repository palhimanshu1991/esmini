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
      desired_speed_(15.0),
      current_speed_(desired_speed_),
      lane_change_duration_(3.0),
      lookahead_dist_(115.0),
      max_deceleration_(-3.0),
      max_acceleration_(3.0),
      lane_ids_available_({0, 0}),
      vehicles_of_interest_({}),
      lane_change_injected(false),
      state_(State::DRIVE),
      lane_change_delay_(1.0),
      lane_change_cooldown_(lane_change_duration_ + lane_change_delay_),
      target_lane_(0),
      desired_thw_(2.0),
      max_imposed_braking_(3.0),
      politeness_(0.5),
      lane_change_acc_gain_(0.2),
      route_(-1),
      initiate_lanechange_(false)
{
    operating_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_LONG);

    if (args && args->properties && args->properties->ValueExists("desiredDistance"))
    {
        desired_distance_ = strtod(args->properties->GetValueStr("desiredDistance"));
    }
    if (args && args->properties && args->properties->ValueExists("desiredSpeed"))
    {
        desired_speed_ = strtod(args->properties->GetValueStr("desiredSpeed"));
    }
    if (args && args->properties && args->properties->ValueExists("laneChangeDuration"))
    {
        lane_change_duration_ = static_cast<float>(strtod(args->properties->GetValueStr("laneChangeDuration")));
    }
    if (args && args->properties && args->properties->ValueExists("lookAheadDistance"))
    {
        lookahead_dist_ = strtod(args->properties->GetValueStr("lookAheadDistance"));
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
    if (args && args->properties && args->properties->ValueExists("thw"))
    {
        desired_thw_ = strtod(args->properties->GetValueStr("thw"));
    }
    if (args && args->properties && args->properties->ValueExists("maxImposedBraking"))
    {
        max_imposed_braking_ = strtod(args->properties->GetValueStr("maxImposedBraking"));
    }
    if (args && args->properties && args->properties->ValueExists("route"))
    {
        route_ = strtoi(args->properties->GetValueStr("route"));
    }
    if (args && args->properties && args->properties->ValueExists("laneChangeAccGain"))
    {
        lane_change_acc_gain_ = strtoi(args->properties->GetValueStr("laneChangeAccGain"));
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
    lane_change_cooldown_ = lane_change_duration_ + lane_change_delay_;
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
    UpdateSurroundingVehicles();
    double acceleration;

    switch (state_)
    {
        case State::DRIVE:
        {
            acceleration = GetAcceleration(object_, vehicles_of_interest_[VoIType::LEAD].vehicle);
            if (acceleration < max_deceleration_)
            {
                acceleration = max_deceleration_;
            }

            if (!lane_change_injected)
            {
                bool adjacent_lanes_available = AdjacentLanesAvailable();
                if (!adjacent_lanes_available)
                {
                    break;
                }
                for (const auto& lane_id : lane_ids_available_)
                {
                    if (lane_id != 0)
                    {
                        initiate_lanechange_ = CheckLaneChangePossible(lane_id);
                        if (initiate_lanechange_ && object_->GetSpeed() > 0)
                        {
                            target_lane_ = lane_id;
                            state_       = State::CHANGE_LANE;
                            break;
                        }
                    }
                }
            }
            break;
        }
        case State::CHANGE_LANE:
        {
            acceleration = 0.0;
            break;
        }
    }

    if (initiate_lanechange_)
    {
        if (AbortLaneChange())
        {
            target_lane_ = 0;
            state_       = State::DRIVE;
        }
        else
        {
            auto lane_change = LaneChangeActionStruct{object_->GetId(), 0, target_lane_, 2, 2, static_cast<float>(lane_change_duration_)};
            player_->player_server_->InjectLaneChangeAction(lane_change);
            lane_change_injected = true;
        }

        initiate_lanechange_ = false;
    }

    // Wait a while before initiating another lane change
    if (lane_change_injected)
    {
        lane_change_cooldown_ -= dt;
        if (state_ == State::CHANGE_LANE &&
            lane_change_cooldown_ <= lane_change_delay_)  // Keep constant long. acceleration during lane_change_duration_
        {
            target_lane_ = 0;
            state_       = State::DRIVE;
        }
        if (lane_change_cooldown_ < 0.0)
        {
            lane_change_cooldown_ = lane_change_delay_ + lane_change_duration_;  // Reset the cooldown
            lane_change_injected  = false;
        }
    }

    current_speed_ += acceleration * dt;

    if (current_speed_ >= desired_speed_)
    {
        current_speed_ = desired_speed_;
    }
    else if (current_speed_ > object_->GetMaxSpeed())
    {
        current_speed_ = object_->GetMaxSpeed();
    }
    else if (current_speed_ < 0.0)
    {
        current_speed_ = 0.0;
    }

    object_->MoveAlongS(current_speed_ * dt);
    gateway_->updateObjectPos(object_->GetId(), 0.0, &object_->pos_);
    gateway_->updateObjectSpeed(object_->GetId(), 0.0, current_speed_);

    Controller::Step(dt);
}

bool ControllerNaturalDriver::AbortLaneChange()
{
    std::vector<Object*> objects_in_radius;
    FilterSurroundingVehicles(objects_in_radius);

    // Check if someone else is already changing to intended lane
    for (const auto& other_object : objects_in_radius)
    {
        ControllerNaturalDriver* nd = GetOtherDriver(other_object);
        if (nd == nullptr)
        {
            continue;  // Other car has no driver
        }

        roadmanager::PositionDiff diff        = {};
        bool                      delta_exist = false;
        for (const auto& veh : vehicles_of_interest_)
        {
            if (other_object == veh.second.vehicle)
            {
                diff        = veh.second.diff;
                delta_exist = true;
            }
        }

        if (!delta_exist)
        {
            object_->pos_.Delta(&other_object->pos_, diff, true, lookahead_dist_);  // Only look ahead or not?
        }

        double freespace = EstimateFreespace(object_, other_object, diff.ds);

        double follow_current_speed = other_object->GetSpeed();
        double desired_gap =
            GetDesiredGap(max_acceleration_, max_deceleration_, follow_current_speed, current_speed_, desired_distance_, desired_thw_);

        if (freespace >= 0 && freespace < desired_gap && nd->lane_change_injected && nd->target_lane_ == this->target_lane_ &&
            !(other_object->pos_.GetLaneId() == object_->pos_.GetLaneId()))
        {
            return true;
        }
    }

    return false;
}

void ControllerNaturalDriver::UpdateSurroundingVehicles()
{
    vehicles_of_interest_ = {};

    std::vector<Object*> objects_in_radius;
    FilterSurroundingVehicles(objects_in_radius);

    for (const auto& obj : objects_in_radius)
    {
        roadmanager::PositionDiff diff = {};
        object_->pos_.Delta(&obj->pos_, diff, true, lookahead_dist_);

        if (diff.dLaneId == 0)
        {
            FindClosestAhead(obj, diff, VoIType::LEAD);
            FindClosestBehind(obj, diff, VoIType::FOLLOWING);
        }
        else if (diff.dLaneId * -SIGN(object_->pos_.GetLaneId()) == 1)  // Always 1 on left lane
        {
            FindClosestAhead(obj, diff, VoIType::LEFT_LEAD);
            FindClosestBehind(obj, diff, VoIType::LEFT_FOLLOW);
        }
        else if (diff.dLaneId * -SIGN(object_->pos_.GetLaneId()) == -1)  // Always -1 on right lane
        {
            FindClosestAhead(obj, diff, VoIType::RIGHT_LEAD);
            FindClosestBehind(obj, diff, VoIType::RIGHT_FOLLOW);
        }
    }
}

// Simple distance calc to find only relevant vehicles around ego
void ControllerNaturalDriver::FilterSurroundingVehicles(std::vector<Object*>& filtered_vehicles)
{
    for (const auto& obj : entities_->object_)
    {
        if (obj->GetId() == object_->GetId())
        {
            continue;
        }

        double relative_distance;
        object_->pos_.Distance(&obj->pos_,
                               roadmanager::CoordinateSystem::CS_ENTITY,
                               roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN,
                               relative_distance,
                               lookahead_dist_);

        if (relative_distance <= lookahead_dist_)
        {
            filtered_vehicles.push_back(obj);
        }
    }
}

ControllerNaturalDriver* ControllerNaturalDriver::GetOtherDriver(scenarioengine::Object* object)
{
    ControllerNaturalDriver* nd;
    if (object->GetControllerActiveOnDomain(ControlDomains::DOMAIN_LONG))
    {
        auto active_controller = object->GetControllerTypeActiveOnDomain(ControlDomains::DOMAIN_LONG);
        if (active_controller == Type::CONTROLLER_TYPE_NATURAL_DRIVER)
        {
            nd = dynamic_cast<ControllerNaturalDriver*>(object->GetAssignedControllerOftype(active_controller));
            return nd;
        }
    }

    return nullptr;
}

/*
    GetAcceleration() and GetDesiredGap() based on IDM (https://en.wikipedia.org/wiki/Intelligent_driver_model)
*/
double ControllerNaturalDriver::GetAcceleration(scenarioengine::Object* follow, scenarioengine::Object* lead)
{
    if (follow == nullptr)  // No car to calculate acceleration for, return 0
    {
        return 0.0;
    }

    int delta = 4;

    double follow_current_speed = follow->GetSpeed();
    double acceleration         = max_acceleration_ * (1 - std::pow(follow_current_speed / desired_speed_, delta));

    if (lead != nullptr)
    {
        double lead_current_speed = lead->GetSpeed();
        double desired_gap =
            GetDesiredGap(max_acceleration_, max_deceleration_, follow_current_speed, lead_current_speed, desired_distance_, desired_thw_);

        roadmanager::PositionDiff diff = {};
        follow->pos_.Delta(&lead->pos_, diff, false, lookahead_dist_);
        double freespace = EstimateFreespace(follow, lead, diff.ds);
        (freespace == 0) ? freespace = SMALL_NUMBER : freespace = freespace;

        acceleration -= max_acceleration_ * std::pow(desired_gap / freespace, 2);
    }

    return acceleration;
}

double ControllerNaturalDriver::GetDesiredGap(double max_acceleration,
                                              double max_deceleration,
                                              double follow_speed,
                                              double lead_speed,
                                              double desired_distance,
                                              double desired_thw)
{
    double ab             = -max_acceleration * max_deceleration;
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

void ControllerNaturalDriver::GetVehicleOfInterestType(int lane_id, VoIType& lead, VoIType& follow)
{
    if (lane_id == lane_ids_available_[0])
    {
        lead   = VoIType::LEFT_LEAD;
        follow = VoIType::LEFT_FOLLOW;
    }
    else
    {
        lead   = VoIType::RIGHT_LEAD;
        follow = VoIType::RIGHT_FOLLOW;
    }
}

bool ControllerNaturalDriver::CheckLaneChangePossible(const int lane_id)
{
    if (lane_change_injected)
    {
        return false;
    }

    VoIType adj_lead, adj_follow;
    GetVehicleOfInterestType(lane_id, adj_lead, adj_follow);

    double new_following_acceleration      = GetAcceleration(vehicles_of_interest_[adj_follow].vehicle, vehicles_of_interest_[adj_lead].vehicle);
    double new_following_pred_acceleration = GetAcceleration(vehicles_of_interest_[adj_follow].vehicle, object_);
    if (new_following_pred_acceleration < -max_imposed_braking_)
    {
        return false;
    }

    double predicted_new_acceleration = GetAcceleration(object_, vehicles_of_interest_[adj_lead].vehicle);
    if (lane_id == route_)
    {
        if (predicted_new_acceleration < -max_imposed_braking_)
        {
            return false;
        }
    }
    else
    {
        double acceleration               = GetAcceleration(object_, vehicles_of_interest_[VoIType::LEAD].vehicle);
        double old_following_acceleration = GetAcceleration(vehicles_of_interest_[VoIType::FOLLOWING].vehicle, object_);
        double old_following_pred_acceleration =
            GetAcceleration(vehicles_of_interest_[VoIType::FOLLOWING].vehicle, vehicles_of_interest_[VoIType::LEAD].vehicle);

        /* Jerk
            If I change lane, how much...
            "predicted_new_acceleration - acceleration" ...more can ego accelerate?
            "politeness_" ...do I care about being in the way of others?
            "new_following_pred_acceleration - new_following_acceleration" ...more can adjacent behind vehicle accelerate?
            "old_following_pred_acceleration - old_following_acceleration" ...more can current following vehicle accelerate?
        */

        double jerk = predicted_new_acceleration - acceleration +
                      politeness_ * (new_following_pred_acceleration - new_following_acceleration + old_following_pred_acceleration -
                                     old_following_acceleration);

        if (jerk <= lane_change_acc_gain_)
        {
            return false;
        }
    }

    return true;
}

double ControllerNaturalDriver::EstimateFreespace(const scenarioengine::Object* follow, const scenarioengine::Object* target, const double ds)
{
    // adjust longitudinal dist wrt bounding boxes
    double adjusted_gap_length = ds;
    double dHeading            = GetAbsAngleDifference(follow->pos_.GetH(), target->pos_.GetH());
    if (dHeading < M_PI_2)  // objects are pointing roughly in the same direction
    {
        // TODO: Needs adjustment if ref is not BB center
        if (ds > 0)
        {
            adjusted_gap_length -=
                (static_cast<double>(follow->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(follow->boundingbox_.center_.x_)) +
                (static_cast<double>(target->boundingbox_.dimensions_.length_) / 2.0 - static_cast<double>(target->boundingbox_.center_.x_));

            return (adjusted_gap_length < 0) ? 0 : adjusted_gap_length;
        }
        else
        {
            adjusted_gap_length +=
                (static_cast<double>(follow->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(follow->boundingbox_.center_.x_)) +
                (static_cast<double>(target->boundingbox_.dimensions_.length_) / 2.0 - static_cast<double>(target->boundingbox_.center_.x_));

            return (adjusted_gap_length > 0) ? 0 : adjusted_gap_length;
        }
    }

    return ds;  // Not pointing roughly same direction, return ref point differences for now
}

void ControllerNaturalDriver::FindClosestAhead(scenarioengine::Object* object, roadmanager::PositionDiff& diff, VoIType type)
{
    if (vehicles_of_interest_[type].vehicle == nullptr && diff.ds >= 0)
    {
        vehicles_of_interest_[type].vehicle = object;
        vehicles_of_interest_[type].diff    = diff;
    }
    else if (diff.ds >= 0 && diff.ds < vehicles_of_interest_[type].diff.ds)
    {
        vehicles_of_interest_[type].vehicle = object;
        vehicles_of_interest_[type].diff    = diff;
    }
}

void ControllerNaturalDriver::FindClosestBehind(scenarioengine::Object* object, roadmanager::PositionDiff& diff, VoIType type)
{
    if (vehicles_of_interest_[type].vehicle == nullptr && diff.ds < 0)
    {
        vehicles_of_interest_[type].vehicle = object;
        vehicles_of_interest_[type].diff    = diff;
    }
    else if (diff.ds < 0 && diff.ds > vehicles_of_interest_[type].diff.ds)
    {
        vehicles_of_interest_[type].vehicle = object;
        vehicles_of_interest_[type].diff    = diff;
    }
}

bool ControllerNaturalDriver::AdjacentLanesAvailable()
{
    double current_s    = object_->pos_.GetS();
    int    current_lane = object_->pos_.GetLaneId();

    id_t road_id = object_->pos_.GetTrackId();
    auto road    = object_->pos_.GetRoadById(road_id);

    auto ls                      = road->GetLaneSectionByS(current_s);
    auto driving_lanes_available = ls->GetNumberOfDrivingLanesSide(current_lane);

    if (driving_lanes_available == 1)
    {
        // Only 1 lane in current direction
        lane_ids_available_[0] = 0;
        lane_ids_available_[1] = 0;

        return false;
    }

    int left_lane_id  = current_lane - SIGN(current_lane);
    int right_lane_id = current_lane + SIGN(current_lane);

    int current_lane_idx = ls->GetLaneIdxById(current_lane);
    int max_lane_idx     = ls->GetNumberOfLanes() - 1;

    bool left_driving, right_driving;
    if (SIGN(current_lane) == -1)
    {
        int left_lanes = ls->GetNumberOfLanesLeft();
        (current_lane_idx == max_lane_idx) ? right_driving = false : right_driving = ls->GetLaneById(right_lane_id)->IsDriving();
        (current_lane_idx == max_lane_idx - left_lanes) ? left_driving = false : left_driving = ls->GetLaneById(left_lane_id)->IsDriving();
    }
    else
    {
        int right_lanes = ls->GetNumberOfLanesRight();
        (current_lane_idx == 0) ? right_driving = false : right_driving = ls->GetLaneById(right_lane_id)->IsDriving();
        (current_lane_idx == right_lanes - 1) ? left_driving = false : left_driving = ls->GetLaneById(left_lane_id)->IsDriving();
    }

    if (!left_driving && right_driving)  // Lane to the left is not driveable, but right lane is.
    {
        lane_ids_available_[0] = 0;
        lane_ids_available_[1] = right_lane_id;
    }
    else if (left_driving && right_driving)  // Lane to left and right are driving lanes
    {
        lane_ids_available_[0] = left_lane_id;
        lane_ids_available_[1] = right_lane_id;
    }
    else if (left_driving && !right_driving)  // Lane to the right is not driveable, but left lane is.
    {
        lane_ids_available_[0] = left_lane_id;
        lane_ids_available_[1] = 0;
    }
    else
    {
        lane_ids_available_[0] = 0;
        lane_ids_available_[1] = 0;
    }

    return true;
}
