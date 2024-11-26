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

#pragma once

#include <string>
#include <array>
#include <unordered_map>
#include "Controller.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_NATURAL_DRIVER_TYPE_NAME "NaturalDriver"

namespace scenarioengine
{
    struct VoI
    {
        scenarioengine::Object* vehicle;
        roadmanager::PositionDiff diff;
    };

    enum State
    {
        DRIVE = 0,
        CHANGE_LANE,
    };

    enum VoIType // VehicleOfInterestType
    {
        LEAD = 0,
        FOLLOWING,
        LEFT_LEAD,
        LEFT_FOLLOW,
        RIGHT_LEAD,
        RIGHT_FOLLOW,
    };

    class ControllerNaturalDriver : public Controller
    {
    public:
        ControllerNaturalDriver(InitArgs* args);

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_NATURAL_DRIVER_TYPE_NAME;
        }
        const char* GetTypeName() override
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return CONTROLLER_TYPE_NATURAL_DRIVER;
        }
        int GetType() override
        {
            return GetTypeStatic();
        }

        void Init();
        void InitPostPlayer();
        void Step(double dt);
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);

        void GetLeadVehicle();
        void GetFollowVehicle();
        bool AdjacentLanesAvailable();
        void UpdateSurroundingVehicles();
        bool VehiclesInEgoLane(std::vector<scenarioengine::Object*> &vehicles);
        bool VehiclesInAdjacentLane(scenarioengine::Object* object, roadmanager::PositionDiff& diff, VoIType type);
        void FindClosestAhead(scenarioengine::Object* object, roadmanager::PositionDiff& diff, VoIType type);
        void FindClosestBehind(scenarioengine::Object* object, roadmanager::PositionDiff& diff, VoIType type);
        void GetAdjacentLeadAndFollow(const int lane_id);
        bool AbortLaneChange();
        ControllerNaturalDriver* GetOtherDriver(scenarioengine::Object* object);
        double EstimateFreespace(const scenarioengine::Object* follow, const scenarioengine::Object* target, const double ds);

        bool CheckLaneChangePossible(const int lane_id);

        void GetVehicleOfInterestType(int lane_id, VoIType &lead, VoIType &follow);
        void ClearVehicleOfInterest(VoIType type);

        double GetAcceleration(scenarioengine::Object* follow, scenarioengine::Object* lead);
        double GetDesiredGap(double max_acceleration, double max_deceleration, double follow_speed, double lead_speed, double desired_distance, double desired_thw);

        void ReportKeyEvent(int key, bool down);
        void SetDesiredSpeed(double desired_speed)
        {
            desired_speed_ = desired_speed;
        }

        double GetMaxAcceleration()
        {
            return max_acceleration_;
        }
        double GetDesiredSpeed()
        {
            return desired_speed_;
        }
        double GetMaxDeceleration()
        {
            return max_deceleration_;
        }
        double GetDesiredDistance()
        {
            return desired_distance_;
        }
        double GetDesiredTHW()
        {
            return desired_thw_;
        }
        State GetState()
        {
            return state_;
        }

    private:
        vehicle::Vehicle vehicle_;
        bool             active_;
        double           desired_distance_;  // target headway time
        double           actual_distance_;
        double           distance_adjustment_t_;
        double           desired_speed_;
        double           current_speed_;
        double           speed_tolerance_;
        double           lane_change_duration_;
        double           adj_rear_dist_;
        double           adj_lead_dist_;
        double           lookahead_dist_;
        double           max_deceleration_;
        double           max_acceleration_;
        std::array<int, 2> lane_ids_available_;
        std::unordered_map<VoIType, VoI> vehicles_of_interest_;
        double           distance_to_adjacent_lead_;
        bool             lane_change_injected;
        State            state_;
        double           lane_change_delay_;
        double           lane_change_cooldown_;
        int              target_lane_;
        double           desired_thw_;
        double           max_imposed_braking_;
        double           politeness_;
        double           lane_change_acc_gain_;
        int              route_;
        bool             initiate_lanechange_;
    };

    Controller* InstantiateNaturalDriver(void* args);
}  // namespace scenarioengine