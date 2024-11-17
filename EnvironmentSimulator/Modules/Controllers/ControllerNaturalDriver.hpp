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
    enum State
    {
        DRIVE = 0,
        FOLLOW,
        TRY_CHANGE_LEFT,
        TRY_CHANGE_RIGHT,
        CHANGE_LANE,
    };

    enum VoIType // VehicleOfInterest
    {
        LEAD = 0,
        LEFT_LEAD,
        LEFT_FOLLOW,
        RIGHT_LEAD,
        RIGHT_FOLLOW,
    };

    struct VoI // VehicleOfInterest
    {
        scenarioengine::Object* vehicle;
    };

    class ControllerNaturalDriver : public Controller
    {
    public:
        ControllerNaturalDriver(InitArgs* args);

        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_NATURAL_DRIVER_TYPE_NAME;
        }
        virtual const char* GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return CONTROLLER_TYPE_NATURAL_DRIVER;
        }
        virtual int GetType()
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
        bool FindVehicleAhead(std::vector<scenarioengine::Object*> vehicles, VoIType type);
        bool FindVehicleBehind(std::vector<scenarioengine::Object*> vehicles, VoIType type);
        void AdjacentLaneActors(const int lane_id, bool &lead, bool& follow);
        bool CheckLaneChangeConditions(bool has_lead, bool has_follow);
        void LeadInAdjacentLane(std::vector<scenarioengine::Object*> vehicles);
        void FollowInAdjacentLane(std::vector<scenarioengine::Object*> vehicles);
        bool AdjacentLanesAvailable(std::array<int, 2> &lane_ids_available);
        bool VehiclesInEgoLane(std::vector<scenarioengine::Object*> &vehicles);
        bool VehiclesInAdjacentLane(std::vector<scenarioengine::Object*> &vehicles, int lane_id);
        void ClearVehicleOfInterest(VoIType type);
        void ReportKeyEvent(int key, bool down);
        void SetDesiredSpeed(double desired_speed)
        {
            desired_speed_ = desired_speed;
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
        float            lane_change_duration_;
        double           rear_dist_;
        double           lookahead_dist_;
        double           max_deceleration_;
        double           max_acceleration_;
        std::array<int, 2> lane_ids_available_;
        std::unordered_map<VoIType, VoI> vehicles_of_interest_;
        double           distance_to_adjacent_lead_;
        bool             lane_change_injected;
        State            state_;
        double           cooldown_period_;
        int              target_lane_;
        double           k_p_;
        double           k_d_;
    };

    Controller* InstantiateNaturalDriver(void* args);
}  // namespace scenarioengine