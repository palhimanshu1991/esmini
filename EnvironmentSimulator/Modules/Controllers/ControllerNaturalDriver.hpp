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
#include "Controller.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_NATURAL_DRIVER_TYPE_NAME "NaturalDriver"

namespace scenarioengine
{
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
        void AdjustToLead(double thw_);
        std::vector<scenarioengine::Object*> VehiclesInEgoLane();
        void ReportKeyEvent(int key, bool down);
        void SetDesiredSpeed(double desired_speed)
        {
            desired_speed_ = desired_speed;
        }

    private:
        vehicle::Vehicle vehicle_;
        bool             active_;
        double           thw_;  // target headway time
        double           thw_adjustment_t_;
        double           desired_speed_;
        double           current_speed_;
        double           speed_tolerance_;
        double           lane_change_duration_;
        double           lateral_dist_;
    };

    Controller* InstantiateNaturalDriver(void* args);
}  // namespace scenarioengine