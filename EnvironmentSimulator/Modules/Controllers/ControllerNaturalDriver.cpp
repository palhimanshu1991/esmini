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
      thw_(1.5),
      thw_adjustment_t_(3.0),
      desired_speed_(15.0), // TODO: Take from road speed
      current_speed_(desired_speed_),
      speed_tolerance_(2.0),
      lane_change_duration_(3.0),
      lateral_dist_(5.0)
{
    operating_domains_ = static_cast<unsigned int>(ControlDomains::DOMAIN_LONG);

    if (args && args->properties && args->properties->ValueExists("thw"))
    {
        thw_ = strtod(args->properties->GetValueStr("thw"));
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
        lane_change_duration_    = strtod(args->properties->GetValueStr("laneChangeDuration"));
    }
    if (args && args->properties && args->properties->ValueExists("lateralDist"))
    {
        lateral_dist_ = strtod(args->properties->GetValueStr("lateralDist"));
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

void ControllerNaturalDriver::Step(double timeStep)
{



    Controller::Step(timeStep);
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