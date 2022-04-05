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
 * This controller simulates a bad or dizzy driver by manipulating
 * the speed and lateral offset in a random way.
 * The purpose is purely to demonstrate how to implement a controller.
 */

//#include "playerbase.hpp"
#include "ControllerFollowRoute.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "OSCManeuver.hpp"
#include "ScenarioEngine.hpp"
#include "LaneIndependentRouter.hpp"

using namespace scenarioengine;

Controller *scenarioengine::InstantiateControllerFollowRoute(void *args)
{
	Controller::InitArgs *initArgs = (Controller::InitArgs *)args;

	return new ControllerFollowRoute(initArgs);
}

ControllerFollowRoute::ControllerFollowRoute(InitArgs *args) : Controller(args)
{
}

void ControllerFollowRoute::Init()
{
	LOG("FollowRoute init");

	Controller::Init();
}

double testtime = 0;
void ControllerFollowRoute::Step(double timeStep)
{
	// LOG("FollowRoute step");
	//LOG("current pos: r=%d, l=%d, s=%f, cwi=%d, cwis=%d", object_->pos_.GetTrackId(), object_->pos_.GetLaneId(), object_->pos_.GetS(), currentWaypointIndex_, waypoints_.size());
	if (object_->pos_.GetRoute() != nullptr)
	{
		if (!pathCalculated_)
		{
			CalculateWaypoints();
		}
	}

	if (pathCalculated_ && currentWaypointIndex_ >= waypoints_.size())
	{
		LOG("No waypoints left");
		object_->SetSpeed(0);
		Controller::Step(timeStep);
		return;
	}

	if (pathCalculated_ && !changingLane_)
	{
		roadmanager::Position nextWaypoint = waypoints_[currentWaypointIndex_];
		roadmanager::Position vehiclePos = object_->pos_;

		bool drivingWithRoadDirection = abs(vehiclePos.GetHRelativeDrivingDirection()) < M_PI_2;
		bool sameRoad = nextWaypoint.GetTrackId() == vehiclePos.GetTrackId();
		bool sameLane = nextWaypoint.GetLaneId() == vehiclePos.GetLaneId();
		if (sameRoad)
		{
			bool nearSPos = abs(vehiclePos.GetS() - nextWaypoint.GetS()) < 25;
			if (!sameLane && nearSPos)
			{
				ChangeLane(nextWaypoint.GetLaneId(), 1);
				changingLane_ = true;
			}
			if ((drivingWithRoadDirection && vehiclePos.GetS() > nextWaypoint.GetS()) ||
				(!drivingWithRoadDirection && vehiclePos.GetS() < nextWaypoint.GetS()))
			{
				currentWaypointIndex_++;
			}
		}
	}

	for (size_t i = 0; i < actions_.size(); i++)
	{
		OSCPrivateAction *action = actions_[i];
		if (action->name_ != "LaneChange")
		{
			continue;
		}

		if (!action->IsActive())
		{
			LOG("ACTION START");
			action->Start(scenarioEngine_->getSimulationTime(), timeStep);
		}
		else if (action->IsActive())
		{
			action->Step(scenarioEngine_->getSimulationTime(), timeStep);
			if (action->state_ != OSCAction::State::COMPLETE)
			{
				action->UpdateState();
			}
		}

		if (action->state_ == OSCAction::State::COMPLETE)
		{
			LOG("ACTION COMPLETED");
			actions_.erase(actions_.begin() + i);
			changingLane_ = false;
		}
	}

	Controller::Step(timeStep);
}

void ControllerFollowRoute::CalculateWaypoints()
{
	roadmanager::Position startPos = object_->pos_;
	roadmanager::Position targetPos = object_->pos_.GetRoute()->all_waypoints_.back();

	roadmanager::LaneIndependentRouter router(odr_);

	std::vector<roadmanager::Node *> pathToGoal = router.CalculatePath(startPos, targetPos, roadmanager::RouteStrategy::SHORTEST);
	if (pathToGoal.empty())
	{
		LOG("Path not found");
	}
	else
	{
		waypoints_ = router.GetWaypoints(pathToGoal, targetPos);
		pathCalculated_ = true;
	}
}

void ControllerFollowRoute::Activate(ControlDomains domainMask)
{
	LOG("FollowRoute activate");

	this->mode_ = Controller::Mode::MODE_ADDITIVE;
	if (object_ != nullptr)
	{
		odr_ = object_->pos_.GetOpenDrive();
	}
	currentWaypointIndex_ = 0;
	pathCalculated_ = false;
	changingLane_ = false;
	waypoints_ = {};
	Controller::Activate(domainMask);
}

void ControllerFollowRoute::ChangeLane(int lane, double time)
{
	LatLaneChangeAction *action_lanechange = new LatLaneChangeAction();
	action_lanechange->name_ = "LaneChange";
	action_lanechange->object_ = object_;
	action_lanechange->transition_.shape_ = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
	action_lanechange->transition_.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
	action_lanechange->transition_.SetParamTargetVal(time);
	action_lanechange->max_num_executions_ = 1;

	LatLaneChangeAction::TargetAbsolute *test = new LatLaneChangeAction::TargetAbsolute;
	test->value_ = lane;
	action_lanechange->target_ = test;
	actions_.push_back(action_lanechange);
}

void ControllerFollowRoute::ReportKeyEvent(int key, bool down)
{
}
