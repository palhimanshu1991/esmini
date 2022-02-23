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
#include <queue>
#include "ControllerFollowRoute.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "OSCManeuver.hpp"
#include "ScenarioEngine.hpp"

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
bool pathCalculated = false;
void ControllerFollowRoute::Step(double timeStep)
{
	// LOG("FollowRoute step");
	// object_->MoveAlongS(timeStep * object_->GetSpeed());
	roadmanager::Route *test = nullptr;
	if (object_->pos_.GetRoute() != nullptr)
	{
		if (!pathCalculated)
		{
			//roadmanager::Position targetWaypoint(5,-1,15,0);
			roadmanager::Position targetWaypoint = object_->pos_.GetRoute()->all_waypoints_.back();
			targetWaypoint.Print();
			std::vector<roadmanager::RoadPath::PathNode *> pathToGoal = CalculatePath(&targetWaypoint, RouteStrategy::SHORTEST);
			LOG("Path to goal size: %d", pathToGoal.size());
			for (roadmanager::RoadPath::PathNode *node : pathToGoal)
			{
				LOG("%d", node->fromRoad->GetId());
			}
			pathCalculated = true;
		}

		// test = object_->pos_.GetRoute();
		// if (test->GetWaypoint(-1)->GetLaneId() != object_->pos_.GetLaneId() &&
		//test->GetWaypoint(-1)->GetTrackId() == object_->pos_.GetTrackId())
		// {
		// 	int laneid = test->GetWaypoint(-1)->GetLaneId();
		// 	LOG("ADD ACTION");
		// 	ChangeLane(laneid, 3);
		// }
	}
	testtime += timeStep;
	if (testtime > 1)
	{
		LOG("Nr of actions: %d", actions_.size());
		testtime = 0;
	}
	for (size_t i = 0; i < actions_.size(); i++)
	{
		OSCPrivateAction *action = actions_[i];
		if (!action->IsActive())
		{
			LOG("ACTION START");
			action->Start(scenarioEngine_->getSimulationTime(), timeStep);
		}
		if (action->IsActive())
		{
			// LOG("ACTION STEP");
			action->Step(scenarioEngine_->getSimulationTime(), timeStep);
			if (action->state_ != OSCAction::State::COMPLETE)
			{
				action->UpdateState();
			}
		}
		if (action->state_ == OSCAction::State::COMPLETE)
		{
			LOG("ACTION COMPLETED");
			LOG("actions before end: %d", actions_.size());
			// action->End();
			actions_.erase(actions_.begin() + i);
			LOG("actions after erase: %d", actions_.size());
		}
	}

	Controller::Step(timeStep);
}

void ControllerFollowRoute::Activate(ControlDomains domainMask)
{
	LOG("FollowRoute activate");

	this->mode_ = Controller::Mode::MODE_ADDITIVE;
	// Trigger* trigger = new Trigger(0);
	// ConditionGroup* conGroup = new ConditionGroup();
	// TrigBySimulationTime* condition = new TrigBySimulationTime();
	// condition->value_ = 2;

	// conGroup->condition_.push_back(condition);
	// trigger->conditionGroup_.push_back(conGroup);
	// event_lanechange->start_trigger_ = trigger;

	// Grab and inspect road network
	roadmanager::OpenDrive *odr = nullptr;
	if (object_ != nullptr)
	{
		odr = object_->pos_.GetOpenDrive();
	}

	if (odr != nullptr)
	{
		for (int i = 0; i < odr->GetNumOfRoads(); i++)
		{
			roadmanager::Road *road = odr->GetRoadByIdx(i);
			LOG("road[%d] id: %d length: %.2f", i, road->GetId(), road->GetLength());
		}
	}

	Controller::Activate(domainMask);
}

void ControllerFollowRoute::ChangeLane(int lane, double time)
{
	LatLaneChangeAction *action_lanechange = new LatLaneChangeAction();
	action_lanechange->object_ = object_;
	action_lanechange->transition_.shape_ = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
	action_lanechange->transition_.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
	action_lanechange->transition_.SetParamTargetVal(time);
	action_lanechange->max_num_executions_ = 1;

	LatLaneChangeAction::TargetAbsolute *test = new LatLaneChangeAction::TargetAbsolute;
	test->value_ = lane;
	action_lanechange->target_ = test;
	actions_.push_back(action_lanechange);

	// Event* event_lanechange = new Event();
	// event_lanechange->action_.push_back(action_lanechange);
	// event_lanechange->priority_ = Event::Priority::OVERWRITE;
	// event_lanechange->name_="HelperLaneChange";
	// event_lanechange->max_num_executions_ = 1;
	// object_->addEvent(event_lanechange);
}

void ControllerFollowRoute::ReportKeyEvent(int key, bool down)
{
}

// Checks if targetlane is in the same direction as the current driving direction,
// Returns true if so, false if not
bool ControllerFollowRoute::TargetLaneIsInDrivingDirection(roadmanager::RoadPath::PathNode *pNode, roadmanager::Road *nextRoad, int targetLaneId)
{
	roadmanager::ContactPointType linkContactPoint = pNode->link->GetContactPointType();
	bool sameDirection = false;
	if (linkContactPoint == roadmanager::ContactPointType::CONTACT_POINT_START)
	{
		LOG("TargetLaneIsInDrivingDirection: CONTACT_POINT_START");
		if (nextRoad->IsSuccessor(pNode->fromRoad, &linkContactPoint))
		{
			LOG("TargetLaneIsInDrivingDirection: CONTACT_POINT_START IsSuccessor");
			sameDirection = sgn(pNode->fromLaneId) != sgn(targetLaneId);
		}
		else if (nextRoad->IsPredecessor(pNode->fromRoad, &linkContactPoint))
		{
			LOG("TargetLaneIsInDrivingDirection: CONTACT_POINT_START IsPredecessor");
			sameDirection = sgn(pNode->fromLaneId) == sgn(targetLaneId);
		}
	}
	else if (linkContactPoint == roadmanager::ContactPointType::CONTACT_POINT_END)
	{
		LOG("TargetLaneIsInDrivingDirection: CONTACT_POINT_END");
		if (nextRoad->IsSuccessor(pNode->fromRoad, &linkContactPoint))
		{
			LOG("TargetLaneIsInDrivingDirection: CONTACT_POINT_END IsSuccessor");
			sameDirection = sgn(pNode->fromLaneId) == sgn(targetLaneId);
		}
		else if (nextRoad->IsPredecessor(pNode->fromRoad, &linkContactPoint))
		{
			LOG("TargetLaneIsInDrivingDirection: CONTACT_POINT_END IsPredecessor");
			sameDirection = sgn(pNode->fromLaneId) != sgn(targetLaneId);
		}
	}
	else
	{
		LOG("Undefined contact point");
	}

	if (!sameDirection)
	{
		LOG("Target waypoint is in opposite direction");
	}

	return sameDirection;
}

// Gets the next pathnode for the nextroad based on current srcnode
std::vector<roadmanager::RoadPath::PathNode *> ControllerFollowRoute::GetNextNodes(roadmanager::Road *nextRoad, roadmanager::RoadPath::PathNode *srcNode)
{
	// Register length of this road and find node in other end of the road (link)

	roadmanager::RoadLink *nextLink = 0;

	if (srcNode->link->GetElementType() == roadmanager::RoadLink::ELEMENT_TYPE_ROAD)
	{
		LOG("GetNextNode: ELEMENT_TYPE_ROAD");
		// node link is a road, find link in the other end of it
		if (srcNode->link->GetContactPointType() == roadmanager::ContactPointType::CONTACT_POINT_END)
		{
			nextLink = nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR);
		}
		else
		{
			nextLink = nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR);
		}
	}
	else if (srcNode->link->GetElementType() == roadmanager::RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
	{
		LOG("GetNextNode: ELEMENT_TYPE_JUNCTION");
		roadmanager::Junction *junction = roadmanager::Position::GetOpenDrive()->GetJunctionById(srcNode->link->GetElementId());
		if (junction && junction->GetType() == roadmanager::Junction::JunctionType::DIRECT)
		{
			LOG("GetNextNode: ELEMENT_TYPE_JUNCTION DIRECT_JUNCTION");
			if (nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR) &&
				nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR)->GetElementId() == junction->GetId())
			{
				// Node link is a direct junction, and it is the successor to the road being checked
				// hence next link is the predecessor of that road
				nextLink = nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR);
			}
			else if (nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR) &&
					 nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR)->GetElementId() == junction->GetId())
			{
				// Node link is a direct junction, and it is the predecessor to the road being checked
				// hence next link is the successor of that road
				nextLink = nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR);
			}
		}
		else
		{
			LOG("GetNextNode: ELEMENT_TYPE_JUNCTION COMMON_JUNCTION");
			if (nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR) &&
				nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR)->GetElementId() == srcNode->fromRoad->GetId())
			{
				// Node link is a non direct junction, and it is the successor to the connecting road being checked
				// hence next link is the predecessor of that connecting road
				nextLink = nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR);
			}
			else if (nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR) &&
					 nextRoad->GetLink(roadmanager::LinkType::PREDECESSOR)->GetElementId() == srcNode->fromRoad->GetId())
			{
				// Node link is a non direct junction, and it is the predecessor to the connecting road being checked
				// hence next link is the successor of that connecting road
				nextLink = nextRoad->GetLink(roadmanager::LinkType::SUCCESSOR);
			}
		}
	}
	std::vector<roadmanager::RoadPath::PathNode *> nextNodes;
	if (nextLink == 0)
	{
		// end of road
		LOG("GetNextNode: End of road, no next link exists");
		return nextNodes;
	}
	std::vector<int> connectingLaneIds = GetConnectingLanes(srcNode, nextRoad);
	if (connectingLaneIds.empty())
	{
		// no existing lanes
		LOG("GetNextNode: No lanes exist");
		return nextNodes;
	}
	for (int nextLaneId : connectingLaneIds)
	{
		// create next node
		roadmanager::RoadPath::PathNode *pNode = new roadmanager::RoadPath::PathNode;
		pNode->dist = srcNode->dist + nextRoad->GetLength();
		pNode->link = nextLink;
		pNode->fromRoad = nextRoad;
		pNode->fromLaneId = nextLaneId;
		pNode->previous = srcNode;
		nextNodes.push_back(pNode);
	}

	return nextNodes;
}

std::vector<int> ControllerFollowRoute::GetConnectingLanes(roadmanager::RoadPath::PathNode *srcNode, roadmanager::Road *nextRoad)
{

	roadmanager::LaneSection *lanesection = nullptr;
	if (srcNode->link->GetType() == roadmanager::LinkType::SUCCESSOR)
	{
		LOG("GetConnectingLanes: SUCCESSOR");
		int nrOfLanesection = srcNode->fromRoad->GetNumberOfLaneSections();
		lanesection = srcNode->fromRoad->GetLaneSectionByIdx(nrOfLanesection - 1);
		LOG("GetConnectingLanes: lanesection id: %d", nrOfLanesection - 1);
	}
	else
	{
		LOG("GetConnectingLanes: PREDCESSOR");
		lanesection = srcNode->fromRoad->GetLaneSectionByIdx(0);
	}

	std::vector<int> connectingLaneIds;
	int nrOfLanes = lanesection->GetNumberOfLanes();
	LOG("GetConnectingLanes: NrOfLanes: %d", nrOfLanes);
	LOG("GetConnectingLanes: currentRoad: %d currentLane: %d", srcNode->fromRoad->GetId(),srcNode->fromLaneId);
	for (size_t i = 0; i < nrOfLanes; i++)
	{	
		roadmanager::Lane *lane = lanesection->GetLaneByIdx(i);
		int currentlaneId = lane->GetId();
		LOG("GetConnectingLanes: CurrentLane: %d", currentlaneId);
		if (lane->IsDriving() && SIGN(currentlaneId) == SIGN(srcNode->fromLaneId))
		{
			int nextLaneId = srcNode->fromRoad->GetConnectingLaneId(srcNode->link, currentlaneId, nextRoad->GetId());
			LOG("GetConnectingLanes: nextRoad: %d nextLaneId: %d", nextRoad->GetId(),nextLaneId);
			if (nextLaneId != 0)
			{
				LOG("GetConnectingLanes: push lane id: %d",nextLaneId);
				connectingLaneIds.push_back(nextLaneId);
			}
		}
	}
	return (connectingLaneIds);
}

// Calculate path to target and returns it as a vector of pathnodes
std::vector<roadmanager::RoadPath::PathNode *> ControllerFollowRoute::CalculatePath(roadmanager::Position *targetWaypoint, RouteStrategy routeStrategy)
{
	using namespace roadmanager;

	std::vector<RoadPath::PathNode *> pathToGoal;
	std::vector<RoadPath::PathNode *> distance;

	OpenDrive *odr = nullptr;
	if (object_ != nullptr)
	{
		odr = object_->pos_.GetOpenDrive();
	}

	Road *startRoad = odr->GetRoadById(object_->pos_.GetTrackId());
	int startLaneId = object_->pos_.GetLaneId();
	Position startPos = object_->pos_;
	ContactPointType contactPoint = ContactPointType::CONTACT_POINT_UNDEFINED;
	RoadLink *nextElement = nullptr;

	Road *targetRoad = odr->GetRoadById(targetWaypoint->GetTrackId());
	int targetLaneId = targetWaypoint->GetLaneId();

	// Look only in forward direction, w.r.t. entity heading
	if (startPos.GetHRelative() < M_PI_2 || startPos.GetHRelative() > 3 * M_PI_2)
	{
		// Along road direction
		contactPoint = ContactPointType::CONTACT_POINT_END;
		nextElement = startRoad->GetLink(LinkType::SUCCESSOR); // Find link to next road or junction
	}
	else
	{
		// Opposite road direction
		contactPoint = ContactPointType::CONTACT_POINT_START;
		nextElement = startRoad->GetLink(LinkType::PREDECESSOR); // Find link to previous road or junction
	}

	auto compare = [](RoadPath::PathNode *a, RoadPath::PathNode *b)
	{ return a->fromRoad->GetLength() < b->fromRoad->GetLength(); };
	std::priority_queue<RoadPath::PathNode *, std::vector<RoadPath::PathNode *>, decltype(compare)> unvisited(compare);
	std::vector<RoadPath::PathNode *> visited;

	// If start and end waypoint are on the same road and same lane,
	// no pathToGoal are needed
	if (startRoad == targetRoad && startLaneId == targetLaneId)
	{
		return pathToGoal;
	}

	if (!nextElement)
	{
		// No link (next road element) found
		return pathToGoal;
	}

	RoadPath::PathNode *pNode = new RoadPath::PathNode;
	pNode->link = nextElement;
	pNode->fromRoad = startRoad;
	pNode->fromLaneId = startLaneId;
	pNode->previous = 0;
	pNode->contactPoint = contactPoint;
	if (contactPoint == ContactPointType::CONTACT_POINT_START)
	{
		pNode->dist = startPos.GetS(); // distance to first road link is distance to start of road
	}
	else if (contactPoint == ContactPointType::CONTACT_POINT_END)
	{
		pNode->dist = startRoad->GetLength() - startPos.GetS(); // distance to end of road
	}

	unvisited.push(pNode);

	bool found = false;
	RoadPath::PathNode *currentNode = nullptr;
	while (!unvisited.empty() && !found)
	{
		currentNode = unvisited.top();
		unvisited.pop();
		LOG("CURRENTNODE: %d", currentNode->fromRoad->GetId());

		RoadLink *link = currentNode->link;
		Road *pivotRoad = currentNode->fromRoad;
		int pivotLaneId = currentNode->fromLaneId;
		Road *nextRoad = nullptr;

		if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD)
		{
			LOG("CalculatePath: ELEMENT_TYPE_ROAD");
			nextRoad = odr->GetRoadById(link->GetElementId());
			LOG("CalculatePath: nextRoad: %d",nextRoad->GetId());
			LOG("CalculatePath: targetRoad: %d",targetRoad->GetId());
			if (nextRoad == targetRoad)
			{
				found = TargetLaneIsInDrivingDirection(currentNode, nextRoad, targetLaneId);
				if (found)
				{
					// Create last node (targetnode) and push to distance vector
					RoadPath::PathNode *targetNode = new RoadPath::PathNode;
					targetNode->previous = currentNode;
					targetNode->dist = currentNode->dist + nextRoad->GetLength();
					targetNode->fromRoad = nextRoad;
					targetNode->fromLaneId = currentNode->fromRoad->GetConnectingLaneId(currentNode->link, currentNode->fromLaneId, nextRoad->GetId());
					targetNode->link = nullptr;
					distance.push_back(targetNode);
				}
			}
			else
			{
				std::vector<RoadPath::PathNode *> nextNodes = GetNextNodes(nextRoad, currentNode);
				LOG("CalculatePath:Road: Size of nextNodes: %d",nextNodes.size());
				for (RoadPath::PathNode *nextNode : nextNodes)
				{
					// Check if next node is already among unvisited
					size_t i;
					for (i = 0; i < distance.size(); i++)
					{
						if (distance[i]->link == nextNode->link)
						{
							// Consider it, i.e. calc distance and potentially store it (if less than old)
							if (nextNode->dist < distance[i]->dist)
							{
								// Replace current node with updated node
								distance[i] = nextNode;
							}
							break;
						}
					}

					if (i == distance.size())
					{
						distance.push_back(nextNode);
					}
					unvisited.push(nextNode);
				}
			}
		}
		else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION)
		{
			LOG("CalculatePath: ELEMENT_TYPE_JUNCTION");
			// check all junction links (connecting roads) that has pivot road as incoming road
			roadmanager::Junction *junction = odr->GetJunctionById(link->GetElementId());
			for (size_t j = 0; j < junction->GetNoConnectionsFromRoadId(pivotRoad->GetId()); j++)
			{
				nextRoad = odr->GetRoadById(junction->GetConnectingRoadIdFromIncomingRoadId(pivotRoad->GetId(), (int)j));
				if (nextRoad == 0)
				{
					return pathToGoal;
				}

				if (nextRoad == targetRoad) // target road reached
				{
					found = TargetLaneIsInDrivingDirection(currentNode, nextRoad, targetLaneId);
					if (found)
					{
						// Create last node (targetnode) and push to distance vector
						RoadPath::PathNode *targetNode = new RoadPath::PathNode;
						targetNode->previous = currentNode;
						targetNode->dist = currentNode->dist + nextRoad->GetLength();
						targetNode->fromRoad = nextRoad;
						targetNode->fromLaneId = currentNode->fromRoad->GetConnectingLaneId(currentNode->link, currentNode->fromLaneId, nextRoad->GetId());
						targetNode->link = nullptr;
						distance.push_back(targetNode);
					}
				}
				else
				{
					std::vector<RoadPath::PathNode *> nextNodes = GetNextNodes(nextRoad, currentNode);
					LOG("CalculatePath:Junction: Size of nextNodes: %d",nextNodes.size());
					for (RoadPath::PathNode *nextNode : nextNodes)
					{
						// Check if next node is already among unvisited
						size_t i;
						for (i = 0; i < distance.size(); i++)
						{
							if (distance[i]->link == nextNode->link)
							{
								// Consider it, i.e. calc distance and potentially store it (if less than old)
								if (nextNode->dist < distance[i]->dist)
								{
									// Replace current node with updated node
									distance[i] = nextNode;
								}
								break;
							}
						}

						if (i == distance.size())
						{
							distance.push_back(nextNode);
						}
						unvisited.push(nextNode);
					}
				}
			}
		}
	}

	if (found)
	{
		LOG("Goal found");
		LOG("CurrentNode road id: %d", currentNode->fromRoad->GetId());
		RoadPath::PathNode *nodeIterator = distance.back();
		while (nodeIterator != 0)
		{
			pathToGoal.push_back(nodeIterator);
			nodeIterator = nodeIterator->previous;
		}
	}
	else
	{
		LOG("Path to target not found");
		for(RoadPath::PathNode* pn : distance)
		{
			LOG("Road:%d Dist:%f Prev: %d",pn->fromRoad->GetId(),pn->dist,pn->previous->fromRoad->GetId());
		}
	}

	return pathToGoal;
}
