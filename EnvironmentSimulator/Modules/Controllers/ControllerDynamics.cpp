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


#include "ControllerDynamics.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "playerbase.hpp"

#include <random>

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerDynamics(void* args)
{
	Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

	return new ControllerDynamics(initArgs);
}

ControllerDynamics::ControllerDynamics(InitArgs* args) :
	length_(5.0), width_(2.0), height_(1.5), mass_(1000), suspension_stiffness_(2.3), friction_slip_(1.2), roll_influence_(1.0),
	Controller(args)
{
	if (args && args->properties)
	{
	}

	vehicle_.Init(length_, width_, height_, mass_, object_->pos_.GetOpenDrive(), suspension_stiffness_, friction_slip_, roll_influence_);
	vehicle_.SetupFlatGround(100);
}

ControllerDynamics::~ControllerDynamics()
{

}

void ControllerDynamics::Init()
{
	printf("dynamics init\n");

	// Detach object from any constraints aligning to road
	object_->pos_.SetAlignMode(roadmanager::Position::ALIGN_MODE::ALIGN_NONE);

	Controller::Init();
}

void ControllerDynamics::Step(double timeStep)
{
	vehicle_.Step(timeStep);

	double x, y, z, h, p, r;
	vehicle_.GetPosition(x, y, z);
	vehicle_.GetRotation(h, p, r);

	for (int i = 0; i < vehicle_.vehicle_->getNumWheels(); i++)
	{
		const btTransform* xform = &(vehicle_.vehicle_->getWheelTransformWS(i));

		gateway_->updateObjectWheelZ(object_->GetId(), 0.0, xform->getOrigin()[2]);

		float wh, wp, wr;
		xform->getRotation().getEulerZYX(wh, wp, wr);
		gateway_->updateObjectWheelAngle(object_->GetId(), 0.0, h);
	}


	gateway_->updateObjectWorldPos(object_->GetId(), 0.0, x, y, z, h, p, r);

	Controller::Step(timeStep);
}

void ControllerDynamics::Activate(ControlDomains domainMask)
{
	printf("dynamics activate\n");

	vehicle_.Reset(
		object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(),
		object_->pos_.GetH(), object_->pos_.GetP(), object_->pos_.GetR(),
		object_->GetSpeed());

	Controller::Activate(domainMask);
}

void ControllerDynamics::ReportKeyEvent(int key, bool down)
{

}
