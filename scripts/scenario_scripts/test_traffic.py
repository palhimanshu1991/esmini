from scenariogeneration import xosc
from scenariogeneration import ScenarioGenerator
from generate_traffic import get_vehicle_positions
import random

class Scenario(ScenarioGenerator):
    def __init__(self):
        ScenarioGenerator.__init__(self)


    def scenario(self, **kwargs):
        # We point to the road we wish to use
        roadfile = "../../resources/xodr/e6mini.xodr"
        road = xosc.RoadNetwork(roadfile=roadfile)

        ## create the storyboard, requires init and when to stop (Trigger), and add the story to it
        sb = xosc.StoryBoard(
            xosc.Init(),
            xosc.ValueTrigger(
                "stop_simulation",
                0,
                xosc.ConditionEdge.none,
                xosc.SimulationTimeCondition(
                    15,
                    xosc.Rule.greaterThan,
                ),
                "stop",
            ),
        )

        egoname = "Ego"

        # We shall define a catalog to find our vehicles in
        catalog = xosc.Catalog()
        catalog.add_catalog("VehicleCatalog", "../xosc/Catalogs/Vehicles")
        catalog.add_catalog("ControllerCatalog", "../xosc/Catalogs/Controllers")


        ## create entities based on entries in the catalog and give them names
        entities = xosc.Entities()
        entities.add_scenario_object(
            egoname, xosc.CatalogReference("VehicleCatalog", "car_white")
        )
        
        init = xosc.Init()
        step_time = xosc.TransitionDynamics(
            xosc.DynamicsShapes.step, xosc.DynamicsDimension.rate, 1
        )
        
        # Define the start conditions for the entities (they need speed and position)
        ego_s = 200
        ego_t = 0
        ego_lid = -2
        ego_rid = 0

        ego_start = (ego_s, ego_t, ego_lid, ego_rid)
        egostart = xosc.TeleportAction(xosc.LanePosition(*ego_start))
        egospeed = xosc.AbsoluteSpeedAction(15, step_time)

        # Add the start conditions to the init stage for each entity
        init.add_init_action(egoname, egostart)
        init.add_init_action(egoname, egospeed)
        
        vehicles = get_vehicle_positions(roadfile=roadfile,
                                         ego_pos=ego_start,
                                         density=1.0,
                                         catalog_path="../../resources/xosc/Catalogs/Vehicles/VehicleCatalog.xosc")

        for i in range(1, len(vehicles)):
            targetname = f"Target{i}"
            man_group = xosc.ManeuverGroup(f"{targetname}_man_group")
            entities.add_scenario_object(
                    targetname, xosc.CatalogReference("VehicleCatalog", vehicles[i]["catalog_name"]),
                    xosc.CatalogReference("ControllerCatalog", "NaturalDriver")
                )
            # entities.scenario_objects[1].entityobject.parameterassignments = desired_speed
            noise_pc = random.uniform(0.8, 1.2)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("DesiredSpeed", 70 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("EgoSpeed", 50 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("DesiredDistance", 5 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("AdjRearDistance", 3.0 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("AdjLeadDistance", 5.0 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("MaxAcc", 5.0 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("Route", vehicles[i]["position"][2])
            entities.scenario_objects[i].controller[0].add_parameter_assignment("Politeness", 0.0 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("LaneChangeDelay", 0.0 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("Thw", 0.5 * noise_pc)
            targetstart = xosc.TeleportAction(xosc.LanePosition(*vehicles[i]["position"]))
            targetspeed = xosc.AbsoluteSpeedAction(15, step_time)
            init.add_init_action(targetname, targetstart)
            init.add_init_action(targetname, targetspeed)
            init.add_init_action(targetname, xosc.ActivateControllerAction(lateral=False, longitudinal=True))

            delete_entity_trigger = xosc.EntityTrigger("delete_entity_trigger", 0, xosc.ConditionEdge.none, xosc.EndOfRoadCondition(0), targetname)
            delete_entity_action = xosc.DeleteEntityAction(targetname)
            delete_entity_event = xosc.Event(f"delete_{targetname}_event", xosc.Priority.override)
            delete_entity_event.add_action(f"delete_{targetname}_action", delete_entity_action)
            delete_entity_event.add_trigger(delete_entity_trigger)
            delete_entity_maneuver = xosc.Maneuver(f"delete_{targetname}_maneuver")
            delete_entity_maneuver.add_event(delete_entity_event)

            deactivate_controller_action = xosc.ActivateControllerAction(False, False)
            deactivate_controller_event = xosc.Event(f"deactivate_{targetname}_event", xosc.Priority.override)
            deactivate_controller_event.add_action(f"deactivate_{targetname}_controller_action", deactivate_controller_action)
            deactivate_controller_event.add_trigger(delete_entity_trigger)
            deactivate_controller_maneuver = xosc.Maneuver(f"deactivate_{targetname}_controller")
            deactivate_controller_maneuver.add_event(deactivate_controller_event)

            man_group.add_actor(targetname)
            man_group.add_maneuver(deactivate_controller_maneuver)
            man_group.add_maneuver(delete_entity_maneuver)
            sb.add_maneuver_group(man_group)
                    
        sb.init = init

        ## return the scenario
        return xosc.Scenario(
            "munich_scenario",
            "esmini",
            xosc.ParameterDeclarations(),
            entities=entities,
            storyboard=sb,
            roadnetwork=road,
            catalog=catalog,
        )

if __name__ == "__main__":
    s = Scenario()
    s.generate(".")
