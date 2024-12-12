from scenariogeneration import xosc, xodr
from scenariogeneration import ScenarioGenerator
from generate_traffic import get_vehicle_positions
import random
import os

class RoadGen(ScenarioGenerator):
    def __init__(self):
        ScenarioGenerator.__init__(self)

    def road(self, **kwargs):
        road = xodr.create_road([xodr.Line(500)], id=0, left_lanes=2, right_lanes=2, lane_width=3.5)

        odr = xodr.OpenDrive("myroad")
        odr.add_road(road)
        odr.adjust_roads_and_lanes()
        return odr

class Scenario(ScenarioGenerator):
    def __init__(self, road_generated):
        ScenarioGenerator.__init__(self)
        self.road_generated = road_generated

    def scenario(self, **kwargs):
        res_rel_path = os.path.join("..", "..", "resources")

        # We point to the road we wish to use
        if self.road_generated:
            roadfile_name = __file__.split(os.path.sep)[-1].split(".py")[0]
            road = xosc.RoadNetwork(roadfile=f"../xodr/{roadfile_name}0.xodr")
        else:
            road = xosc.RoadNetwork(roadfile="../xodr/e6mini.xodr")

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
        vehicle_catalog_path = "../xosc/Catalogs/Vehicles"
        controller_catalog_path = "../xosc/Catalogs/Controllers"
        catalog.add_catalog("VehicleCatalog", vehicle_catalog_path)
        catalog.add_catalog("ControllerCatalog", controller_catalog_path)


        ## create entities based on entries in the catalog and give them names
        entities = xosc.Entities()
        entities.add_scenario_object(
            egoname, xosc.CatalogReference("VehicleCatalog", "car_white")
        )
        
        init = xosc.Init()
        step_time = xosc.TransitionDynamics(
            xosc.DynamicsShapes.step, xosc.DynamicsDimension.rate, 1
        )
        
        ego_s = 200
        ego_t = 0
        ego_lid = -2
        ego_rid = 0

        # A tuple to feed the get_vehicle_position later
        ego_start = (ego_s, ego_t, ego_lid, ego_rid)
        egostart = xosc.TeleportAction(xosc.LanePosition(*ego_start))
        egospeed = xosc.AbsoluteSpeedAction(15, step_time)

        # Add the start conditions to the init stage for ego only
        init.add_init_action(egoname, egostart)
        init.add_init_action(egoname, egospeed)
        init.add_init_action(egoname, xosc.ActivateControllerAction(lateral=False, longitudinal=True))
        
        vehicles = get_vehicle_positions(roadfile=os.path.join(res_rel_path, "xodr", road.road_file),
                                         ego_pos=ego_start,
                                         density=1.0,
                                         catalog_path=os.path.join(res_rel_path, "xosc", vehicle_catalog_path, "VehicleCatalog.xosc"))

        # Iterate over all vehicle positions, and create targets with suitable parameters and positions
        for i in range(1, len(vehicles)):
            targetname = f"Target{i}"
            target_set_speed = 20 # m/s
            man_group = xosc.ManeuverGroup(f"{targetname}_man_group")
            entities.add_scenario_object(
                    targetname, xosc.CatalogReference("VehicleCatalog", vehicles[i]["catalog_name"]),
                    xosc.CatalogReference("ControllerCatalog", "NaturalDriver")
                )
            noise_pc = random.uniform(0.8, 1.2)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("DesiredSpeed", target_set_speed * 3.6 * noise_pc) # Takes kph
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
            targetspeed = xosc.AbsoluteSpeedAction(target_set_speed, step_time)
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
    road_generated = False

    # Uncomment lines below to generate traffic on scenario-generation road. Also change the roadfile in xosc.RoadNetwork
    # road = RoadGen()
    # road.generate("..")
    # road_generated = True

    s = Scenario(road_generated)
    s.generate("../../resources/")
