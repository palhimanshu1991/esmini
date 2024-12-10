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
        
        egoname = "Ego"
        targetname = "Target"

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
        for i in range(1, 2):
            entities.add_scenario_object(
                    f"{targetname}{i}", xosc.CatalogReference("VehicleCatalog", vehicles[i]["catalog_name"]),
                    xosc.CatalogReference("ControllerCatalog", "NaturalDriver")
                )
            # entities.scenario_objects[1].entityobject.parameterassignments = desired_speed
            noise_pc = random.uniform(0.9, 1.1)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("DesiredSpeed", 70 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("EgoSpeed", 50 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("DesiredDistance", 5 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("AdjRearDistance", 3.0 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("AdjLeadDistance", 20.0 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("MaxAcc", 4.0 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("Politeness", 0.1 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("LaneChangeDelay", 5.0 * noise_pc)
            entities.scenario_objects[i].controller[0].add_parameter_assignment("Thw", 1.5 * noise_pc)
            targetstart = xosc.TeleportAction(xosc.LanePosition(*vehicles[i]["position"]))
            targetspeed = xosc.AbsoluteSpeedAction(15, step_time)
            init.add_init_action(f"{targetname}{i}", targetstart)
            init.add_init_action(f"{targetname}{i}", targetspeed)
            init.add_init_action(f"{targetname}{i}", xosc.ActivateControllerAction(lateral=False, longitudinal=True))
                    
        
        ## create the storyboard, requires init and when to stop (Trigger), and add the story to it
        sb = xosc.StoryBoard(
            init,
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
        # sb.add_maneuver(target_takeoff_maneuver, targetname)

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
    sce = Scenario()
    foldername = "tmp"
    files = sce.generate(foldername)

    # uncomment the following lines to display the scenario using esmini
    from scenariogeneration import esmini
    esmini(sce, "../../", generation_path=foldername,resource_path="../../resources/xosc",args="--path ../../models/", window_size= "600 600 800 600")
