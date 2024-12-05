import random
import xml.etree.ElementTree as ET
import os
from scenariogeneration import xosc
from scenariogeneration import ScenarioGenerator


class Scenario(ScenarioGenerator):
    def __init__(self):
        ScenarioGenerator.__init__(self)

    def get_lanesection_ids(self, lane_element):
        lane_ids = []
        for side in ["left", "right"]:
            element = lane_element.find(f"laneSection/{side}")
            if element is not None:
                for lane in element.findall("lane"):
                    if lane.get("type") in ["driving", "offRamp", "onRamp"]:
                        lane_ids.append(int(lane.get("id")))


        return lane_ids
     
    def parse_road(self, roadfile: str) -> dict:
        ret_dict = { "total_road_length" : 0.0 ,
                     "drivable_lanes_length" : 0.0 }

        tree = ET.parse(roadfile)
        root = tree.getroot()
        road_elem = root.findall('road')
        if road_elem is None:
            print("No road in file")
            return
        
        for element in road_elem:
            road_id = int(element.get("id"))
            if int(element.get("junction")) == 1:
                   continue # Skip junctions
            lanes = self.get_lanesection_ids(element.find("lanes"))
            ret_dict[road_id] = {
                "length":  float(element.get("length")),
                "lane_ids": lanes
            }
            ret_dict["total_road_length"] += ret_dict[road_id]["length"]
            ret_dict["drivable_lanes_length"] += ret_dict[road_id]["length"] * len(lanes)

        return ret_dict

    def get_vehicle_types(self) -> list:
        file_name = __file__.split("/")[-1]
        file_path = __file__.split(file_name)[0]
        
        # Parse the XML file using ElementTree
        xml_file = os.path.join(file_path, "..", "resources", "xosc", "Catalogs", "Vehicles", "VehicleCatalog.xosc")
        tree = ET.parse(xml_file)
        root = tree.getroot()

        # Find all Vehicle elements
        vehicles = root.findall(".//Vehicle")

        vehicle_data = []
        for vehicle in vehicles:
            name = vehicle.get("name")
            veh_type = vehicle.get("vehicleCategory") 
            # Find the length in the BoundingBox/Dimensions node
            dimensions = vehicle.find(".//BoundingBox/Dimensions")
            if veh_type == "car" and "trailer" not in name and dimensions is not None:
                length = dimensions.get("length")
                if name == "car_blue":
                    length = "4.5"
                if length is not None and length.replace('.', '', 1).isdigit():
                    vehicle_data.append({"name": name, "length": float(length)})
                else:
                    print(f"Skipping vehicle {name} due to invalid length: {length}")

        return vehicle_data

    def scenario(self, **kwargs):
        # We point to the road we wish to use
        roadfile = "/home/slundel6/git-repos/esmini/resources/xodr/Hello_world_overtake0.xodr"
        # roadfile = "/home/slundel6/git-repos/esmini/resources/xodr/Munich8_fixed.xodr"
        road = xosc.RoadNetwork(roadfile=roadfile)
        road_dict = self.parse_road(roadfile)
        
        egoname = "Ego"

        # We shall define a catalog to find our vehicles in
        catalog = xosc.Catalog()
        catalog.add_catalog("VehicleCatalog", "../xosc/Catalogs/Vehicles")
        catalog.add_catalog("ControllerCatalog", "../xosc/Catalogs/Controllers")

        vehicles = self.get_vehicle_types()

        ## create entities based on entries in the catalog and give them names
        entities = xosc.Entities()
        entities.add_scenario_object(
            egoname, xosc.CatalogReference("VehicleCatalog", "car_white")
        )
        
        init = xosc.Init()
        step_time = xosc.TransitionDynamics(
            xosc.DynamicsShapes.step, xosc.DynamicsDimension.rate, 1
        )
        
        # create init (0 starting speed)

        # Define the start conditions for the entities (they need speed and position)
        ego_s = 400
        ego_t = 0
        ego_lid = -2
        ego_rid = 1
        egostart = xosc.TeleportAction(xosc.LanePosition(ego_s, ego_t, ego_lid, ego_rid))
        egospeed = xosc.AbsoluteSpeedAction(15, step_time)

        # Add the start conditions to the init stage for each entity
        init.add_init_action(egoname, egostart)
        init.add_init_action(egoname, egospeed)
        
        t_id = 0
        car_factor = 2 # cars/100m
        car_density = int(100/car_factor)
        for road_id in list(road_dict)[2:]:
            section_length = road_dict[road_id]["length"]
            # import pdb
            # pdb.set_trace()
            for lane_id in road_dict[road_id]["lane_ids"]:
                min_sample = 0
                for s in range(0, int(section_length), car_density):
                    target_type = vehicles[int(random.uniform(0, len(vehicles)-1))]
                    target_name = target_type["name"]
                    target_length = target_type["length"]
                    min_sample = s + target_length # add a car length to avoid on top of eachother

                    if car_density > section_length:
                        continue # No cars on roads shorter than density

                    s_noise = random.uniform(min_sample, min_sample + car_density - target_length)

                    targetname = f"Target_{t_id}"
                    entities.add_scenario_object(
                        targetname, xosc.CatalogReference("VehicleCatalog", target_name),
                        xosc.CatalogReference("ControllerCatalog", "NaturalDriver")
                    )
                    if (ego_s - target_length < s_noise < ego_s + target_length) and lane_id == ego_lid and road_id == ego_rid:
                        continue # Don't place a target on top of ego

                    if s_noise > section_length - target_length or s_noise < target_length: # Max noise
                        continue

                    targetstart = xosc.TeleportAction(xosc.LanePosition(s_noise, 0, lane_id, road_id))
                    activatecontroller = xosc.ActivateControllerAction(False, True)
                    targetspeed = xosc.AbsoluteSpeedAction(15, step_time)
                    init.add_init_action(targetname, targetstart)
                    init.add_init_action(targetname, targetspeed)
                    init.add_init_action(targetname, activatecontroller)
                    t_id += 1
                    
        
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
    foldername = "/home/slundel6/git-repos/esmini/resources/"
    files = sce.generate(foldername)

    # uncomment the following lines to display the scenario using esmini
    from scenariogeneration import esmini

    esmini(sce, "/home/slundel6/git-repos/esmini", generation_path=foldername, window_size= "600 600 800 600")
