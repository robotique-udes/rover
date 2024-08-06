from ament_index_python.packages import get_package_share_directory
import json

class Location:
    def __init__(self, latitude: float, longitude: float):
        self.latitude = latitude
        self.longitude = longitude

class Route():
    def __init__(self):
        self.waypoints = []

    def add_waypoint(self, waypoint: Location):
        self.waypoints.append(waypoint)

class RouteManager():
    def __init__(self, ui_node):
        self.ui_node = ui_node
        self.routes = []
        self.current_route = None

        package_share_directory = get_package_share_directory("rover_gui")
        self.saved_locations_path = package_share_directory + "/../../../../src/rover/rover_gui/saved_files/saved_routes.json"
        
    def get_locations(self):
        return self.locations
    
    def add_location_to_route(self, location: Location):
        self.current_route.add_waypoint(location)
    
    def delete_location(self, index):
        self.current_route.waypoints.pop(index)

    def set_current_route(self, route_name: str):
        for route in self.routes:
            if route.name == route_name:
                self.current_route = route
                return True
        self.ui_node.get_logger().info(f"Route '{route_name}' not found.")
        return False
        
    def load_routes(self):
        try:
            with open(self.saved_locations_path, "r") as f:
                data = json.load(f)
                for route_data in data["routes"]:
                    route = Route()
                    route.name = route_data["name"]
                    for waypoint_data in route_data["waypoints"]:
                        location = Location(
                            latitude = float(waypoint_data["latitude"]),
                            longitude = float(waypoint_data["longitude"])
                        )
                        route.add_waypoint(location)
                    self.routes.append(route)

        except FileNotFoundError:
            with open(self.saved_locations_path, "w") as f:
                json.dump({"routes": []}, f)
        except json.JSONDecodeError:
            self.ui_node.get_logger().info("Error decoding JSON from file. The file may be empty or contain invalid JSON.")

    def save_routes(self):
        data = {"routes": []}
        for route in self.routes:
            route_data = {
                "name": route.name,
                "waypoints": [
                    {
                        "latitude": waypoint.latitude,
                        "longitude": waypoint.longitude
                    } for waypoint in route.waypoints
                ]
            }
            data["routes"].append(route_data)

        try:
            with open(self.saved_locations_path, "w") as f:
                json.dump(data, f, indent=2)
            self.ui_node.get_logger().info("Routes saved successfully.")
        except IOError:
            self.ui_node.get_logger().info("Error saving routes to file.")

    def __del__(self):
        self.save_routes()