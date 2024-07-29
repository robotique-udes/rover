from navigation.location_manager import Location

class Route(Location):
    def __init__(self):
        self.waypoints = []

    def add_waypoint(self, waypoint: Location):
        self.waypoints.append(waypoint)

class RouteManager():
    def __init__(self):
        self.routes = []
        
    def add_location(self, route: Route):
        self.locations.append(route)

    def get_locations(self):
        return self.locations
