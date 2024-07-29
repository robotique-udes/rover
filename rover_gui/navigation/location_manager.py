class Location:
    def __init__(self, latitude: float, longitude: float, name: str = ""):
        self.latitude = latitude
        self.longitude = longitude
        self.name = name

class LocationManager():
    def __init__(self):
        self.locations = []
        
    def add_location(self, location: Location):
        self.locations.append(location)

    def get_locations(self):
        return self.locations