from ament_index_python.packages import get_package_share_directory

class Location:
    def __init__(self, latitude: float, longitude: float, name: str = ""):
        self.latitude = latitude
        self.longitude = longitude
        self.name = name

class LocationManager():
    def __init__(self):
        self.locations = []

        package_share_directory = get_package_share_directory("rover_gui")
        self.saved_locations_path = package_share_directory + "/../../../../src/rover/rover_gui/saved_files/saved_locations.txt"
        
    def get_locations(self):
        return self.locations
    
    def add_location(self, location: Location):
        self.locations.append(location)
    
    def delete_location(self, index):
        self.locations.pop(index)
        
    def load_locations(self):
        try:
            with open(self.saved_locations_path, "r") as f:
                for line in f:
                    parts = line.strip().split(";")
                    if len(parts) == 3:
                        name, latitude, longitude = parts

                        location = Location(float(latitude), float(longitude), name)
                        self.locations.append(location)

        except FileNotFoundError:
            with open(self.saved_locations_path, "w") as f:
                pass
