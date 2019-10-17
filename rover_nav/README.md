# Basic setup
### Dependencies
* mapviz
	
### Installing mapviz
https://github.com/swri-robotics/mapviz

### Launching mapviz only
roslaunch mapviz mapviz.launch 

### Launching mapviz + simulated GPS values
roslaunch rover_nav simulation.launch

### Displaying gps coordinated on map
1. Add gps plugin (Add -> gps -> Ok)
2. Select topic "/pos" (First point is at coordinates 45.5,-73.5)
3. Add tile_map plugin (Add -> tile_map -> Ok)

# How to get Google Maps satelite imagery in Mapviz
### Dependencies
* mapviz
* docker

**Note:** An internet connection is needed the first time, however, the tile map is cached so it can then be used offline.

### Installing Docker
Follow this tutorial, starting at "Install using the repository"
https://docs.docker.com/v17.12/install/linux/docker-ce/ubuntu/#uninstall-old-versions

### Setting up and running mapproxy server:
Follow instructions of section 1 only.
https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite

### To do at every boot (<em>TODO: automate this process on startup</em>):
* sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
* Base url for the tile map plugin (if not there already): http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
