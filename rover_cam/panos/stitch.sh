#!/bin/sh
# $1=pano_dir
# $2=pano_num

# Assembles a Hugin .pto project file using equirectangular projection.
pto_gen -o $1'/pano/panorama.pto' -p 1 -f 60 $1'/input_imgs/img1.jpg' $1'/input_imgs/img2.jpg' $1'/input_imgs/img3.jpg'

# Control point detector for hugin.
cpfind --multirow --celeste -o $1'pano/panorama.pto' $1'pano/panorama.pto'
# Remove all non-credible control points.
cpclean -o $1'pano/panorama.pto' $1'pano/panorama.pto'
# Find vertical lines and assigns vertical control points to them.
linefind -o $1'pano/panorama.pto' $1'pano/panorama.pto'
# Control point optimization.
autooptimiser -a -m -l -s -o $1'pano/panorama.pto' $1'pano/panorama.pto'
# Change some output options of the project file
pano_modify --canvas=AUTO --crop=AUTO -o $1'pano/panorama.pto' $1'pano/panorama.pto'
# Stitching panorama
hugin_executor --stitching --prefix=$1'pano/panorama.pto' $1'pano/panorama.pto'
# Convert .tif to .jpg
convert $1'pano/panorama.tif' $1'pano/panorama_'$2'.jpg'

# Remove unwanted format (.tif and .pto) 
rm $1'pano/panorama.pto'
rm $1'pano/panorama.tif'


