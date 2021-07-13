#!/bin/sh

ffmpeg -framerate 8 -i ./results/images/res_%03d.png -vf "crop=trunc(iw/2)*2:trunc(ih/2)*2" -c:v libx264 -r 30 -pix_fmt yuv420p $1
