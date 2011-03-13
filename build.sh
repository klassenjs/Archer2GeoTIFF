#!/bin/bash

## Build script for Linux

#g++ main.cpp ArcherDataset.cpp ArcherRectify.cpp -lgdal1.5.0 -I/usr/include/gdal -O3 -o Archer2GeoTIFF

## Build script for Mac

g++ main.cpp ArcherDataset.cpp ArcherRectify.cpp -lgdal -lproj -I/usr/local/osgeo/include -L/usr/local/osgeo/lib -O3 -o Archer2GeoTIFF

