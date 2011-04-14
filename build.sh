#!/bin/bash


TYPE=`uname -s`

if [ "${TYPE}" = "Darwin" ]; then

	## Build script for Mac
	g++ main.cpp ArcherDataset.cpp ArcherRectify.cpp \
		-lgdal -lproj \
		-I/usr/local/osgeo/include \
		-L/usr/local/osgeo/lib \
		-O3 -o Archer2GeoTIFF
else
	## Build script for Linux
	g++ main.cpp ArcherDataset.cpp ArcherRectify.cpp \
		-lgdal1.6.0 \
		-lproj \
		-I/usr/include/gdal \
		-O3 -o Archer2GeoTIFF
	#g++ main.cpp ArcherDataset.cpp ArcherRectify.cpp -Wl,-R/home/jimk/apps/lib -L/home/jimk/apps/lib -lgdal -lproj -lopenjpeg -I/home/jimk/apps/include -O3 -o Archer2GeoTIFF

fi

