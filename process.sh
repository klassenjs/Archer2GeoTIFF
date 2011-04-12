#!/bin/bash

##################
#### Settings ####
##################

### Maximum number of processes to run at once (=number of image strips processed at once)
MAX_PAN=10  # Set to use all CPU cores, but don't cause swapping
MAX_HSI=8   # Set to use all CPU cores, but don't cause swapping

### Output resolution of GeoTIFFs, higher needs more memory per process.
PAN_RES=.3048  # Pan-chromatic output resolution (meters/px)
HSI_RES=1.0    # Hyper-spectral output resoluton (meters/px)

### Projection Settings
# Grand Forks
#AVERAGE_HEIGHT=259.0 # meters above mean sea level (WGS84) 
#OUTPUT_PROJECTION="+init=epsg:26914" # Output projection (proj4 syntax)

# St. Paul
AVERAGE_HEIGHT=213.0 # meters above mean sea level (WGS84)
OUTPUT_PROJECTION="+init=epsg:26915" # Output projection (proj4 syntax)

### Path to Archer2GeoTIFF binary
Archer2GeoTIFF="Archer2GeoTIFF"

################################################################
#### Main Script - Shouldn't need to change below this line ####
################################################################

SOURCE=$1
DEST=$2

RUNNING=0

# Process Archer -> GeoTIFF
for i in "$SOURCE"/*pan ; do 
	$Archer2GeoTIFF -r $PAN_RES -e $AVERAGE_HEIGHT -p $OUTPUT_PROJECTION -i $i -o "$DEST"/`basename $i.tif` &
	RUNNING=`expr 1 + $RUNNING`
	if([ $RUNNING -ge $MAX_PAN ]); then
		wait
		RUNNING=0
	fi
done
for i in "$SOURCE"/*hsi ; do 
	$Archer2GeoTIFF -r $HSI_RES -e $AVERAGE_HEIGHT -p $OUTPUT_PROJECTION -i $i -o "$DEST"/`basename $i.tif` &
	RUNNING=`expr 1 + $RUNNING`
	if([ $RUNNING -ge $MAX_HSI ]); then
		wait
		RUNNING=0
	fi
done 
wait
