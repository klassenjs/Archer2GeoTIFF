#!/bin/bash

#### Settings
PAN_RES=.3048  # Pan-chromatic output resolution (meters/px)
HSI_RES=1.0    # Hyper-spectral output resoluton (meters/px)

AVERAGE_HEIGHT=259.0 # meters above mean sea level (WGS84)
OUTPUT_PROJECTION="+init=epsg:26914" # Output projection (proj4 syntax)
Archer2GeoTIFF="/home/archer/Archer/Archer2GeoTIFF/Archer2GeoTIFF"
Archer2GeoTIFF="/Users/jimk/Archer/Archer2GeoTIFF/build/release/Archer2GeoTIFF"
Archer2GeoTIFF="Archer2GeoTIFF"

SOURCE=$1
DEST=$2

# Process Archer -> GeoTIFF
for i in "$SOURCE"/*pan ; do 
	$Archer2GeoTIFF -r $PAN_RES -e $AVERAGE_HEIGHT -p $OUTPUT_PROJECTION -i $i -o "$DEST"/`basename $i.tif`
done
for i in "$SOURCE"/*hsi ; do 
	$Archer2GeoTIFF -r $HSI_RES -e $AVERAGE_HEIGHT -p $OUTPUT_PROJECTION -i $i -o "$DEST"/`basename $i.tif`
done

# Create compressed GeoTIFFs
mkdir -p "$DEST/compressed"
for i in "$DEST"/*pan.tif ; do 
	gdal_translate -of GTiff -co COMPRESS=JPEG -co TILED=YES $i "$DEST"/compressed/`basename $i`
done
for i in "$DEST"/*hsi.tif ; do 
	gdal_translate -of GTiff -co COMPRESS=LZW  $i $DEST/compressed/`basename $i`
done


