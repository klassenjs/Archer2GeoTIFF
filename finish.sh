 export GDAL_CACHEMAX=1280

 for i in *vrt ; do  gdal_translate -co COMPRESS=DEFLATE -co ZLEVEL=2 -co PREDICTOR=1 -co TILED=YES $i `basename $i .vrt`.tif ; done

 for i in *tif ; do  gdaladdo -r average --config COMPRESS_OVERVIEW DEFLATE --config ZLEVEL_OVERVIEW 2 --config PREDICTOR_OVERVIEW 1 $i 2 4 8 16 &  done

