/*
 *  ArcherRectify.cpp
 *  Archer2GeoTiff
 *
 *  Created by Jim Klassen on 3/21/09.
 *  Copyright (C) 2009 James Klassen
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "ArcherRectify.h"
#include <gdalwarper.h>

ArcherRectify::ArcherRectify(ArcherDataset *ads)
{
	this->archer = ads;

	printf("bbox: [ %lf %lf %lf %lf ] meters_per_px_est: %lf\n",
		ads->GetBBox()->minx, ads->GetBBox()->miny, ads->GetBBox()->maxx, ads->GetBBox()->maxy,
		ads->GetEstResolution());
}

ArcherRectify::~ArcherRectify()
{
	if(this->gdalDestDS)
		GDALClose( this->gdalDestDS );
}

int ArcherRectify::CreateOutputDataset(char* filename, BBox* bbox, double meters_per_pixel, char* projection)
{
	int imgsize_x = ceil((bbox->maxx - bbox->minx) / meters_per_pixel);
	int imgsize_y = ceil((bbox->maxy - bbox->miny) / meters_per_pixel);
	
	printf("Creating output image %d x %d\n", imgsize_x, imgsize_y);

	GDALAllRegister();
	GDALDriverManager *gdal_drivers = GetGDALDriverManager();
	GDALDriver *driver = gdal_drivers->GetDriverByName("GTiff");
	GDALDataset *dst_ds = driver->Create(filename, imgsize_x, imgsize_y, 1, GDT_Byte, (char**) NULL);
	
	/* Set geotransform in new dataset */
	double top_left[2] = { bbox->minx, bbox->maxy };
	double geo_transform[6] = { top_left[0], meters_per_pixel, 0, top_left[1], 0, -meters_per_pixel };
	dst_ds->SetGeoTransform(geo_transform);
	
	/* Set output projection in new dataset */
	OGRSpatialReference srs;
	char *SRS_WKT = NULL;
	srs.importFromProj4(projection);
	srs.exportToWkt( &SRS_WKT );
	dst_ds->SetProjection( SRS_WKT );
	CPLFree( SRS_WKT );

	this->gdalDestDS = dst_ds;
	return TRUE;
}

void ArcherRectify::UseExistingDataset(GDALDataset* ds) 
{
	this->gdalDestDS = ds;
}


/* Use the Geotransform of the output image to calculate where a ground coordinate will fall within the image */
void ground_to_output(double gt[6], double ground_x, double ground_y, double *img_x, double *img_y) 
{
		double invdet = gt[1]*gt[5] - gt[2]*gt[4];
		double xx     = ground_x - gt[0];
		double yy     = ground_y - gt[3];
			
		*img_x = (gt[5] * xx - gt[2] * yy) / invdet;
		*img_y = (-gt[4] * xx + gt[1] * yy) / invdet;
}



/* GDAL Transformer Function for Archer Dataset */
/* Parameters:
 *   	pTransformerArg 	application supplied callback data used by the transformer.
 *   	bDstToSrc 	if TRUE the transformation will be from the destination coordinate space to the source coordinate system, otherwise the transformation will be from the source coordinate system to the destination coordinate system.
 *   	nPointCount 	number of points in the x, y and z arrays.
 *   	x 	input X coordinates. Results returned in same array.
 *   	y 	input Y coordinates. Results returned in same array.
 *   	z 	input Z coordinates. Results returned in same array.
 *   	panSuccess 	array of ints in which success (TRUE) or failure (FALSE) flags are returned for the translation of each point.
 *
 * Returns:
 *   TRUE if the overall transformation succeeds (though some individual points may have failed) or FALSE if the overall transformation fails.  
 */
 
 typedef struct {
	GDALDataset* hSrcDS;
	GDALDataset* hDstDS;
	ArcherDataset *archer;
 } AITParam;
 
int ArcherImageTransformer(void *pTransformerArg, 
							int bDstToSrc, int nPointCount, 
							double *x, double *y, double *z, int *panSuccess)
{
	AITParam *ait = (AITParam*)pTransformerArg;
	ArcherDataset *archer = ait->archer;

	IPoint src_image_point;
	Point  ground;
	
	if(archer == NULL)
		return FALSE;
	
	double gt[6];
	ait->hDstDS->GetGeoTransform(gt);
	if(bDstToSrc) /* Transform from dest to src */
	{
		
		for(int i = 0; i < nPointCount; i++) {
			/* Transform from destination px to ground */
			printf("%d: %lf %lf %lf <- ", i, x[i], y[i], z[i]);
			double xx = x[i];
			double yy = y[i];
			
			x[i] = gt[0] + xx*gt[1] + yy*gt[2];
			y[i] = gt[3] + xx*gt[4] + yy*gt[5];
		
			/* Transform from ground to source px */
			/* ??? */
			panSuccess[i] = TRUE;

			printf("%lf %lf %lf\n", x[i], y[i], z[i]);
		}
	} 
	else
	{
		for(int i = 0 ; i < nPointCount; i++) {
			/* Transform from source px to ground */
			printf("%d: %lf %lf %lf -> ", i, x[i], y[i], z[i]);
			src_image_point.x = floor(x[i]);
			src_image_point.y = floor(y[i]);
			
			ground = archer->ImageToGround(src_image_point);
	
			/* Transform from ground to dest px */
			double invdet = gt[1]*gt[5] - gt[2]*gt[4];
			double xx     = ground.x - gt[0];
			double yy     = ground.y - gt[3];
				
			x[i] = (gt[5] * xx - gt[2] * yy) / invdet;
			y[i] = (-gt[4] * xx + gt[1] * yy) / invdet;
			
			panSuccess[i] = TRUE;
			printf("%lf %lf %lf\n", x[i], y[i], z[i]);
		}
	}	
	return TRUE;
}

int ArcherRectify::run()
{
	GDALWarpOptions *psWarpOptions = GDALCreateWarpOptions();
	GDALDataset *hSrcDS = this->archer->GetGDALDataset();
	GDALDataset *hDstDS = this->gdalDestDS;
	
	/* Source and destination datasets */
	psWarpOptions->hSrcDS = hSrcDS;
	psWarpOptions->hDstDS = hDstDS;
	
	psWarpOptions->nBandCount = 0; /* 0 selects all image bands */
	psWarpOptions->pfnProgress = GDALTermProgress;
	
	AITParam param;
	param.hSrcDS = hSrcDS;
	param.hDstDS = hDstDS;
	param.archer = this->archer;
	
	psWarpOptions->pTransformerArg = (void*)&param;
	psWarpOptions->pfnTransformer = ArcherImageTransformer;
	
	GDALWarpOperation oOperation;
	oOperation.Initialize( psWarpOptions );
	oOperation.ChunkAndWarpImage( 0, 0,
									GDALGetRasterXSize( hDstDS ),
									GDALGetRasterYSize( hDstDS ) );
//	oOperation.WarpRegion( 0, 0,
//									GDALGetRasterXSize( hDstDS ),
//									GDALGetRasterYSize( hDstDS ),
//									0, 0,
//									GDALGetRasterXSize( hSrcDS ),
//									GDALGetRasterYSize( hSrcDS ) );
									
	GDALDestroyWarpOptions( psWarpOptions );

	return 0;
}
