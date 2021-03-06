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
		delete this->gdalDestDS;
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

	// TODO: Make photo follow average plane heading to minimize unused pixels .. requires rotate bbox
	//double phi = -this->archer->GetINSData()[0].heading;
	//double geo_transform[6] = { top_left[0], meters_per_pixel * cos(phi), meters_per_pixel * sin(phi), top_left[1], meters_per_pixel * sin(phi), -meters_per_pixel * cos(phi) };

	// Make photo with North = UP;
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


/* Helper functions for ::run */
typedef float outImageType;
#define IMAGE_GDAL_TYPE GDT_Float32

inline
void set_px(outImageType* img, short *counts, int dst_x, int dst_y, int x, int y, outImageType value) 
{
		//printf("Set: %d, %d = %f\n", x, y, value);
		if(x >= 0 && x < dst_x && y >= 0 && y < dst_y) {
			img[x+(y*dst_x)] += value;
			counts[x+(y*dst_x)]++;
		}
}

inline
float gaussian(float x, float y)
{
	return expf(-sqrtf(x*x + y*y));
	
 	//if(r == 0) return 1;
	//r = r * 3.14159;
	//return sinf(r)*sinf(r)/(r*r);
	
	//return 2 - r;
}

int ArcherRectify::run()
{
	
	GDALAllRegister();
	outImageType* dst_image;
	short* counts;
	double dst_gt[6];

	GDALDataset *src_ds = this->archer->GetGDALDataset();
	GDALDataset *dst_ds = this->gdalDestDS;

	dst_ds->GetGeoTransform(dst_gt);
	
	/* Need to do this for each band */
	/* If we have enough mem do multiple simultaniously so can reuse calculations */
	GDALRasterBand *src_image = src_ds->GetRasterBand( 1 );

	/* Get source and destination image sizes */
	const int src_XSize = src_image->GetXSize();
	const int src_YSize = src_image->GetYSize();
	const int dst_XSize = dst_ds->GetRasterXSize();
	const int dst_YSize = dst_ds->GetRasterYSize();

	double *dst_x = (double*)malloc(sizeof(double) * src_XSize * 1024);
	double *dst_y = (double*)malloc(sizeof(double) * src_XSize * 1024);

	/* Create input scanline buffer */
	outImageType *scanline;
	scanline = (outImageType*) malloc(sizeof(outImageType) * src_XSize);

	/* Create output buffer */
	size_t memory_req = sizeof(outImageType) * dst_XSize * dst_YSize;
	printf("Mallocing %.0f MB for output buffer\n", (float)memory_req / (1024.0 * 1024.0));
	dst_image = (outImageType*)malloc(memory_req);
	bzero(dst_image, memory_req);
	
	counts = (short*)malloc(sizeof(short) * dst_XSize * dst_YSize);
	bzero(counts, sizeof(short) * dst_XSize * dst_YSize);
	
	const int progress_interval = src_YSize / 100;
	int progress = progress_interval;
	
	/* Setup src px -> ground */
	ArcherINSRow *ins = this->archer->GetINSData();
	const int number_of_px   = src_XSize; /*const*/
	const double radians_per_px = (this->archer->ArcherENVIHdr.FOV * DEG_TO_RAD) / number_of_px; /*const*/

	/* For each pixel in the source, place it in the destination */
	int Ys = 0;
	while(Ys < src_YSize) {
		for(int Yi = 0; Yi < 1024 && (Yi+Ys) < src_YSize; Yi++) {
			for(int X = 0; X < src_XSize; X++) {
				int Y = Yi + Ys;
	//			IPoint src_px;
				Point ground;

	//			src_px.x = X;
	//			src_px.y = Y;

				/* Src px to ground */
	//			ground = this->archer->ImageToGround(src_px);
				{ /* image to ground inlined */
					// Note '-' sign it appears the sensor is 'backwards', i.e. 0 is right, 6144 is left
					double lens_angle = -radians_per_px * float(X - (number_of_px / 2.0)); /* x */
					double elevation  =  ins[Y].alt - 213.36;   // TODO: Should get from elevation model /* x y */

					// Calculate x,y on ground relative to center of plane along direction of plane
					double rel_plane_x = sin(lens_angle + ins[Y].roll) * elevation;
					double rel_plane_y = sin(ins[Y].pitch) * elevation;

					// Now rotate plane rel coordinates by Yaw + heading into plane rel north oriented
					double phi = ins[Y].heading + ins[Y].yaw;

					double rel_ground_x =  ( rel_plane_x * cos(phi)) + (rel_plane_y * sin(phi));
					double rel_ground_y =  (-rel_plane_x * sin(phi)) + (rel_plane_y * cos(phi));

					// Convert from plane relative to ground coordinate system by adding plane centerpoint offset
					ground.x = rel_ground_x + ins[Y].x;
					ground.y = rel_ground_y + ins[Y].y;
					ground.z = 0;
				}
				
				/* Ground to dest_px */
				double invdet = dst_gt[1]*dst_gt[5] - dst_gt[2]*dst_gt[4];
				double xx     = ground.x - dst_gt[0];
				double yy     = ground.y - dst_gt[3];
					
				dst_x[X+(Yi*src_XSize)] = (dst_gt[5] * xx - dst_gt[2] * yy) / invdet;
				dst_y[X+(Yi*src_XSize)] = (-dst_gt[4] * xx + dst_gt[1] * yy) / invdet;
			}
		}
		for(int Yi = 0; Yi < 1024 && (Yi+Ys) < src_YSize; Yi++) {
			int Y = Yi + Ys;
			progress--;
			if(progress <= 0) {
				printf("%.0f %%\n", 100.0 * (float)Y / (float)src_YSize);
				progress = progress_interval;
			}
			src_image->RasterIO( GF_Read, 0, Y, src_XSize, 1, scanline, src_XSize, 1, IMAGE_GDAL_TYPE, 0, 0);
			for(int X = 0; X < src_XSize; X++) {
				int dst_xi, dst_yi;

				/* Perform Gaussian Kernel */
				for(dst_yi = floor(dst_y[X+(Yi*src_XSize)]) - 1; dst_yi < floor(dst_y[X+(Yi*src_XSize)]) + 2; dst_yi++)
					for(dst_xi = floor(dst_x[X+(Yi*src_XSize)]) - 1; dst_xi < floor(dst_x[X+(Yi*src_XSize)]) + 2; dst_xi++) {
	//					set_px(dst_image, counts, dst_XSize, dst_YSize, dst_xi, dst_yi, scanline[X] * gaussian(dst_x - dst_xi, dst_y - dst_yi));
						{ /* set_px inlined */
							if(dst_xi >= 0 && dst_xi < dst_XSize && dst_yi >= 0 && dst_yi < dst_YSize) {
									int offset = dst_xi+(dst_yi*dst_XSize);
									//dst_image[offset] += scanline[X] * expf(-sqrtf((dst_x-dst_xi)*(dst_x-dst_xi) + (dst_y-dst_yi)*(dst_y-dst_yi)));
									dst_image[offset] += scanline[X] * (1.0-abs(dst_x[X+(Yi*src_XSize)]-dst_xi)) * (1.0-abs(dst_y[X+(Yi*src_XSize)]-dst_yi));
									counts[offset]++;
							}
						}
					}
			}
		}
		Ys += 1024;
	}	
	
	
	/* Scale pixels */
	for(int i = 0; i < dst_XSize * dst_YSize; i++) {
		if(counts[i] > 0)
			dst_image[i] = dst_image[i] / (float)counts[i];
	}
		
	/* Rescale output buffer to 0-256 */
	outImageType min = dst_image[0];
	outImageType max = dst_image[0];
	for(int i = 0; i < dst_XSize * dst_YSize; i++) {
		if(dst_image[i] < min) min = dst_image[i];
		if(dst_image[i] > max) max = dst_image[i];
	}
	float scale = 256 / (max - min);
	for(int i = 0; i < dst_XSize * dst_YSize; i++) {
		dst_image[i] = (dst_image[i] - min) * scale;
	}
	
	/* Write destination buffer to file */
	GDALRasterBand *dst_band = dst_ds->GetRasterBand( 1 );
	dst_band->SetNoDataValue(0.0);
	dst_band->RasterIO( GF_Write, 0, 0, dst_XSize, dst_YSize, dst_image, dst_XSize, dst_YSize, IMAGE_GDAL_TYPE, 0, 0);
	
	/* Build pyrimids */
	int anOverviewList[3] = { 2, 4, 8 };
	dst_ds->BuildOverviews("NEAREST", 3, anOverviewList, 0, NULL, GDALDummyProgress, NULL);
	
	/* Cleanup */
	free(scanline);
	free(dst_image);
	
	return 0;
}
