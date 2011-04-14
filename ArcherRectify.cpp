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

int ArcherRectify::CreateOutputDataset(char* filename, BBox* bbox, double meters_per_pixel, int nBands, char* projection)
{

	double phi = bbox->phi;
	int imgsize_x = ceil(( cos(phi) * (bbox->maxx - bbox->minx) + sin(phi) * (bbox->maxy - bbox->miny))  / meters_per_pixel);
	int imgsize_y = ceil((-sin(phi) * (bbox->maxx - bbox->minx) + cos(phi) * (bbox->maxy - bbox->miny)) / meters_per_pixel); 
	char* options[] = {
		"INTERLEAVE=BAND",
		"TILED=YES",
		"COMPRESS=DEFLATE",
		"ZLEVEL=3",
		"PREDICTOR=1",	
		NULL
	}; 

	printf("Creating output image %d x %d at %lf m/px rotated %lf degrees\n", imgsize_x, imgsize_y, meters_per_pixel, phi*RAD_TO_DEG);

	GDALAllRegister();
	GDALDriverManager *gdal_drivers = GetGDALDriverManager();
	GDALDriver *driver = gdal_drivers->GetDriverByName("GTiff");
	GDALDataset *dst_ds = driver->Create(filename, imgsize_x, imgsize_y, nBands, GDT_Byte, options);
	
	/* Set geotransform in new dataset */
	double minx = 0;
	//double miny = 0;
	//double maxx =  (bbox->maxx - bbox->minx) * cos(phi) + (bbox->maxy - bbox->miny) * sin(phi);
	double maxy = -(bbox->maxx - bbox->minx) * sin(phi) + (bbox->maxy - bbox->miny) * cos(phi);
	
	double tl_x =  minx * cos(-phi) + maxy * sin(-phi) + bbox->minx;
	double tl_y = -minx * sin(-phi) + maxy * cos(-phi) + bbox->miny;
	
	// TODO: Make photo follow average plane heading to minimize unused pixels .. requires rotate bbox
	double geo_transform[6] = { tl_x, meters_per_pixel * cos(phi), meters_per_pixel * sin(phi), tl_y, meters_per_pixel * sin(phi), -meters_per_pixel * cos(phi) };

	// Make photo with North = UP;
	//double geo_transform[6] = { top_left[0], meters_per_pixel, 0, top_left[1], 0, -meters_per_pixel };
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
	float* counts;
	double dst_gt[6];

	GDALDataset *src_ds = this->archer->GetGDALDataset();
	GDALDataset *dst_ds = this->gdalDestDS;

	dst_ds->GetGeoTransform(dst_gt);
	
	/* Get source and destination image sizes */
	const int src_XSize = src_ds->GetRasterXSize();
	const int src_YSize = src_ds->GetRasterYSize();
	const int dst_XSize = dst_ds->GetRasterXSize();
	const int dst_YSize = dst_ds->GetRasterYSize();
	const int band_count = src_ds->GetRasterCount();

	int start_band, end_band;
	int bands = (band_count>13)? 13 : band_count;

	/* Create input scanline buffer */
	outImageType *scanline;
	scanline = (outImageType*) malloc(sizeof(outImageType) * src_XSize * bands);

	/* Create output buffer */
	size_t memory_req = sizeof(outImageType) * dst_XSize * dst_YSize * bands;
	printf("Mallocing %.0f MB for output buffer\n", (float)memory_req / (1024.0 * 1024.0));
	dst_image = (outImageType*)malloc(memory_req);
	counts = (float*)malloc(sizeof(float) * dst_XSize * dst_YSize * bands);
	
	const int progress_interval = 4 * src_YSize / 100;
	int progress = progress_interval;
	
	/* Setup src px -> ground */
	ArcherINSRow *ins = this->archer->GetINSData();
	float ground_elevation = this->archer->fAverageGroundElevation;
	const int number_of_px   = src_XSize; /*const*/
	const double radians_per_px = (this->archer->ArcherENVIHdr.FOV * DEG_TO_RAD) / number_of_px; /*const*/

    /* Handle bands in blocks of 10 to limit memory usage in HSI case */
    for(start_band = 0; start_band < band_count; start_band += 13)  {
        end_band = (start_band + 13 > band_count)? band_count : start_band + 13;
	bands = end_band - start_band;
	//printf("start: %d, bands: %d\n", start_band, bands);

	/* Clear Output Buffer */
	bzero(dst_image, memory_req);
	bzero(counts, sizeof(float) * dst_XSize * dst_YSize * bands);
	/* For each pixel in the source, place it in the destination */
	for(int Y = 0; Y < src_YSize; Y++) {
		progress--;
		if(progress <= 0) {
			printf("%.0f %%\n", 100.0 * ((float)start_band/(float)band_count + ((float)bands/(float)band_count * (float)Y / (float)src_YSize)) );
			progress = progress_interval;
		}
		if(ins[Y].unk3 != 5) { /* Looks like 1,4 => no lock, 5 => valid data */
			continue;
		}
		/* Fetch the bands into memory */
		for(int band = 0; band < bands ; band++) {
			src_ds->GetRasterBand( start_band+band+1 )->RasterIO( GF_Read, 0, Y, src_XSize, 1, scanline+(src_XSize*band), src_XSize, 1, IMAGE_GDAL_TYPE, 0, 0);
		}
		//src_ds->RasterIO( GF_Read, 0, Y, src_XSize, 1, scanline, src_XSize, 1, IMAGE_GDAL_TYPE, band_count, NULL, 0, 0, 0);	

		for(int X = 0; X < src_XSize; X++) {
			Point ground;
			double dst_x, dst_y;
			int dst_xi, dst_yi;
	
			{ /* image to ground inlined */
				// Note '-' sign it appears the sensor is 'backwards', i.e. 0 is right, 6144 is left
				
				double lens_angle = -radians_per_px * float(X - (number_of_px / 2.0)); /* x */
				double elevation  =  ins[Y].alt - ground_elevation;   // TODO: Should get from elevation model /* x y */

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
				
			dst_x = (dst_gt[5] * xx - dst_gt[2] * yy) / invdet;
			dst_y = (-dst_gt[4] * xx + dst_gt[1] * yy) / invdet;
			
			//printf("%.0lf %.0lf\n", dst_x, dst_y);

			for(int band = 0; band < bands ; band++) {
				/* Perform Gaussian Kernel */
				for(dst_yi = floor(dst_y) - 1; dst_yi < floor(dst_y) + 2; dst_yi++)
					for(dst_xi = floor(dst_x) - 1; dst_xi < floor(dst_x) + 2; dst_xi++) {
						{ /* set_px inlined */
							if(dst_xi >= 0 && dst_xi < dst_XSize && dst_yi >= 0 && dst_yi < dst_YSize) {
									int offset = dst_xi+(dst_yi*dst_XSize)+(band*dst_XSize*dst_YSize);
									
									// Use gaussian pixel weighting (sinc*sinc) -- better/slower
									//float weight = expf(-sqrtf((dst_x-dst_xi)*(dst_x-dst_xi) + (dst_y-dst_yi)*(dst_y-dst_yi)));
									// Use linear pixel weighting -- approx/faster
									float weight = (1.0-abs(dst_x-dst_xi)) * (1.0-abs(dst_y-dst_yi));

									int p = scanline[X+src_XSize*band] * weight;

									// linear (amplitude/coherent light) add
									//dst_image[offset] += p;
									//counts[offseet] += weight;
									
									// uncorrelated source power add (sqrt of sum of squares)
									dst_image[offset] += p * p;
									counts[offset] += weight * weight;
							}
						}
					}
			}
		}
	}
	/* Scale pixels */
	/* TODO: Is this a valid thing to do for the HSI images? */
	for(int band = 0; band < bands; band++) {
//		printf("Normalizing and writing band: %d\n", band);
		long start = band * dst_XSize * dst_YSize;
		long end   = start + dst_XSize * dst_YSize;
		
		/* Rescale output buffer to 0=NODATA, 1-255 */
		/* TODO: this would be better to do with a mask band but then we'd need GDAL 1.8+ */
		outImageType min = INFINITY; //dst_image[start];
		outImageType max = -INFINITY; // dst_image[start];

		for(int i = start; i < end; i++) {
			if(counts[i] > 0.01) { // Don't count nodata pixels
				dst_image[i] = dst_image[i] / (float)counts[i];
				dst_image[i] = sqrt(dst_image[i]);

				if(dst_image[i] < min) min = dst_image[i];
				if(dst_image[i] > max) max = dst_image[i];
			}
		}

		float scale = 255 / (max - min); // Scale 1-255
		for(int i = start; i < end; i++) {
			if(counts[i] > 0.01) 
				dst_image[i] = ((dst_image[i] - min) * scale) + 1; // +1 to allow 0 to be nodata.
			else
				dst_image[i] = 0; // If counts = 0 then set NODATA.
		}

		/* Write destination buffer to file */
		GDALRasterBand *dst_band = dst_ds->GetRasterBand( start_band+band+1 );
		dst_band->SetNoDataValue(0.0);
		dst_band->RasterIO( GF_Write, 0, 0, dst_XSize, dst_YSize, dst_image+start, dst_XSize, dst_YSize, IMAGE_GDAL_TYPE, 0, 0);
	}
    }
	/* Build pyrimids */
	int anOverviewList[3] = { 2, 4, 8 };
	dst_ds->BuildOverviews("AVERAGE", 3, anOverviewList, 0, NULL, GDALDummyProgress, NULL);
	
	/* Cleanup */
	free(scanline);
	free(dst_image);
	
	return 0;
}
