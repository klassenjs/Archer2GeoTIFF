/*
 *  ProcessArcher.cpp
 *  
 *
 *  Created by Jim Klassen on 9/7/08.
 *  Copyright (C) 2008 James Klassen
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

#include<stdlib.h>
#include<stdexcept>
#include<math.h>
#include "ProcessArcher.h"
#include<gdal_priv.h>
#include<ogr_spatialref.h>

ArcherINSData::ArcherINSData() 
{
	this->used_rows = 0;
	this->allocated_rows = 0;
	this->Data = NULL;
	
	this->Hdr.FOV = 40.15;
	this->Hdr.RollBias = 0;
	this->Hdr.PitchBias = 0;
	this->Hdr.HeadingBias = 0;
	this->Hdr.LongitudeBias = 0;
	this->Hdr.LatitudeBias = 0;
	this->Hdr.AltitudeBias = 0;
}

ArcherINSData::~ArcherINSData() 
{
	if(this->Data) {
		free(this->Data);
	}
}

void ArcherINSData::setParams(double FOV, double RollBias, double PitchBias, double HeadingBias, 
									double LongitudeBias, double LatitudeBias, double AltitudeBias,
									int Samples)
{
	this->Hdr.FOV = FOV;
	this->Hdr.RollBias = RollBias;
	this->Hdr.PitchBias = PitchBias;
	this->Hdr.HeadingBias = HeadingBias;
	this->Hdr.LongitudeBias = LongitudeBias;
	this->Hdr.LatitudeBias = LatitudeBias;
	this->Hdr.AltitudeBias = AltitudeBias;
	this->Hdr.Samples = Samples;
}

void ArcherINSData::set_bbox(double minx, double miny, double maxx, double maxy)
{
	this->bbox[0] = minx;
	this->bbox[1] = miny;
	this->bbox[2] = maxx;
	this->bbox[3] = maxy;
}


/* Expand the current BBOX to include the passed in point */
void ArcherINSData::expand_bbox(double x, double y)
{
	if(this->bbox[0] > x) this->bbox[0] = x;
	if(this->bbox[1] > y) this->bbox[1] = y;
	if(this->bbox[2] < x) this->bbox[2] = x;
	if(this->bbox[3] < y) this->bbox[3] = y;
}

void ArcherINSData::calculate_bbox()
{
	int min_x = 0;
	int max_x = this->Hdr.Samples;
	
	int min_y = 0;
	int max_y = this->length();

	double meters_per_px_est = 0;

	/* Init the BBOX to the first center point */
	ArcherINSRow air = this->get_Data(0);
	this->set_bbox(air.x, air.y, air.x, air.y);
	
	/* Check the end points of each scan and use that to determine a bbox */
	
	for(int y = min_y; y < max_y; y++) {
		double gr_x, gr_y;
		this->image_to_ground(min_x, y, &gr_x, &gr_y);
		this->expand_bbox(gr_x, gr_y);

		this->image_to_ground(max_x, y, &gr_x, &gr_y);
		this->expand_bbox(gr_x, gr_y);
		
		meters_per_px_est += this->get_Data(y).alt - 213.36; /* Round off could kill this */
	}
	
	this->meters_per_px_est = (meters_per_px_est / (max_y - min_y)) * (this->Hdr.FOV*3.14159 / 180) / this->Hdr.Samples;
}

double* ArcherINSData::get_bbox()
{
	return this->bbox;
}

void ArcherINSData::append(int sequence, 
				double roll, double pitch, double yaw, 
				double lon, double lat, double alt, 
				double time, 
				double x, double y, double heading) 
{
	if(this->allocated_rows <= this->used_rows) {
		this->allocated_rows += 1024;
		this->Data = (ArcherINSRow*) realloc(this->Data, this->allocated_rows * sizeof(ArcherINSRow));
	}
	this->Data[used_rows].sequence = sequence;
	this->Data[used_rows].roll     = roll;
	this->Data[used_rows].pitch    = pitch;
	this->Data[used_rows].yaw      = yaw;
	this->Data[used_rows].lat      = lat;
	this->Data[used_rows].lon      = lon;
	this->Data[used_rows].alt      = alt;
	this->Data[used_rows].time     = time;
	this->Data[used_rows].x        = x;
	this->Data[used_rows].y        = y;
	this->Data[used_rows].heading  = heading;
	(this->used_rows)++;
}

void ArcherINSData::image_to_ground(int img_x, int img_y, double *ground_x, double *ground_y)
{
		double degrees_to_rad = 3.14159 / 180;
		ArcherINSRow d = this->get_Data(img_y);

		int number_of_px   = this->Hdr.Samples;
		double radians_per_px = (this->Hdr.FOV*degrees_to_rad) / number_of_px;

		// Note '-' sign it appears the sensor is 'backwards', i.e. 0 is right, 6144 is left
		double lens_angle = -radians_per_px * float(img_x - (number_of_px / 2.0));
		double elevation  =  d.alt - 213.36;   //Should get from elevation model

		// Calculate x,y on ground relative to center of plane along direction of plane
		double rel_plane_x = sin(lens_angle + d.roll) * elevation;
		double rel_plane_y = sin(d.pitch) * elevation;

		// Now rotate plane rel coordinates by Yaw + heading into plane rel north oriented
		double phi = d.heading + d.yaw;
	
		double rel_ground_x =  ( rel_plane_x * cos(phi)) + (rel_plane_y * sin(phi));
		double rel_ground_y =  (-rel_plane_x * sin(phi)) + (rel_plane_y * cos(phi));

		// Convert from plane relative to ground coordinate system by adding plane centerpoint offset
		*ground_x = rel_ground_x + d.x;
		*ground_y = rel_ground_y + d.y;
}

ArcherINSRow* ArcherINSData::get_Data() 
{
	return(this->Data);
}

ArcherINSRow ArcherINSData::get_Data(int row)
{
	if(row >= this->used_rows || row < 0)
		throw(std::out_of_range("Array out of bounds"));
	return(this->Data[row]);
}

int ArcherINSData::length()
{
	return used_rows;
}

const char* version() 
{
	return("1.0");
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

void ground_to_output_int(double gt[6], double ground_x, double ground_y, int *img_x, int *img_y) 
{
	double x, y;
	ground_to_output(gt, ground_x, ground_y, &x, &y);
	*img_x = floor(x);
	*img_y = floor(y);
}


ProcessArcher::ProcessArcher(char* in_file, char* out_file, ArcherINSData* ins)
{
	this->in_file = in_file;
	this->out_file = out_file;
	this->ins = ins;
}

ProcessArcher::~ProcessArcher()
{
	return;
}


GDALDataset* createOutputDS(char* filename, double bbox[4], double meters_per_pixel, int *imgsize_x, int *imgsize_y) 
{
		*imgsize_x = ceil((bbox[2] - bbox[0]) / meters_per_pixel);
		*imgsize_y = ceil((bbox[3] - bbox[1]) / meters_per_pixel);
		
		printf("Creating output image %d x %d\n", *imgsize_x, *imgsize_y);

		GDALDriverManager *gdal_drivers = GetGDALDriverManager();
		GDALDriver *driver = gdal_drivers->GetDriverByName("GTiff");
		GDALDataset *dst_ds = driver->Create(filename, *imgsize_x, *imgsize_y, 1, GDT_Byte, (char**) NULL);
		
		double top_left[2] = { bbox[0], bbox[3] };
		double geo_transform[6] = { top_left[0], meters_per_pixel, 0, top_left[1], 0, -meters_per_pixel };
		dst_ds->SetGeoTransform(geo_transform);
		
		OGRSpatialReference srs;
		char *SRS_WKT = NULL;
		srs.SetUTM(15, 1);
		srs.SetWellKnownGeogCS("WGS84");
		srs.exportToWkt( &SRS_WKT );
		dst_ds->SetProjection( SRS_WKT );
		CPLFree( SRS_WKT );
		
		return dst_ds;
}

float sinc(float x, float y) {
	x = sqrtf(x*x + y*y);
	if(x == 0) return 1;
	x = x * 3.14159;
	return sinf(x)/x;
}

typedef float outImageType;

void set_px(outImageType* img, int dst_x, int dst_y, int x, int y, outImageType value) {
		if(x >= 0 && x < dst_x && y >= 0 && y < dst_y)
			img[x+(y*dst_x)] = value;
}


int ProcessArcher::run(double meters_per_pixel)
{
	GDALDataset *src_ds, *dst_ds;
	GDALRasterBand *src_image, *dst_band;
	GDALAllRegister();
	int dst_x, dst_y;
	outImageType* dst_image;
	double dst_gt[6];

	this->ins->calculate_bbox();
	
	/* Print information gathered eariler for 'feel good' */
	printf("bbox: [ %lf %lf %lf %lf ] rows: %d meters_per_px_est: %lf\n",
		this->ins->bbox[0], this->ins->bbox[1], this->ins->bbox[2], this->ins->bbox[3],
		this->ins->length(), this->ins->meters_per_px_est);
	
	src_ds = (GDALDataset*) GDALOpen( this->in_file, GA_ReadOnly );
	dst_ds = createOutputDS( this->out_file, ins->get_bbox(), meters_per_pixel, &dst_x, &dst_y);

	dst_image = (outImageType*)malloc(sizeof(outImageType) * dst_x * dst_y);
	bzero(dst_image, sizeof(char) * dst_x * dst_y);
	dst_ds->GetGeoTransform(dst_gt);

	/* Need to do this for each band */
	/* If we have enough mem do multiple simultaniously so can reuse calculations */
	src_image = src_ds->GetRasterBand( 1 );

	int src_XSize = src_image->GetXSize();
	int src_YSize = src_image->GetYSize();
	
	outImageType *scanline;
	scanline = (outImageType*) malloc(sizeof(outImageType) * src_XSize);
	for(int Y = 0; Y < src_YSize; Y++) {
		//printf("%d\n", Y);
		src_image->RasterIO( GF_Read, 0, Y, src_XSize, 1, scanline, src_XSize, 1, GDT_Float32, 0, 0);
		for(int X = 0; X < src_XSize; X++) {
			double gx, gy;
			double oxd, oyd;
			int    ox, oy;
			this->ins->image_to_ground(X, Y, &gx, &gy);
			ground_to_output(dst_gt, gx, gy, &oxd, &oyd);
			/* Perform Gaussian Kernel */
			oy = floor(oyd);
			ox = floor(oxd);
			set_px(dst_image, dst_x, dst_y, ox, oy, scanline[X]);
			//for(oy = floor(oyd) - 1; oy < floor(oyd) + 2; oy++)
				//for(ox = floor(oxd) - 1; ox < floor(oxd) + 2; ox++)
					//printf("ox: %d oy: %d oxd: %lf oyd: %lf sinc: %lf px: %f\n", ox, oy, oxd, oyd, sinc(oxd - ox, oyd - oy), (outImageType)((float)scanline[X] * sinc(oxd - ox, oyd - oy)));
					//set_px(dst_image, dst_x, dst_y, ox, oy, scanline[X] * sinc(oxd - ox, oyd - oy));
		}
	}
	
	dst_band = dst_ds->GetRasterBand( 1 );
	dst_band->SetNoDataValue(0.0);
	dst_band->RasterIO( GF_Write, 0, 0, dst_x, dst_y, dst_image, dst_x, dst_y, GDT_Float32, 0, 0);
	
	int anOverviewList[3] = { 2, 4, 8 };
	dst_ds->BuildOverviews("NEAREST", 3, anOverviewList, 0, NULL, GDALDummyProgress, NULL);
	
	delete src_ds;
	delete dst_ds;
	free(scanline);
	free(dst_image);
	
	return 0;
}












