/*
 *  ArcherDataset.cpp
 *  
 *
 *  Created by Jim Klassen on 3/20/09.
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

#include "ArcherDataset.h"
#include <stdexcept>
#include <string>

BBox::BBox(double minx, double miny, double maxx, double maxy)
{
	this->minx = minx;
	this->miny = miny;
	this->maxx = maxx;
	this->maxy = maxy;
	this->phi = 0;
}

void BBox::Expand(Point pt)
{
	double phi = this->phi;

	// Shift BBOX to 0,0 and rotate.
	double minx = 0;
	double miny = 0;
	double maxx = this->maxx - this->minx;
	double maxy = this->maxy - this->miny;
	double pt_x = pt.x - this->minx;
	double pt_y = pt.y - this->miny;

	// Rotate about (0,0) by phi so we are in BBOX coordinates
	double minx_r = 0;
	double miny_r = 0;
	double maxx_r =   maxx * cos(phi) + maxy * sin(phi);
	double maxy_r = - maxx * sin(phi) + maxy * cos(phi);
	double pt_x_r =   pt_x * cos(phi) + pt_y * sin(phi);
	double pt_y_r = - pt_x * sin(phi) + pt_y * cos(phi);

	// Expand the BBOX in its corrdinates
	if(minx_r > pt_x_r) minx_r = pt_x_r;
	if(miny_r > pt_y_r) miny_r = pt_y_r;
	if(maxx_r < pt_x_r) maxx_r = pt_x_r;
	if(maxy_r < pt_y_r) maxy_r = pt_y_r;
	
	// Rotate back
	minx =   minx_r * cos(-phi) + miny_r * sin(-phi);
	miny = - minx_r * sin(-phi) + miny_r * cos(-phi);
	maxx =   maxx_r * cos(-phi) + maxy_r * sin(-phi);
	maxy = - maxx_r * sin(-phi) + maxy_r * cos(-phi);
	pt_x =   pt_x_r * cos(-phi) + pt_y_r * sin(-phi);
	pt_y = - pt_x_r * sin(-phi) + pt_y_r * cos(-phi);
	
	// Shift back
	this->maxx = maxx + this->minx;
	this->minx = minx + this->minx;
	this->maxy = maxy + this->miny;
	this->miny = miny + this->miny;
}

void ArcherDataset::readHDRFile(const char* src)
{

	char *hdr_file_name = (char*)malloc(strlen(src)+4);
	sprintf(hdr_file_name, "%s.hdr", src); 
	FILE *hdr_file = fopen(hdr_file_name, "r");
	free(hdr_file_name);
	
	/* Set Defaults for Archer System 8 PAN*/
	this->ArcherENVIHdr.FOV = 40.150;
	this->ArcherENVIHdr.RollBias = 0.0;
	this->ArcherENVIHdr.PitchBias = 0.0;
	this->ArcherENVIHdr.HeadingBias = 0.0;
	this->ArcherENVIHdr.LongitudeBias = 0.0;
	this->ArcherENVIHdr.LatitudeBias = 0.0;
	this->ArcherENVIHdr.AltitudeBias = 0.0;
	this->ArcherENVIHdr.Wavelength = NULL;
	
	char buffer[1024];
	double val;
	if(hdr_file) {
		while(!feof(hdr_file)) {
			fgets(buffer, 1024, hdr_file);
			if(sscanf(buffer, "FOV = %lf\n", &val))
				this->ArcherENVIHdr.FOV = val;
			else if(sscanf(buffer, "Roll Bias = %lf\n", &val))
				this->ArcherENVIHdr.RollBias = val * DEG_TO_RAD;
			else if(sscanf(buffer, "Pitch Bias = %lf\n", &val))
				this->ArcherENVIHdr.PitchBias = val * DEG_TO_RAD;
			else if(sscanf(buffer, "Heading Bias = %lf\n", &val))
				this->ArcherENVIHdr.HeadingBias = val * DEG_TO_RAD;
			else if(sscanf(buffer, "Longitude Bias = %lf\n", &val))
				this->ArcherENVIHdr.LongitudeBias = val;
			else if(sscanf(buffer, "Latitude Bias = %lf\n", &val))
				this->ArcherENVIHdr.LatitudeBias = val;
			else if(sscanf(buffer, "Altitude Bias = %lf\n", &val))
				this->ArcherENVIHdr.AltitudeBias = val;
		}
		fclose(hdr_file);
	}
}

void ArcherDataset::printINSData()
{
	for(int i = 0; i < this->ins_rows; i++) 
		printf("%ld %lf %lf %lf %lf %lf %lf %d %d %d %lf %d, %lf %lf %lf\n", 
				this->ins_data[i].sequence,
				this->ins_data[i].roll,
				this->ins_data[i].pitch,
				this->ins_data[i].yaw,
				this->ins_data[i].lon,
				this->ins_data[i].lat,
				this->ins_data[i].alt,
				this->ins_data[i].unk1,
				this->ins_data[i].unk2,
				this->ins_data[i].unk3,
				this->ins_data[i].time,
				this->ins_data[i].unk4,
				this->ins_data[i].x,
				this->ins_data[i].y,
				this->ins_data[i].heading
				);
	if(this->bbox)
		printf("BBox: [ %lf %lf %lf %lf ]\n", this->bbox->minx, this->bbox->miny, this->bbox->maxx, this->bbox->maxy);

}

void ArcherDataset::readINSFile(const char* fname)
{
	FILE *in_file = fopen(fname, "r");
	int alloc_rows = 1024;
	int used_rows = 0;
	this->ins_data = (ArcherINSRow*)malloc(1024 * sizeof(ArcherINSRow));
	
	while(!feof(in_file)) {
			if(used_rows >= alloc_rows) {
				alloc_rows += 1024;
				this->ins_data = (ArcherINSRow*) realloc(this->ins_data, alloc_rows * sizeof(ArcherINSRow));
			}
	
			fscanf(in_file, "%ld %lf %lf %lf %lf %lf %lf %d %d %d %lf %d\n",
				&this->ins_data[used_rows].sequence,
				&this->ins_data[used_rows].roll,
				&this->ins_data[used_rows].pitch,
				&this->ins_data[used_rows].yaw,
				&this->ins_data[used_rows].lon,
				&this->ins_data[used_rows].lat,
				&this->ins_data[used_rows].alt,
				&this->ins_data[used_rows].unk1,
				&this->ins_data[used_rows].unk2,
				&this->ins_data[used_rows].unk3,
				&this->ins_data[used_rows].time,
				&this->ins_data[used_rows].unk4);
			this->ins_data[used_rows].yaw *= DEG_TO_RAD; /* File is in degrees, we need radians */
			
			/* Use the biases included from the HDR file so we don't have to calc this every time */
			this->ins_data[used_rows].roll  += this->ArcherENVIHdr.RollBias;
			this->ins_data[used_rows].pitch += this->ArcherENVIHdr.PitchBias;
			this->ins_data[used_rows].yaw   += this->ArcherENVIHdr.HeadingBias;
			this->ins_data[used_rows].lat   += this->ArcherENVIHdr.LatitudeBias;
			this->ins_data[used_rows].lon   += this->ArcherENVIHdr.LongitudeBias;
			this->ins_data[used_rows].alt   += this->ArcherENVIHdr.AltitudeBias;
			used_rows++;
	}
	this->ins_rows = used_rows;
	fclose(in_file);
}

/* This function fills in the necessary derived parameters from the INS data */
void ArcherDataset::calculateINSParameters()
{
	const int rows = this->ins_rows;
	int row;
	
	projPJ src = this->WGS84;
	projPJ dst = this->dest_proj;
	
	double easting, northing;
	
	/* Calculate the projected ground coordinates of the plane's track */
	for(row = 0; row < rows; row++) {
		northing = this->ins_data[row].lat * DEG_TO_RAD;
		easting  = this->ins_data[row].lon * DEG_TO_RAD;
		
		pj_transform(src, dst, 1, 0, &easting, &northing, NULL);

		this->ins_data[row].x = easting;
		this->ins_data[row].y = northing;
	}
	
	/* TODO: Assumes more than 100 lines in photo */
	for(row = 0; row < rows - 100; row++) {
		this->ins_data[row].heading = M_PI_2 - atan2( this->ins_data[row+100].y - this->ins_data[row].y, 
												this->ins_data[row+100].x - this->ins_data[row].x);
	}
	for(; row < rows; row++) {
		this->ins_data[row].heading = M_PI_2 - atan2( this->ins_data[row].y - this->ins_data[row-100].y, 
												this->ins_data[row].x - this->ins_data[row-100].x);
	
	}
}

/* Calculate estimated ground BBOX (assumes no elevation) */
void ArcherDataset::estimateGroundBBOX()
{
	int min_x = 0;
	int max_x = this->image_width;
	
	int min_y = 0;
	int max_y = this->ins_rows;

	double meters_per_px_est = 0;

	/* Init the BBOX to the first center point */
	this->bbox = new BBox(this->ins_data[0].x, this->ins_data[0].y, this->ins_data[0].x, this->ins_data[0].y);
	
	//printf("%lf %lf\n", this->ins_data[max_y-1].x - this->ins_data[0].x, this->ins_data[max_y-1].y - this->ins_data[0].y);
	this->bbox->phi = 0; // Disable image rotation;
	if(this->bRotateImage)
		this->bbox->phi = atan2( this->ins_data[max_y-1].y - this->ins_data[0].y, 
												this->ins_data[max_y-1].x - this->ins_data[0].x);
	/* Check the end points of each scan and use that to determine a bbox */
	for(int y = min_y; y < max_y; y++) {
		if(this->ins_data[y].unk3 == 5) {
			this->bbox->Expand( this->ImageToGround(min_x, y) );
			this->bbox->Expand( this->ImageToGround(max_x, y) );
		
			meters_per_px_est += this->ins_data[y].alt - this->fAverageGroundElevation; /* Round off could kill this */
		}
	}
	
	this->meters_per_px_est = (meters_per_px_est / (max_y - min_y)) * (this->ArcherENVIHdr.FOV * DEG_TO_RAD) / this->image_width;

}

GDALDataset* ArcherDataset::GetGDALDataset()
{
	return this->gdal_ds;
}

BBox* ArcherDataset::GetBBox()
{
	return this->bbox;
}

double ArcherDataset::GetEstResolution()
{
	return this->meters_per_px_est;
}

int ArcherDataset::GetRasterCount()
{
	return this->gdal_ds->GetRasterCount();
}

ArcherINSRow *ArcherDataset::GetINSData()
{
	return this->ins_data;
}

const Point ArcherDataset::ImageToGround(IPoint pt)
{
	Point ground;
	ArcherINSRow *d = &this->ins_data[pt.y];

	int number_of_px   = this->image_width; /*const*/
	double radians_per_px = (this->ArcherENVIHdr.FOV * DEG_TO_RAD) / number_of_px; /*const*/

	// Note '-' sign it appears the sensor is 'backwards', i.e. 0 is right, 6144 is left
	double lens_angle = -radians_per_px * float(pt.x - (number_of_px / 2.0)); /* x */
	double elevation  =  d->alt - this->fAverageGroundElevation;   // TODO: Should get from elevation model /* x y */

	// Calculate x,y on ground relative to center of plane along direction of plane
	double rel_plane_x = sin(lens_angle + d->roll) * elevation;
	double rel_plane_y = sin(d->pitch) * elevation;

	// Now rotate plane rel coordinates by Yaw + heading into plane rel north oriented
	double phi = d->heading + d->yaw;

	double rel_ground_x =  ( rel_plane_x * cos(phi)) + (rel_plane_y * sin(phi));
	double rel_ground_y =  (-rel_plane_x * sin(phi)) + (rel_plane_y * cos(phi));

	// Convert from plane relative to ground coordinate system by adding plane centerpoint offset
	ground.x = rel_ground_x + d->x;
	ground.y = rel_ground_y + d->y;
	ground.z = 0;
	
	return ground;

}

const Point ArcherDataset::ImageToGround(int img_x, int img_y)
{
	IPoint pt;
	pt.x = img_x;
	pt.y = img_y;
	return this->ImageToGround(pt);
}

ArcherDataset::ArcherDataset(const char* proj, const char* src)
{
	ArcherDataset(proj, src, 259.0, false);
}
ArcherDataset::ArcherDataset(const char* proj, const char* src, float fAverageGroundElevation, bool bRotateImage)
{
	this->WGS84 = pj_init_plus("+init=epsg:4326"); // WGS84
	if(proj)
		this->dest_proj = pj_init_plus(proj); // Should be NULL if failed... then don't reproject
	else
		this->dest_proj = pj_init_plus("+init=epsg:26915"); // Default to UTM15N Meters
	
	this->fAverageGroundElevation = fAverageGroundElevation;
	this->bRotateImage = bRotateImage;
	
	/* Open the GDAL dataset */
	if(!src)
		throw(std::out_of_range("Source dataset not specified"));  
	
	printf("Trying to open dataset using GDAL (%s)\n", src);
	
	GDALAllRegister();
	this->gdal_ds = (GDALDataset*) GDALOpen( src, GA_ReadOnly );
	printf("GDAL DS: %p\n", this->gdal_ds);
	if(this->gdal_ds == NULL)
		throw(std::out_of_range("Could not open source dataset"));
	/* Read parameters from the ENVI Header info from the GDAL Dataset */
	this->image_height = this->gdal_ds->GetRasterYSize();
	this->image_width = this->gdal_ds->GetRasterXSize();
	printf("Width: %d, Height: %d, Bands: %d\n", this->gdal_ds->GetRasterXSize(), this->gdal_ds->GetRasterYSize(), this->gdal_ds->GetRasterCount());
	
	/* Get non-standard parameters from the .hdr file */
	this->readHDRFile(src);
	printf("FOV: %lf\n", this->ArcherENVIHdr.FOV);
	
	/* Open the inertial nav system dataset */
	char *ins_fname = (char*) malloc(strlen(src));
	strcpy(ins_fname, src);
	strcpy(ins_fname + strlen(ins_fname) - 3, "txt");
	this->readINSFile(ins_fname);
	
	if(this->ins_rows != this->image_height) {
		printf("Warning inertial nav. data does not match image dimensions.\n");
	}

	/* Calculated derived INS values (x,y,heading)*/
	this->calculateINSParameters(); 

	/* Calculate estimated BBOX and resolution on the ground */
	this->estimateGroundBBOX();

	//this->printINSData();
	if(this->bbox)
		printf("BBox: [ %lf %lf %lf %lf ]\n", this->bbox->minx, this->bbox->miny, this->bbox->maxx, this->bbox->maxy);

}

ArcherDataset::~ArcherDataset()
{
	/* Free inertial nav data */
	if(this->ins_data)
		free(this->ins_data);

	/* Close GDAL dataset */
	if(this->gdal_ds != NULL)
		delete this->gdal_ds;

	/* Free projections */
	if(this->WGS84 != NULL)
		pj_free(this->WGS84);
	if(this->dest_proj != NULL)
		pj_free(this->dest_proj);
		
	/* Free wavelength list */
	if(this->ArcherENVIHdr.Wavelength)
		free(this->ArcherENVIHdr.Wavelength);
	
	/* Free bounding box */
	if(this->bbox)
		delete this->bbox;
}
