/*
 *  ArcherDataset.h
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

#ifndef _ARCHER_DATASET_H
#define _ARCHER_DATASET_H

#include <proj_api.h>
#include <gdal_priv.h>
#include <ogr_spatialref.h>

typedef struct {
	long sequence; // Unique ID of image row
	double roll;   // Roll of Plane in radians
	double pitch;  // Pitch of Plane in radians
	double yaw;    // Yaw of Plane in radians (file is in degrees)
	double lon;    // Longitude in degrees WGS84
	double lat;    // Latitude in degrees WGS84
	double alt;    // Altitude in meters WGS84
	int unk1;      // 0
	int unk2;      // 0
	int unk3;      // 5
	double time;   // Time in seconds with an unknown offset
	int unk4;      // 8192
	double x;	   // Derived: Easting
	double y;      // Derived: Northing
	double heading; // Derived: Plane heading in radians
} ArcherINSRow;

typedef struct {
	double x;
	double y;
	double z;
} Point;

typedef struct {
	int x;
	int y;
} IPoint;

class BBox
{
	public:
		double minx;
		double miny;
		double maxx;
		double maxy;
		double phi; //TODO: supported rotated bboxes 
		
		BBox(double minx, double miny, double maxx, double maxy);
		void Expand(Point pt);
};

class ArcherDataset
{
	public:
		ArcherDataset(const char* proj, const char* src);
		~ArcherDataset();
	
		GDALDataset* GetGDALDataset();
		BBox* GetBBox();
		double GetEstResolution();
		int GetRasterCount();
		ArcherINSRow* GetINSData();
		
		const Point ImageToGround(IPoint pt);
		const Point ImageToGround(int x, int y);
	private:
		int ins_rows;
		ArcherINSRow* ins_data;

		GDALDataset* gdal_ds;
		projPJ WGS84;
		projPJ dest_proj;
	public:	
		struct {
			double FOV; // Field of view of lens in degrees (this is used in the camera calcs)
			double RollBias; // radians (file is in degrees)
			double PitchBias; // radians (file is in degrees)
			double HeadingBias; // radians (file is in degrees)
			double LongitudeBias; // degrees
			double LatitudeBias; // degrees
			double AltitudeBias; // meters
			double *Wavelength; // Array of wavelengths of each band in nm
		} ArcherENVIHdr;
		
		int image_height;
		int image_width;
		int image_bands;
		
		BBox *bbox;
		double meters_per_px_est;
		
		void printINSData();
		
		/* Helper functions for the initializer */
		void readHDRFile(const char* src);
		void readINSFile(const char* filename);
		void calculateINSParameters();
		void estimateGroundBBOX();

};

#endif
