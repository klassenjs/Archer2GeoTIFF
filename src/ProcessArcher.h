/*
 *  ProcessArcher.h
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

typedef struct {
	long sequence; // Unique ID of image row
	double roll;   // Roll of Plane in radians
	double pitch;  // Pitch of Plane in radians
	double yaw;    // Yaw of Plane in radians (file is in degrees)
	double lat;    // Latitude in degrees WGS84
	double lon;    // Longitude in degrees WGS84
	double alt;    // Altitude in meters WGS84
	int unk1;      // 0
	int unk2;      // 0
	int unk3;      // 5
	double time;   // Time in seconds with an unknown offset
	int unk4;      // 8192
	double x;	   // Derived: Easting
	double y;      // Derived: Northing
	double heading; // Derived: Plane heading in degrees
} ArcherINSRow;

typedef struct {
		double FOV; // Field of view of lens in degrees (this is used in the camera calcs)
		double RollBias; // degrees (the rest should all be zero and processed in py code)
		double PitchBias; // degrees
		double HeadingBias; // degrees
		double LongitudeBias; // degrees
		double LatitudeBias; // degrees
		double AltitudeBias; // meters
		int    Samples; // sensor width in px
} ArcherENVIHdr;


class ArcherINSData {
public:
	ArcherINSData();
	~ArcherINSData();
	void setParams(double FOV, double RollBias, double PitchBias, double HeadingBias, 
					    double LongitudeBias, double LatitudeBias, double AltitudeBias, int Samples);

	void append(int sequence, double roll, double pitch, double yaw, 
			double lon, double lat, double alt, double time, 
			double x, double y, double heading);
	
	void set_bbox(double minx, double miny, double maxx, double maxy);
	double* get_bbox();
													
	void image_to_ground(int img_x, int img_y, double *ground_x, double *ground_y);
			
	ArcherINSRow* get_Data();
	ArcherINSRow  get_Data(int row);
	int           length();
	
	double bbox[4];	
	double meters_per_px_est;
	void calculate_bbox();

private:
	void expand_bbox(double x, double y);
	
	int used_rows;
	int allocated_rows;
	ArcherINSRow* Data;
	ArcherENVIHdr Hdr;
};

class ProcessArcher {
public:
	ProcessArcher(char* file_in, char* file_out, ArcherINSData *ins);
	~ProcessArcher();
	
	int run(double meters_per_pixel);
	
private:
	char* in_file;
	char* out_file;
	ArcherINSData *ins;
};

const char* version();
void ground_to_output(double gt[6], double ground_x, double ground_y, double *img_x, double *img_y);
void ground_to_output_int(double gt[6], double ground_x, double ground_y, int *img_x, int *img_y);
