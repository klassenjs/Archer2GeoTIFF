/*
 *  ArcherRectify.h
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

#ifndef _ARCHER_RECTIFY_H
#define _ARCHER_RECTIFY_H

#include <proj_api.h>
#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include "ArcherDataset.h"

/* A class to convert a ArcherDataset to a DOQ. */
class ArcherRectify
{
	public:
			ArcherRectify(ArcherDataset *ads);
			~ArcherRectify();
			
			int CreateOutputDataset(char* filename, BBox* bbox, double meters_per_px, int nBands, char* projection);
			void UseExistingDataset(GDALDataset* ds);

			
			int run();
	private:
			ArcherDataset *archer;
			GDALDataset* gdalDestDS;
			

};

#endif
