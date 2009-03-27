/*  main.cpp
 *  Archer2GeoTiff
 * 
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
#include<stdio.h>
#include<string.h>
#include "main.h"
#include "ArcherDataset.h"
#include "ArcherRectify.h"

void usage(char* argv0)
{
	printf("Usage: %s [-p projection] [-r resolution (ground units/px)] -i input -o output\n", argv0);
	exit(1);
}

int main(int argc, char* argv[]) {
	char *projectionString = NULL;
	char *srcDataset  = NULL;
	char *destDataset = NULL;
	char *resolution  = NULL;
	double res;

	int i = 1;
	
	while(i < argc) {
		if(strcmp(argv[i], "-p") == 0) {
			projectionString = argv[i];
		} else if(strcmp(argv[i], "-o") == 0) {
			destDataset = argv[++i];
		} else if(strcmp(argv[i], "-i") == 0) {
			srcDataset = argv[++i];
		} else if(strcmp(argv[i], "-r") == 0) {
			resolution = argv[++i];
		} else {
			usage(argv[0]);
		}
		i++;
	}
	if(destDataset == NULL || srcDataset == NULL)
		usage(argv[0]);
	
	printf("New ArcherDataset(%s) -> %s\n", srcDataset, destDataset);
	ArcherDataset *archerDataset = new ArcherDataset(projectionString, srcDataset);
	ArcherRectify *archerRectifier = new ArcherRectify(archerDataset);
	
	if(resolution == NULL)
		res = archerDataset->GetEstResolution();
	else
		res = atof(resolution);
	
	archerRectifier->CreateOutputDataset(destDataset, archerDataset->GetBBox(),
														res, archerDataset->GetRasterCount(), projectionString);
	archerRectifier->run();
	delete archerRectifier;
	delete archerDataset;
	
	printf("Done.\n");
	return(0);
}
