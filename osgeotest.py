#!/usr/bin/python

# Copyright (C) 2008 James Klassen
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
#import mapscript
import osgeo.gdal
from osgeo.gdalconst import *
import osgeo.gdalnumeric
#import osgeo.ogr
import osgeo.osr
#import geos
import pyproj
import math
import numpy

class ArcherINS(object):
	def __init__(self, fname):
		self.filename = 0
		self.fileh = None
		self.rows = 0

		self.FOV = 40.15 # degrees
		self.RollBias = 0 # deg
		self.PitchBias = 0 # deg
		self.HeadingBias = 0 # deg
		self.LongitudeBias = 0 #deg
		self.LatitudeBias = 0 # deg
		self.AltitudeBias = 0 # meters

		self.Data = []
		self.BBOX = []
		
		if(fname):
			self.Open(fname)
			self.Read()


	def Open(self,fname):
		self.filename = fname
		self.fileh = open(fname, 'r')
		return self
		
	def Close(self):
		if(self.fileh):
			close(self.fileh)

	def Read(self):
		if(self.fileh):
			f = self.fileh
			pj = pyproj.Proj("+proj=utm +zone=15 +ellps=GRS80 +datum=NAD83 +units=m +no_defs")

			minx = 1000000000000000
			miny = 1000000000000000
			maxx = 0
			maxy = 0

			old_x = 0
			old_y = 0
			old_heading = numpy.zeros(100)
			old_heading_idx = 0

			for line in f:
				sequence, roll, pitch, yaw, lon, lat, alt, unk1, unk2, unk3, time, shutter = line.split()

				roll  = math.degrees(float(roll))  + self.RollBias
				pitch = math.degrees(float(pitch)) + self.PitchBias
				yaw   = float(yaw)   + self.HeadingBias
				lon   = float(lon)   + self.LongitudeBias
				lat   = float(lat)   + self.LatitudeBias
				alt   = float(alt)   + self.AltitudeBias

				x, y  = pj(lon, lat)

				if(x < minx):
					minx = x
				if(y < miny):
					miny = y
				if(x > maxx):
					maxx = x
				if(y > maxy):
					maxy = y
				if(x <> old_x and y <> old_y):	
					heading = 90 - math.degrees(math.atan2( y - old_y, x - old_x))
					#print 'Inst head = ', heading
					old_heading_idx = ( old_heading_idx + 1 ) % 100
					old_heading[old_heading_idx] = heading
					heading = old_heading.mean()
					#print 'Ave head = ', heading

			
				old_x = x
				old_y = y

				self.Data.append( { 'sequence':sequence, 'roll':roll, 'pitch':pitch, 'yaw':yaw, 'lon':lon, 'lat':lat, 'alt':alt, 'time':time, 'x':x, 'y':y, 'heading':heading  } )			
				self.rows = self.rows + 1
			# Silly buffer to approx bbox otherwise is just range of centerpoints
			self.BBOX = [ minx-1000, miny-1000, maxx+1000, maxy+1000 ]
		

class ElevationModel:
	def __init__(self):
		return

	def getElev(self, Easting, Northing):
		return 213.36	# flat at 700ft for now

class processArcher(object):
	def __init__(self, basename):
		ins_ds = ArcherINS(basename+'.txt')
		src_ds = osgeo.gdal.Open(basename+'.pan', GA_ReadOnly)

		print 'Driver: ', src_ds.GetDriver().ShortName,'/', src_ds.GetDriver().LongName
		print 'Size is ',src_ds.RasterXSize,'x',src_ds.RasterYSize, 'x',src_ds.RasterCount
		print 'Projection is ',src_ds.GetProjection()
		    
		geotransform = src_ds.GetGeoTransform()
		if not geotransform is None:
			print 'Origin = (',geotransform[0], ',',geotransform[3],')'
			print 'Pixel Size = (',geotransform[1], ',',geotransform[5],')'	


		# This is Archer data so we don't expect the image to be georeferenced yet.
		print ins_ds.BBOX, ins_ds.rows

		src_image = src_ds.GetRasterBand(1).ReadAsArray()

		dst_ds, dst_image = self.createOutput(basename+".tiff", ins_ds.BBOX, .33)
		dst_gt = dst_ds.GetGeoTransform()
		
		for Y in range(0, src_ds.RasterYSize, 1):
			print Y
			for X in range(0, src_ds.RasterXSize):
				x, y = self.transform_px(ins_ds, src_ds, X, Y)
				xout, yout = self.ground_to_output(dst_gt, x, y)
				#print X, Y, ' -> ', x, y, ' -> ' , xout, yout, src_image[Y][X]
				## WARNING this is Nearest Neighbor(ish)
				## xout and yout should be float and should be applied as
				## a sync (to the first zero) to the image
				## where the sync is taken and added to the pixels
				## pixels should be float and scaled to 0-255 before save
				dst_image[yout][xout] = src_image[Y][X]


		dst_ds.GetRasterBand(1).WriteArray( dst_image )


	def transform_px(self, ins_ds, src_ds, X, Y):
		# X, Y are source image pixel coordinates
		degrees_to_rad = 3.14159 / 180

		number_of_px   = src_ds.RasterXSize
		degrees_per_px = ins_ds.FOV / number_of_px

		# Note '-' sign it appears the sensor is 'backwards'
		lens_angle = - degrees_per_px * (float(X - (number_of_px / 2.0)))
		elevation  = ins_ds.Data[Y]['alt'] - 213.36 # Should get from elevation model

		rel_ground_x = math.sin(degrees_to_rad * (lens_angle + ins_ds.Data[Y]['roll'])) * elevation
		rel_ground_y = math.sin(degrees_to_rad * ins_ds.Data[Y]['pitch']) * elevation

		# Now rotate rel coordinates by Yaw into real ground coordinates
		# WARNING is YAW == HEADING or the 'error' from HEADING?

		yaw = degrees_to_rad * ins_ds.Data[Y]['yaw']
		yaw = degrees_to_rad * ins_ds.Data[Y]['heading'] + yaw
	
		ground_x =  ( rel_ground_x * math.cos(yaw)) + (rel_ground_y * math.sin(yaw))
		ground_y =  (-rel_ground_x * math.sin(yaw)) + (rel_ground_y * math.cos(yaw))

		x = ground_x + ins_ds.Data[Y]['x']
		y = ground_y + ins_ds.Data[Y]['y']

		return(x, y)

	def ground_to_output(self, gt, x, y):
		# Ground = GeoTransform * Image
		# Image  = 1/|GeoTransform| * GeoTransform^-1 * Ground

		invdet = gt[1]*gt[5] - gt[2]*gt[4] 
		xx     = x - gt[0]
		yy     = y - gt[3]
			
		X = int(math.floor( (gt[5] * xx - gt[2] * yy) / invdet ))
		Y = int(math.floor( (-gt[4] * xx + gt[1] * yy) / invdet ))

		return(X, Y)

	def createOutput(self, fname, bbox, meters_per_px=1):
		imgsize_x = int(math.ceil((bbox[2] - bbox[0]) / meters_per_px))
		imgsize_y = int(math.ceil((bbox[3] - bbox[1]) / meters_per_px))

		print imgsize_x, imgsize_y
	
		driver = osgeo.gdal.GetDriverByName("GTiff")
		dst_ds = driver.Create(fname, imgsize_x, imgsize_y)

		print 'Size is ',dst_ds.RasterXSize,'x',dst_ds.RasterYSize

		top_left = ( bbox[0], bbox[3]) # minx, maxy)

		dst_ds.SetGeoTransform( [ top_left[0], meters_per_px, 0, top_left[1], 0, -meters_per_px ] )

		srs = osgeo.osr.SpatialReference()
		srs.SetUTM( 15, 1 )
		srs.SetWellKnownGeogCS( 'WGS84' )
		dst_ds.SetProjection( srs.ExportToWkt() )

		raster = numpy.zeros( (imgsize_y, imgsize_x) )    
		#dst_ds.GetRasterBand(1).WriteArray( raster )	
		
		return(dst_ds, raster)

p = processArcher(sys.argv[1])
