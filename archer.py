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
import osgeo.gdal
from osgeo.gdalconst import *
import osgeo.gdalnumeric
import osgeo.osr
import pyproj
import math
import numpy
import ProcessArcher

class ArcherINS(object):
	def __init__(self, fname):
		self.filename = 0
		self.fileh = None
		self.rows = 0

		## Need to read this from .hdr file
		self.FOV = 40.15 # degrees
		self.RollBias = 0 # deg
		self.PitchBias = 0 # deg
		self.HeadingBias = 0 # deg
		self.LongitudeBias = 0 #deg
		self.LatitudeBias = 0 # deg
		self.AltitudeBias = 0 # meters
		self.Samples = 6144

		self.Data = ProcessArcher.ArcherINSData()
		self.Data.setParams(self.FOV, 0, 0, 0, 0, 0, 0, self.Samples)
		
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

				sequence = int(sequence)				# Line sequence number
				roll  = float(roll)  + math.radians(self.RollBias)   	# Absolute roll in radians
				pitch = float(pitch) + math.radians(self.PitchBias)  	# Absolute pitch in radians
				yaw   = math.radians(float(yaw) + self.HeadingBias)  	# Absolute yaw in radians
				lon   = float(lon)   + self.LongitudeBias            	# Camera position WGS84 degrees
				lat   = float(lat)   + self.LatitudeBias		# Camera position WGS84 degrees
				alt   = float(alt)   + self.AltitudeBias	        # Camera position WGS84 meters

				time  = float(time)					# Time in seconds since who knows when

				# Reproject from lat/lon to UTM15
				x, y  = pj(lon, lat)

				# Calculate crude bounding box
				if(x < minx):
					minx = x
				if(y < miny):
					miny = y
				if(x > maxx):
					maxx = x
				if(y > maxy):
					maxy = y
				if(x <> old_x and y <> old_y):	
					heading = (math.pi/2.0) - (math.atan2( y - old_y, x - old_x))
					#print 'Inst head = ', heading
					old_heading_idx = ( old_heading_idx + 1 ) % 100
					old_heading[old_heading_idx] = heading
					heading = old_heading.mean()
					#print 'Ave head = ', heading
			
				old_x = x
				old_y = y

				self.Data.append(sequence, roll, pitch, yaw, lon, lat, alt, time, x, y, heading)			
				self.rows = self.rows + 1
			# Silly buffer to approx bbox otherwise is just range of centerpoints
			# Now done in C++: self.Data.set_bbox(minx - 1000, miny - 1000, maxx + 1000, maxy + 1000)


class processArcher(object):
	def __init__(self, basename):
		ins_ds = ArcherINS(basename+'.txt')

		pA = ProcessArcher.ProcessArcher(basename+'.pan', basename+'.tiff', ins_ds.Data)
		pA.run(0.1524) # 6"


p = processArcher(sys.argv[1])
