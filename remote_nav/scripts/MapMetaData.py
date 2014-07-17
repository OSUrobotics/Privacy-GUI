#!/usr/bin/python
import yaml

class MapMetaData(yaml.YAMLObject):
	yaml_tag = u'!MapMetaData'

	def __init__(self, image, resolution, origin, negate, occupied_thresh, free_thresh):
		self.image = image
		self.resolution = resolution
		self.origin = origin
		self.negate = negate
		self.occupied_thresh = occupied_thresh
		self.free_thresh = free_thresh

def yaml_to_meta_data(file_name):
	# Open the file -- no error cehcking here
	fo = open(file_name)

	# Convert yaml -- no error checking here either
	file_text = fo.read()
	meta_data = yaml.load("--- !MapMetaData \n" + file_text)

	fo.close()

	# return MapMetaData object
	return meta_data