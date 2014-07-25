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

class RegisteredMapMetaData(yaml.YAMLObject):
	yaml_tag = u'!RegisteredMapMetaData'

	def __init__(self, semantic_map, slam_map, origin, resolution, slam_width, slam_height, semantic_width, semantic_height, semantic_to_slam, slam_to_semantic, objects):
		self.slam_map = slam_map
		self.semantic_map = semantic_map
		self.resolution = resolution
		self.origin = origin
		self.slam_width = slam_width
		self.slam_height = slam_height
		self.semantic_height = semantic_height
		self.semantic_width = semantic_width
		self.slam_to_semantic = slam_to_semantic
		self.semantic_to_slam = semantic_to_slam

def yaml_to_meta_data(file_name):
	# Open the file -- no error cehcking here
	fo = open(file_name)

	# Convert yaml -- no error checking here either
	file_text = fo.read()
	meta_data = yaml.load("--- !MapMetaData \n" + file_text)

	fo.close()

	# return MapMetaData object
	return meta_data