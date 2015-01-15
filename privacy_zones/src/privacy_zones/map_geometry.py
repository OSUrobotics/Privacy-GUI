import numpy as np
import cv2
import os
import yaml
from shapely.geometry import Point, LinearRing, GeometryCollection
from easy_markers.generator import MarkerGenerator
from visualization_msgs.msg import Marker, MarkerArray

class Zone(LinearRing):
    def __init__(self, zone_dict, geom):
        super(Zone, self).__init__([geom.px_to_world_coords((p['x'], p['y'])) for p in zone_dict['Points']])
        self.mode = zone_dict['Mode']
        self.name = zone_dict['Name']

        self.gen = MarkerGenerator()
        self.gen.ns = '/map_zones'
        self.gen.type = Marker.LINE_STRIP
        self.gen.scale = [0.3]*3
        self.gen.frame_id = 'map'


    def to_marker(self, color=(1,0,0,1)):
        self.gen.color = color
        marker = self.gen.marker(points=[(c[0], c[1], 0) for c in self.coords])
        marker.text = self.name
        return marker

class Zones(dict):
    def __init__(self, zone_list, geom):
        super(Zones, self).__init__([(z['Name'], Zone(z, geom)) for z in zone_list])

    def in_which(self, point):
        return [z for z in self.values() if z.contains(point)]

    def to_marker_array(self, colors=dict()):
        ma = MarkerArray(markers=[z.to_marker() for z in self.values()])
        for i, m in enumerate(ma.markers): m.id = i
        return ma

class MapGeometry:
    def __init__(self, yaml_path):
        self.yaml_path = yaml_path
        with open(yaml_path, 'r') as map_info_file:
            self.map_info = yaml.load(map_info_file)
        self.get_map_size()

    def get_map_size(self,):
        map_path = os.path.join(os.path.split(self.yaml_path)[0], self.map_info['image'])
        height, width, _ = cv2.imread(map_path).shape
        self.size = (0, height)

    def px_to_world_coords(self, px):
        return (self.size - np.array(px)) * (-1,1) * self.map_info['resolution'] + self.map_info['origin'][:2]

    def world_coords_to_px(self, coords):
        return (np.array(coords) - self.map_info['origin'][:2]) / self.map_info['resolution']

if __name__ == '__main__':
    import yaml
    mapInfoPath = '/home/lazewatd/wu_maps/jolley4.yaml'
    with open(mapInfoPath, 'r') as map_yaml:
        mapInfo = yaml.load(map_yaml)

    geom = MapGeometry(mapInfoPath)

    # px = (0,0)
    # world = geom.px_to_world_coords(px)
    # print '%s -> %s' % (str(px), str(world))
    # print '%s <- %s' % (str(geom.world_coords_to_px(world)), str(px))

    # px = (500,500)
    # world = geom.px_to_world_coords(px)
    # print '%s -> %s' % (str(px), str(world))
    # print '%s <- %s' % (str(geom.world_coords_to_px(world)), str(px))

    zones = yaml.load(open('/home/lazewatd/Dropbox/dev/wheelchair_ws/src/Privacy-GUI/privacy_zones/jolley4_zones.yaml', 'r'))
    z = Zones(zones['Zone List'], geom)
    # zz = Zone(z, geom)