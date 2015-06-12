import numpy as np
import cv2
import os
import yaml
from shapely.geometry import Point, Polygon, GeometryCollection
from easy_markers.generator import MarkerGenerator
from visualization_msgs.msg import Marker, MarkerArray
from privacy_zones.msg import Zone as ZoneMsg
from geometry_msgs.msg import Point32

class Zone(Polygon):
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
        marker = self.gen.marker(points=[(c[0], c[1], 0) for c in self.exterior.coords])
        marker.text = self.name
        return marker

    def to_msg(self):
        zone_msg = ZoneMsg(name=self.name)
        zone_msg.zone.points = [Point32(c[0], c[1], 0) for c in self.exterior.coords]
        return zone_msg

class Zones(dict):
    def __init__(self, zone_list, geom):
        super(Zones, self).__init__([(z['Name'], Zone(z, geom)) for z in zone_list])

    def in_which(self, point):
        return [z for z in self.values() if z.contains(point)]

    def to_marker_array(self, colors=dict()):
        ma = MarkerArray(markers=[z.to_marker(colors[n]) if n in colors else z.to_marker() for n, z in self.iteritems()])
        for i, m in enumerate(ma.markers): m.id = i
        return ma

class MapGeometry:
    def __init__(self, yaml_path=None, map_metadata=None):
        if yaml_path is not None:
            self.yaml_path = yaml_path
            with open(yaml_path, 'r') as map_info_file:
                self.map_info = yaml.load(map_info_file)
            self._get_map_size()
        if map_metadata is not None:
            self.map_info = dict(
                origin=[
                    map_metadata.origin.position.x,
                    map_metadata.origin.position.y,
                    map_metadata.origin.position.z
                ],
                resolution=map_metadata.resolution,
            )
            self._size = (map_metadata.width, map_metadata.height)

    def _get_map_size(self):
        map_path = os.path.join(os.path.split(self.yaml_path)[0], self.map_info['image'])
        height, width, _ = cv2.imread(map_path).shape
        self._size = (width, height)

    def get_map_dims(self):
        return np.abs(self.px_to_world_coords(self._size) - self.px_to_world_coords((0,0)))

    def px_to_world_coords(self, px):
        return ((0, self._size[1]) - np.array(px)) * (-1,1) * self.map_info['resolution'] + self.map_info['origin'][:2]

    def world_coords_to_px(self, coords):
        c = ((np.array(coords) - self.map_info['origin'][:2]) / self.map_info['resolution'])
        c[:,1] = self._size[1] - c[:,1]
        return c

if __name__ == '__main__':
    import yaml
    mapInfoPath = '/home/lazewatd/wu_maps/jolley4.yaml'
    with open(mapInfoPath, 'r') as map_yaml:
        mapInfo = yaml.load(map_yaml)

    geom = MapGeometry(mapInfoPath)

    px = (0,0)
    world = geom.px_to_world_coords(px)
    print '%s -> %s' % (str(px), str(world))
    print '%s <- %s' % (str(geom.world_coords_to_px(world)), str(px))
    print 'map size in meters:', geom.get_map_dims()

    # px = (500,500)
    # world = geom.px_to_world_coords(px)
    # print '%s -> %s' % (str(px), str(world))
    # print '%s <- %s' % (str(geom.world_coords_to_px(world)), str(px))

    # zones = yaml.load(open('/home/lazewatd/Dropbox/dev/wheelchair_ws/src/Privacy-GUI/privacy_zones/jolley4_zones.yaml', 'r'))
    # z = Zones(zones['Zone List'], geom)
    # zz = Zone(z, geom)