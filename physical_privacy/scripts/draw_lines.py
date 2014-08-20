#!/usr/bin/env python
import rospy
import yaml
import rospkg
import pprint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

neutral = ColorRGBA()
neutral.a = 0.6
neutral.r = 1.0
neutral.g = 1.0
neutral.b = 1.0

private = ColorRGBA()
private.a = 1.0
private.r = 1.0
private.g = 0.0
private.b = 0.0

public = ColorRGBA()
public.a = 1.0
public.g = 1.0

color_map = {
    0: neutral,
    1: private,
    2: public
}

global_frame = "/map"


if __name__ == '__main__':
    rospy.init_node('draw_lines')
    # We use rospack to find the filepath for physical_privacy, lest the user does not roslaunch.
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('physical_privacy')

    #Our zones are stored in a .yaml file.
    zones_file = rospy.get_param('draw_lines/zones', package_path + "/zones/test.yaml")
    f = open(zones_file)
    zones = yaml.safe_load(f)

    #A nice pretty printer for debuggin.
    pp = pprint.PrettyPrinter(indent=4)

    #Publishes a MarkerArray for zone displaying in rviz.
    pub = rospy.Publisher("/zones", MarkerArray)
    lines_array = MarkerArray()


    i = 0
    for zone in zones['Zone List']:
        line = Marker()
        line.header.stamp = rospy.Time.now()
        #TODO: Must find solidarity in frame_id
        line.header.frame_id = global_frame
        line.ns = "physical_privacy"
        line.id = i

        line.type = line.LINE_STRIP
        line.action = line.ADD

        line.pose.orientation.w = 1.0

        line.scale.x = 0.1

        line.color = color_map[zone['Mode']]

        line.lifetime = rospy.Duration(0)

        for point in zone['Points']:
            new_point = Point()
            new_point.x = point['x'] 
            new_point.y = point['y']
            line.points.append(new_point)
        point = Point()

        #We must loop around to close our path.
        point.x = zone['Points'][0]['x'] 
        point.y = zone['Points'][0]['y']
        line.points.append(point)

        #make a text label this line.
        text = Marker()
        text.text = zone['Name']
        text.header.frame_id = global_frame
        text.ns = "labels"
        text.id = i
        text.type = text.TEXT_VIEW_FACING
        text.action = text.ADD
        text.pose.position.x = point.x
        text.pose.position.y = point.y
        text.pose.orientation.w = 1.0
        text.scale.z = 0.5
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.lifetime = rospy.Duration(0)
        
        i += 1

        lines_array.markers.append(line)
        lines_array.markers.append(text)
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        pub.publish(lines_array)
        r.sleep()

    rospy.spin()
