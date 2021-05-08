import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud

rospy.init_node("test")
pub = rospy.Publisher('/centroids', PointCloud, queue_size=10)

rospy.sleep(1)

p = PointCloud()

p1 = Point(232,207,0)
p.points.append(p1)
p.header.frame_id = 'map'

pub.publish(p)


