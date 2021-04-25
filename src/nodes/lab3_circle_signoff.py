import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
rospy.init_node("circle", anonymous = False)
pub = rospy.Publisher('/path_planner/a_star_planning', GridCells, queue_size = 10)
rospy.sleep(1)
print('making points')

coords = [[2,0],
    [3,0],
    [4,0],
    [5,0],
    [1,1],
    [2,1],
    [5,1],
    [6,1],
    [0,2],
    [1,2],
    [6,2],
    [7,2],
    [0,3],
    [7,3],
    [0,4],
    [7,4],
    [0,5],
    [1,5],
    [6,5],
    [7,5],
    [1,6],
    [2,6],
    [5,6],
    [6,6],
    [2,7],
    [3,7],
    [4,7],
    [5,7]]

pointCoords = []

for i in coords:
    p = Point()
    p.x = (i[0] + .5) * .3 - 5
    p.y = (i[1] + .5) * .3 - 5
    pointCoords.append(p)
  
msg = GridCells()
msg.cell_height = .3   #dims are equal to map resolution
msg.cell_width = .3
msg.cells = pointCoords
msg.header.frame_id = 'map'
pub.publish(msg)


print(len(coords))
