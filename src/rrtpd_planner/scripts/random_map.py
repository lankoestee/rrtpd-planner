import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import copy

rmap = OccupancyGrid()

def publish_map():

    def map_cb(msg):
        global rmap
        map_data = copy.deepcopy(msg)
        map=np.zeros((map_data.info.width,map_data.info.height),dtype=int)
        k=0
        for j in range(map_data.info.height):
            for i in range(map_data.info.width):
                if map_data.data[k]==-1 or map_data.data[k]>=50:
                    map[i,j]=100
                else:
                    map[i,j]=0
                k+=1
        successNum = 0
        while successNum < 5:
            rospy.loginfo("successNum: %d" % successNum)
            x = np.random.randint(0, map_data.info.width)
            y = np.random.randint(0, map_data.info.height)
            if map[x, y] != 0:
                continue
            isbreak = False
            for i in range(-12, 13):
                if map[x+i, y] != 0 or map[x, y+i] != 0:
                    isbreak = True
                    break
            if isbreak:
                continue
            successNum += 1
            for i in range(-12, 13):
                for j in range(-12, 13):
                    map[x+i, y+j] = 100
        k=0
        map_data.data=np.zeros(map_data.info.width*map_data.info.height,dtype=int)
        for j in range(map_data.info.height):
            for i in range(map_data.info.width):
                map_data.data[k]=map[i,j]
                k+=1
        rospy.loginfo("Map processer finished")
        
        rmap.header.frame_id = "map"
        rmap.info.resolution = map_data.info.resolution
        rmap.info.width = map_data.info.width
        rmap.info.height = map_data.info.height
        rmap.info.origin = map_data.info.origin
        rmap.data = map_data.data

    rospy.init_node('random_map')
    rate = rospy.Rate(1)
    map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
    rospy.Subscriber("/omap", OccupancyGrid, map_cb)

    while not rospy.is_shutdown():
        map_pub.publish(rmap)  # 发布地图消息
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_map()
    except rospy.ROSInterruptException:
        pass