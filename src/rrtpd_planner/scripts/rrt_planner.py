# planner.py
# 用于实现路径规划

import rospy
import numpy as np
import copy
from scipy.spatial import distance
from scipy.interpolate import splev
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
np.set_printoptions(threshold = np.inf)
np.set_printoptions(suppress = True)
np.set_printoptions(linewidth=100)

# 定义全局变量
T = 20.0 # 轨迹时间长度（秒）
Flatarea = 0.15  # 障碍物膨胀距离（米）
Bstage = 3  # B样条阶数

# 定义节点类
class tree_node:
    def __init__(self, point):
        self.x = point[0]
        self.y = point[1]
        self.parent = None
        self.children = []
    def add_child(self, child):
        self.children.append(child)
        child.parent = self
    def delete_child(self, child):
        self.children.remove(child)
        child.parent = None
    def position(self):
        return (self.x, self.y)

# 计算两点之间的曼哈顿距离
def manhattan_dist(current, goal):
    return abs(current.x - goal.x) + abs(current.y - goal.y)

# 检查节点是否在地图范围内且可通过，并进行节点膨胀
def is_valid_node(map, node):
    rows = len(map)
    cols = len(map[0])
    if node[0] < 0 or node[0] >= rows or node[1] < 0 or node[1] >= cols:
        return False
    if map[node[0]][node[1]] >= 50:
        return False
    return True

# 检查两点之间是否有障碍物, 地图中100表示障碍物
def is_line_blocked(map, start, goal):
    if start[0] == goal[0] and start[1] == goal[1]:
        return False
    x1 = start[0]
    y1 = start[1]
    x2 = goal[0]
    y2 = goal[1]
    if abs(x1 - x2) >= abs(y1 - y2):
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
        for x in range(x1, x2+1):
            y = int(round(y1 + (y2 - y1) * (x - x1) / (x2 - x1)))
            if map[x][y] >= 50:
                return True
    else:
        if y1 > y2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
        for y in range(y1, y2+1):
            x = int(round(x1 + (x2 - x1) * (y - y1) / (y2 - y1)))
            if map[x][y] >= 50:
                return True
    return False

# 使用rrt算法求解路径
def rrt(start, goal, map):
    def is_goal(point):
        return distance.euclidean(point, goal) < 20
    
    rospy.loginfo("Waiting for path planning")
    # 初始化树
    root = tree_node(start)
    node_list = [root]
    # 在地图上随机撒点
    for i in range(100000):
        # 随机生成节点
        x = np.random.randint(0, map.shape[0])
        y = np.random.randint(0, map.shape[1])
        node = (x, y)
        # 检查节点是否有效
        if not is_valid_node(map, node):
            continue
        # 寻找最近节点（可优化）
        dist = []
        for j in range(len(node_list)):
            dist.append(distance.euclidean(node, node_list[j].position()))
        nearest_node = node_list[dist.index(min(dist))]
        # 判断是否有障碍物
        if is_line_blocked(map, nearest_node.position(), node):
            continue
        # 将节点加入树
        new_node = tree_node(node)
        node_list.append(new_node)
        nearest_node.add_child(new_node)
        # 判断是否到达目标点
        if is_goal(node):
            break
    # 从目标点回溯路径
    path = []
    path.append(goal)
    node = node_list[-1]    
    while node.parent != None:
        path.append(node.position())
        node = node.parent
    path.append(node.position())
    path.reverse()
    rospy.loginfo("Path planning finished")
    return path

# 点迹加密
def denser(path, n=1):
    oldpath = copy.deepcopy(path)
    for i in range(n):
        newpath = []
        for j in range(len(oldpath)-1):
            newpath.append(oldpath[j])
            newpath.append((int(oldpath[j][0]/2+oldpath[j+1][0]/2), int(oldpath[j][1]/2+oldpath[j+1][1]/2)))
        newpath.append(oldpath[-1])
        oldpath=newpath
    return newpath

# 单向缩减路径
def single_reduce(map, path):
    path_reduced = []
    path_reduced.append(path[0])
    while path_reduced[-1][0] != path[-1][0] or path_reduced[-1][1] != path[-1][1]:
        for i in range(len(path)-1, 0, -1):
            # 判断是否有障碍物
            if is_line_blocked(map, path_reduced[-1], path[i]):
                continue
            path_reduced.append(path[i])
            break
    return path_reduced

# B样条拟合
def bspline_fit(path):
    path = np.array(path)
    x = path[:, 0]
    y = path[:, 1]
    l=len(x)  

    t=np.linspace(0,1,l-Bstage+1,endpoint=True)
    t=np.append([0]*Bstage,t)
    t=np.append(t,[1]*Bstage)
    tck=[t,[x,y],Bstage]

    u3=np.linspace(0,1,(max(l*2,500)),endpoint=True)
    out = splev(u3,tck)
    
    newpath = np.vstack((out[0], out[1])).T
    return newpath

# 用于接收地图信息的回调函数
def map_callback(msg):
    rospy.loginfo("Received a /map message!")
    rospy.loginfo("Map size (w x h): %d x %d m=%d" % (msg.info.width,msg.info.height,msg.info.width*msg.info.height))
    rospy.loginfo("Origin: (%d, %d)" % (msg.info.origin.position.x,msg.info.origin.position.y))
    rospy.loginfo("Resolution: %f" % msg.info.resolution)
    global map_data
    map_data=msg
    if len(map_data.data) > 0:
        path_publish()

# 地图数据处理
def map_process():
    rospy.loginfo("Waiting for map processer")
    map=np.zeros((map_data.info.width,map_data.info.height),dtype=int)
    k=0
    for j in range(map_data.info.height):
        for i in range(map_data.info.width):
            if map_data.data[k]==-1 or map_data.data[k]>=50:
                map[i,j]=100
            else:
                map[i,j]=0
            k+=1
    # 以Flatarea为膨胀尺寸（以米为单位），结合地图分辨率，膨胀障碍物
    newmap=np.zeros((map.shape[0],map.shape[1]),dtype=int)
    newmap[0,0]=100
    newmap[map.shape[0]-1,0]=100
    newmap[0,map.shape[1]-1]=100
    newmap[map.shape[0]-1,map.shape[1]-1]=100
    for i in range(1,map.shape[0]-1, 4):
        for j in range(1,map.shape[1]-1, 4):
            if map[i,j]==100:
                newmap[i,j]=100
                if map[i+1,j]==0 and map[i-1,j]==0 and map[i,j+1]==0 and map[i,j-1]==0:
                    continue
                for k in range(int(Flatarea/map_data.info.resolution)):
                    rowDis = int(np.sqrt((Flatarea/map_data.info.resolution)**2-k**2))
                    for t in range(-rowDis, rowDis+1):
                        if j-k > 0 and j-k < map.shape[1] and i+t > 0 and i+t < map.shape[0]:
                            newmap[i+t, j-k] = 100
                        if j+k > 0 and j+k < map.shape[1] and i+t > 0 and i+t < map.shape[0]:
                            newmap[i+t, j+k] = 100
    
    rospy.loginfo("Map size %d x %d" % (map.shape[0],map.shape[1]))
    rospy.loginfo("Map processer finished")
    return newmap

# 加入中点
def add_midpoint(path):
    rospy.loginfo("Waiting for midpoint")
    path = np.array(path)
    path_new = []
    for i in range(len(path) - 1):
        path_new.append(path[i])
        path_new.append((int((path[i][0] + path[i + 1][0]) / 2), int((path[i][1] + path[i + 1][1]) / 2)))
    path_new.append(path[-1])
    rospy.loginfo("Midpoint finished")
    return path_new

# 发布路径信息
def path_publish():
    map=map_process()
    # 以米为单位的地图尺寸
    rospy.loginfo("Waiting for path publisher")
    path=Path()
    path.header.frame_id='map'
    path.header.stamp=rospy.Time.now()
    start=[int(0.8/map_data.info.resolution),int(0.8/map_data.info.resolution)]
    goal=[int(11.3/map_data.info.resolution),int(4.5/map_data.info.resolution)]
    rospy.loginfo("Start point: (%d, %d)" % (start[0], start[1]))
    if map[start[0],start[1]]==100 or map[goal[0],goal[1]]==100:
        if map[start[0],start[1]]==100:
            rospy.logerr("Start point is not accessible")
        if map[goal[0],goal[1]]==100:
            rospy.logerr("Goal point is not accessible")
        return
    
    start=tuple(start)
    goal=tuple(goal)
    path_list=rrt(start, goal, map)
    path_list=single_reduce(map, path_list)
    path_list=denser(path_list, 3)
    path_list=single_reduce(map, path_list)
    path_list=denser(path_list, 3)
    path_list=single_reduce(map, path_list)
    path_list=denser(path_list, 2)
    path_list=bspline_fit(path_list)

    for i in range(len(path_list)):
        rospy.loginfo("Path point %d: (%d, %d)" % (i, path_list[i][0], path_list[i][1]))
        pose=PoseStamped()
        pose.header.frame_id='map'
        pose.header.stamp=rospy.Time.now()
        pose.pose.position.x=path_list[i][0]*map_data.info.resolution+map_data.info.origin.position.x
        pose.pose.position.y=path_list[i][1]*map_data.info.resolution+map_data.info.origin.position.y
        pose.pose.position.z=0
        path.poses.append(pose)
    path_pub=rospy.Publisher('/path',Path,queue_size=10)
    rate=rospy.Rate(10)
    isfirst=False
    while not rospy.is_shutdown():
        path_pub.publish(path)
        rate.sleep()
        if not isfirst:
            rospy.loginfo("Path publisher are working")
            isfirst=True

if __name__ == '__main__':
    # 初始化map_listener节点
    rospy.init_node('map_listener')

    # 订阅/map信息
    rospy.Subscriber('/map',OccupancyGrid,map_callback)

    # 循环等待回调函数
    rospy.spin()
