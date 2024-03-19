import numpy as np
import math
import rospy
import tf
import copy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from scipy.spatial import KDTree

Freq=100
Flatarea=0.2

map_data=None
path_data=None
pathx=None
pathy=None

vMax = 1.8  # It can be changed
vMin = 0.8  # It can be changed
limitSteer = np.pi/6

startPoint = np.array([0.8, 0.8])
endPoint = np.array([11.3, 4.5])

Cm1=0.287
Cm2=0.0545
Cr0=0.0218
Cr2=0.00035
B_r = 3.3852
C_r = 1.2691
D_r = 0.1737
B_f = 2.579
C_f = 1.2
D_f = 0.192
m = 0.041
Iz = 27.8e-6
l_f = 0.029
l_r = 0.033
g = 9.8
Nf = m*g*l_r/(l_f+l_r)
Nr = m*g*l_f/(l_f+l_r)

global xC, throttle_delta

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ep = 0.0
        self.ei = 0.0
        self.ed = 0.0
        self.dt = 1/Freq
    def update_e(self, e):
        self.ed = e - self.ep
        self.ei += e
        self.ep = copy.deepcopy(e)       
    def get_uc(self):
        u = self.kp*self.ep+self.ki*self.ei+self.kd*self.ed
        if u > limitSteer:
            u = limitSteer        
        elif u < -limitSteer:
            u = -limitSteer
        return u
    def get_uv(self):
        u = self.kp*self.ep+self.ki*self.ei+self.kd*self.ed
        return u
    
crossPID = PID(100, 0, 1800)  # It can be changed
verticalPID = PID(5, 0, 2)

def update_model(xC, D, delta, dt):
    global hpath, path_pub

    def slope(xC):
        phi = xC[2]
        v_x = xC[3]
        v_y = xC[4]
        omega = xC[5]
        
        alpha_f = math.atan2((l_f*omega+v_y), abs(v_x)) - delta
        alpha_r = math.atan2((v_y-l_r*omega), abs(v_x))
        F_fy = D_f * math.sin(C_f * math.atan(-B_f * alpha_f))
        F_fx = -Cr0 * Nf - Cr2 * v_x * v_x
        F_ry = D_r * math.sin(C_r * math.atan(-B_r * alpha_r))
        F_rx = (Cm1*D-Cm2*D*v_x-Cr0*Nr-Cr2*v_x*v_x)

        dx = np.zeros(8)
        dx[0] = v_x*math.cos(phi) - v_y*math.sin(phi)
        dx[1] = v_y*math.cos(phi) + v_x*math.sin(phi)
        dx[2] = omega
        dx[3] = 1/m*(F_rx + F_fx*math.cos(delta) - F_fy*math.sin(delta) + m*v_y*omega)
        dx[4] = 1/m*(F_ry + F_fx*math.sin(delta) + F_fy*math.cos(delta) - m*v_x*omega)
        dx[5] = 1/Iz*(F_fx*math.sin(delta)*l_f + F_fy*l_f*math.cos(delta)- F_ry*l_r)
        dx[6] = v_x
        dx[7] = v_y

        return dx

    k1 = slope(xC)
    k2 = slope(xC + dt/2*k1)
    k3 = slope(xC + dt/2*k2)
    k4 = slope(xC + dt*k3)
    xC = xC + dt/6*(k1 + 2*k2 + 2*k3 + k4)

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = xC[0]
    pose.pose.position.y = xC[1]
    pose.pose.position.z = 0.0
    hpath.poses.append(pose)
    path_pub.publish(hpath)

    return xC

def map_cb(msg):
    global map_data
    if map_data is None:
        map_data = msg
        rospy.loginfo("Received map")
        k = 0
        map = np.zeros((map_data.info.width, map_data.info.height))
        # 将map_data.data转换为二维数组
        for j in range(map_data.info.height):
            for i in range(map_data.info.width):
                if map_data.data[k] == -1:
                    map[i, j] = 100
                else:
                    map[i, j] = map_data.data[k]
                k += 1
        # 以Flatarea为膨胀尺寸（以米为单位），结合地图分辨率，膨胀障碍物
        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i, j] == 100:
                    for k in range(int(Flatarea / map_data.info.resolution)):
                        if i + k < map.shape[0]:
                            map[i + k, j] = 100
                        if i - k >= 0:
                            map[i - k, j] = 100
                        if j + k < map.shape[1]:
                            map[i, j + k] = 100
                        if j - k >= 0:
                            map[i, j - k] = 100
        rospy.loginfo("Map processer finished")

def path_cb(msg):
    global path_data, pathx, pathy
    if path_data is None:
        path_data = msg
        rospy.loginfo("Received path")
        pathx = []
        pathy = []
        # 将path_data.poses转换为二维数组
        for i in range(len(path_data.poses)):
            pathx.append(path_data.poses[i].pose.position.x)
            pathy.append(path_data.poses[i].pose.position.y)
        rospy.loginfo("Path processer finished")

def follow_path(xC, realTime):
    global pathx, pathy
    leadingNum = int(len(pathx)/100)
    # 生成KDTree
    path = np.vstack((pathx, pathy)).T
    kdtree = KDTree(path)
    # 寻找最近点
    dist, idx = kdtree.query([xC[0], xC[1]])
    if idx < len(pathx)-leadingNum:
        p1 = path[int(idx+leadingNum/3)]
        p2 = path[idx+leadingNum]
    elif idx == len(path)-1:
        p1 = path[-2]
        p2 = path[-1]
        if xC[1] > endPoint[1]:
            rospy.loginfo("Final Time: %f", realTime)
            rospy.signal_shutdown("Car has been arrived!")
    else:
        p1 = path[idx]
        p2 = path[-1]
    # 计算前方路径弯曲程度，给出合适速度
    if idx + leadingNum < len(pathx):
        realLeading = leadingNum + 50
    else:
        realLeading = len(pathx) - idx - 1
    vFit = 0
    if realLeading > 50:
        # 计算前方路径的弯曲程度
        thetaSum = 0
        for i in range(1, leadingNum):
            x1 = pathx[idx+1]-pathx[idx]
            y1 = pathy[idx+1]-pathy[idx]
            x2 = pathx[idx+i+1]-pathx[idx]
            y2 = pathy[idx+i+1]-pathy[idx]
            if np.cross((x1, y1), (x2, y2)) < 0.0001:
                continue
            thetaSum += np.arccos((x1*x2+y1*y2)/(np.sqrt(x1**2+y1**2)*np.sqrt(x2**2+y2**2)))
        # rospy.loginfo("thetaSum: %f", thetaSum)
        vFit = vMax-(vMax-vMin)*thetaSum*1
        if vFit < vMin:
            vFit = vMin
    else:
        vFit = vMax
    # rospy.loginfo("vFit: %f", vFit)
    # 横向控制器
    def cross_track(xC):
        # 计算向量v1和v2
        v1 = [p2[0]-p1[0], p2[1]-p1[1]]
        v2 = [xC[0]-p1[0], xC[1]-p1[1]]

        # 计算投影向量
        proj_v2 = [v1[0]*v2[0]+v1[1]*v2[1]/(v1[0]**2+v1[1]**2)*v1[0],
                v1[0]*v2[0]+v1[1]*v2[1]/(v1[0]**2+v1[1]**2)*v1[1]]

        # 计算距离
        ey = abs((v2[0]-proj_v2[0])**2+(v2[1]-proj_v2[1])**2)**0.5

        # 计算方向
        cross_product = v1[0]*v2[1]-v1[1]*v2[0]
        if cross_product > 0:
            ey = -ey

        crossPID.update_e(ey)
        return crossPID.get_uc()
    
    def vertical_track(xC):
        v = (xC[3]**2+xC[4]**2)**0.5
        ey = vFit-v
        verticalPID.update_e(ey)
        return verticalPID.get_uv()
    
    # 计算油门和方向盘角度
    D = vertical_track(xC)
    delta = cross_track(xC)    
    return D, delta

# 辅助函数
def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


# 发布base_link坐标系
def publish_base_link(xC):
    br = tf.TransformBroadcaster()
    br.sendTransform((xC[0], xC[1], 0.0),
                     tf.transformations.quaternion_from_euler(0, 0, xC[2]),
                     rospy.Time.now(),
                     "base_link",
                     "odom")

if '__main__' == __name__:
    xC = np.zeros(8)
    throttle_delta = np.zeros(2)
    rospy.init_node('dynamic')
    rospy.Subscriber("/map", OccupancyGrid, map_cb)
    rospy.Subscriber("/path", Path, path_cb)
    path_pub=rospy.Publisher('/hpath', Path, queue_size=10)
    hpath = Path()
    hpath.header.frame_id = 'map'
    hpath.header.stamp = rospy.Time.now()
    rate = rospy.Rate(Freq)
    isStart = False
    realTime = 0
    while not rospy.is_shutdown():
        if map_data is not None and path_data is not None and pathx is not None and pathy is not None:
            if not isStart:
                xC[0] = pathx[0]
                xC[1] = pathy[0]
                xC[2] = np.arctan((pathy[1]-pathy[0])/(pathx[1]-pathx[0]))
                isStart = True
            throttle, delta = follow_path(xC, realTime)
            # rospy.loginfo("throttle: %f, delta: %f", throttle, delta)
            realTime += 1.0/Freq
            xC = update_model(xC, throttle, delta, 1.0/Freq)
            publish_base_link(xC)
        rate.sleep()
