import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('odom_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    told = False
    while not rospy.is_shutdown():
        # 发布odom坐标系相对于map坐标系的静态变换，单位为米和弧度
        br.sendTransform((0.0, 0.0, 0.0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "/odom",
                         "/map")
        if not told:
            rospy.loginfo("Odom to Map transform published!")
            told = True
        rate.sleep()
