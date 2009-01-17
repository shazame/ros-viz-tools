#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_algorithm_viewer' )
import rospy
from rviz_algorithm_viewer.msg import Cluster2, ClusterField
import geometry_msgs.msg
import random
import tf

def rand_cluster(n):
  clust_f = ClusterField()
  clust_f.name = "Cluster %d" % n

  points_nb = random.randint( 1, 30 )

  for i in range(points_nb):
    point = geometry_msgs.msg.Point()

    point.x = random.uniform( -20, 20 )
    point.y = random.uniform( -20, 20 )
    point.z = random.uniform( -20, 20 )

    clust_f.points.append( point )

  return clust_f

def gen_clusters():
  clust_msg = Cluster2()
  clust_msg.header.frame_id = "/base_link"
  clust_msg.header.stamp = rospy.Time.now()

  cluster_nb = random.randint( 1, 10 )

  for i in range(cluster_nb):
    clust_msg.clusters.append( rand_cluster(i) )

  return clust_msg

def send_clusters():
  pub = rospy.Publisher( 'test_cluster', Cluster2 )
  rospy.init_node( 'test_cluster' )

  rospy.loginfo( 'Cluster test started. Sending cluster points...' )

  br = tf.TransformBroadcaster()

  while not rospy.is_shutdown():
    pub.publish( gen_clusters() )

    br.sendTransform((0, 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "base_link",
        "map")

    rospy.sleep( 0.5 )

if __name__ == '__main__':
  try:
    send_clusters()
  except rospy.ROSInterruptException:
    pass
