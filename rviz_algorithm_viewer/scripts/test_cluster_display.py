#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_algorithm_viewer' )
import rospy
from rviz_algorithm_viewer.msg import Cluster2, ClusterField
import geometry_msgs.msg
import random
import tf

cluster_radius = 5
max_cluster_pts = 50
max_clusters = 15
fps = 10

def rand_cluster(n, c_x, c_y, c_z):
  """x, y and z are the coordinates of the cluster center"""
  clust_f = ClusterField()
  clust_f.name = "Cluster %d" % n

  points_nb = random.randint( 1, max_cluster_pts )

  for i in range(points_nb):
    point = geometry_msgs.msg.Point()

    point.x = c_x + random.uniform( -1 * cluster_radius, cluster_radius )
    point.y = c_y + random.uniform( -1 * cluster_radius, cluster_radius )
    point.z = c_z + random.uniform( -1 * cluster_radius, cluster_radius )

    clust_f.points.append( point )

  return clust_f

def gen_clusters():
  clust_msg = Cluster2()
  clust_msg.header.frame_id = "/base_link"
  clust_msg.header.stamp = rospy.Time.now()

  cluster_nb = random.randint( 1, max_clusters )
  for i in range(cluster_nb):
    x = random.uniform( -20, 20 )
    y = random.uniform( -20, 20 )
    z = random.uniform( -20, 20 )
    clust_msg.clusters.append( rand_cluster(i, x, y, z) )

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

    rospy.sleep( 1. / fps )

if __name__ == '__main__':
  try:
    send_clusters()
  except rospy.ROSInterruptException:
    pass
