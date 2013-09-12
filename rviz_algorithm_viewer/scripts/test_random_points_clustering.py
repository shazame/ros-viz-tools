#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_algorithm_viewer' )
import rospy
from rviz_algorithm_viewer.msg import Cluster2, ClusterField
import geometry_msgs.msg
import tf

import cv
import cv2
import numpy as np
import numpy.random as r

points_number = 1000
cluster_number = 20
range_size = 20
fps = 10

def get_points_label( pts ):
  retval, bestLabels, centers = cv2.kmeans(
      data=np.array(pts, np.float32),
      K=cluster_number,
      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 1, 10),
      attempts=1,
      flags=cv2.KMEANS_RANDOM_CENTERS)

  return bestLabels

def gen_cluster_msg( pts, labels ):
  clust_msg = Cluster2()
  clust_msg.header.frame_id = "/base_link"
  clust_msg.header.stamp = rospy.Time.now()

  # Creating the clusters
  cluster_nb = labels.max() + 1
  for i in range( cluster_nb ):
    cluster = ClusterField()
    cluster.name = str(i)
    clust_msg.clusters.append( cluster )

  # Adding the points to the right clusters
  points_nb = pts.shape[0]
  for i in range( points_nb ):
    point = geometry_msgs.msg.Point()
    point.x = pts[i][0]
    point.y = pts[i][1]
    point.z = pts[i][2]

    clust_msg.clusters[ labels[i] ].points.append( point )

  return clust_msg

def send_clusters():
  pub = rospy.Publisher( 'test_cluster', Cluster2 )
  rospy.init_node( 'test_cluster' )

  rospy.loginfo( 'Cluster test started. Sending cluster points...' )

  br = tf.TransformBroadcaster()

  while not rospy.is_shutdown():
    points = np.random.randn( points_number, 3 )
    points = points / 2.5 * range_size

    pub.publish( gen_cluster_msg( points, get_points_label( points ) ))

    br.sendTransform((0, 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "base_link",
        "map")

    rospy.sleep( 1. / fps )


if __name__ == '__main__':
  #try:
  send_clusters()
  #except rospy.ROSInterruptException:
  #  pass
