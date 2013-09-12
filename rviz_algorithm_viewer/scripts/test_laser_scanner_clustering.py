#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_algorithm_viewer' )
import rospy
from rviz_algorithm_viewer.msg import Cluster2, ClusterField
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg
import tf; br = tf.TransformBroadcaster()
pub = rospy.Publisher( 'test_cluster', Cluster2 )

import cv
import cv2
import numpy as np
import math

cluster_number = 10

def get_points_label( pts ):
  retval, bestLabels, centers = cv2.kmeans(
      data=np.array(pts, np.float32),
      K=cluster_number,
      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 1, 10),
      attempts=1,
      flags=cv2.KMEANS_PP_CENTERS)
      #flags=cv2.KMEANS_RANDOM_CENTERS)

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

def send_clusters( points ):
  pub.publish( gen_cluster_msg( points, get_points_label( points ) ))

  br.sendTransform((0, 0, 0),
      tf.transformations.quaternion_from_euler(0, 0, 0),
      rospy.Time.now(),
      "base_link",
      "map")

def get_laser_scan_pts( msg ):
  i = 0;

  points = []
  theta = msg.angle_min
  #for theta in np.arange(, , ):
  while theta < ( msg.angle_max - msg.angle_increment/2 ):
    if i < len( msg.ranges ):
      r = msg.ranges[i];
      x = r * math.cos(theta);
      y = r * math.sin(theta);

      points.append([ x, y, 0 ])

      theta += msg.angle_increment
      i += 1

  send_clusters( np.asarray( points ))

def listen_and_send_clusters():
  rospy.init_node( 'test_cluster' )
  rospy.Subscriber( "scan", LaserScan, get_laser_scan_pts )
  rospy.loginfo( 'Cluster test started. Reading points from scan /topic and sending cluster points...' )
  rospy.spin()

if __name__ == '__main__':
  try:
    listen_and_send_clusters()
  except rospy.ROSInterruptException:
    pass
