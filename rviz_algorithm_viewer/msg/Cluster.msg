# This message corresponds to the general output of clustering
# algorithm: a list of points with the cluster they belong to.

# Time of data emission, coordinate frame ID.
Header header

# Array of 3D points with a label corresonding to the cluster they
# belong to. Each point should be interpreted as a 3D point in the frame
# given in the header.
ClusterPoint[] points
