# This message is another interpretation of the general output of
# clustering algorithm: a list of clusters of points.

# Time of data emission, coordinate frame ID.
Header header

# Array of 3D points with a label corresonding to the cluster they
# belong to. Each point should be interpreted as a 3D point in the frame
# given in the header.
ClusterField[] clusters
