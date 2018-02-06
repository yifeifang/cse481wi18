Now that we can segment horizontal surfaces from our scene, we will see how to segment objects.
To do so, we will use the [Euclidean Clustering](http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction) algorithm, as described in the PCL tutorials.
Euclidean clustering essentially groups points that are close together.
You must set a "closeness" threshold, such that points within this threshold are considered to be part of the same cluster.
Another useful feature is that you can set a minimum and a maximum number of points that a cluster can contain.
This helps to filter out noise (isolated single points) or parts of the surface that weren't segmented out.

We will be modifying our existing `segmentation.cpp` and `segmentation.h`.

# Crop the scene
Reinstate the cropper and do the segmentation on the cropped scene, so that the wall is cut off:
After tweaking some of the parameters, this is what I get:
![image](https://cloud.githubusercontent.com/assets/1175286/25830838/b2ab228a-3414-11e7-8522-31a183402eb6.png)

Another way to get rid of the wall is to do planar segmentation again, but this time for a plane perpendicular to the X (with a greater tolerance to account for the fact that the view is skewed somewhat).
However, usually, you will just command the robot to navigate to a pose that is perpendicular to any walls and you can crop them out easily.

# Add object segmentation function
Add this function to `segmentation.h` and `segmentation.cpp`.
Notice that we are representing objects not as miniature point clouds, but as `PointIndices` in the original point cloud.
This technique is more efficient.
It also gives you more flexibility, since it's always possible to reify the point cloud at a later time.

```cpp
void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices);
```

# Extract the scene above the plane
Now, we will implement `SegmentSurfaceObjects`.
First, we will extract the scene above the plane.
To do this, we will use `pcl::ExtractIndices` again, but this time calling `setNegative(true)`.

```cpp
pcl::ExtractIndices<PointC> extract;
pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
extract.setInputCloud(cloud);
extract.setIndices(surface_indices);
extract.setNegative(true);
extract.filter(above_surface_indices->indices);

ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());
```

In the above code, we are getting the indices of the points above the surface.
To do this, we use `setNegative(true)`, which gives us the opposite set of indices from `surface_indices`.
The `filter` method is overloaded so that it can output indices or a point cloud, which you can figure out from carefully looking at the inherited public methods of [pcl::ExtractIndices](http://docs.pointclouds.org/1.7.1/classpcl_1_1_extract_indices.html).

In your `Segmenter::Callback`, call `SegmentSurfaceObjects`:
```cpp
std::vector<pcl::PointIndices> object_indices;
SegmentSurfaceObjects(cloud, table_inliers, &object_indices);
```

Try building and running your code.
You should see it report a non-zero number of points above the surface.

# Do the clustering
Now, in `SegmentSurfaceObjects`, you can implement the actual clustering:
```cpp
double cluster_tolerance;
int min_cluster_size, max_cluster_size;
ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

pcl::EuclideanClusterExtraction<PointC> euclid;
euclid.setInputCloud(cloud);
euclid.setIndices(above_surface_indices);
euclid.setClusterTolerance(cluster_tolerance);
euclid.setMinClusterSize(min_cluster_size);
euclid.setMaxClusterSize(max_cluster_size);
euclid.extract(*object_indices);

// Find the size of the smallest and the largest object,
// where size = number of points in the cluster
size_t min_size = std::numeric_limits<size_t>::max();
size_t max_size = std::numeric_limits<size_t>::min();
for (size_t i = 0; i < object_indices->size(); ++i) {
  // TODO: implement this
  size_t cluster_size = ???;
}

ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
         object_indices->size(), min_size, max_size);
```

In the for loop, implement some code to figure out how many points are in the smallest cluster and how many points are in the largest cluster.
This will be useful for tweaking your `ec_min_cluster_size` and `ec_max_cluster_size` params later.

# Visualize the results
First, let's visualize the point cloud above the surface.
After publishing the table marker in `Segmenter::Callback`, reify the point cloud above the table and publish it.
In this code snippet, we are publishing to a third publisher passed into Segmenter called `above_surface_pub_`, but you can repurpose your `surface_points_pub_` if you want, just remember to uncomment its usage earlier.
```cpp
// We are reusing the extract object created earlier in the callback.
extract.setNegative(true);
extract.filter(*cloud_out);
pcl::toROSMsg(*cloud_out, msg_out);
above_surface_pub_.publish(msg_out);
```

By viewing only this point cloud in RViz, you can see what input you are giving to the Euclidean clustering algorithm:
![image](https://cloud.githubusercontent.com/assets/1175286/25832474/9c979b4e-341f-11e7-8933-270363d66cf8.png)

Now, let's draw boxes around each of the objects.

```cpp
for (size_t i = 0; i < object_indices.size(); ++i) {
  // Reify indices into a point cloud of the object.
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  *indices = object_indices[i];
  PointCloudC::Ptr object_cloud(new PointCloudC());
  // TODO: fill in object_cloud using indices

  // Publish a bounding box around it.
  visualization_msgs::Marker object_marker;
  object_marker.ns = "objects";
  object_marker.id = i;
  object_marker.header.frame_id = "base_link";
  object_marker.type = visualization_msgs::Marker::CUBE;
  GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                            &object_marker.scale);
  object_marker.color.g = 1;
  object_marker.color.a = 0.3;
  marker_pub_.publish(object_marker);
}
```

Note that we have to create a unique ID for each object.

You should see something like this:
![image](https://cloud.githubusercontent.com/assets/1175286/25832677/d5d135d6-3420-11e7-8207-81b67eee9fd3.png)

Some of the objects are actually just noise in the data.
Luckily, you can filter them out by increasing the minimum size of a cluster:
```
rosparam set ec_min_cluster_size 20
```

One issue that will occur is that if you change a parameter and the number of objects decreases, the box for the objects that disappeared will still be there.
This is because RViz markers are "added" to the scene and are not deleted unless you specify `action=DELETE`.
You can auto-delete them by setting the `lifetime` field on the marker, or you can refresh the RViz display by toggling the Marker display off and on.

As you adjust the minimum cluster size, you can look at the output of your node to see what the smallest cluster size is:
```
/point_cloud_demo SegmentSurfaceObjects:102: Found 10 objects, min size: 11, max size: 5306
# Adjusted ec_min_cluster_size to 20
/point_cloud_demo SegmentSurfaceObjects:102: Found 7 objects, min size: 27, max size: 5306
```

If you have objects that are close together, you can experiment with adjusting the `ec_cluster_tolerance` param:
```
rosparam set ec_cluster_tolerance 0.05
```

The cup, which was once in two parts, is now joined into one cluster.
However, it incorrectly joins other objects.
This is a limitation of the Euclidean clustering algorithm.
![image](https://cloud.githubusercontent.com/assets/1175286/25832754/6d8dcc86-3421-11e7-87b6-412e2a60e76a.png)

As you work on your projects, you should think about what perception challenges you have.
Although the segmentation pipeline presented in this lab could be useful, you may also need to make tweaks or design your own algorithm.
For example, if you know that "floating" objects are unlikely, you can merge a "floating" cluster with the nearest cluster that reaches the surface level.
Or, perhaps you don't have a tabletop at all and are actually looking for a something floating in the air (like a beer tap).
Then, you will need to do Euclidean clustering directly on a cropped point cloud, without searching for a surface first.

# Color and geometry-based clustering
Euclidean clustering works only if there is a gap in between the objects (as specified by `ec_cluster_tolerance`).
If there is no gap between objects (e.g., two books placed next to each other in a shelf), then you may need to use a more advanced clustering algorithm.

[Region-growing segmentation](http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php#region-growing-segmentation) and [color-based region-growing segmentation](http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php#region-growing-rgb-segmentation) both work similarly to Euclidean cluster extraction.
However, they also give you the option to specify thresholds on the smoothness of the normals or on the average color.
For example, the color-based region-growing segmentation can be used to segment two differently-colored books placed against each other.