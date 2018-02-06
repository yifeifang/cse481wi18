When running your code in simulation, you may want to have multiple point clouds at different places in the map.
This tutorial explains how to do so.

First, make sure that your saved point clouds have been transformed into the base_link frame.

However, instead of publishing the clouds in the base_link frame, you will publish them to imaginary frames with custom names, such as `shelf_1` or `delivery_spot`.

```py
cloud = camera.read_cloud(path) # cloud is a sensor_msgs.msg.PointCloud2
cloud.header.frame_id = 'shelf_1'
pub.publish(cloud)
```

Now you need to connect your imaginary frames to the map frame.
To do this, use [static_transform_publisher](http://wiki.ros.org/tf#static_transform_publisher):
```
rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms
```

E.g.,
```
rosrun tf static_transform_publisher 3 0 0 0 0 0 1 map shelf_1 100
```

You can launch `static_transform_publisher` from launch files and repeat this process for multiple point clouds:
```xml
<launch>
  <node pkg="tf" type="static_transform_publisher" name="shelf1_broadcaster" args="3 0 0 0 0 0 1 map shelf1 100" />
  <node pkg="tf" type="static_transform_publisher" name="delivery_spot_broadcaster" args="0 7 0 0 0 0 1 map delivery_spot 100" />
</launch>
```