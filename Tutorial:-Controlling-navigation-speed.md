The navigation speed of the robot is specified through a couple of launch files.
In order to modify these launch files, you will need to create a copy of them and run those copies instead.

# Make copies of the launch files
In addition to copying the main `fetch_nav.launch` file, we need to copy an additional file that is included by `fetch_nav.launch`.
You can see what files are included in `fetch_nav.launch` by running `rosed fetch_navigation fetch_nav.launch`.

```bash
cd fetch_api/launch
roscp fetch_navigation fetch_nav.launch .
roscp fetch_navigation move_base.launch.xml .
ls
```

You should see two new files: fetch_nav.launch and move_base.launch.xml.

# Make a copy of the config files
If you look at `move_base.launch.xml`, you will see that loads two configuration files related to the `move_base` node: `move_base.yaml` and `fetch/move_base.yaml`.

Create a folder named `config` if it doesn't exist and copy these two files:
```bash
cd fetch_api/config
```

Both files are named `move_base.yaml`, so `roscp` will ask you which one to copy.
This command copies the main `move_base.yaml` (double check the numbers, which may be different for you):
```bash
> roscp fetch_navigation move_base.yaml .
You have chosen a non-unique filename, please pick one of the following:
1) /opt/ros/indigo/share/fetch_navigation/config/freight/move_base.yaml
2) /opt/ros/indigo/share/fetch_navigation/config/move_base.yaml
3) /opt/ros/indigo/share/fetch_navigation/config/fetch/move_base.yaml
#? 2
/opt/ros/indigo/share/fetch_navigation/config/move_base.yaml
```

This command copies the Fetch-specific configuration:
```bash
> roscp fetch_navigation move_base.yaml fetch_move_base.yaml
You have chosen a non-unique filename, please pick one of the following:
1) /opt/ros/indigo/share/fetch_navigation/config/freight/move_base.yaml
2) /opt/ros/indigo/share/fetch_navigation/config/move_base.yaml
3) /opt/ros/indigo/share/fetch_navigation/config/fetch/move_base.yaml
#? 3
/opt/ros/indigo/share/fetch_navigation/config/fetch/move_base.yaml
```

# Edit the launch files
To run the modified versions of the launch files, you will need to start navigation using this command from now on:
```bash
roslaunch fetch_api fetch_nav.launch
```

Also be sure to edit any launch files you have that include `fetch_nav.launch`:
```xml
<include file="$(find fetch_api)/launch/fetch_nav.launch">
```

However, you also need to edit your copy of `fetch_nav.launch` to use your copy of `move_base.launch.xml` and the two `move_base.yaml`s.

Edit `fetch_nav.launch` and replace:
```xml
<arg name="move_base_include" default="$(find fetch_navigation)/launch/include/move_base.launch.xml" />
```
with:
```xml
<arg name="move_base_include" default="$(find fetch_api)/launch/move_base.launch.xml" />
```

Edit `move_base.launch.xml` and replace:
```xml
<rosparam file="$(find fetch_navigation)/config/move_base.yaml" command="load" />
<rosparam file="$(find fetch_navigation)/config/$(arg name)/move_base.yaml" command="load" />
```
with (notice the three differences on the two lines):
```xml
<rosparam file="$(find fetch_api)/config/move_base.yaml" command="load" />
<rosparam file="$(find fetch_api)/config/fetch_move_base.yaml" command="load" />
```

# Edit the configuration
To limit the robot's speed, edit `fetch_move_base.yaml` and set the `max_vel_x` parameter from 1.0 to something slower, like 0.3.
`max_vel_x` is in meters/second, `{min,max}_vel_theta` are in radians/second.

You can also experiment with the tolerances in `move_base.yaml` (`yaw_goal_tolerance` and `xy_goal_tolerance`).
However, we have not tested how tight of tolerances the robot can actually achieve.

You can refer to the course repository for a working example of the launch/config files and the edits we made.