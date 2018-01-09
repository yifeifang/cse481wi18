In this lab, we will see how to make web interfaces for controlling your robot.
For your projects, you may use whatever web programming framework you like.
But, these labs will use [Polymer](https://www.polymer-project.org/).

# Robot web tools tutorials
Go to the [Robot Web Tools](http://robotwebtools.org/tools.html) website.
Notice that each of the three libraries, roslibjs, ros2djs, and ros3djs, have tutorials.
Tutorial 1 ([Writing an actionlib client](http://wiki.ros.org/roslibjs/Tutorials/ActionlibClient)) and Tutorial 3 ([Basic ROS functionality](http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality)) of the [roslibjs tutorials](http://wiki.ros.org/roslibjs/Tutorials) will be especially helpful.

# Polymer tutorials
The idea of Polymer is to create custom HTML elements that can be built in a modular way and reused elsewhere.

In this lab, we will help you use Polymer, but you may find Polymer's own tutorials useful:
- [Quick tour](https://www.polymer-project.org/2.0/start/quick-tour)
- [Build an element](https://www.polymer-project.org/2.0/start/first-element/intro)
- [Build an application](https://www.polymer-project.org/2.0/start/toolbox/set-up)

# Initialize your frontend
You can store the code for your web frontend anywhere, but we recommend placing it in the `frontend` folder of the relevant ROS package:
```
cd web_teleop
mkdir frontend
cd frontend
polymer init
```

Select `polymer-2-application`.
Type `web-teleop` as the application name and accept the defaults for the other two options:
```
Application name web-teleop
Main element name (web-teleop-app)
Brief description of the application
```

# Add ROS web components
Look at the contents of the `frontend` folder.
The `bower_components` folder contains the other elements (aka components) that you are using in your application.
You should add this folder to a `.gitignore` file in the frontend folder:
```
cd frontend
echo bower_components/ > .gitignore
```

You can use web components developed by anyone in the world by adding it to your `bower.json` file.
You can also run `bower install --save GITHUB_ID/REPO_NAME` in your frontend folder.

You can find web components to use at `https://webcomponents.org`.
The ROS web components are located at: https://www.webcomponents.org/collection/jstnhuang/ros-element-collection

For this project, you will need `ros-websocket`, `ros-topic`, and `ros-action-client`.
Clicking on one of those will show you the documentation for that element.

To import them into your project:
```
cd frontend
bower install --save jstnhuang/ros-websocket
bower install --save jstnhuang/ros-topic
bower install --save jstnhuang/ros-action-client
```

# Using other frameworks
You can use [Robot Web Tools](http://wiki.ros.org/roslibjs/Tutorials) to interface with ROS using other web programming frameworks.
As of Winter 2018, the Robot Web Tools project has had issues with its CDN not working, so please contact the course staff if you have problems.