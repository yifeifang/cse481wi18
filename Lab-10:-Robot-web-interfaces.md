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

You can find web components to use at https://webcomponents.org.
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

# Web application structure
- **index.html**: Sets up the webpage to use web components. It is a simple webpage that imports your "main" web component, `web-teleop-app`.
- **src/**: Here are where you place your elements.
- **bower.json**: Lists the elements you will want to import into your project. When other developers clone your code, they can run `bower update` to make `bower` download all the needed elements and save them into the `bower_components/` folder.
- **bower_components/**: All the elements you are using, not written by you, are saved here. This should not be checked into source control. Instead, developers will run `bower update`.

We recommend flattening the structure of the `src` folder:
```
cd frontend
mv src/web-teleop-app/web-teleop-app.html src/web-teleop-app.html
rm -r src/web-teleop-app
```

Next, edit `index.html` and change the import path:
```diff
- <link rel="import" href="/src/web-teleop-app/web-teleop-app.html">
+ <link rel="import" href="/src/web-teleop-app.html">
```

Edit `src/web-teleop-app.html` to change the import path:
```diff
- <link rel="import" href="../../bower_components/polymer/polymer-element.html">
+ <link rel="import" href="../bower_components/polymer/polymer-element.html">
```

# Build and development cycle
To see your website, run `polymer serve` from the frontend folder.
Then, visit the URL given back to you in a web browser.

By default, this serves the website on `localhost`, meaning that you have to view the webpage on the same computer.
If you are developing remotely (such as via SSH), then specify the hostname of the computer you are developing on:
```
polymer serve --hostname HOSTNAME
```
Now, on your own web browser, visit HOSTNAME:8080 (or whatever port your webpage is being served on).

After making edits, you can simply refresh the web browser to see your updated site.

# Connect to the websocket server
Edit `src/web-teleop-app.html`.
First, import the `<ros-websocket>` element :

```diff
<link rel="import" href="../bower_components/polymer/polymer-element.html">
+ <link rel="import" href="../bower_components/ros-websocket/ros-websocket.html">
```

Add the `<ros-websocket>` element and a title to your app:
```html
<ros-websocket auto ros="{{ros}}"
  url="{{url}}"
  on-connection="_handleConnection"
  on-close="_handleClose"
  on-error="_handleError"></ros-websocket>

<h1>Fetch teleop</h1>
```

Look at the [documentation](https://www.webcomponents.org/element/jstnhuang/ros-websocket/elements/ros-websocket) for `<ros-websocket>` to learn more about the element and its API.

- **auto**: Adding this boolean property means that the element will try to establish the websocket connection immediately.
- **ros**: After the connection is made, the element will store a data structure representing the connection to a variable. We store this in a variable called `ros` using the double curly-brace notation.
- **url**: This property gives the URL of the websocket server. The websocket server's URL is inferred from the URL of the webpage and saved into the `url` variable using the double curly-brace notation. You can override it with a different URL by assigning a new URL to the `url` variable.
- **on-EVENT**: When elements fire events, you can handle the events by adding `on-EVENTNAME` attributes and supplying the function names of the handlers.

Now, add the handlers:

```js
static get properties() {
  return {};     
}        
               
_handleConnection() {
  this.status = 'Connected to the websocket server.';
  console.log(this.status);
}
_handleClose() {
  this.status = 'Closed connection to the websocket server.';
  console.log(this.status);
}
_handleError() {
  this.status = 'Error connecting to the websocket server.';
  console.log(this.status);
}
```

And display the status in the HTML section of your element:
```html
<h1>Fetch teleop</h1>
{{status}}
```

## Test the connection
If your backend is running, then when you refresh the page, you should see "Connected to the websocket server."
If your backend is not running, it should say "Closed connection to the websocket server."
If you or your backend team has not reached the end of Lab 9 yet, then you can run:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

Now, the app should connect to the websocket server after you reload the page.

# Display the torso height

# Send a new torso height

# Camera image

# Using other frameworks
You can use [Robot Web Tools](http://wiki.ros.org/roslibjs/Tutorials) to interface with ROS using other web programming frameworks.
As of Winter 2018, the Robot Web Tools project has had issues with its CDN not working, so please contact the course staff if you have problems.