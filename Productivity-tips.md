# Workspaces
Workspaces are like virtual desktops.
It can help you to stay organized when you have many windows open.

Search for "Unity Tweak Tool" in the Ubuntu software center and install it.
Click on "Workspace settings" and then set the number of horizontal workspaces to 4 and the number of vertical workspaces to 1.

Now, pressing Ctrl-RIGHT will move you to the workspace on the right, and similarly for Ctrl-LEFT.
Pressing Ctrl-Shift-RIGHT will pick up the current window and move it to the workspace on the right.

# tmux
Programming in ROS involves opening up a lot of terminal windows.
You should use [tmux](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/tmux.md) or screen to manage them.
tmux allows you to have multiple terminals in a single terminal window, which can help you to stay organized.

tmux is especially helpful when SSH-ing into a robot.
This will allow you to SSH once into a computer and open many terminals at once within that one SSH session.
Additionally, if your SSH connection dies to the spotty wifi, your tmux session will survive and you can get right back to work once you reconnect.