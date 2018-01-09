# Course computers
All of the course computers have ROS Indigo and Fetch software installed.
You are free to install other software on these computers.
You should keep the computers up to date with software updates.

**Important**: Keep all of your code backed up on Github or elsewhere.
Some of the course computers have had trouble booting in the past.

# Working remotely
Because we only have a limited number of course computers, you may want to bring your own laptop to class.
ROS Indigo can only be installed on Ubuntu Linux 14.04.
You have a few options for working with your own computers:
* SSH
* TeamViewer
* Virtual machine
* Dual-boot

## SSH: Secure shell
SSH will be the most common way to access the course computers and the robot.
If you have a Mac, you can run SSH from the Terminal app.
If you have Windows, you can install the [Chrome SSH extension](https://chrome.google.com/webstore/detail/secure-shell/pnhechapfaindjhompbnflcldabbghjo?hl=en) or use PuTTY.

To SSH into a course computer, run:
```
ssh teamN@HOSTNAME
```
Where teamN is team1, team2, etc. and HOSTNAME is the name of your computer (roomba, unimate, silverarm, etc.)

If you are not on CSE-Local or are working from home, run:
```
ssh teamN@HOSTNAME.cs.washington.edu
```

Typing in your password may get tiresome.
To set up passwordless authentication, generate a pair of SSH keys and copy them to your team's computer:
```
ssh-keygen # Press Enter to accept defaults, password is optional
ssh-copy-id teamN@HOSTNAME
```

Even without having to type your password, it may be annoying to open multiple SSH windows.
You can use tools like [tmux](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/tmux.md) to maintain multiple virtual terminals within a single SSH session.

* [More documentation on SSH](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/ssh.md)

Note that using SSH, you will not be able to access any GUIs.

## TeamViewer
You may install TeamViewer, which is remote desktop software that allows you to see GUIs and control the course computers.

## Virtual machines
You can install Ubuntu 14.04 on a virtual machine, such as VirtualBox or VMWare.
CSE offers free software here: https://www.cs.washington.edu/lab/software

If you are using a VM and are having trouble getting Gazebo to load, try upgrading OpenGL and forcing software rendering:
```
sudo apt-get install libgl1-mesa-dri-lts-trusty
export LIBGL_ALWAYS_SOFTWARE=1
roslaunch fetch_gazebo playground.launch
```

At this point, Gazebo will run but it may be quite slow.
If your computer has a good graphics card, try enabling 3D acceleration in your VM settings and unsetting LIBGL_ALWAYS_SOFTWARE=1:

```
unset LIBGL_ALWAYS_SOFTWARE
```

If Gazebo works, then you are good to go! If it only works with LIBGL_ALWAYS_SOFTWARE=1, then add this to your .bashrc:
```
export LIBGL_ALWAYS_SOFTWARE=1
```

## Dual-boot
You can install Ubuntu 14.04 on your own computer.
You can search online for how to do this or ask the course staff for help.

## Ubuntu 14.04 setup
If you have installed Ubuntu 14.04 via dual-booting or a virtual machine, follow the [course software installation instructions](https://github.com/cse481wi18/cse481wi18/tree/indigo-devel/course_setup).