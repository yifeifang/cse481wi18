# Start ROS
```
roscore
```

# Start the Fetch simulator
```
roslaunch fetch_gazebo playground.launch
```

The very first time you run the simulator on a new computer, it will take some time to start up.
If it doesn't appear to work after a minute or so, shut down the simulator with Ctrl-C and restart it.
It may take several tries.

You should see the following:
![image](https://cloud.githubusercontent.com/assets/1175286/24824318/18f2ebdc-1bbe-11e7-92aa-daf69c40bc35.png)

# Get the starter code
## Generate SSH keys
Generate a new SSH key pair if you don't already have one.
A blank password is fine:
```
ssh-keygen
```

## Set up SSH keys on Github
Go to your [Github SSH keys settings](https://github.com/settings/keys) and paste the contents of your ~/.ssh/id_rsa.pub into a new SSH key:
```
gedit ~/.ssh/id_rsa.pub
```

## Create a workspace
We recommend using [Catkin tools](https://catkin-tools.readthedocs.io/en/latest/), which is a replacement for `catkin_make` from the ROS tutorials.
Catkin tools is already installed on the course computers.

This code will migrate your existing Catkin workspace to use Catkin tools:
```
cd ~/catkin_ws
catkin init
catkin clean -bdy
catkin build
```

## Download the course starter code
```
cd ~/catkin_ws/src
git clone git@github.com:cse481wi18/cse481c.git
cd cse481c
```

Build your workspace:
```
catkin build
```

The build should complete successfully with this message, which appears whenever you build a new package for the first time:
```
[build] Note: Workspace packages have changed, please re-source setup files to use them.
```
This means that all of your current terminal windows will not be aware of this package (and the package name cannot be tab-completed).
The .bashrc files on the course computers are configured to re-source setup files, so you can just source your .bashrc:
```
source ~/.bashrc
```

Test that the package built successfully by running:
```
rosrun applications hello_world.py
```

You should see:
```
/hello_world main:8: Hello world!
```

# Save the starter code to your team's repository
You should have received an invitation to join the CSE 481C Github organization: https://github.com/cse481wi18.

You should keep your code and data backed up to the cloud, since our course computers have had some issues in the past.
Your team will back up your code to Github as part of the CSE 481C Github organization.

To do this, first change the url of the `origin` remote.
**Change teamNUMBER** to team1, team2, etc.:
```
git remote set-url origin git@github.com:cse481wi18/teamNUMBER.git
git push -u origin indigo-devel
```

Do this now.
Verify that your code has been pushed by visiting https://github.com/cse481wi18/teamNUMBER.
If you do not see your code, switch to the `indigo-devel` branch.
If possible, change default branch from `master` to `indigo-devel` and delete the `master` branch.