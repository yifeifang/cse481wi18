You can use some basic text to speech software on the robot and play sounds.
**Note:** the robot's speaker is not very loud.

[sound_play](http://wiki.ros.org/sound_play) provides an API for playing sounds.
We have provided a wrapper around it in `fetch_api/src/fetch_api/sound.py`

# Run the demo
```
rosrun sound_play soundplay_node.py
rosrun applications sound_demo.py
```

# API
Make sure you are running `sound_play`. In simulation, you will need to run this yourself. On the real robot, 
Create the sound object:
```py
import fetch_api
sound = fetch_api.RobotSound()
```

To say something:
```py
sound.say('Hello world')
```

To play a sound:
```py
robot_sound.play_sound('E04.wav')
```

The sounds are stored as WAV files in the `fetch_api/sounds` directory.
We have included several sounds from the [Willow Garage sound library](https://github.com/aramadia/willow-sound).
Feel free to download the whole set and try other sounds.