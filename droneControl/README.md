# running droneControl 
`roslaunch root_framework gcsRoot.launch`

(separate terminal)
`rosrun root_framework keyboardPress.py`

To control, click into the keyboardPress terminal.

t = takeoff/land
l = loiterMode
m = missionMode


## LoiterMode
Whilst in loiter mode, you can use the keyboard controls to move the the drone 
`wasd`  for XY moves
`zx`    for up/down
`qe`    for rotation

`h` - setpoint at 0,0,7.5m
`b` - fly directly below the powerline

## messageControl 
pilotHandler for all messages to Mavros Setpoint


## Adding a new pilot
To add a new pilot, you will need to:
droneCore = add a new state under _onKeyboardPress
messageControl = add a new subscriber with topic `/onboard/setpoint/{YourPilotHere}`

#