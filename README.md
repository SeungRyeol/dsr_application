# dsr_application

# *Package used*
https://github.com/doosan-robotics/doosan-robot

# *application list*

## - teleop_joy_dsr.py
Use the joystick to control each axis.
You can change it to your settings via joy_params.yaml. (In this example, we used an EX switch.)

1. A_BUTTON + B_BUTTON -> Go to target
2. HOME_BUTTON -> Go to home
3. L1_BUTTON + R1_BUTTON -> Save pos
4. Stick Axes -> Move jog

## - motion_to_yaml.py
Save the settings for the motion function as a yaml file.
Convert your settings to yaml using the joystick and keyboard.