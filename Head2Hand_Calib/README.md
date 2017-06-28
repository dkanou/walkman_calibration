fsm_plugin
===============
FSM = (Start, MoveHand)

"Start" : initialization from keyboard which hand to start moving.
"MoveHand" : moves the hand every time the user uses the "next" command. First a set of movements are produced and in sequence the same movements are produced with some rotation at the wrist of that hand.

commands
===============
Starting:
$ roscore
$ gazebo (drop "IIT Bigman Ctrl" robot model in the world)
$ CommunicationHandler ~/advr-superbuild/configs/ADVR_shared/bigman/configs/config_walkman.yaml
$ NRTDeployer ~/advr-superbuild/configs/ADVR_shared/bigman/configs/config_walkman.yaml
$ XBotGUI ~/advr-superbuild/configs/ADVR_shared/bigman/configs/config_walkman.yaml

GUI (in PI):
> Start the HomingExample. After the robot is set to the homing position, stop it.
> Start in this sequence: IkRosSharedMemoryPublisher, OpenSotIk, ManipulationPlugin, Head2Hand_Calib

Head2Hand_Calib_cmd:
> rosservice call /Head2Hand_Calib_cmd "left" : choose left hand to move
> rosservice call /Head2Hand_Calib_cmd "right" : choose right hand to move
> rosservice call /Head2Hand_Calib_cmd "next" : proceed to the next movement
