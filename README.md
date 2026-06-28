# The Advanced SimRacing SimulatorFramework
A more advanced car controller for the Godot game engine.

## Description
Custom rigidbody car controller with raycast suspension for the Godot game engine , fork frome Dechode. \n
Refractor A lot of codebase of Dechode to enhance them into A new level, including advanced Engine model,and Linear Clutch Model(Karnopp), More stable spring-based Diff Model and more! \n
Adding Advanced Tween based EngineSound! \n
And also ForceFeedBackNode within My godot_sdl_haptic Plugin! \n

This project will be a framework till I have time to add more preset , track and models for it;
Features:
- RWD, FWD and AWD drivetypes available
- Pacejka 89 , 2012 and TMeasy , RectPatch Brush tire model available.
- Tire wear
- Fuel consumption using BSFC
- Choose between preloaded limited slip diff, open diff and locked diff/solid axle
- Manual clutch with adjustable clutch friction force
- Manual and automatic gearbox

This project would not have been possible without Wolfes written tutorial of his own car simulator physics. Also huge thank you to Bastiaan Olij for his vehicle demo. See the links in the Acknowledments section for more info.

## Controls

Keyboard and Joys:
- Project -> InputMap -> Bind as you want!

Wheels that compatible with SDL3:
- Wheels itself
- Sequential support
- 3 padel support

## License
The project code is licensed under the GPL2 License - see the LICENSE.md file for details.  \n
This project also contains models and textures owned by their authors , they may not have Public Domain License , You shall ask SpeedDreams Developers and Modders because it was what I what to Replicate (at least before I can move the whole SpeedDreams codebase to A RHI).


## Acknowledgments
* [Kenney car kit](https://www.kenney.nl/assets/car-kit)
* [Bastiaan Olij - Vehicle demo](https://github.com/BastiaanOlij/vehicle-demo/)
* [Wolfe, written tutorial of his GDSim vehicle physics](https://www.gtplanet.net/forum/threads/gdsim-v0-4a-autocross-and-custom-setups.396400/)
* [Racer.nl, Alot of great documentation about physics of racing sims](http://www.racer.nl/)
