# Change Log
All notable changes to this project will be documented in this file.
 
The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).
 
## wolf_controller - [0.0.8] - 2022-02-15
 
sha 7cbb2f88217efc3c99aa1f7174f230172555d512

### Added
- License

### Changed
 
### Fixed

## wolf_controller - [0.0.7] - 2022-02-13
 
sha a77bde6b414113977ea6b1e624c39f362bc0cfc7

### Added
- activate push recovery and step reflex in the param files
- plot capture point info
- state estimation integrate velocity, but it needs testing and at the actual state doesn't work

### Changed
- send input device commands only if the robot is in the active state
- push recovery based on capture point
- integrated other services in the keyboard node
 
### Fixed
- angular velocities bug
- arm startup problem

## wolf_controller - [0.0.6] - 2022-02-06
 
sha 0db116a4324dc8565576ae621997f5dbeb11faf2

### Added
- base_footprint
- controller test with EIGEN MALLOC checks
- step reflex
- separate sliders for the velocities

### Changed
- reorganized foot trajectory files
 
### Fixed
- perform init() only one time

## wolf_controller - [0.0.5] - 2022-02-01
 
sha f1aeaa4ea273b50f028eff620d71e12026062c4

### Added
- new ROS services (reset_base, set_swing_frequency, etc...)
- new ROS services to the keyboard node

### Changed
- working on [issue-2](https://github.com/graiola/wolf/issues/2)
- clean the ros topics
- reduce xbot logger verbosity
- cleaned up various branches
 
### Fixed
- bug fix in the foothold reset about the terrain estimation
