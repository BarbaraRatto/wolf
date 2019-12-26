Use external rt_logger and create a library for the wpg
Experiments on the real robot
Fix device interface
Simplify math in the commands interface
Change class names according to the paper
Fix the state machine based on the paper
Fix the swing frequency definition based on https://cseweb.ucsd.edu/classes/sp16/cse169-a/slides/CSE169_13.pdf
IK with joint limits
Update the cartesio when ready
Add external api for the weights, to be used for a ML thesis
Arm task
change names dls -> wb
push recovery
terrain estimator
step reflexes
tests and profiling
check the FIXME around the code clean up
startup procedure and solver reset (state machine)
clean the descriptions, load the homing from the srdf
remove sensors and clean up the urdfs!
matlab/txt files logger: create an async thread to write to file when triggered, using a buffer.
add com/icp to stabilize the robot at low freq swings ML thesis
