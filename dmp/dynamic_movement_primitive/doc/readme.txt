
DMP Message:
FillMe

DMP Controller:
FillMe

DMP Transform:
The DMPTransform is used to encode DMPs relative to the initial hand configuration.
Thus, when encoding the DMP from a demonstrated pose trajectory, the
initial (first) pose is being "subtracted" from all poses in the trajectory such 
that each DMP starts from zero. The >initial_start< variable in the DMP message
will contain this initial pose, i.e. the first pose of the demonstrated trajectory.
The >initial_goal< variable will contain the subtracted initial goal during demonstration.

 

 