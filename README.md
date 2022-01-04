# fastDDP

### Notes
* Need System class to store S attributes
* DDP.m is just a function to define, same with ddp cost and ddp trajectory
* Need main file that has main func, makes an S object and calls these funcs
* ddp.m appears to just return the optimal computed control deviations from
the input control sequence. This is effectively just one iteration of DDP.
Each trajectory that we saw before is really an iteration, not a candidate
trajectory, and the optimal trajectory is just the last one. This might be
why it appears to only run the backward pass once (although with a new
while loop inside of it), yet the forward pass is iterated multiple times.

### TODO