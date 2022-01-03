# fastDDP

### Notes
* Need System class to store S attributes
* DDP.m is just a function to define, same with ddp cost and ddp trajectory
* Need main file that has main func, makes an S object and calls these funcs

### TODO
* Need to switch all vectors (state and control) to Eigen::VectorXd. Then, store relevant quantities like number of states and controls as System variables so you can easily make new relevantly-sized vectors and arrays in other functions
