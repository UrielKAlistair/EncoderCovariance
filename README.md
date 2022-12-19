# EncoderCovariance
Takes encoder data and updates the position and position covariance of the robot by publishing an odometry message.

Steps to get good encoder data:

- [x] Get proper encoder counts from elec.  
- [ ] Ensure no mechanical trouble with the encoder and tyre.  
Now that we have counts, we can do position math. But for that we need wheel base and wheel radius.  
- [x] Tune wheel radius
- [ ] Tune wheel dist  
That done, we can compute the covariance matrix and publish the odometry.  
- [ ] Make a decent error model for the covariance.
