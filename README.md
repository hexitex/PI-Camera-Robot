# PI-Camera-Robot
Controls a 3 axis pi camera robot using multi-threaded lambda functions based on cheap steppers, also can control upto 16 servos. 
See example output file movie1.mp4 in list of files

Sequence as follows:

Startup align turn left/right forwards/backwards roll left/right positions of steppers, this happens after every complete seq (current 9000 shots)

General movements for each of the steppers very slowly, freeze motion of all motors and take shot every five seconds.

After 100 shots make a short movie 25 frames per second no compression so that every frame can be inspected (security camera function)

When 9000 shots have been completed compile the 90 mini movies into 1 movie that lasts about 6 minutes and covers 12 hour period. then start sequence again.

Detects night and day and changes camera mode to suit best as possible without too long an exposure so that moving objects don't get blured.



