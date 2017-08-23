# CarND-Controls-PID 

Self-Driving Car Engineer Nanodegree Program

# ReadMe

---
This repository contains my (Anton Varfolomeev) implementation of the PID-control project

---

## Reflections on solution

There seems to be more heuristics and intuitions than rigorous mathematics in this project.

Short history of search (and results):

* PID controller implementation (rather straightforward, UpdateError() and TotalError() methods of the PID class)
* Manual PID parameters search. For it, I used three command line parameters (event without proper names, just numbers, sorry). 
    The purpose was to find set of parameters to pass the whole loop. It took about half an hour to obtain 'working' numbers,
    They were: 

    ```Kp = 0.04, Ki = 0.01, Kd = 10```

* Throttle heuristics: I decided not to use PID - controller with some destination speed, but made throttle proportional 
to the square of the cosine of the current angle. This implementation allows me to achieve the speed of 55 mph 
on straight segments of the road - and to remain on the road on curves

    ``` throttle = cos(angle) * cos(angle) * 0.85 ```
* Twiddle algorithms implementation (Twiddle() method of the PID class)

Then there was a long story of Twiddle optimization:

* While 'twiddling', it is possible that some set of parameters will send our car off the road. To continue training,
we need to reset not only PID controller, but the simulator too. Thanks to the forums, I found the command to 
perform the task

* At the beginning we can use rather short segments for coarse parameters update, but for fine-tuning it is better to use
longer and longer segments. I achieve it by 20% 'twiddle period' increase on each 'total error' (MSE) decrease

* When our car is on the straight segment, we can receive a lot of low CTE values, it will lead to the low MSE, which will 
dominate all future errors and reduce the probability of new parameters acceptance. To prevent it, I pause MSE calculation
on straight segments (when CTE is low).

### Note to the reviewer:
Due to the implementation differences, the same controller with the same parameters will show different results
on different computers.
I assigned default values so that it works on three different machines (one of them Ubuntu 16.04, and two 
others - Windows 10 with Ubuntu shell) - but I am not sure if it will work on your system.
  
