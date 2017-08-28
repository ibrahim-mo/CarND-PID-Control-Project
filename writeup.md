## Writeup for the PID Control project

### By: Ibrahim Almohandes, Aug. 27, 2017

[//]: # (Image References)

[image1]: ./output/figure_0.01_0.0_0.0.png
[image2]: ./output/figure_0.05_0.0_0.0.png
[image3]: ./output/figure_0.1_0.0_0.0.png
[image4]: ./output/figure_0.5_0.0_0.0.png
[image5]: ./output/figure_0.05_0.0001_0.0.png
[image6]: ./output/figure_0.05_0.0005_0.0.png
[image7]: ./output/figure_0.05_0.001_0.0.png
[image8]: ./output/figure_0.05_0.001_0.5.png
[image9]: ./output/figure_0.05_0.001_1.0.png
[image10]: ./output/figure_0.05_0.001_2.0.png
[image11]: ./output/figure_0.06_0.0012_0.75.png
[image12]: ./output/figure_0.250829_0.00256535_5.91271_3.png
[video1]: ./pid_video.mov

To successfully steer the autonomous vehicle around the track, we need to come up with good (suboptimal) values for the PID Controller's hyper-parameters (Kp, Ki, and Kd), as in the following equation:

>Steering = - TotalError() = - (Kp * p_error + Kd * d_error + Ki * i_error); where  
>p_error = CTE (cross-track error)  
>i_error = sum (CTE), over t = [1, n]  
>d_error = d(CTE) / d(t)
>
>Notice the negative sign is applied to steering to counter-effect the calculated error.

Multiple methods can be used to extract the PID hyper-parameters which will be finally plugged into the vehicle steering equation.

Let's first discuss the effect of each of the three parameters on the CTE error, and hence the steering:

1. Increasing the proportional coefficient (Kp) will decrease the rise time. However, if we keep increasing Kp, the controller output will overshoot. Having Kp alone means we will always have a steady-state error, as the output is proportional to it.

2. Adding a non-zero integral coefficient (Ki) will help eliminate (eventually) the steady-state error. However, due to the summing effect, we need to start from a very small value (compared to Kp). And similar to Kp, increasing Ki will decrease the rise time, but increasing it more will cause overshooting of the output.

3. While Kp and Ki contribute to the controller components that are based on current and previous error observations, adding a non-zero differential coefficient (Kd) will contribute to predicting the future trend of the output based on the CTE error rate. Increasing Kd will reduce the settling time of the output and hence provide more stability into the system (i.e., smoother and safer steering in this case).


Now, it's time to describe how I came up with the PID hyper-parameters that I used to control the steering function of the autonomous vehicle. In my project implementation, I applied a combination of two techniques (as in the following order):

**A.** The fist method to apply is **manual tuning**. In this method, we fine tune Kp, Ki, and Kd until we reach good initial values that allow the vehicle to steer around the whole track (perhaps with a few deviations). For this manual tuning method, I am applying the following steps:

1. Initialize _Ki_ and _Kd_ to zeros, and manually fine tune _Kp_ by increasing or decreasing it until the output starts to smoothly oscillate around the reference (zero) line with a small overshoot. Here, I tried the values **0.01, 0.05, 0.1, and 0.5**, and I found out that the first value among these to satisfy this condition better is **0.1**, and let's now call this value _Ku_. Then, we actually take half of this value as our initial _Kp_ (as we actually want to have a small offset below the reference line). Hence, I chose an initial ```Kp = 0.5 * Ku = 0.5 * 0.1 = 0.05```.

2. Then, we fine tune _Ki_. As _Ki_ gets multiplied by an integral (accumulative) term, we need to play with much smaller values (than those of _Kp_). Here, I tried the values **0.0001, 0.0005, and 0.001**. I found out that both **0.0005, and 0.001** provide a smaller rise time (before smoothly oscillating around the reference line) than **0.001**, and **0.001** was slightly better than **0.0005**. Hence, I chose an initial ```Ki = 0.001```.

3. Then we fine tune _Kd_ until we reach a faster rise time after a small disturbance. As _Kd_ gets multiplied by a differential term, we will play with higher _Ki_ values than those of _Kp_. Here, I tried the values **0.5, 1.0, and 2.0**. We can see here that _Kd_ - with any of the chosen values - have stabilized the the output (though with a small oscillation) after a short rise delay. We can see that both **0.5, and 2.0** - at least in one occasion - cause a much larger overshoot than the small overshoot produced by **1.0** (around **-0.4/0.4**). Hence, ```Kd = 1.0``` becomes our best candidate.

Please see the PID controller's output response with different sets of manually tuned parameters, which illustrates on how these hyper-parameters were gradually tuned:

![image 1][image1]
![image 2][image2]
![image 3][image3]
![image 4][image4]
![image 5][image5]
![image 6][image6]
![image 7][image7]
![image 8][image8]
![image 9][image9]
![image 10][image10]

After running the vehicle in the simulator with these values, I can see the car successfully completes the track without leaving the road or causing accidents. However, the steering was far from ideal, as the car stepped over the red hashed lines as well as the double yellow lines at multiple places, and touched the ledges at a few others. (In real life, this can be scary to potential riders and should be avoided). Hence, there comes a need to do better by applying a computerized method that can fine tune these parameters even further, like **twiddle**, or **SGD** (Stochastic Gradient Descent).

The purpose of the manual fine tuning method is to come up with relatively good initial estimates of the hyper-parameters, which are approximate in nature (hence, can vary from one person to another). If the values are good enough, we can simply use them, but if not satisfactory, then we can rely on an automated approach to smooth these values more (for a more stable and safer steering of the vehicle).

I also tried another tuning method called the **Ziegler–Nichols** method which combines both manual tuning and mathematical derivation of the PID hyper-parameters. For details about this method, as well as the previous **manual tuning** method, please see the [PID controller](https://en.wikipedia.org/wiki/PID_controller) Wikipedia article. After applying this method, I got values that are close to ones I extracted from the **manual tuning** method (```Ku=0.1, Tu=100 -> Kp=0.06, Ki=0.0012, Kd=0.75```). However, it produced them with less effort, and the autonomous vehicle behavior in the simulator was very similar as well. For me, this worked as a double confirmation. The next step is to improve the vehicle steering using an automated tuning method, starting from either set of hyper-parameters (I chose the former). The following diagram shows a simulation of the PID controller's output response after applying the **Ziegler–Nichols** method.

![image 11][image11]

**B.** Staring from the initial set of hyper-parameters (```Kp=0.05, Ki=0.001, Kd=1.0```), it's now time to apply an automated fine-tuning method called **twittle** (a tweak of the known gradient descent algorithm), which was described in _Sebastian Thrun_'s lectures (on _Udacity_). The method will help us improve the PID controller output response (i.e., vehicle steering). I followed a similar approach to the one described in the lectures, but with some necessary modifications to make it work with the simulator. First, I found out that I need to set the number of iterations to 300, so the vehicle can have enough data (including areas of the road with very steep curvature, hence more prone to large steering errors). In addition, I cannot use an explicit loop (as it's only implicit by triggering the lambda function one simulator event at a time), so I've had to make it work as if there were an actual loop. The way to do this was to use a global iterator that increments only at each lambda step (or simulator-triggered event). Then, I implemented a small finite state machine (FSM) that transitions at each 2*n iterations (for each comparison between total error and best error, of the sum of squares of CTE). Finally, I used a criteria of error tolerance (= 0.001) to stop the iterative fine tuning of the PID hyper-parameters, which should be - by now - good enough to try them out on the simulator in full autonomous mode.

Finally, I ran the _pid_ program along with the simulator (after disabling the **twittle** code by commenting out line 34 in _main.cpp_ (```#define APPLY_TWIDDLE```), and applying the final tuning values we got, as described in section **B.**). We then feed the steering data that we was generated by main.cpp, and feed it into our Python simulation program (_pid_sim.py_). I ran _pid_ and the simulator for three tracks and got the output shown in the following diagram. We can see that most of the time the output slightly oscillates around the reference (zero) line. However, although there are cases where the controller output overshoots, the vehicle stayed on track all the time. I think controlling the speed with another PID controller can help with these extreme points, as most likely the vehicle was driving very fast at an extreme curve and was trying to adjust itself, hence producing such large instantaneous errors.

![image 12][image12]

I recorded a video of the simulator with the vehicle driven in autonomous mode with its steering controlled by the _pid_ program. You can watch the video I recorded in the [pid_video.mov](./pid_video.mov) file. Please note this video was recorded using Apple's QuickTime player, so it may, or may not, be playable by other media players.

In conclusion, I would like to mention that I spent a tremendous amount of time on this project to make it look better and to gather multiple resources from the lectures, forum articles, and even Wikipedia. In the process, I've learnt a lot about PID controllers. And as I mentioned earlier, one of the improvements that can be made to this project is to control the speed of the vehicle using an additional PID controller, hence providing a much smoother and safer autonomous driving.

Thanks!
