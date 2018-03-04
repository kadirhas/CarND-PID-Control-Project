# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

This is the 4th project of the second term on Udacity Self-Driving Car Engineer Nanodegree Program. In this project, a PID controller is implemented to keep the car on the track.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Discussion

### PID controller
P parameter affects how the controller responds to the current error. It respons similar to the spring: if the error is higher, it "forces" the corrective input more. Derivative part of the controller contains information about future. It acts as a damper, since it does not respond a steady state error. This also means that D variable increases the steady state error and make the system more sluggish. Another problem is, if the error changes suddenly, it applies "impulse" as the force, since the derivative of a step is infinite. Usually a filter is applied to remove this effect, however on this project It was not necessary since the sensor is perfect and there was no sudden changes on the crosstrack error. Integrator variable sums up the all errors, it is about the "past" of the system. It removes the steady state error, and it can also remove the bias in the system if there is any. The downside is, it increases the oscillations. If there is a limit on the input of the system (such as maximum steering angle) it also creates a problem which is known as integrator wind up, but there are also many ways of deal with this issue. 
### Implementation
On this project, the frequency of the control loop is assumed as constant. Because of that the time difference parameter is not seen on the I and D controllers. 
There are two controllers for the vehicle, one for steering and another one for the throttle. First, the speed controller is tuned, then the lateral controller is implemented. Even though there are limits for both inputs, anti windup algorithms are not implemented because I is not used for the speed controller and the I term is relatively very small for the lateral controller. 
### Tuning
It is tuned manually since I am experienced with PID tuners. The speed controller is tuned very easily: a P controller with 0.5 was good enough. There is a small steady state error which can be removed via the I parameter. I didn't implemented it because it might create wind up problem and it is already satisfactory for the lateral control. For the lateral control tuning, first 0 is applied as the steering to see if there is any bias. It has been observed that there is a small bias towards right, which implies that I parameter should be used on the final controller for nonoscillating system. After applying 0.5 P controller, the vehicle was oscillating too much so I've started to increase the D parameter, while decreasing the P. Then I used very small I parameter. Then I've decided to increase the velocity of the vehicle. Since the PID controller does not have any information about the dynamics of the vehicle, the parameters should be tuned according to velocity for a more robust control. So I've decreased the P and increased D even more. The final parameters are:  [P,I,D] (0.11,0.0000001,4.0) for the lateral control and (0.5,0,0) for the speed control.


