# Term2 - Model Predictive Control Project Writeup
Self-Driving Car Engineer Nanodegree Program

---

## Reflection


### 1. Describe the model in detail.
My project adopts the `Kinematic Model` for the predictive controller. It has four states `[x, y, ψ, v]` to describe the states of the vehicle, and two actuators, steering angle `δ` and acceleration `a`. To calculate the predicted future states and actuators, I introduced two error functions `Cross Track Error (cte)` and `Orientation Error (eψ)` to measure how well the vehicle follows a reference line. So my model consists of six input state vector `[x, y, ψ, v, cte, eψ]` and two output actuators `[δ, a]`. The following equations are used to predict (update) the state vector after `dt` time interval elapses. The variable `Lf` is a constant value that measures the distance between the front of the vehicle and its center of gravity. It determines the turn rate of the vehicle.
> x[t+1]​ = x[t]​ + v[t] * ​cos(ψ[t​]) ∗ dt
>
> y[t+1] = y[t] + v[t] * sin(ψ[t]​) ∗ dt
>
> ψ[t+1] = ψ[t] + v[t] / Lf * δ[t] * dt
>
> v[t+1] = v[t] + a[t] * dt
>
> cte[t+1] = cte[t] + v[t] * sin(ψ[t]​) ∗ dt
>
> eψ[t+1] = eψ[t] + v[t] / Lf * δ[t] * dt

In the model prediction procedure, my MPC utilizes upper equations to predict a successor state `S(t+1)`, then by minimizing the power of 2 cost to the referencing state `Ref_S(t+1)`, it finds out an optimized pair of actuators `[δ[t], a[t]]`.


### 2. Discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally, provides some details of the previous values tried.
In general, we want N to be larger to cover as many steps as possible, and dt to be smaller enough to predict a successor state that doesn't deviate much from previous one. However, considering the environment usually changes in the near future, and the limited computation power also doesn't allow us to do much calculations. Being said that, it doesn't necessarily have to predict too many steps further. So a reasonable combination of N and dt should be some values that just cover an appropriate range without much deviation in between states.

I started this project with an intial N=20 and dt=0.05, assuming to predict 20 states in one second might be suffice. But in my working PC, the computation effort couldn't keep up with the demands. With all penalty factors set to be one, the vehicle couldn't even navigate through the track under 30Mph speed. Then I chose N=10, dt=0.1 as in the Q&A shown, it works decent enough for me. So I ended up just using them without devoting much effort in trying out other values.


### 3. Describe waypoints preprocessing before fitting to a polynomial, if any.
Because the simulator gives me the referencing waypoints and vehicle position, orientation in the map coordinate. It's a bit awkward to work with, especially the objective requires us to take aspect of the vehicle (from the vehicle's coordinate). The way I took to preprocess waypoints is exactly what's shown in the Q&A video. That is, by subtracting waypoints to the vehicle position, and rotating axes for 90 degrees in counter-clockwise, the transformed waypoints have the same coordinate as the vehicle. Another benefit is our vehicle is now at the point where `x`=0, `y`=0 and heading `ψ`=0, it's very easy to calculate `cte`=polyeval(coeffs, 0) especially `eψ`=-atan(coeffs[1]).


### 4. Discusses in details of how to deal with 100ms actuation latency.
The method I used to deal with latency is the way stated in the section 7 of Lesson 19. Because the vehicle is at `x`=0, `y`=0 and `ψ`=0 initially, provided I have the steering angle `δ` and throttle value `a`. So just plugin these values into equations above under the circumstance `dt`=0.1, I get a new set of initial states that describing where vehicle will be after 100ms. Then I Feed this state vector into MPC solver, the resulting actuators are just the values to control the vehicle due to latency.
