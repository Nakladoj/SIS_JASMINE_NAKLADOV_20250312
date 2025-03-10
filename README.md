# AE 544 Analytical Dynamics Programming Project 01
### By: Jasmine Nakladov

## Introduction
This project asks to write three (3) numerical programs to simulate three scnerios, all of which will include the approach or hit of a singularity or ambiguity. The scenarios are as follows:

1. Singularity for Asymmetric Euler Angle Sets
2. Ambiguity of Euler Parameters / Quaternions
3. Classical Rodriques Parameters

This will be accomplished by integrating an attitude or dynamics equation with initial conditions. The angular velocity vecotr initial condition will be manipulated to ensure the singularity/ambiguity is approached. An additional goal of this project is to investigate whether different numerical integrators affect the singularity problem in different ways. The investiaget integrators will be as follows: *ode45*, *ode4*, and *ode15s*. 

Finally, a 3D animation will be included to aid in analyzing each simulation.

Any references used in this project are located at the bottom of this .md file.

## Singularity for Asymmetric Euler Angle Sets
The first scenario that will be examined is the singularity of Asymmetric Euler Angles. The specific sequence chosen was the classic 3-2-1 Yaw Pitch Roll rotation. For this rotation, geometric singularity occurs at a pitch angle of +/- $90^\circ$, where the two remaining angles are not uniquely defined. The following process will be followed for Scenarios 2. and 3. as well. 

The following equations were used for this simulation, gathered from the textbook's **Appendix B.2 Mapping Between Body Angular Velocity Vector and the Euler
Angle Rates**:

$$
\dot{\theta} = [B(\theta)] \omega
$$

$$
[B(\theta)] = \frac{1}{cos{\theta_2}}
\begin{bmatrix}
0 & sin{\theta_3} & cos{\theta_3} \\
0 & cos{\theta_2} cos{\theta_3} & -cos{\theta_2} sin{\theta_3} \\
cos{\theta_2} & sin{\theta_2} sin{\theta_3} & sin{\theta_2} cos{\theta_3}
\end{bmatrix}
$$

In the matrix equation, it can be clearly seen that a singularity occurs at $\theta_2 = 90^\circ$, as this would cause the $\frac{1}{cos{\theta_2}}$ term to be divided by 0. A function was created to solve the $\dot{\theta} = [B(\theta)] \omega$ equation, as seen in the attached *.m* code. In this function, each Euler angle $\theta$ is extracted from the integrator's input state vector, with ode45 being the first integrator tested. The inputs to the function are the Euler angles and the body's angular velocity. The function then defines the above $[B(\theta)]$ matrix and $\dot{\theta}$ equation, which will use the integrator's updated values of the angular rates as inputs.

Above the function, the body angular velocity vector, initial angles, timespan, and the ode's input state vector are defined. The angular velocity vector, defined as *omega_body*, is important and serves as the main way to manipulate the angles towards singularity. The actual values of the $\omega$ vector are arbitrary, and many variations were tested to see which would bring the pitch angle, $\theta_2$, close to the singularity. The final values chosen for this part of the project are as follows:

$$
[B(\theta)] = 
\begin{bmatrix}
0.1 \\
0.2 \\
0.1
\end{bmatrix}
 rad/s
$$

The initial Euler angles were all set to 0 rad, and the timespan was set to 100 seconds.

theta0 = [0 0 0]'; % Initial condition
tspan = [0 60]; % Timespan

[t, theta] = ode45(@(t, theta) ypr(theta, omega_body), tspan, theta0);
theta = rad2deg(theta);


% Creating function to calculate angular rates = [B(theta)]*omega
% From the textbook, Appendix B.2
% angular rates = [B(theta)]*omega



## Ambiguity of Euler Parameters / Quaternions

## Classical Rodriques Parameters

## References
1. Project info
2. textbook
3. ode45 mtlab help
4. ChatGPT was consulted for help with MathJax syntax.
