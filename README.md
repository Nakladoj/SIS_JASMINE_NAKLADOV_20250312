# AE 544 Analytical Dynamics Programming Project 01
### By: Jasmine Nakladov

## Introduction
This project asks to write three (3) numerical programs to simulate three scnerios, all of which will include the approach or hit of a singularity or ambiguity. The scenarios are as follows:

1. Singularity for Asymmetric Euler Angle Sets
2. Ambiguity of Euler Parameters / Quaternions
3. Classical Rodriques Parameters

This will be accomplished by integrating an attitude or dynamics equation with initial conditions. The angular velocity vecotr initial condition will be manipulated to ensure the singularity/ambiguity is approached. An additional goal of this project is to investigate whether different numerical integrators affect the singularity problem in different ways. The investiaget integrators will be as follows: *ode45* for a nonstiff example, and *ode15s* for a stiff example.

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

The initial Euler angles were all set to 0 rad, and the timespan was set to 100 seconds. For the integrator, the input state vector contains $\theta$ and time provided by the function at each integration step. The *ode45* function then repeatedly calls on the predefined *angular_rates* function, the timespan, and the initial angular conditions, outputting $\dot{\theta}$ and integrating for the Euler angles. Finally, in the code, $\theta$ is converted to degrees for an easier visual and plotted alongside the time *t*. The output can be seen below:

<p align="center">
  <img src="https://github.com/user-attachments/assets/c446f309-f566-4a05-8376-0ae87a3ef221" alt="Singularity for Asymmetric Euler Angles Figure Using ode45" width="500">
</p>

With this code, the pitch angle approaches $-90^\circ$, reaching up to $-89.8484^\circ$. When the pitch angle approaches this value in the figure at around 18, 45, 70, and 95 seconds, gimbal lock occurs the remaining two angles, yaw and roll, react accordingly. Both angles experience discontinuities and/or sharp jumps at the same time as gimbal lock occurs. This is because when this happens, the system loses one of its degrees of freedom, causing the yaw and roll angles to become unstable. 

The same process was then repeated with the ode15s function, used to solve stiff problems as opposed to ode45's nonstiff problems. The only difference between this and the previously described code is the integrator. Below are the results:

<p align="center">
  <img src="https://github.com/user-attachments/assets/fdc93cdb-94ef-409c-aa40-8e34202ba24d" alt="Singularity for Asymmetric Euler Angles Figure Using ode15s" width="500">
</p>

As can be seen..........

## Ambiguity of Euler Parameters / Quaternions
The second scenario that will be examined is the ambiguity of the Euler Parameters.


## Classical Rodriques Parameters

## References
1. Project info
2. Class notes
3. Textbook
4. Matlab Help Center
   3.1 Matlab ode45: https://www.mathworks.com/help/matlab/ref/ode45.html
   3.2 Matlab Choose an ode Solver: https://www.mathworks.com/help/matlab/math/choose-an-ode-solver.html
   3.3 Matlab quat3dcm: https://www.mathworks.com/help/aerotbx/ug/quat2dcm.html
5. ChatGPT was consulted for help with MathJax syntax, as well as inserting figures into the .md file.
