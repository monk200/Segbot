# Reworked Matlab Simulation
Yorihisa Yamamoto is/was an employee at Mathworks and created a simulation for controlling a self-balancing robot made of Legos. His work creating a Segbot can be found [here](https://www.mathworks.com/matlabcentral/fileexchange/19147-nxtway-gs-self-balancing-two-wheeled-robot-controller-design). For the reworked simulation, the parameters were measured and filled into the model by Prof. Block so there is slightly less information in the section below (but the numbers are significantly more accurate). This simulation was also significantly easier to tune and I was able to both find stable poles for it and manipulate the poles to see how it behaves outside of the stable region.  

## Input Parameters
These parameters should be entered into the appropriate lines at the top of the [<code>equations.m</code>](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Reworked%20Simulation%20Files/equations.m) file.  

| Variable | Calculation | Value | Units | Description |
| ----------- | ----------- | -----------| -----------| -----------|
| g | - | 9.81 | m/sec^2 | Gravity acceleration |
| m | wheel weight + Mrotor | 0.0615 | kg | Wheel weight |
| R | - | 0.067 | m | Wheel radius |
| Jw | [m * (R^2)] / 2 | 1.3804e-4 | kg * m^2 | Wheel inertia moment |
| M | - | 0.5764 | kg | Body weight |
| L | - | 0.049 | m | Distance of center of mass from the wheel axis before motors |
| Jpsi | M * (L^2) / 3 | 4.6131e-4 | kg * m^2 | Body pitch inertia moment |
| Jm | Jmotor * (gear ratio ^2) | 2.285e-4 | kg * m^2 | DC motor inertia moment |
| Rm | - | 1.875 | ohm | DC motor resistance |
| Kb | Kt (assumption) | 0.30646 | V sec/rad | DC motor back EMF constant |
| Kt | - | 0.30646 | Nm/A | DC motor torque constant |
| n | - | 1 | - | Gear ratio |
| fm | assumption | 0.0022 | - | Friction coefficient between body and DC motor |
| fw | assumption | 0 | - | Friction coefficient between wheel and floor |

## Tuning the Simulation
Continuing work in the [<code>equations.m</code>](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Reworked%20Simulation%20Files/equations.m) file, the last line defines <code>K</code>. This set of simulation files only has the three-state files available. This means that the only K gain to edit is <code>K</code> and the Simulink file that should be used is [<code>segbot.slx</code>](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Reworked%20Simulation%20Files/segbot.slx).  

Typing <code>eig(A-B*K)</code> into the Command Window will output the eigenvalues of the 3-state system with the current K gains. When this is done with the gains currently listed in the file, the eigenvalues are [-446.4162, -8.9051, -0.9308]'. These poles are stable, which can be checked by noticing that all the poles are in the left half plane and that the simulation does balance the Segbot. To change the poles, use the place function in Matlab with <code>K = place(A, B, poleArray)</code>.  

The Simulink file can also be edited to adjust the type of disturbance applied to the system. The Pulse Generator block labeled "Finger Pushing At Top of Body (Newtons)" can be edited to increase the amplitude, corresponding to the pushing force. The other parameters can also be edited but for basic stability the goal is to tune the Segbot to stabilize after a 4.5 N tap. By default, the Simulink file only has the tap set to be 0.5 N.  

## Results
The current poles set in the Matlab file are stable for the system, and the running simulation can be seen in the video below. Below there is also an image of the three plots being monitored to ensure a stable system. You can see how in the Psi_body plot below, the body of the Segbot waivers but stabilizees in less than 4 seconds, with very little overshoot. The Theta Wheel plot shows how the wheels rotate as they work to correct Psi_body but they do not rotate when the Segbot finds its balancing point. Lastly, the motor voltage plot shows that the control effort stays within a reasonable range (between 6V and -6V in this model) and does not waiver back and forth. All of these plots support that this is a stable system.  

![Stable Pole Plots](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Reworked%20SImulation%20Files/plots.PNG)  

[Simulation Video on Github](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Reworked%20SImulation%20Files/Stable%20Poles.mp4)  
[![Stable Simulation](https://i9.ytimg.com/vi/ORlbtnldfrQ/mq2.jpg?sqp=CMCEkYUG&rs=AOn4CLD_6UKGUA3llFuLTRHpBtmPb5CrUw)](https://youtu.be/3cc0C4fP93k)  

## Further Analysis
Further analysis on the simulation can be done to understand how the system works and how it behaves with different poles. First I tried setting the K gains to be noticably more responsive than the stable gains. Setting <code>K = [-34.3159, -3.6375, -3.3290]</code> produces poles at [-446.4162, -8.9051, -10]'. Below again are plots and a link to a video of the simulation running. The Psi_body plot already shows how there is a much more dramatic adjustment of the Segbot's tilt, but it still is able to stabalize. The portions of Theta Wheel where the Segbot is adjusting its position are much faster, also meaning that the distance doesn't need to be as large to create a counter-balancing force. The biggest issue with these gains can be seen in the motor voltage plot. The control input actually saturates very slightly at 6V, the maximum voltage the motor simulation can handle. (At this point it is worthwhile to note that the simulation motor voltage is scaled down from a 10V maximum to 6V) There is also an overshoot into the negative voltages to handle the initial control effort overshooting the amount of speed needed.  

![More Responsive Poles Plots](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Reworked%20SImulation%20Files/Manipulating%20Poles/more%20responsive.png)  
[![More Responsive Simulation](https://i9.ytimg.com/vi/d-1ndJblGV4/mq2.jpg?sqp=CPCNkYUG&rs=AOn4CLBU40kSTlXPUmkKxll7eIoSN37k2w)](https://youtu.be/d-1ndJblGV4)  

Increasing the responsiveness of the system even more can show even more dramatic issues. This amount of responsiveness can be dangerous to even try on real hardware because the control effort has the potential to burn out the motors or cause other undue stress on the parts. Here <code>K = [-63.5382, -8.7203, -10.4572]</code> making the poles [-446.4162, -15, -20]'. The plots still show that the Segbot comes back to equilibrium but the dramatic oscilation sseen on the Psi_body plot and the motor voltage plot show clear issues. The sharp changes are hard to control and end up causing a lot of overshoot. Not to mention, the motor voltage saturates in both the positive and negative direction (this is what would burn out the motors on real hardware).  

![Too Responsive Poles Plots](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Reworked%20SImulation%20Files/Manipulating%20Poles/too%20responsive.png)  
[![Too Responsive Simulation](https://i9.ytimg.com/vi/qGNAavwwsLA/mq2.jpg?sqp=CJyQkYUG&rs=AOn4CLAy2k0ghNSgdPa8YB49bJ55ECOvug)](https://youtu.be/qGNAavwwsLA)  

As a last excersise with manipulating the poles of the system, next I made the system not responsive enough. I set <code>K = [-47.2416, -13.2226, -18.4865]</code> which made the poles [-80, -50, -60]'. The plots for this system show an insane amount of oscillation. In fact, in the plots there is a point just before the 3 second mark where it just barely starts to approach its balancing point and then it recieves the second tap which adds enough energy that the controller can't keep the Segbot from tipping over entirely. Because of how dramatic the oscillations are, the motor voltage saturates again trying to rescue it from its inevitable fate.  

![Not Responsive Poles Plots](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Reworked%20SImulation%20Files/Manipulating%20Poles/not%20responsive%20enough.png)  
[![Not Responsive Simulation](https://i9.ytimg.com/vi/98DRiGnAHTE/mq2.jpg?sqp=CPSUkYUG&rs=AOn4CLAcfTJszh9wsLgm46E3AepAPw_O4w)](https://youtu.be/98DRiGnAHTE)  

Returning back to using the stable poles, the simulation can still be changed to explore other behaviors of the controller. If the "Finger Pushing At Top of Body (Newtons)" block is changed to have a 50% pulse width and an amplitude of 0.4 N, it will simulate pushing the Segbot gently for 2 seconds. The video below shows how the Segbot pushes back on the "finger" (the force is applied from the left) to stay balanced. This makes sense because it needs to counteract the force being applied to stay upright.  

[![Pushing the Segbot](https://i9.ytimg.com/vi/trDw0ImDjyA/mq2.jpg?sqp=CPSUkYUG&rs=AOn4CLADverP_Pjyg6w_unwn7q2HV-o7Sg)](https://youtu.be/trDw0ImDjyA)  

Now, setting the amplitude of the same block to 0 N and editing the "Psi Offset" block in the bottom right of the Simulink file to 0.2 radians will tell the Segbot to drive forward for 2 seconds. The video below shows how when the Segbot gets the initial moment of movement, its body leans forward and quickly adjusts back. It also leans backwards for a moment when it stops moving to counteract the forces caused by being in motion and then stopping.  

[![Driving Forward](https://i9.ytimg.com/vi/pj7e5s4WAzk/mq2.jpg?sqp=CKCXkYUG&rs=AOn4CLD1Waesy1a2AsGNwCQcy5uWRImIVw)](https://youtu.be/pj7e5s4WAzk)  

These analysis are all useful in gaining a deeper understanding of how the control algorithm behaves and highlighting the physics of how an inverted pendulum needs to be balanced.  
