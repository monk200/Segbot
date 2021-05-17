# Initial Matlab Simulation
Yorihisa Yamamoto is/was an employee at Mathworks and created a simulation for controlling a self-balancing robot made of Legos. His work creating a Segbot can be found [here](https://www.mathworks.com/matlabcentral/fileexchange/19147-nxtway-gs-self-balancing-two-wheeled-robot-controller-design). Some of the parameters in the simulation are educational guesses and others don't translate very well unless the Segbot design is similar to Yamamoto's design. Nonetheless, it helps create a proof-of-concept that balancing a similar Segbot will work and allows students to experience tuning a balancing controller without needing the physical hardware. Since the first portion of this project had to be done remotely and with no hardware, the parameters are estimated and I did my best to tune the simulation. Ultimately, the simulation is still unstable after many hours of trying different pole placements but I hypothesize this is because the parameters are not accurate enough or because the frame of the Segbot makes it much more difficult to balance.  

## Input Parameters
The parameters below were estimated and calculated as well as possible given the sometimes surprisingly limited information on the previously chosen parts. The motor parameters specifically, however, were estimated using real-world information provided by Prof. Block about a similar motor. These parameters should be entered into the appropriate lines at the top of the [<code>equations.m</code>](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Initial%20Simulation%20Files/equations.m) file. Not all of these parameters are listed in the file but all of them were calculated for the sake of data collection.  

| Variable | Calculation | Value | Units | Description |
| ----------- | ----------- | -----------| -----------| -----------|
| g | - | 9.81 | m/sec^2 | Gravity acceleration |
| m | 0.03 (estimated wheel weight) + 0.032 (given Mrotor) | 0.062 | kg | Wheel weight |
| R | - | 0.0325 | m | Wheel radius |
| Jw | [m * (R^2)] / 2 | 3.2744e-5 | kg * m^2 | Wheel inertia moment |
| M | 0.197 (battery) + 2 * 0.982 (both motors) + 0.04 (estimated PCB + components) + 0.066 (given Mstator) + 1.746 (frame) | 2.245 | kg | Body weight |
| W | - | 0.20 | m | Body width |
| D | - | 0.20 | m | Body depbth |
| H | - | 0.29 | m | Body height |
| L | sqrt[(13.805-2)^2 + (5-3.897)^2] | 11.86 | m | Distance of center of mass from the wheel axis before motors |
| Jpsi | M * (L^2) / 3 | 105.2603 | kg * m^2 | Body pitch inertia moment |
| Jphi | M * (W^2 + D^2) / 12 | 0.1796 | kg * m^2 | Body yaw inertia moment |
| Jm | (given Jmotor) * (gear ratio ^2) = 1.1e-7 * (1/45)^2 | 5.432e-11 | kg * m^2 | DC motor inertia moment |
| Rm | 12V / 1.8A | 6.667 | ohm | DC motor resistance |
| Kb | Kt (assumption) | 0.4903 | V sec/rad | DC motor back EMF constant |
| Kt | 9 kg * cm / 1.8A | 0.4903 | Nm/A | DC motor torque constant |
| n | - | 1 | - | Gear ratio |
| fm | assumption | 0.0022 | - | Friction coefficient between body and DC motor |
| fw | assumption | 0 | - | Friction coefficient between wheel and floor |

## Tuning the Simulation
Continuing work in the [<code>equations.m</code>](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Initial%20Simulation%20Files/equations.m) file, the last two lines define <code>K3</code> and <code>K</code>. For this case, focus on the 3-state system. This means that the only K gain to edit is <code>K3</code> and the Simulink file that should be used is [<code>segbot3state.slx</code>](https://github.com/monk200/Segbot/blob/main/Matlab_Simulation/Initial%20Simulation%20Files/segbot.slx).  

Typing <code>eig(A3-B3*K3)</code> into the Command Window will output the eigenvalues of the 3-state system with the current K3 gains. When this is done with the gains currently listed in the file, the eigenvalues are [1136.6, 0.80, -0.80]', which makes the system unstable because the third pole is the only one in the left half plane. This can be visually verified by running the Simulink file. To try to stabilize the system, use the place function in Matlab to place the poles in the left half plane directly. The syntax for this would be <code>K3 = place(A3, B3, poleArray)</code>.  

The Simulink file can also be edited to adjust the type of disturbance applied to the system. The Pulse Generator block labeled "Finger Pushing At Top of Body (Newtons)" can be edited to increase the amplitude, corresponding to the pushing force. The other parameters can also be edited but for basic stability the goal is to tune the Segbot to stabilize after a 4.5 N tap.  

## Results
Unfortunately, I was not able to find stable poles for the system even after hours of trying. This could be because my estimates for the model were too inaccurate or because the physical design itself is not easy to balance. The most likely way that the parameter estimates are off is that more calculations need to be made with respect to where each part is stored in the Segbot frame. The design has enough space that the weight is not centered over the wheel axis and the mass is more distributed. Another point that I didn't consider when creating the frame model is that a top-heavy object is actually easier to balance than a bottom-heavy object because it requires a smaller control input. This was not modeled in the simulation but the frame does design for the battery to sit low, which might have made the sweet spot for balance more difficult to achieve in the real world.  

Prof. Block was working with me throughout each step of the design process and approved of each of my finalized choices up to just before working with the simulation. This makes me confident that despite some potentially non-optimal choices, this Segbot design is still capable of working in real life and with some minor improvements. Below is a link to a video of the simulation when it was as stabilized as I managed to get it. I did not end up saving the poles for this behavior because they still needed a notable amount of changes to be viable.  

[![Initial Simulation](https://youtu.be/ORlbtnldfrQ)
