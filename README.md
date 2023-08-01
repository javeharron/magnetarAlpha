# Magnetar
Magnetar is a distance-ranging coil gun platform.

## Description
This system is designed to utilize commercially available LiDAR modules for distance-based energy correction. As a projectile from a coil gun flies through the air energy is lost through drag. The goal of this project was to create a model to correct for this loss so projectiles impact with consistent energy.

This system utilizes a simple revolving design to house multiple projectiles for subsequent firing. These projectiles are standard-length Nerf darts combined with a ferromagnetic insert (M4x12mm Bolt). To calculate the corrected velocity necessary to overcome drag energy losses an iterative equation is used. Essentially, out to a set amount of resolution drag losses are added to the projectile's target impact velocity. The target velocity is found using the kinetic energy formula or: 

# $`E_k = \frac{1}{2}mv^2`$

From this target velocity drag is added over set time intervals modeled by the equation: 

# $` V_i = V_t + \frac{\frac{1}{2}\rhoV_t^2AC_d}{M}R`$

In this equation, $`V_i`$ is the initial velocity after a set time interval of inverse drag. $`V_t`$ is the initial/target velocity of the projectile and R is the resolution of the time interval (e.g. 0.001 for millisecond intervals). This is continuously run until:
# $`V_it = d`$

Finally the voltage necessary to achieve this velocity is calculated using the formula of: 
#U = $`\frac{1}{2}`$

### Items
The Magnetar project files include the following: 
- Fully modular and customizable software.
- A modular data logging program and corresponding visual studio project files to intercept serial port communication for dumping into .XLSX formats.
- All 3D files in .STL format for additive manufacturing.
- Data from testing the system at various 1-meter distances.
- Full circuit schematic of the design.
- Flow charts for all code present.
- Sample BOM for at-home fabrication.

### Configurable features
Currently, the firmware allows for the configuration of:
- Pins for circuit control and HID.
- Capacitance of system.
- Efficiency constant of the coil.
- Mass of dart projectile.
- Cross-sectional area of the projectile.
- Density of environment.
- Maximum voltage for capacitors.
- Table output.
- Switching between full and auto-ranging power.
- Target energy.


