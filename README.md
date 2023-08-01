# Magnetar
Magnetar is a distance ranging coil gun platform.

## Description
This system is designed to utilize commercially available LiDAR modules for distance based energy correction. As a projectile from a coil gun flies through the air energy is lost through drag. The goal of this project was to create a model to correct for this loss so projectiles impact with a consistent energy.

This system utilizes a simple revolving design to house multiple projectiles for subsequent firing. These projectiles are standard length nerf darts combined with a ferro magnetic insert (M4x12mm Bolt).

### Items
The Magnetar project files include the following: 
- Fully modular and customizable software.
- A modular datalogging program and corresponding visual studio projwct files to intercept serial port communication for dumping into .XLSX formats.
- All 3D files in .STL format for additive manufacturing.
- Data from testing the system at various 1-meter distances.
- Full circuit schematic of the design.
- Flow charts for all code present.
- Sample BOM for at home fabrication.

### Configurable features
Currently the firmware allows for the configuration of:
- Pins for circuit control and HID.
- Capacitance of system.
- Efficiency constant of coil.
- Mass of dart projectile.
- Cross sectional area of projectile.
- Density of environment.
- Maximum voltage for capacitors.
- Table output.


