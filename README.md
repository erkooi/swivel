# Model and animation of a F35B swivel in OpenSCAD

For a video with animations of the swivel and more information please also check my channel "Eric Kooistra - Hobby" on YouTube: https://www.youtube.com/channel/UCnQhySYKmPyZsY2G8S3CUbA

## 1. Overview
### 1.1 3D print swivel model
The swivel model consists of four identical tube segments, one as input tube, two for the mid tube, and one as output tube. After 3D printing, the two segments for the mid tube are glued together and then bolted to the input segment and to the output segment. The bolts should be so tight that the tubes maintain their position, but can be rotated manually.

![3D printed swivel model](Pictures/swivel_bolt.jpg)

### 1.2 Animate swivel movements in OpenSCAD
The swivel model movements are animated in OpenSCAD. The swivel.scad design in OpenSCAD can:

- show the swivel model segments,
- animate transition between horizontal swivel for forward flight, and swivel vertical down for hovering,
- animate small adjustments around swivel vertical down, for pitch (forward - backward) and yaw (left - right, like with rudder) control during hovering.

In addition the swivel model can also animate swivel movements in OpenSCAD that are possible, but that not suited for use in an airplane: 

- animate swivel moving from horizontal to vertical full down to horizontal to vertical full up to horizontal in the ZX plane,
- animate thrust vectoring around swivel horizontal for forward flight.

The swivel is suitable for forward flight and for hovering, and for transition between forward flight and hovering. The amount of off center pointing of the swivel output tube is controlled by rotating the mid tube with respect to the input tube and to the output tube, whereby these rotations are equal but in opposite directions.

The swivel is not suitable for thrust vectoring during forward flight. This is due to that pointing the swivel output sligthly up, left, down, right needs to be done by rotating the input tube, which in a real application cannot instantaneously reach a new angle. Furthermore rotating the input tube by a full circle to reach all off center pointings would complicate the wiring and construction in a real application.

![Swivel model in OpenSCAD](Pictures/select_3.1_start.jpg)

### 1.3 Analyse swivel movements in Python numpy
The swivel model is formed from four tube segments using **rotate** and **translate** from OpenSCAD or using **multmatrix** for 4D transformation matrices for rotation and translation. The result is identical. These 4D matrix equations from swivel_assembly.scad are then used with Python numpy in swivel_functions.py, for further analysis of the swivel movement and thrust vector pointing in a jupyter notebook at python/swivel.ipynb (also available as python/swivel.html).

## 2. Model definitions

The XYZ coordinates and corresponding zero angle and positive angle directions are defined as:

```
From openscad/libraries/linear_algebra.scad:

* Right-handed coordinates X, Y, Z:
  . right hand fingers point from X to Y points tumb to Z
  . screw from X to Y goes to Z
 
      z
      |          . angleYZ
      |--- y     . angleZX
     /           . angleXY
    x
 
* Euler angles
  - https://en.wikipedia.org/wiki/Euler_angles
  - positive rotation about:
    . X-axis from Y to Z = phi, is roll, aileron, banking
    . Y-axis from Z to X = theta, is pitch, elevator, elevation
    . Z-axis from X to Y = psi, is yaw, rudder, heading
 
* Spherical coordinates P(r, phi, theta)
  - https://en.wikipedia.org/wiki/Spherical_coordinate_system
    . r is radial distance from origin to P
    . phi is angle from X to projection of P in XY plane
    . theta is angle from Z to radial line through P
```
The swivel is at the tail of an airplane, with up in positive Z direction and right in positive Y direction. For forward flight the airplane flies in negative X direction, and the swivel thrust vector points horizontal in positive X direction.

The alpha_tilt angle beteen the tube segments yields a maximum pointing range of 0 to 4 * alpha_tilt degrees, when the mid tube is rotated by theta_mid = 0 to 180 degrees. For alpha_tilt > 22.5 degrees the swivel can be pointed slightly forward as well, which makes it possible to use the swivel for pitch control during hovering.

When the mid tube is rotated, then the swivel output also rotates in the YZ plane. To keep the swivel thrust vector pointing of theta_output in the ZX plane it is therefore necessary to compensate for this by counter rotating the input tube via phi_input. The phi_input rotation of the input tube also provides swivel yaw control during hovering.

![Swivel model for forward flight](swivel_0_side_zx_mid.jpg)

![Swivel model for hover flight](swivel_4_alpha_is_90_output.jpg)

## 3. Swivel movement as function of swivel input rotation and mid rotation
### 3.1 Exact equations for swivel control
The matrix equations that yield the swivel output position, are available as function SwivelOutputPosition() in both swivel_functions.scad and swivel_functions.py. These matrix equations are expanded into gonio formules in SwivelOutputPosition_Gonio(), that is also available in both swivel_functions.scad and swivel_functions.py. This expansion yields huge formula for the swivel output position as function of swivel dimensions, the rotation of the swivel input tube (phi_input) and the swivel mid tube (theta_mid).

Rotating the mid tube by theta_mid changes the pointing of the output tube, for horizontal pointing theta_mid = 0, for maximum angle theta_mid = 180 degrees. The function phi_output_yz(theta_mid) shows how the phi_input of input tube needs to be counter rotated to keep the swivel pointing in the ZX plane during transition:

![phi_output_yz(theta_mid)](phi_output_yz_as_function_of_theta_mid.jpg)

The function theta_output_zx(theta_mid) shows how the output pointing depends on the rotation by theta_mid of the mid tube:

![theta_output_zx(theta_mid)](theta_output_zx_as_function_of_theta_mid.jpg)

### 3.2 Harmonic approximation using the DFT
The exact formula for swivel output control is huge, because it has in the order of hundred terms. Therefore it may need to be approximated to be able to implement it in a real time application, using e.g.:

* Precalculated lookup table with values from the exact formula
* Apprimation formula that is close to the exact formula, but much simpler to calculate

The function for phi_output_yz(theta_mid) is almost linear, the deviation suggests that adding the first harmonic frequency component deviation yields a good approximation. The function for theta_output_zx(theta_mid) looks like the first halve of a sinus, this suggest that the first harmonic frequency component of concat(theta_output_zx(theta_mid), -theta_output_zx(theta_mid)) yields a good approximation. The harmonic components are obtained using the Discrete Fourier Transform (DFT) for real input signals (rDFT). With analysis and plot from the swivel.ipynb jupyter note book this results in:

* phi_output_yz(theta_mid) ~= phi_output_horizontal + theta_mid / 2 - phi_output_f1_ampl * sin(theta_mid)
* theta_output_zx(theta_mid) ~= theta_output_horizontal + theta_output_f1_ampl * sin(theta_mid / 2)

and inverse:

* theta_mid ~= 2 * arcsin(theta_output_zx - theta_output_horizontal) / theta_output_f1_ampl)

where theta_output_f1_ampl ~= 4 alpha_tilt is the maximum angle for theta_output (because -1 <= x <= +1 for arcsin(x)).

### 3.3 Real input DFT support in OpenSCAD
The dft.scad implements the real input DFT in OpenSCAD. The implementation is uses matrix multiplication and is not optimezed like an Fast Fourier Transform (FFT). However with dft.scad it is possible to calculate the harmonic approximation of the swivel control entirely in OpenSCAD.

## 4. Conclusions
- The maximum swivel output angle is four times the tilt angle of the swivel tube segments. Using a tilt angle > 22.5 degrees therefore enables an maximum output angle > 90 degrees, which is necessary for pitch control beyond vertical down during hovering.
- The swivel is suited for forward flight (thrust vector pointing horizontal) and hover fligth (thrust vector pointing near vertical down)
- The swivel can transition smoothly between horizontal and vertical down, while keeping the thrust vector pointing in the ZX plane.
- The mid tube needs to rotate between 0 and 180 degrees to transition the swivel pointing between horizontal (zero) output angle and maximum output angle.
- The input tube needs to rotate between 0 and 90 degrees to compensate for rotations of the mid tube and by some more degrees e.g. 10, to provide yaw control.
- The swivel is not suitable for thrust vectoring near horizontal during forward flight, because would require full circle and instantaneous control of the input tube orientation.

## 5. Files

Add library path to OPENSCADPATH environment variable.

### 5.1 Swivel/openscad
* swivel.scad : main program with select options to 3D print, show and animate the swivel
* swivel_functions.scad : functions to calculate the orientation of the swivel
* swivel_functions_unit_test.scad : unit test for swivel_functions.scad
* swivel_assembly.scad : assemble the swivel module from its parts
* swivel_parts.scad : module parts for the swivel, the swivel model consists of four identical segment tubes

Dependencies:
* openscad/libraries/ from https

### 5.2 Swivel/python
Add library path to PYTHONPATH environment variable.

* swivel.ipynb, swivel.html - jupyter notebook to determine harmonic approximation of phi_output_yz(theta_mid) and theta_output_zx(theta_mid)
* swivel_functions.py - functions to calculate the orientation of the swivel
* try_swivel.py - try swivel pointings
