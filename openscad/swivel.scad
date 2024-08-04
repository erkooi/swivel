//------------------------------------------------------------------------------
// Copyright 2022 E. Kooistra
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//------------------------------------------------------------------------------
//
// Author: E. Kooistra, March 2022
// Purpose: Create swivel
// Description:
//   Selections via select value to:
//   . Show static model of the swivel parts
//   . Form the swivel with certain orientation
//   . Animate the swivel for range of orientations
//   . 3D print swivel part
//
// Animation:
//   Animation repeatedly loops $t = 0:1/Steps:1, with Steps defined in GUI
//   menu View/Animate.
//   . Use e.g. FPS = 10 frames per second in GUI for movie, or FPS = 1 to see
//     individual frames.
//   . The animation stops when Steps in GUI is set to 0 or left empty.
//   . The animation restarts when Steps in GUI is set to 1 and then set to > 0.
//     E.g. to fill in Steps 80, first fill in the 0 and then the 8 to avoid
//     that the animation first uses Steps 8 and then continues with 80.
//
// Rerun file:
//   . Clear Console using right-mouse Clear
//   . Without animation Openscad reruns after the file is saved, or use
//     Preview F5 icon '>>' to rerun
//
// Coordinates (see openscad/math/linear_algebra.scad)
//
//     z
//     |         . angleYZ
//     |--- y    . angleZX
//    /          . angleXY
//   x

include <math_constants.scad>;
use <linear_algebra.scad>;
use <numscad.scad>;
use <swivel_functions.scad>;
use <swivel_parts.scad>;
use <swivel_assembly.scad>;

//------------------------------------------------------------------------------
// Select
//------------------------------------------------------------------------------
//   0   StaticSwivel() = 4 times SegmentTube() with x_spacer for exploded view
//   1   SegmentTube() for 3D print
//
//   2   FormSwivel() steer theta_mid with InputTube marker kept up
//   2.1 FormSwivel() animate theta_mid with InputTube marker kept up
//
//   3   FormSwivel() steer theta_mid with OutputTube pointing to phi_output_yz
//   3.1 FormSwivel() animate theta_mid with OutputTube in ZX plane pointing to phi_output_yz_vertical_down
//   3.2 FormSwivel() animate theta_mid with OutputTube in ZX plane pointing to phi_output_yz_vertical_down
//                    and to phi_output_yz_vertical_up
//
//   4   Formswivel() animate OutputTube hovering around phi_output_yz_vertical_down, using rDFT and
//                    harmonics approximation, as in swivel.ipynb
//   5   Formswivel() animate OutputTube transition between horizontal and phi_output_yz_vertical_down as 3.1,
//                    but using rDFT and harmonics approximation, as in swivel.ipynb
//
//   6   FormSwivel() show multiple swivel pointings in one view
//   7   FormSwivel() animate OutputTube sweep out and back one time, while rotating several times,
//                    to create a spiral pattern
//   8   FormSwivel() animate OutputTube sweep out and back several times, while rotating one time,
//                    to create a star pattern
//   9   Trial

select = 3.1;


//------------------------------------------------------------------------------
// Verbosity
//------------------------------------------------------------------------------

// 0   : do only what is needed
// > 0 : do more to e.g. echo status and verify results
verbosity = 1;

if (verbosity > 1) {
    echo("$fa = ", $fa);
    echo("$fs = ", $fs);
    echo("$fn = ", $fn);
}

//------------------------------------------------------------------------------
// Parameters
//------------------------------------------------------------------------------

// Swivel tube segment
R_wall_inner = 45;   // Tube inner radius
d_wall = 4;          // Tube wall thickness
alpha_tilt = 25;     // Segment slope angle
x_input = 50;        // Length of input segment
x_mid = 80;          // Length of middle segment
x_output = 40;       // Length of output segment
x_body = 50;         // Default length of segment for 3D print
x_spacer = 20;       // Segment x_spacer default 0, > 0 to better view them

swivel_tilt_max = SwivelTiltMax(alpha_tilt);  // Maximum thrust angle

// Optional segment bottom
no_bottom = false;
with_bottom = true;
R_bottom_hole = 1;  // Center drill hole in bottom

// Optional segment rotation markers
L_marker = 8;       // Place markers when > 0
d_marker = 1;       // Depth in the body
h_marker = 1;       // Height outside the body

//------------------------------------------------------------------------------
// Orientation
//------------------------------------------------------------------------------
// Swivel OutputTube() moves from straight to maximum tilted when theta goes
// from 0 to 180 or from 360 to 180 (= 0 to -180). Whether the swivel
// OutputTube() moves down, up, left, right etc., dependents on the InputTube()
// rotation angle phi_input_yz.
output_tube_pointing = "down";

// MidTube rotation angle theta_mid around x-axis in yz plane, positive
// theta_mid is:
// . from +z to -y, for MidTube with respect to InputTube
// . from +z to +y, for OutputTube with respect to MidTube
// The theta_mid is with respect to InputTube marker up, theta_mid is:
// . 0 for InputTube marker up long side aligned with MidTube short side
// . 0 - 180 OutputTube points to -y, so angle XY < 0
// . 180 for InputTube marker up long side aligned with MidTube long side
// . 180 - 360 OutputTube points to +y, so angle XY > 0
// For theta_mid = 0 or 360 the phi_input_yz angle is don't care and therefore
// undefined. This then causes 180 degrees flip rotation of the MidTube.
// Therefore use theta_mid + f_eps to avoid theta_mid = 0 or 360.
mid_tube_rotation = "positive";
//mid_tube_rotation = "negative";

theta_mid_for_thrust_straight = 0;
theta_mid_for_thrust_angle_max = 180;

// InputTube rotation angle phi_input_yz around x-axis in yz plane, phi_input
// is:
// . phi_input_yz = phi_input_yz_for_input_marker_up = 90 for markers on long
//   side of InputTube() on top, so with positive z in the ZX plane (y = 0).
phi_input_yz_for_input_marker_up = PhiInputYzForInputMarkerUp();  // = 90
phi_input_yz_vertical_down = PhiInputYzVertical("down");
phi_input_yz_vertical_up = PhiInputYzVertical("up");

// OutputTube pointing down angle in -z-axis direction in YZ plane
phi_output_yz_vertical_down = PhiOutputYzVertical("down");
phi_output_yz_vertical_up = PhiOutputYzVertical("up");

theta_output_zx_horizontal = ThetaOutputZxHorizontal();
theta_output_zx_vertical_down = ThetaOutputZxVertical("down");
theta_output_zx_vertical_up = ThetaOutputZxVertical("up");
theta_output_zx_max = ThetaOutputZxMax(alpha_tilt);


//------------------------------------------------------------------------------
// Run selection
//------------------------------------------------------------------------------
echo(select = select);
if (select == 0) {
    StaticSwivel(x_spacer, x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
                 with_bottom, R_bottom_hole,
                 L_marker, d_marker, h_marker);

} else if (select == 1) {
    // SegmentTube() for 3D print
    //$fa = 1;
    //$fs = 0.01;
    $fn = 100;
    segmentColor = "Red";
    markerColor = "Red";
    SegmentTube(segmentColor, x_body, R_wall_inner, d_wall, alpha_tilt,
                with_bottom, R_bottom_hole,
                markerColor, L_marker, d_marker, h_marker);

} else if (select == 2) {
    // Steer swivel via theta_mid and keep InputTube marker up
    theta_mid = 0;
    phi_input_yz = 0;
    //phi_input_yz = phi_input_yz_for_input_marker_up;  // = 90

    // Apply phi_input_yz_for_input_marker_up
    FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
               no_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
               phi_input_yz, theta_mid);

    if (verbosity > 0) {
        // Echo swivel status
        position_vector = SwivelOutputPosition(x_input, x_mid, x_output,
                                               phi_input_yz,
                                               theta_mid, alpha_tilt);
        echo_SwivelOutputPosition(phi_input_yz,
                                  theta_mid, alpha_tilt,
                                  position_vector);

        thrust_vector = SwivelThrustVector(x_input, x_mid, x_output,
                                           phi_input_yz,
                                           theta_mid, alpha_tilt);
        echo_SwivelThrustVector(phi_input_yz,
                                theta_mid, alpha_tilt,
                                thrust_vector);
    }

} else if (select == 2.1) {
    // Animate theta_mid with phi_input_yz_for_input_marker_up.
    // . move swivel from horizontal, down, to horizontal.
    // . phi_output_yz rotates in YZ plane from 180, 270 to 360 and then jumps
    //   back to 180.
    turn_back_theta = true;
    // Use ? to assign theta_mid in this scope. When theta_mid is assigned
    // within if-else then that is a different scope, that gets lost after
    // the if-else is left, causing theta_mid to be undefined.
    theta_mid_positive = ((turn_back_theta)
                             ?   ($t < 0.5 ? $t * 360 : (1 - $t) * 360)
                             :   ($t * 360)) + f_eps;
    theta_mid_negative = -1 * theta_mid_positive;
    theta_mid = (mid_tube_rotation == "positive") ? theta_mid_positive : theta_mid_negative;

    // Apply phi_input_yz_for_input_marker_up
    FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
               no_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
               phi_input_yz_for_input_marker_up, theta_mid);

    if (verbosity > 0) {
        // Echo swivel status
        phi_output_yz = toAngle360(fAngleYZ(SwivelOutputPosition(x_input, x_mid, x_output,
                                                                 phi_input_yz_for_input_marker_up,
                                                                 theta_mid, alpha_tilt)));
        echo(time = $t);
        echo(theta_mid = theta_mid);
        echo(phi_output_yz = phi_output_yz);
    }

} else if (select == 3) {
    // Steer theta_mid and adjust phi_input_yz to get phi_output_yz.
    theta_mid = 180;  // 0 is straight, 180 is maximum down
    phi_output_yz = phi_output_yz_vertical_down;

    // Determine phi_input_yz for requested phi_output_yz and given theta_mid
    phi_input_yz = DeterminePhiInputYzForPhiOutputYzAndThetaMid(x_input, x_mid, x_output, alpha_tilt,
                                                                phi_output_yz, theta_mid);

    // Apply phi_input_yz
    FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
               no_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
               phi_input_yz, theta_mid);

    if (verbosity > 0) {
        // Echo swivel status
        position_vector_with_input_marker_up = SwivelOutputPosition(x_input, x_mid, x_output,
                                                                    phi_input_yz_for_input_marker_up,
                                                                    theta_mid, alpha_tilt);
        echo_SwivelOutputPosition(phi_input_yz_for_input_marker_up,
                                  theta_mid, alpha_tilt,
                                  position_vector_with_input_marker_up);

        echo(theta_mid = theta_mid);
        echo(phi_input_yz = phi_input_yz);
        echo(phi_output_yz = phi_output_yz);

        // Verify swivel status
        position_vector = SwivelOutputPosition(x_input, x_mid, x_output,
                                               phi_input_yz,
                                               theta_mid, alpha_tilt);
        thrust_vector = SwivelThrustVector(x_input, x_mid, x_output,
                                           phi_input_yz,
                                           theta_mid, alpha_tilt);

        position_angle_YZ = toAngle360(fAngleYZ(position_vector));
        thrust_angle_YZ = toAngle360(fAngleYZ(thrust_vector));
        thrust_angle_ZX = toAngle360(fAngleZX(thrust_vector));
        thrust_angle_XR = toAngle360(fAngleXR(thrust_vector));
        echo(position_angle_YZ = position_angle_YZ);
        echo(thrust_angle_YZ = thrust_angle_YZ);
        echo(thrust_angle_ZX = thrust_angle_ZX);
        echo(thrust_angle_XR = thrust_angle_XR);

        assert(abs(position_angle_YZ - thrust_angle_YZ) < f_eps,
            "Swivel position_vector angle_YZ != Swivel thrust_vector angle_YZ");
        assert(abs(position_angle_YZ - phi_output_yz) < f_eps,
            "Swivel position_vector angle_XY != phi_output_yz");
        assert(abs(thrust_angle_YZ - phi_output_yz) < f_eps,
            "Swivel thrust_vector angle_XY != phi_output_yz");
        assert(abs(thrust_angle_ZX - theta_output_zx_horizontal - thrust_angle_XR) < f_eps,
            "Swivel thrust_vector angle_ZX - theta_output_zx_horizontal != angle_XR");
    }

} else if (select == 3.1) {
    // Animate theta_mid 0:360 with OutputTube pointing to
    // phi_output_yz_vertical_down
    // . move swivel from horizontal, down, to horizontal.
    // . keep phi_output_yz = phi_output_yz_vertical_down
    // . when turn_back_theta = false then theta_mid is 0 to 360 and
    //   phi_input_yz then rotates in YZ plane from 90, 0,360 to 270 and then
    //   jumps back to 90, with so a jump of 180. Use turn_back_theta = true
    //   for theta_mid 0 to 180 back to 0 to avoid this 180 jump in
    //   phi_input_yz after each repetition.
    // . add +f_eps in same direction as theta_mid, to avoid 180 degrees jump
    //   in phi_input_yz between theta_mid = 0 and +0.
    turn_back_theta = true;
    // Use ? to assign theta_mid in this scope. When theta_mid is assigned
    // within if-else then that is a different scope, that gets lost after
    // the if-else is left, causing theta_mid to be undefined.
    theta_mid_positive = ((turn_back_theta)
                             ?   ($t < 0.5 ? $t * 360 : (1 - $t) * 360)
                             :   ($t * 360)) + f_eps;
    theta_mid_negative = -1 * theta_mid_positive;
    theta_mid = (mid_tube_rotation == "positive") ? theta_mid_positive : theta_mid_negative;
    phi_output_yz = phi_output_yz_vertical_down;

    // Determine phi_input_yz for requested phi_output_yz and given theta_mid
    phi_input_yz = DeterminePhiInputYzForPhiOutputYzAndThetaMid(x_input, x_mid, x_output, alpha_tilt,
                                                                phi_output_yz, theta_mid);

    // Apply phi_input_yz
    FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
               no_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
               phi_input_yz, theta_mid);

    if (verbosity > 0) {
        // Echo swivel status
        echo(time = $t,
             theta_mid = theta_mid,
             phi_input_yz = phi_input_yz,
             phi_output_yz = phi_output_yz);
    }

} else if (select == 3.2) {
    // Animate theta_mid 0:360 with OutputTube pointing to phi_output_yz_vertical_down
    // and then again 360:720 with OutputTube pointing to phi_output_yz_vertical_up.
    // This only works for full down / up swing, because theta_mid needs to rotate
    // full circle twice, so that phi_output_yz can be changed at theta_mid = 0, without
    // causing a jump in phi_input_yz. Therefore it is not possible to swing between
    // less than full down / less than full up, without having a jump in phi_input_yz.
    // . move swivel from horizontal, down, horizontal, up, horizontal.
    // . keep phi_output_yz = phi_output_yz_vertical_down first round and then
    //   at phi_output_yz_vertical_up second round
    // . phi_input_yz then rotates in YZ plane from 90, 0,360 to 270, 180, 90,
    //   without jumps.
    twice = 2;
    phi_output_yz = $t < 1/twice ? phi_output_yz_vertical_down : phi_output_yz_vertical_up;
    theta_mid = toAngle360($t * twice * 360 + f_eps);

    // Determine phi_input_yz for requested phi_output_yz and given theta_mid
    phi_input_yz = DeterminePhiInputYzForPhiOutputYzAndThetaMid(x_input, x_mid, x_output, alpha_tilt,
                                                                phi_output_yz, theta_mid);

    // Apply phi_input_yz
    FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
               no_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
               phi_input_yz, theta_mid);

    if (verbosity > 0) {
        // Echo swivel status
        echo(time = $t,
             theta_mid = theta_mid,
             phi_input_yz = phi_input_yz,
             phi_output_yz = phi_output_yz);
    }

} else if (select == 4) {
    // Animate OutputTube hovering around phi_output_yz_vertical_down with angle
    // psi_hover and radius r_hover, using hover approximations from
    // swivel.ipynb.
    r_hover_max = theta_output_zx_max - theta_output_zx_vertical_down;
    r_hover = r_hover_max;

    // Use fixed psi_hover angle or animate psi_hover angles
    //animate = false;
    animate = true;
    // . If animate == false then use fixed special psi_hover angles in XY
    //   plane from swivel.ipynb, to cross verify between scad and ipynb
    hover_pitch_error_max_psi = 0;
    hover_yaw_error_max_psi = 105.820312;
    psi_hover = animate ? $t * 360 : hover_pitch_error_max_psi;
    //psi_hover = animate ? $t * 360 : hover_yaw_error_max_psi;

    pitch = -1 * r_hover * cos(psi_hover);  // vary theta_output for x direction
    yaw = r_hover * sin(psi_hover);  // vary phi_output_yz for y direction

    // Prepare single harmonic approximations
    // . for phi_output_yz from theta_mid (to keep swivel motion in ZX plane)
    phi_output_yz_rdft_bin_arr = rDftBinsOfPhiOutputYzAsFunctionOfThetaMid(alpha_tilt,
                                                                           output_tube_pointing,
                                                                           mid_tube_rotation);
    phi_ampl_arr = phi_output_yz_rdft_bin_arr[0];
    phi_output_yz_f0_ampl = phi_ampl_arr[0];  // = DC = InitPhiOutputYzHorizontal()
    phi_output_yz_f1_ampl = phi_ampl_arr[1];  // = few degrees for the small sinus like deviation from linear
    echo(phi_output_yz_f1_ampl = phi_output_yz_f1_ampl);

    // . for theta_output from theta_mid (to point swivel thrust for transition
    //   between horizontal for forward flight, and down for hover).
    theta_output_rdft_bin_arr = rDftBinsOfThetaOutputZxAsFunctionOfThetaMid(alpha_tilt,
                                                                            output_tube_pointing,
                                                                            mid_tube_rotation);
    theta_ampl_arr = theta_output_rdft_bin_arr[0];
    theta_output_f0_ampl = theta_ampl_arr[0];  // = theta_output_zx_horizontal
    theta_output_f1_ampl = theta_ampl_arr[1];  // ~= swivel_tilt_max
    theta_output_zx_max = theta_output_f1_ampl;  // maximum for this approximation
    echo(theta_output_f1_ampl = theta_output_f1_ampl);

    // Determine approximate theta_mid from theta_output
    theta_output_zx = theta_output_zx_vertical_down + pitch;
    hover_theta_mid = ApproximateThetaMidAsFunctionOfThetaOutputZx(theta_output_f0_ampl,
                                                                   theta_output_f1_ampl,
                                                                   theta_output_zx);

    // Determine approximate phi_output_yz from approximate theta_mid
    hover_phi_offset = ApproximatePhiOutputYzAsFunctionOfThetaMid(phi_output_yz_f0_ampl,
                                                                  phi_output_yz_f1_ampl,
                                                                  hover_theta_mid);
    hover_phi_adjust = phi_output_yz_vertical_down - hover_phi_offset;

    // Adjust phi_input_yz to achieve wanted phi_output_yz
    hover_phi_input_yz = phi_input_yz_vertical_down + yaw + hover_phi_adjust;

    // Apply hover_phi_input_yz and hover_theta_mid for pitch and yaw
    FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
               no_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
               hover_phi_input_yz, hover_theta_mid);

    if (verbosity > 0) {
        // Verify swivel status
        thrust_vector = SwivelThrustVector(x_input, x_mid, x_output,
                                           hover_phi_input_yz,
                                           hover_theta_mid, alpha_tilt);

        // pitch
        thrust_angle_zx = theta_output_zx_horizontal + toAngle360(fAngleXR(thrust_vector));
        hover_pitch = thrust_angle_zx - theta_output_zx_vertical_down;
        hover_pitch_error = pitch - hover_pitch;

        // yaw
        thrust_angle_yz = toAngle360(fAngleYZ(thrust_vector));
        hover_yaw = thrust_angle_yz - phi_output_yz_vertical_down;
        hover_yaw_error = yaw - hover_yaw;

        echo(psi_hover = psi_hover,
             hover_pitch = hover_pitch,
             hover_yaw = hover_yaw,
             hover_pitch_error = hover_pitch_error,
             hover_yaw_error = hover_yaw_error);
    }

} else if (select == 5) {
    // Prepare single harmonic approximations
    // . for phi_output_yz from theta_mid (to keep swivel motion in ZX plane)
    phi_output_yz_rdft_bin_arr = rDftBinsOfPhiOutputYzAsFunctionOfThetaMid(alpha_tilt,
                                                                           output_tube_pointing,
                                                                           mid_tube_rotation);
    phi_ampl_arr = phi_output_yz_rdft_bin_arr[0];
    phi_output_yz_f0_ampl = phi_ampl_arr[0];  // = DC = InitPhiOutputYzHorizontal()
    phi_output_yz_f1_ampl = phi_ampl_arr[1];  // = few degrees for the small sinus like deviation from linear
    echo(phi_output_yz_f1_ampl = phi_output_yz_f1_ampl);

    // . for theta_output from theta_mid (to point swivel thrust for transition
    //   between horizontal for forward flight, and down for hover).
    theta_output_rdft_bin_arr = rDftBinsOfThetaOutputZxAsFunctionOfThetaMid(alpha_tilt,
                                                                            output_tube_pointing,
                                                                            mid_tube_rotation);
    theta_ampl_arr = theta_output_rdft_bin_arr[0];
    theta_output_f0_ampl = theta_ampl_arr[0];  // = theta_output_zx_horizontal
    theta_output_f1_ampl = theta_ampl_arr[1];  // ~= swivel_tilt_max
    theta_output_zx_max = theta_output_f1_ampl;  // maximum for this approximation
    echo(theta_output_f1_ampl = theta_output_f1_ampl);

    // Animate OutputTube transition
    init_phi_input_yz = 0;  // use 0 for swivel movement in ZX plane
    theta_output_zx = theta_output_zx_horizontal + ($t < 0.5 ? 2 * $t * theta_output_zx_max : 2 * (1 - $t) * theta_output_zx_max);

    // Determine approximate theta_mid from theta_output_zx
    transition_theta_mid = ApproximateThetaMidAsFunctionOfThetaOutputZx(theta_output_f0_ampl,
                                                                        theta_output_f1_ampl,
                                                                        theta_output_zx);
    echo(theta_output_zx = theta_output_zx);
    echo(transition_theta_mid = transition_theta_mid);

    // Determine approximate phi_output_yz from approximate theta_mid
    transition_phi_offset = ApproximatePhiOutputYzAsFunctionOfThetaMid(phi_output_yz_f0_ampl,
                                                                       phi_output_yz_f1_ampl,
                                                                       transition_theta_mid);
    transition_phi_adjust = phi_output_yz_vertical_down - transition_phi_offset;

    // Adjust phi_input_yz to achieve wanted phi_output_yz
    transition_phi_input_yz = phi_input_yz_vertical_down + init_phi_input_yz + transition_phi_adjust;

    // Apply transition_phi_input_yz and transition_theta_mid for pitch and yaw
    FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
               no_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
               transition_phi_input_yz, transition_theta_mid);

    if (verbosity > 0) {
        // Verify swivel status
        thrust_vector = SwivelThrustVector(x_input, x_mid, x_output,
                                           transition_phi_input_yz,
                                           transition_theta_mid, alpha_tilt);

        // pitch
        thrust_angle_zx = theta_output_zx_horizontal + toAngle360(fAngleXR(thrust_vector));
        transition_theta_output = thrust_angle_zx;
        transition_theta_output_error = theta_output_zx - transition_theta_output;

        // yaw
        thrust_angle_yz = toAngle360(fAngleYZ(thrust_vector));
        transition_phi_output_yz = thrust_angle_yz;
        transition_phi_output_yz_error = phi_output_yz_vertical_down - transition_phi_output_yz;

        echo(transition_theta_output = transition_theta_output,
             transition_phi_output_yz = transition_phi_output_yz,
             transition_theta_output_error = transition_theta_output_error,
             transition_phi_output_yz_error = transition_phi_output_yz_error);
    }

} else if (select == 6) {
    // Show multiple swivel pointings in one view
    step = 60;
    for (theta_mid = [0:step:360-1]) {
        for (phi_output_yz = [0:step:360-1]) {
            // Determine phi_input_yz for requested phi_output_yz and given theta_mid
            phi_input_yz = DeterminePhiInputYzForPhiOutputYzAndThetaMid(x_input, x_mid, x_output, alpha_tilt,
                                                                        phi_output_yz, theta_mid);

            // Show each phi_input_yz
            if (step > 15) {
                FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
                           no_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
                           phi_input_yz, theta_mid);
            }

            if (verbosity > 0) {
                // Verify that position_angle_yz == thrust_angle_yz == phi_output360
                position_vector = SwivelOutputPosition(x_input, x_mid, x_output,
                                                       phi_input_yz,
                                                       theta_mid, alpha_tilt);
                thrust_vector = SwivelThrustVector(x_input, x_mid, x_output,
                                                   phi_input_yz,
                                                   theta_mid, alpha_tilt);

                // Map position_vector YZ angle and thrust_vector YZ angle to range [0, 360> of phi_output_yz
                position_angle_yz = toAngle360(fAngleYZ(position_vector));
                thrust_angle_yz = toAngle360(fAngleYZ(thrust_vector));
                phi_output_yz360 = toAngle360(phi_output_yz);

                echo(theta_mid = theta_mid,
                     position_vector = num_zero_if_close(position_vector, f_eps),
                     thrust_vector = num_zero_if_close(thrust_vector, f_eps),
                     angles_yz = phi_output_yz, phi_output_yz360, position_angle_yz, thrust_angle_yz);

                if (theta_mid != 0) {
                    assert(abs(position_angle_yz - thrust_angle_yz) < f_eps,
                        "position_angle_yz != thrust_angle_yz");
                    if (abs(position_vector.y) > f_eps || abs(position_vector.z) > f_eps) {
                        assert(abs(position_angle_yz - phi_output_yz360) < f_eps,
                            "position_angle_yz != phi_output_yz360");
                    }
                }
            }
        }
    }

} else if (select == 7) {
    // Spiral swivel pointings out max_theta_output degrees and back in to 0,
    // and phi_output_yz rotating nof_rotations times
    nof_rotations = 5;
    // theta_mid as in select 3.1 with turn_back_theta = true.
    // The required theta_mid is about two times theta_output. The theta_output
    // is the deviation angle of the output thrust vector from horizontal.
    max_theta_output = 30;
    max_theta_mid = max_theta_output * 2;
    exponential = 1;  // use > 1 to increase theta_mid more slowly for small theta_mid
    // t_outward = 0   to 0.5 for t =   0 - < 0.5
    // t_inward  = 0.5 to 0   for t = 0.5 - < 1.0
    t_outward = $t^exponential / 0.5^(exponential - 1);
    t_inward = (1 - $t)^exponential / 0.5^(exponential - 1);
    theta_mid = (($t < 0.5)
                  ?   t_outward * max_theta_mid
                  :   t_inward * max_theta_mid) + f_eps;
    phi_output_yz = phi_output_yz_vertical_down + $t * 360 * nof_rotations;

    // Determine phi_input_yz for requested phi_output_yz and given theta_mid
    phi_input_yz = DeterminePhiInputYzForPhiOutputYzAndThetaMid(x_input, x_mid, x_output, alpha_tilt,
                                                                phi_output_yz, theta_mid);

    FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
               no_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
               phi_input_yz, theta_mid);


} else if (select == 8) {
    // Swing nof_pointings times between less than full up and less than full
    // down, and at the same time rotate phi_output_yz full circle to create a
    // star pattern. This is only feasible with jumps in phi_input_yz, when the
    // swivel crosses the horizontal direction (see 3.2).
    // Use max_theta_output as maximum deviation angle of the output thrust
    // vector from horizontal. Use theta_mid is about twice theta_output.
    nof_pointings = 4;
    max_theta_output = 10;
    max_theta_mid = max_theta_output * 2;
    // . timeline
    tp = nof_pointings * $t;  // tp time has increased by 1 per pointing
    ti = floor(tp);  // ti pointing index in range 0 to nof_pointings - 1
    // . create pointings, using theta_mid is 0 to +max_theta_mid to 0 for each
    //   pointing
    sawtooth_up = 2 * (tp % 1);  // one sawtooth up per pointing
    sawtooth_down = 2 - sawtooth_up;  // one sawtooth down per pointing
    triangle = (tp % 1) < 0.5 ? sawtooth_up : sawtooth_down;
    theta_mid = toAngle360(triangle * max_theta_mid + f_eps);

    // . start with vertical down
    phi_output_yz = phi_output_yz_vertical_down;

    // Determine phi_input_yz for requested phi_output_yz and given theta_mid
    phi_input_yz = DeterminePhiInputYzForPhiOutputYzAndThetaMid(x_input, x_mid, x_output, alpha_tilt,
                                                                phi_output_yz, theta_mid);

    // . create star pattern, by rotating phi_input_yz_star by
    //   360 / nof_pointings, after each pointing
    phi_input_yz_offset = 360 * ti / nof_pointings;  // rotate in steps
    //phi_input_yz_offset = 360 * tp / nof_pointings;  // rotate gradually

    phi_input_yz_star = phi_input_yz + phi_input_yz_offset;

    if (verbosity > 0) {
        thrust_vector = SwivelThrustVector(x_input, x_mid, x_output,
                                           phi_input_yz_star,
                                           theta_mid, alpha_tilt);
        thrust_angle_xr = fAngleXR(thrust_vector);

        echo(nof_pointings = nof_pointings);
        echo(ti = ti,
             phi_input_yz_star = phi_input_yz_star,
             theta_mid = theta_mid,
             thrust_angle_xr = thrust_angle_xr);
    }

    // Apply phi_input_yz_star
    FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
               no_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
               phi_input_yz_star, theta_mid);

} else if (select == 9) {

}
