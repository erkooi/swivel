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
// Author: E. Kooistra, Jul 2024
// Purpose: Library to calculate the orientation of the swivel
// Description:
// . Unit tests for swivel_functions.scad
// . Put unit test in separate file, because if it is kept within
//   swivel_functions.scad, then that slows down simulation and animation of
//   files that use swivel_functions, due to the time consuming rDFT.

include <math_constants.scad>;
use <numscad.scad>;
use <swivel_functions.scad>;

//------------------------------------------------------------------------------
// UNIT TEST for swivel_functions.scad
//------------------------------------------------------------------------------

x_input = 50;
x_mid = 80;
x_output = 40;
phi_input_yz = 0;
theta_mid = 60;
alpha_tilt = 25;


//------------------------------------------------------------------------------
// Echo position_vector
//------------------------------------------------------------------------------
position_vector = SwivelOutputPosition_Gonio(x_input, x_mid, x_output, phi_input_yz, theta_mid, alpha_tilt);
echo_SwivelOutputPosition(phi_input_yz, theta_mid, alpha_tilt, position_vector);


//------------------------------------------------------------------------------
// Echo thrust_vector
//------------------------------------------------------------------------------
thrust_vector = SwivelThrustVector(x_input, x_mid, x_output, phi_input_yz, theta_mid, alpha_tilt);
thrust_vector_111 = SwivelThrustVector(1, 1, 1, phi_input_yz, theta_mid, alpha_tilt);
echo(thrust_vector = thrust_vector);
echo_SwivelThrustVector(phi_input_yz, theta_mid, alpha_tilt, thrust_vector);
echo(thrust_vector_111 = thrust_vector_111);
echo_SwivelThrustVector(phi_input_yz, theta_mid, alpha_tilt, thrust_vector_111);


//------------------------------------------------------------------------------
// Verify SwivelOutputPosition_Gonio() == SwivelOutputPosition()
//------------------------------------------------------------------------------
echo("Verify SwivelOutputPosition_Gonio() == SwivelOutputPosition()");
step = 30;
for (theta_mid = [0:step:360]) {
    for (phi_input_yz = [0:step:360]) {
        position_vector       = SwivelOutputPosition(      x_input, x_mid, x_output, phi_input_yz, theta_mid, alpha_tilt);
        position_vector_gonio = SwivelOutputPosition_Gonio(x_input, x_mid, x_output, phi_input_yz, theta_mid, alpha_tilt);

        assert(abs(position_vector[0] - position_vector_gonio[0]) < f_eps, "position_vector.x != position_vector_gonio.x");
        assert(abs(position_vector[1] - position_vector_gonio[1]) < f_eps, "position_vector.y != position_vector_gonio.y");
        assert(abs(position_vector[2] - position_vector_gonio[2]) < f_eps, "position_vector.z != position_vector_gonio.z");
    }
}


//------------------------------------------------------------------------------
// Verify harmonic approximations
//------------------------------------------------------------------------------
output_tube_pointing = "down";
mid_tube_rotation = "positive";
B = 8;   // number of bins (harmonics including DC bin 0) to verify
N = 32;  // DFT size, choose B <= ceil(N / 2)

// Verify phi_output_yz(theta_mid)
echo();
echo("Verify phi_output_yz(theta_mid)");
phi_output_yz_bin_arr = rDftBinsOfPhiOutputYzAsFunctionOfThetaMid(alpha_tilt,
                                                                  B, N);
phi_output_yz_ampl_arr = phi_output_yz_bin_arr[0];
phi_output_yz_angle_arr = phi_output_yz_bin_arr[1];

// . expected values from swivel.ipynb for N = 1024
phi_output_yz_ampl_exp = [0.00000000000000,
                          2.81600296266200,
                          0.06920119381489,
                          0.00226742433106,
                          0.00008358041842,
                          0.00000328628332,
                          0.00000013459664,
                          0.00000000567019,
                          0.00000000024384,
                          0.00000000001065,
                          0.00000000000047];

phi_output_yz_angle_exp = [  0.0,
                            90.0,
                           -90.0,
                            90.0,
                           -90.0,
                            90.0,
                           -90.0,
                            90.0,
                           -90.0,
                            90.0,
                           -90.0];

phi_ampl_exp = num_slice(phi_output_yz_ampl_exp, 0, B - 1);
phi_angle_exp = num_slice(phi_output_yz_angle_exp, 0, B - 1);

echo(phi_output_yz_ampl_arr = phi_output_yz_ampl_arr);
echo(phi_output_yz_ampl_exp = phi_ampl_exp);
echo(phi_output_yz_angle_arr = phi_output_yz_angle_arr);
echo(phi_output_yz_angle_exp = phi_angle_exp);

assert (num_allclose(phi_output_yz_ampl_arr, phi_ampl_exp, rtol=1e-05, atol=1e-08), "Wrong phi_output_yz_ampl_arr");
assert (num_allclose(phi_output_yz_angle_arr, phi_angle_exp, rtol=1e-05, atol=1e-08), "Wrong phi_output_yz_angle_arr");


// Verify theta_output_zx(theta_mid)
echo();
echo("Verify theta_output_zx(theta_mid)");
theta_output_zx_bin_arr = rDftBinsOfThetaOutputZxAsFunctionOfThetaMid(alpha_tilt,
                                                                      mid_tube_rotation,
                                                                      B, N);
theta_output_zx_ampl_arr = theta_output_zx_bin_arr[0];
theta_output_zx_angle_arr = theta_output_zx_bin_arr[1];

// . expected values from swivel.ipynb for N = 1024
theta_output_zx_ampl_exp = [90,  // = DC = ThetaOutputZxHorizontal()
                            99.179360973761930609,
                             0,
                             0.802431588024201337,
                             0,
                             0.017673589176769072,
                             0,
                             0.000515964517513393,
                             0,
                             0.000017236506752680,
                             0,
                             0.000000623287702789,
                             0,
                             0.000000023746520276,
                             0];

theta_output_zx_angle_exp = [  0,
                             -90.0,
                               0,
                              90.0,
                               0,
                             -90.0,
                               0,
                              90.0,
                               0,
                             -90.0,
                               0,
                              90.0,
                               0,
                             -90.0,
                               0];

theta_ampl_exp = num_slice(theta_output_zx_ampl_exp, 0, B - 1);
theta_angle_exp = num_slice(theta_output_zx_angle_exp, 0, B - 1);
echo(theta_output_zx_ampl_arr = theta_output_zx_ampl_arr);
echo(theta_output_zx_ampl_exp = theta_ampl_exp);
echo(theta_output_zx_angle_arr = theta_output_zx_angle_arr);
echo(theta_output_zx_angle_exp = theta_angle_exp);

assert (num_allclose(theta_output_zx_ampl_arr, theta_ampl_exp, rtol=1e-05, atol=1e-08), "Wrong theta_output_zx_ampl_arr");
assert (num_allclose(theta_output_zx_angle_arr, theta_angle_exp, rtol=1e-05, atol=1e-08), "Wrong theta_output_zx_angle_arr");

//------------------------------------------------------------------------------
// All asserts when OK when program gets to here
//------------------------------------------------------------------------------
echo();
echo("PASSED");
