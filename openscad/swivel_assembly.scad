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
// Purpose: Library to assemble the swivel module from its parts.
// Description:
//  . Static swivel with spacer between the tube segments
//  . Form swivel from tube segments using rotate and translate from OpenSCAD
//    or using 4D transformation matrices for rotation and translation. The
//    result is identical, so the matrix equations can also be used in Python.

use <linear_algebra.scad>;
use <swivel_parts.scad>;

// The swivel assembly is constructed with InputTube() long side markers up,
// so with phi_input_yz = 90. However the zero position for the long side
// markers is better defined at phi_input_yz = 0. Therefore use
// c_phi_input_yz_construction to compensate for this construction offset, by
// subracting it from the input phi_input_yz in FormSwivel() and
// FormSwivel_Gonio().
include <swivel_constants.scad>;


//------------------------------------------------------------------------------
// Swivel assembly
//------------------------------------------------------------------------------

// Create exploded view of the swivel in horizontal direction, by means of
// x_spacer > 0
module StaticSwivel(x_spacer, x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
                    use_bottom, R_bottom_hole, L_marker, d_marker, h_marker) {
    InputTube(x_input, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker);
    translate([x_spacer + x_input, 0, 0])
        MidTube(x_mid, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker);
    translate([x_spacer*2 + x_input + x_mid, 0, 0])
        OutputTube(x_output, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker);
}

// Create the swivel, using translate() and rotate() transformations
module FormSwivel(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
                  use_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
                  phi_input_yz, theta_mid) {
    rotate([phi_input_yz - c_phi_input_yz_construction, 0, 0]) union() {
        InputTube(x_input, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker);
        translate([x_input, 0, 0]) rotate([0, alpha_tilt, 0]) rotate([theta_mid, 0, 0]) rotate([0, -alpha_tilt, 0]) union() {
            MidTube(x_mid, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker);
            translate([x_mid, 0, 0]) rotate([0, -alpha_tilt, 0]) rotate([-theta_mid, 0, 0]) rotate([0, alpha_tilt, 0])
                OutputTube(x_output, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker);
        }
    }
}

// Create the swivel, using 4D matrices
// FormSwivel_Gonio = FormSwivel, but using gonio matrix equations
module FormSwivel_Gonio(x_input, x_mid, x_output, R_wall_inner, d_wall, alpha_tilt,
                        use_bottom, R_bottom_hole, L_marker, d_marker, h_marker,
                        phi_input_yz, theta_mid) {
    mRot_yz(phi_input_yz - c_phi_input_yz_construction) union() {
        InputTube(x_input, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker);
        mTrans(x_input, 0, 0) mRot_zx(alpha_tilt) mRot_yz(theta_mid) mRot_zx(-alpha_tilt) union() {
            MidTube(x_mid, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker);
            mTrans(x_mid, 0, 0) mRot_zx(-alpha_tilt) mRot_yz(-theta_mid) mRot_zx(alpha_tilt)
                OutputTube(x_output, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker);
        }
    }
}

