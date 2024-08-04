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
// Purpose: Library to create the parts for the swivel
// Description:
//  . The InputTube, MidTube and OutputTube each consist of SegmentTube
//    sections.
//  . The SegmentTube consists of the difference between an outer and an inner
//    SegmentBody.

include <shape_constants.scad>;  // include is needed for constants
use <triangles.scad>;   // use is sufficient for module and function

//------------------------------------------------------------------------------
// Swivel parts
//------------------------------------------------------------------------------

marker_color = "Purple";

module InputTube(x_input, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker) {
    SegmentTube("Green", x_input, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole,
                marker_color, L_marker, d_marker, h_marker);
}

module MidTube(x_mid, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker) {
    translate([x_mid/2, 0, 0]) union() {
        // use d_eps to overlap the Segments
        translate([-d_eps, 0, 0]) mirror([0, 0, 1])
            SegmentTube("Orange", x_mid/2 + d_eps, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole,
                        marker_color, L_marker, d_marker, h_marker);
        mirror([1, 0, 0]) translate([-d_eps, 0, 0]) mirror([0, 0, 1])
            SegmentTube("Orange", x_mid/2 + d_eps, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole,
                        marker_color, L_marker, d_marker, h_marker);
    }
}

module OutputTube(x_output, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole, L_marker, d_marker, h_marker) {
    translate([x_output, 0, 0]) mirror([1, 0, 0])
        SegmentTube("Red", x_output, R_wall_inner, d_wall, alpha_tilt, use_bottom, R_bottom_hole,
                    marker_color, L_marker, d_marker, h_marker);
}


//------------------------------------------------------------------------------
// Segment tube
//------------------------------------------------------------------------------

module SegmentTube(segment_color, h, R_wall_inner, d_wall, alpha_tilt,
                   use_bottom, R_bottom_hole,
                   marker_color=undef, L_marker=0, d_marker=0, h_marker=0) {
    // Create tube segment:
    // . with thickness d_wall,
    // . centered around x axis between x = 0 and x = h,
    // . rotated around x axis (so in YZ plane) such that long side markers are
    //   at positive Y in XY plane, to define initial swivel YZ angle = 0
    //   degrees,
    // . with optional bottom at x = 0, and with angle alpha_tilt applied at
    //   x = h.
    difference() {
        // Create tube by subracting inner body from outer body.
        difference() {
            // Outer body
            SegmentBody(segment_color, h, R_wall_inner + d_wall, alpha_tilt,
                        marker_color, L_marker, d_marker, h_marker);
            // Inner body, with optional bottom
            if (use_bottom == true) {
                // use d_wall for bottom and to extend inner body beyond outer body
                translate([-d_wall / cos(alpha_tilt), 0, 0])
                    SegmentBody(segment_color, h, R_wall_inner, alpha_tilt);
            } else {
                // use d_eps to extend inner body beyond outer body
                translate([-d_eps, 0, 0])
                    SegmentBody(segment_color, h + 2*d_eps, R_wall_inner, alpha_tilt);
            }
        }
        // Create small center hole to guide drilling a skrew hole in the
        // bottom use d_eps to extend cylinder beyond the bottom thickness.
        translate([h + d_eps, 0, 0]) rotate([0, -90 + alpha_tilt, 0])
            cylinder(d_wall + 2*d_eps, R_bottom_hole, R_bottom_hole, center = false);
    }
}

module SegmentBody(segment_color, h, R_wall_inner, alpha_tilt,
                   marker_color=undef, L_marker=0, d_marker=0, h_marker=0) {
    // Create body centered around x axis between x = 0 and x = h, with angle
    // alpha_tilt applied at x = h.
    // The 180 degrees markers have length L_marker, go d_marker deep in the
    // body and h_marker high outside the body.
    L = R_wall_inner / cos(alpha_tilt);
    union() {
        color(marker_color)
        if (L_marker > 0) {
            W_bottom = L_marker / 2;
            W_top = W_bottom / 2;
            // Left top, left bottom, rigth top and right bottom
            x_offset = 1.0*L_marker;
            translate([x_offset, 0, R_wall_inner - d_marker]) mirror([1, 0, 0])
                TriangleMarker(L_marker, W_bottom, W_top, d_marker + h_marker);
            mirror([0, 0, 1]) translate([x_offset, 0, R_wall_inner - d_marker]) mirror([1, 0, 0])
                TriangleMarker(L_marker, W_bottom, W_top, d_marker + h_marker);
            translate([h + L * sin(alpha_tilt) - x_offset, 0, R_wall_inner - d_marker])
                TriangleMarker(L_marker, W_bottom, W_top, d_marker + h_marker);
            mirror([0, 0, 1]) translate([h - L * sin(alpha_tilt) - x_offset, 0, R_wall_inner - d_marker])
                TriangleMarker(L_marker, W_bottom, W_top, d_marker + h_marker);
        }
        color(segment_color)
        hull() {
            // use d_eps to apply hull with 3D cylinder that is almost a 2D circle
            // use center = false to have exact height h
            // use +-90 to have cylinder extend in +-d_eps x direction, so that
            // height h is exact
            rotate([0, 90, 0]) cylinder(h=d_eps, r1=R_wall_inner, r2=R_wall_inner, center=false);
            translate([h, 0, 0]) rotate([0, -90 + alpha_tilt, 0]) cylinder(h=d_eps, r1=L, r2=L, center=false);
        }
    }
}
