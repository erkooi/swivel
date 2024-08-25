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
//                      July 2024 updated
// Purpose: Library to calculate the orientation of the swivel
// Description:
//  . position vector of OutputTube in FormSwivel (in swivel_assembly.scad)
//    with respect to origin. Calculated using matrices or using the gonio
//    formulas that follow from elaborating the sequence of matrices.
//  . harmonic approximation formulas for swivel control, using rDFT to
//    determine the real input DFT of the exact formula output
//  . thrust vector of OutputTube pointing in FormSwivel.
//  . When position_vector and thrust_vector point horizontal along the x-axis,
//    then angle YZ is undef, because then phi_input_yz can have any value.


include <math_constants.scad>;
include <swivel_constants.scad>;
use <numscad.scad>;
use <linear_algebra.scad>;
use <dft.scad>;


//------------------------------------------------------------------------------
// Coordinates (see linear_algebra.scad)
//------------------------------------------------------------------------------
//     z
//     |         . angleYZ
//     |--- y    . angleZX
//    /          . angleXY
//   x

//------------------------------------------------------------------------------
// Swivel position
//------------------------------------------------------------------------------
// Horizontal swivel is lined along the x-axis:
// . input of InputTube() at x = 0,
// . output of OutputTube() at x = x_input + x_mid + x_output,
// . positive x is swivel thrust direction, so negative x is airplane movement
//   direction
//
// Angle defintions:
// * alpha_tilt:
//   . Fixed tilted angle of the SegmentTube() at which the tubes connect to
//     eachother.
//   . The InputTube and MidTube and the MidTube and OutputTube can rotate
//     along the tilted plane at which they connect.
//   . Maximum tilted swivel thrust angle is 4 * alpha_tilt, because:
//     . InputTube() and MidTube() can angle between 0 and 2 * alpha_tilt
//     . MidTube() and OutputTube() can angle between 0 and 2 * alpha_tilt
//   . For alpha_tilt > 22.5 the swivel thrust angle can point beyond 90
//     degrees of vertical down, so with a thrust component in forward, i.e.
//     negative x direction, to force the airplane backward. This provides
//     pitch control during hovering.
// * theta_mid is rotation of the MidTube() in the alpha_tilt plane:
//   . rotates InputTube() // MidTube() and MidTube() \\ OutputTube() in equal,
//     but opposite directions.
//   . The swivel tubes all remain in the same plane, because the InputTube
//     and OutputTube always rotate in equal, but opposite theta_mid directions
//     with repect to the MidTube.
//   . theta_mid is 0 for horizontal, straight swivel:
//     . when markers on long side of InputTube() and OutputTube() line up with
//       the markers on the short side of the MidTube().
//   . theta_mid != 0 for tilted swivel.
//   . theta_mid is 180 for maximum tilted swivel:
//     . when markers on long sides of the tubes all line up
//     . pointing down to -z when phi_input_yz = 90 = PhiInputYzVertical("down")
//     . pointing up to +z when phi_input_yz = -90 = PhiInputYzVertical("up")
//   . mid_tube_rotation = 'positive' or 'negative'
//     . For positive the MidTube() rotates counter clockwise in tilted alpha
//       plane with respect to the InputTube() and clockwise with respect to
//       the OutputTube().
//     . The mid_tube_rotation defines the swivel mounting, regarding YZ
//       orientation, in an airplane. When the mid_tube_rotation is changed
//       then the InputTube YZ angle needs to be changed 180 degrees, to keep
//       the swivel pointing down.
//   . rotating the MidTube() by theta_mid steers the thrust vector output
//     angle theta_output_xr with respect to the x-axis, but also affects the
//     phi_output_yz angle. Therefore the phi_input_yz angle needs to be
//     adjusted dependent on theta_mid, to keep phi_output_yz constant, so that
//     the swivel thrust output keeps pointing in the ZX plane.
// * theta_output_xr is angle with respect to x-axis of OutputTube() thrust
//   vector:
//     . theta_output_xr_= 0, for straight swivel with theta_mid = 0,
//     . theta_output_xr = 4 * alpha_tilt, for maximum tilted swivel with
//       theta_mid = 180.
// * theta_output_zx is angle in ZX plane, for swivel pointing down in ZX plane:
//     . theta_output_zx = theta_output_zx_horizontal + theta_output_xr
// * phi_input_yz is rotation of swivel InputTube() in YZ plane about the
//   x-axis:
//   . phi_input_yz = 90 = PhiInputYzForInputMarkerUp(), defines when markers
//     on long input side of InputTube() are on top, so on the +z axis.
//   . when theta_mid rotates in positive direction from 0 to 180, then
//     phi_input_yz needs to counter rotate in negative direction from 180 to
//     90 to tilt the swivel down and to keep the swivel in the ZX plane:
//     . InitPhiInputYzHorizontal("down", "positive") = 180
//     . start horizontal with phi_input_yz = 180, so marker on long side of
//       InputTube() at -y, then:
//         theta_mid  phi_input_yz
//                 0  180 =  180 = InitPhiInputYzHorizontal("down", "positive")
//                               : horizontal,
//                90  135 =  135 : horizontal to down
//               180   90 =   90 = PhiInputYzVertical("down")
//                               : down
//               270   45 =   45 : down to horizontal
//           360,  0    0 =    0 : horizontal
//           450, 90  315 =  -45 : horizontal to up
//           540,180  270 =  -90 = PhiInputYzVertical("up")
//                               : up
//           630,270  225 = -135 : up to horizontal
//           720,  0  180 = -180 : horizontal
//     . two rotations of theta_mid cause one opposite rotation of phi_input_yz.
//   . when theta_mid rotates in negative direction from 0 to -180, then
//     phi_input_yz needs to counter rotate in positive direction from 0 to 90
//     to tilt the swivel down and to keep the swivel in the ZX plane.
//   . Therefore the swivel can be mounted either with phi_input_yz = 180 or 0,
//     and then the theta_mid rotation direction for moving down is
//     respectively positive 0 to 180, or negative 0 to -180.
// * phi_output_yz is the rotation of the swivel in YZ plane about the x-axis:
//   . The swivel has same phi_output_yz as the OutputTube(), because the
//     opposite control of theta_mid keeps the swivel tubes in one plane.
//   . The OutputTube position vector and OutputTube thrust vector both have
//     the same phi_output_yz angle, because the swivel tubes are in one plane.
//   . Keep swivel in vertical ZX plane for all theta_mid:
//     . phi_output_yz = -90 = PhiInputYzVertical("down")
//     . phi_output_yz = 90 = PhiInputYzVertical("up")
//
// Swivel control:
// . Use theta_mid = 0 tot 180 and phi_input_yz = 180 to 90 to move swivel from
//   horizontal (theta_mid = 0) to down (theta_mid = 180), keeping the swivel
//   in ZX plane, with phi_output_yz = PhiOutputYzVerticalDown() = -90.
// . For the swivel in an airplane only pointing down in -z direction is needed.
//   During hover the swivel points down and can use steering around
//   phi_output_yz = -90 for yaw, and around theta_output_zx = 180 for pitch.
// . During forward flight the swivel is kept horizontal, so no thrust
//   vector steering.
// . During transision the swivel moves in the ZX plane between theta_output_zx
//   = 90 (horizontal for forward flight) and theta_output_zx = 190 (maximum
//   down for hover).
// . Thrust vectoring near horizontal direction is not feasible, because this
//   requires being able to instantaneously change the phi_input_yz angle.
//   Such a jump in phi_input_yz is not physically possible by a motor, because
//   it is not smooth.
//   Furthermore:
//   . the ranges for theta_mid and phi_input_yz are typically limited to less
//     than 360 degrees for practical reasons of wiring to the motors.
//   . rotating the MidTube via theta_mid can only move the OutputTube over 0
//     to SwivelTiltXrMax() degrees (e.g. 0 to 100 degrees in case alpha_tilt =
//     25 degrees), so a positive tilt range. To steer from positive tilt to
//     negative tilt then requires a 180 degree jump in phi_input_yz when
//     theta_mid goes throuhg 0.
// . use phi_output_yz != 0 in addition to phi_input_yz to control the yaw:
//   . phi_output_yz > 0, swivel output to right, so yaw from Y to X
//   . phi_output_yz < 0, swivel output to left, so yaw from Y to -X

//------------------------------------------------------------------------------
// Swivel phi_input_yz control analysis
//------------------------------------------------------------------------------

// InputTube YZ angle to have phi_output_yz_arr[theta_mid = 0] = 0.
// . 270 = -ThetaMidToPhiOutputYzCrosstalkMax()
function PhiInputYzForAnalysis() = 270;

// OutputTube YZ angle to have phi_output_yz_arr[theta_mid = 0] = 0. With
// phi_input_yz == PhiInputYzForAnalysis() then phi_output_yz(0) = 0
function PhiOutputYzForAnalysis() = 0;

// Maximum crosstalk between MidTube rotation and OutputTube YZ angle
function ThetaMidToPhiOutputYzCrosstalkMax() = 90;

//------------------------------------------------------------------------------
// Swivel phi_input_yz control angle from y to z in YZ plane
//------------------------------------------------------------------------------

// When phi_input_yz = 90, then the markers on long side of InputTube() are on
// top, so with positive z in the ZX plane (y = 0).
function PhiInputYzForInputMarkerUp() = 90;

// When theta_mid = 0, then the swivel is straight and phi_input_yz can have
// any value. Initialize phi_input_yz to prepare for moving the swivel in down
// or up direction. The initial phi_input_yz depends on whether theta_mid will
// be rotated in positive or negative direction to theta_mid = 180 = -180
// degrees, because around theta_mid = 0 (is horizontal, straight swivel) the
// swivel output YZ direction changes sign.
function InitPhiInputYzHorizontal(output_tube_pointing, mid_tube_rotation) =
    output_tube_pointing == "down"
        ? mid_tube_rotation == "positive" ? 180 : 0
        : mid_tube_rotation == "positive" ? 0 : 180;

// When theta_mid = 180, then the swivel has maximum tilt angle, because then
// the long sides of the three tubes line up. The swivel then points down to
// -z in the ZX plane when phi_input_yz = 90 and up to +z in the ZX plane when
// phi_input_yz = 90 - 180 = -90 = 270.
function PhiInputYzVertical(output_tube_pointing) =
    let(phi_output_yz_vertical = PhiOutputYzVertical(output_tube_pointing))
    toAngle360(phi_output_yz_vertical + 180);


//------------------------------------------------------------------------------
// Swivel phi_output_yz thrust angle from y to z in YZ plane
//------------------------------------------------------------------------------

// Thrust phi_output_yz angle in YZ plane for swivel vertical down or up in the
// ZX plane. The phi_output_yz is constant for all theta_mid, to keep the
// thrust output in the ZX plane for all swivel pointings.
function PhiOutputYzVertical(output_tube_pointing) =
    output_tube_pointing == "down" ? 270 : 90;

// Initial phi_output_yz angle in YZ plane when theta_mid = 0, so horizontal,
// straight swivel.
function InitPhiOutputYzHorizontal(output_tube_pointing, mid_tube_rotation) =
    InitPhiInputYzHorizontal(output_tube_pointing, mid_tube_rotation);


//------------------------------------------------------------------------------
// Swivel theta_output_xr thrust angle with respect to x-axis
//------------------------------------------------------------------------------
// Maximum swivel thrust angle.
function SwivelTiltXrMax(alpha_tilt) = alpha_tilt * 4;

// XR angle for vertical swivel.
function SwivelTiltXrVertical() = 90;


//------------------------------------------------------------------------------
// Swivel theta_output_zx thrust angle from z to x
//------------------------------------------------------------------------------
// Horizontal swivel thrust theta_output_zx angle in ZX plane.
function ThetaOutputZxHorizontal() = 90;

// Swivel thrust theta_output_zx from theta_output_xr
function ThetaOutputZxAngle(theta_output_xr, output_tube_pointing) =
    let(theta_output_zx_horizontal = ThetaOutputZxHorizontal(),
        theta_output_zx = output_tube_pointing == "down"
                              ? theta_output_zx_horizontal + theta_output_xr
                              : theta_output_zx_horizontal - theta_output_xr
    )
    theta_output_zx;

// Maximum swivel thrust theta_output_zx angle for swivel output down.
function ThetaOutputZxMax(alpha_tilt) =
    ThetaOutputZxHorizontal() + SwivelTiltXrMax(alpha_tilt);

// Minimum swivel thrust theta_output_zx angle for swivel output up.
function ThetaOutputZxMin(alpha_tilt) =
    ThetaOutputZxHorizontal() - SwivelTiltXrMax(alpha_tilt);

// Vertical down or up swivel thrust theta_output_zx angle in ZX plane.
function ThetaOutputZxVertical(output_tube_pointing) =
    output_tube_pointing == "down" ? 180 : 0;


//------------------------------------------------------------------------------
// Position vector from origin to swivel output point
//------------------------------------------------------------------------------
// . equivalent calculations as done by module FormSwivel()
function SwivelOutputPosition(x_input, x_mid, x_output, phi_input_yz, theta_mid, alpha_tilt) =
    fRot_yz(phi_input_yz - c_phi_input_yz_construction) *
    fTrans(x_input, 0, 0) * fRot_zx(alpha_tilt) * fRot_yz(theta_mid) * fRot_zx(-alpha_tilt) *
    fTrans(x_mid, 0, 0) * fRot_zx(-alpha_tilt) * fRot_yz(-theta_mid) * fRot_zx(alpha_tilt) * fTrans(x_output, 0, 0) *
    [0, 0, 0, 1];


module echo_SwivelOutputPosition(phi_input_yz, theta_mid, alpha_tilt, position_vector) {
    echo(phi_input_yz = phi_input_yz);
    echo(theta_mid = theta_mid);
    echo(alpha_tilt = alpha_tilt);
    echo(SwivelOutputPosition = position_vector);
    echo(SwivelOutputPosition_angle_YZ = fAngleYZ(position_vector));
    echo(SwivelOutputPosition_angle_ZX = fAngleZX(position_vector));
    echo(SwivelOutputPosition_angle_XY = fAngleXY(position_vector));
    echo(SwivelOutputPosition_angle_XR = fAngleXR(position_vector));
    echo();
}


// SwivelOutputPosition_Gonio = SwivelOutputPosition, but using gonio equations
// . equivalent calculations as done by module FormSwivel_Gonio()
function SwivelOutputPosition_Gonio(x_input, x_mid, x_output, phi_input_yz, theta_mid, alpha_tilt) =
    //
    // Rx = 1 0  0    Ry = c 0 s    Rz = c -s 0
    //      0 c -s         0 1 0         s  c 0
    //      0 s  c        -s 0 c         0  0 1
    //
    // Rxp * Txi * Rya * Rxt- * Rya- * Txm * Rya- * Rxt * Rya * Txo * [0, 0, 0, 1] =
    // * Txo = [xo, 0, 0, 1];
    //
    // * Rya = [ca * xo,
    //          0,
    //          -sa * xo,
    //          1];
    //
    // * Rxt = [ca * xo,
    //          st * (-sa * xo),
    //          ct * (-sa * xo),
    //          1];
    //
    // * Rya- = [ca * (ca * xo) - sa * (ct * (-sa * xo)),
    //           st * (-sa * xo),
    //           sa * (ca * xo) + ca * (ct * (-sa * xo)),
    //           1];
    //
    // * Txm = [xm + ca * (ca * xo) - sa * (ct * (-sa * xo)),
    //          st * (-sa * xo),
    //          sa * (ca * xo) + ca * (ct * (-sa * xo)),
    //          1];
    //
    // * Rya- = [ca * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) - sa * (sa * (ca * xo) + ca * (ct * (-sa * xo))),
    //           st * (-sa * xo),
    //           sa * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) + ca * (sa * (ca * xo) + ca * (ct * (-sa * xo))),
    //           1];
    //
    // * Rxt- = [ca * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) - sa * (sa * (ca * xo) + ca * (ct * (-sa * xo))),
    //           ct * (st * (-sa * xo)) - st * (sa * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) +
    //               ca * (sa * (ca * xo) + ca * (ct * (-sa * xo)))),
    //           st * (st * (-sa * xo)) + ct * (sa * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) +
    //               ca * (sa * (ca * xo) + ca * (ct * (-sa * xo)))),
    //           1];
    //
    // * Rya = [ca * (ca * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) - sa * (sa * (ca * xo) +
    //              ca * (ct * (-sa * xo)))) + sa * (st * (st * (-sa * xo)) + ct * (sa * (xm + ca * (ca * xo) -
    //              sa * (ct * (-sa * xo))) + ca * (sa * (ca * xo) + ca * (ct * (-sa * xo))))),
    //          ct * (st * (-sa * xo)) - st * (sa * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) +
    //              ca * (sa * (ca * xo) + ca * (ct * (-sa * xo)))),
    //          -sa * (ca * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) - sa * (sa * (ca * xo) +
    //              ca * (ct * (-sa * xo)))) + ca * (st * (st * (-sa * xo)) + ct * (sa * (xm + ca * (ca * xo) -
    //              sa * (ct * (-sa * xo))) + ca * (sa * (ca * xo) + ca * (ct * (-sa * xo))))),
    //          1];
    //
    // * Txi = [xi + ca * (ca * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) - sa * (sa * (ca * xo) +
    //               ca * (ct * (-sa * xo)))) + sa * (st * (st * (-sa * xo)) + ct * (sa * (xm + ca * (ca * xo) -
    //                   sa * (ct * (-sa * xo))) + ca * (sa * (ca * xo) + ca * (ct * (-sa * xo))))),
    //               ct * (st * (-sa * xo)) - st * (sa * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) +
    //                   ca * (sa * (ca * xo) + ca * (ct * (-sa * xo)))),
    //               -sa * (ca * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) - sa * (sa * (ca * xo) +
    //                   ca * (ct * (-sa * xo)))) + ca * (st * (st * (-sa * xo)) + ct * (sa * (xm + ca * (ca * xo) -
    //                   sa * (ct * (-sa * xo))) + ca * (sa * (ca * xo) + ca * (ct * (-sa * xo))))),
    //          1];
    //
    // * Rxp = [xi + ca * (ca * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) - sa * (sa * (ca * xo) +
    //                    ca * (ct * (-sa * xo)))) + sa * (st * (st * (-sa * xo)) + ct * (sa * (xm + ca * (ca * xo) -
    //                    sa * (ct * (-sa * xo))) + ca * (sa * (ca * xo) + ca * (ct * (-sa * xo))))),
    //               cp * (ct * (st * (-sa * xo)) - st * (sa * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) +
    //                    ca * (sa * (ca * xo) + ca * (ct * (-sa * xo))))) - sp * (-sa * (ca * (xm + ca * (ca * xo) -
    //                    sa * (ct * (-sa * xo))) - sa * (sa * (ca * xo) + ca * (ct * (-sa * xo)))) +
    //                    ca * (st * (st * (-sa * xo)) + ct * (sa * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) +
    //                    ca * (sa * (ca * xo) + ca * (ct * (-sa * xo)))))),
    //               sp * (ct * (st * (-sa * xo)) - st * (sa * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) +
    //                    ca * (sa * (ca * xo) + ca * (ct * (-sa * xo))))) + cp * (-sa * (ca * (xm + ca * (ca * xo) -
    //                    sa * (ct * (-sa * xo))) - sa * (sa * (ca * xo) + ca * (ct * (-sa * xo)))) +
    //                    ca * (st * (st * (-sa * xo)) + ct * (sa * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) +
    //                    ca * (sa * (ca * xo) + ca * (ct * (-sa * xo)))))),
    //          1];
    //
    //       = [xi + ca * A - sa * B,
    //               cp * (ct * (st * (-sa * xo)) - st * (C)) - sp * (-sa * (A) + ca * (B)),
    //               sp * (ct * (st * (-sa * xo)) - st * (C)) + cp * (-sa * (A) + ca * (B)),
    //          1];
    //
    //          with:
    //            A = ca * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) -
    //                sa * (sa * (ca * xo) + ca * (ct * (-sa * xo)));
    //            B = st * (st * (-sa * xo)) + ct * (C);
    //            C = sa * (xm + ca * (ca * xo) - sa * (ct * (-sa * xo))) +
    //                ca * (sa * (ca * xo) + ca * (ct * (-sa * xo)));
    //
    // Recombine terms:
    //
    //       = [xi + xm * (ca2 + sa2 * ct) + xo * (ca4 + sa4 * ct2 + -sa2 * st2 + ca2 * sa2 * (4 * ct - ct2 - 1)),
    //               xo * cp * ct * -st * sa +
    //                    cp * -st * E +
    //                    sp * sa * F +
    //                   -sp * ca * G,
    //               xo * sp * ct * -st * sa +
    //                    sp * -st * E +
    //                   -cp * sa * F +
    //                    cp * ca * G,
    //          1];
    //
    //          with:
    //            E = xm * sa +
    //                xo * (sa * ca2 * (2 - ct) +
    //                      sa3 * ct);
    //            F = xm * ca +
    //                xo * (ca3 +
    //                      sa2 * ca * (2 * ct - 1));
    //            G = xm * sa * ct +
    //                xo * (-sa * st2 +
    //                       sa3 * ct2 +
    //                       sa * ca2 * (2 * ct - ct2));
    //
    //        = [xi + xm * Xm + xo * Xo,
    //                cp * P +  sp * Q,
    //                sp * P + -cp * Q,
    //           1];
    //
    //           with:
    //             P = xm * Ym + xo * Yo;
    //             Q = xm * Zm + xo * Zo;
    //
    let(xo = x_output,
        xm = x_mid,
        xi = x_input,
        ca = cos(alpha_tilt),
        sa = sin(alpha_tilt),
        ct = cos(theta_mid),
        st = sin(theta_mid),
        ca2 = ca * ca,
        ca3 = ca * ca2,
        ca4 = ca2 * ca2,
        sa2 = sa * sa,
        sa3 = sa * sa2,
        sa4 = sa2 * sa2,
        ca2sa2 = ca2 * sa2,
        ct2 = ct * ct,
        st2 = st * st,
        cp = cos(phi_input_yz - c_phi_input_yz_construction),
        sp = sin(phi_input_yz - c_phi_input_yz_construction),
        Xm = ca2 + sa2 * ct,
        Xo = ca4 + sa4 * ct2 + -sa2 * st2 + ca2 * sa2 * (4 * ct - ct2 - 1),
        Ym = sa * -st,
        Yo = sa * -st * (ca2 * (2 - ct) + ct * (1 + sa2)),
        Zm = ca * sa * (1 - ct),
        Zo = ca * sa * (st2 + (sa2 - ca2) * (2 * ct - 1 - ct2))
    )
    [xi + xm * Xm                  + xo * Xo,
          xm * (cp * Ym + sp * Zm) + xo * (cp * Yo + sp * Zo),
          xm * (sp * Ym - cp * Zm) + xo * (sp * Yo - cp * Zo),
     1];


//------------------------------------------------------------------------------
// Swivel thrust vector along OutputTube
//------------------------------------------------------------------------------

// Thrust vector of swivel output tube
// . thrust vector is difference vector of position vector for two swivels that
//   have different OutputTube lengths.
function SwivelThrustVector(x_input, x_mid, x_output, alpha_tilt, phi_input_yz, theta_mid) =
    SwivelOutputPosition(x_input, x_mid, x_output, phi_input_yz, theta_mid, alpha_tilt) -
    SwivelOutputPosition(x_input, x_mid,        0, phi_input_yz, theta_mid, alpha_tilt);

module echo_SwivelThrustVector(phi_input_yz, theta_mid, alpha_tilt, thrust_vector) {
    echo(phi_input_yz = phi_input_yz);
    echo(theta_mid = theta_mid);
    echo(alpha_tilt = alpha_tilt);
    echo(SwivelThrustVector = thrust_vector);
    echo(SwivelThrustVector_angle_YZ = fAngleYZ(thrust_vector));
    echo(SwivelThrustVector_angle_ZX = fAngleZX(thrust_vector));
    echo(SwivelThrustVector_angle_XY = fAngleXY(thrust_vector));
    echo(SwivelThrustVector_angle_XR = fAngleXR(thrust_vector));
    echo();
}

//------------------------------------------------------------------------------
// Swivel control
//------------------------------------------------------------------------------

// Determine phi_input_yz for requested phi_output_yz_request and given
// theta_mid, because theta_mid != 0 also yields some OutputTube YZ rotation.
function DeterminePhiInputYzForPhiOutputYzAndThetaMid(x_input, x_mid, x_output, alpha_tilt,
                                                      phi_output_yz_request, theta_mid) =
    // First determine the phi_output_yz angle of the OutputTube due to
    // theta_mid using phi_input_yz = 0. Then adjust phi_input_yz such that
    // phi_input_yz + phi_output_yz yields phi_output_yz_request.
    let(position_vector = SwivelOutputPosition(x_input, x_mid, x_output,
                                               0,
                                               theta_mid, alpha_tilt),
        phi_output_yz = fAngleYZ(position_vector),
        phi_input_yz = phi_output_yz_request - phi_output_yz
    )
    phi_input_yz;


//------------------------------------------------------------------------------
// Swivel harmonic approximation for phi_output_yz(theta_mid)
// . to keep swivel motion in ZX plane,
// . see swivel.ipynb for more description and plots
//------------------------------------------------------------------------------

function rDftBinsOfPhiOutputYzAsFunctionOfThetaMid(alpha_tilt,
                                                   B=8, N=16) =
    // Calculate DC and first harmonic using real input rDFT of sinus like
    // part in exact phi_output_yz(theta_mid).
    //
    // Input:
    // . B: return ampl, phase of bins 0:B-1, so B values in
    //      rdft_phi_output_yz_diff_bin_arr.
    // . N: number of points in theta_mid_arr and therefore of the DFT. The
    //      maximum number of frequency bins is K = N // 2 + 1, so B <= K.
    //      It is not necessary to choose large N >> 2 * B for more accuracy.
    // Return:
    // . rdft_phi_output_yz_diff_bin_arr: rDFT bin polar values for
    //   phi_output_yz(theta_mid).
    let(// First calculate exact phi_output_yz(theta_mid_arr) for N points, so
        // that with rDFT this yields N // 2 + 1 bins, with DC at bin 0.
        phi_input_yz_for_analysis = PhiInputYzForAnalysis(),
        phi_output_yz_for_analysis = PhiOutputYzForAnalysis(),

        // Prepare theta_mid_arr, 0 <= theta_mid < 360 degrees
        n_arr = num_range(0, 1, N - 1),
        theta_mid_arr = [ for (n = n_arr) n / N * 360 ],  // one period of 0 to 360

        // Calculate exact phi_output_yz_arr
        // . can use x_input = 1, x_mid = 1, x_output = 1, because swivel thrust
        //   vector pointing direction is independent of swivel tube lengths.
        thrust_vector_arr = [ for (n = n_arr) SwivelThrustVector(1, 1, 1, alpha_tilt,
                                                                 phi_input_yz_for_analysis,
                                                                 theta_mid_arr[n]) ],
        phi_output_yz_arr = [ for (n = n_arr) toAngle360(fAngleYZ(thrust_vector_arr[n])) ],

        // . replace phi_output_yz_arr[theta_mid = 0] = NaN, when OutputTube is
        //   horizontal at x-axis (so z/y = 0/0)
        phi_output_yz_arr_corrected = concat(phi_output_yz_for_analysis,
                                             num_slice(phi_output_yz_arr, 1, N - 1)),

        // Determine deviation of phi_output_yz from linear
        // phi_output_yz_horizontal + theta_mid / 2. Keep DC level of
        // phi_output_yz_horizontal in phi_output_yz_diff_arr, because that
        // then will appear in the DC bin 0.
        phi_output_yz_diff_arr = [ for (n = n_arr) phi_output_yz_arr_corrected[n] - theta_mid_arr[n] / 2 ],

        // Determine harmonics in phi_output_yz_diff_arr using rDFT
        // . scale k = 0 DC by 1 / N
        // . scale k > 0 harmonics by 2 / N to get their amplitude
        rdft_phi_output_yz_diff_arr = rdft(phi_output_yz_diff_arr),
        rdft_phi_output_yz_diff_re_arr = dft_scale_dc_harmonics(rdft_phi_output_yz_diff_arr[0], N),
        rdft_phi_output_yz_diff_im_arr = dft_scale_dc_harmonics(rdft_phi_output_yz_diff_arr[1], N),
        rdft_phi_output_yz_diff_ample_arr = num_complex_abs(rdft_phi_output_yz_diff_re_arr,
                                                            rdft_phi_output_yz_diff_im_arr),
        rdft_phi_output_yz_diff_angle_arr = num_complex_angle(rdft_phi_output_yz_diff_re_arr,
                                                              rdft_phi_output_yz_diff_im_arr, f_eps),
        // . polar bin values
        ampl_arr = num_slice(rdft_phi_output_yz_diff_ample_arr, 0, B - 1),
        angle_arr = num_slice(rdft_phi_output_yz_diff_angle_arr, 0, B - 1),
        rdft_phi_output_yz_diff_bin_arr = [ampl_arr, angle_arr]
    )
    rdft_phi_output_yz_diff_bin_arr;


function ApproximatePhiOutputYzCrosstalk(theta_mid, f1_ampl) =
    // Approximate crosstalk phi_output_yz(theta_mid) using the first harmonic
    // from DFT.
    //
    // Equation:
    //   phi_output_yz_crosstalk = theta_mid / 2 - f1_ampl * sin(theta_mid)
    //   . for theta_mid in [0:360>
    //
    // Input:
    // . f1_ampl = few degrees for the small sinus like deviation from linear
    // Return:
    // . phi_output_yz_crosstalk: Equation result for theta_mid_arr
    let(phi_output_yz_crosstalk = toAngle360(theta_mid / 2) - f1_ampl * sin(theta_mid))
    phi_output_yz_crosstalk;


function ApproximatePhiOutputYzAsFunctionOfThetaMid(theta_mid, f1_ampl) =
    // Approximate phi_output_yz(theta_mid) using first harmonic from DFT
    //
    // Equation:
    //   phi_output_yz_approx(theta_mid) ~= -phi_input_yz_for_analysis + phi_output_yz_crosstalk
    //
    //     with: phi_output_yz_crosstalk = theta_mid / 2 - f1_ampl * sin(theta_mid)
    //           for theta_mid in [0:360>
    //
    // Input:
    // . f1_ampl = few degrees for the small sinus like deviation from linear
    // Return:
    // . phi_output_yz_approx: approximate phi_output_yz(theta_mid). This
    //   needs to be compensated via phi_input_yz to keep swivel motion in ZX
    //   plane.
    let(phi_input_yz_for_analysis = PhiInputYzForAnalysis(),
        phi_output_yz_crosstalk = ApproximatePhiOutputYzCrosstalk(theta_mid, f1_ampl),
        phi_output_yz_approx = toAngle360(-phi_input_yz_for_analysis + phi_output_yz_crosstalk)
    )
    phi_output_yz_approx;


//------------------------------------------------------------------------------
// Swivel harmonic approximation for theta_output_xr(theta_mid)
// . to point swivel thrust for transition between horizontal for forward
//   flight, and down for hover
// . see swivel.ipynb for more description and plots
//------------------------------------------------------------------------------

function rDftBinsOfThetaOutputXrAsFunctionOfThetaMid(alpha_tilt,
                                                     B=8, N=16) =
    // Calculate DC and first harmonic using real input rDFT of sinus like
    // signal that is created from exact theta_output_xr(theta_mid).
    //
    // Input:
    // . B: return ampl, phase of bins 0:B-1, so B values in
    //      rdft_theta_output_xr_bin_arr.
    // . N: number of points in theta_mid_arr and therefore of the DFT. The
    //      maximum number of frequency bins is K = N // 2 + 1, so B <= K.
    //      It is not necessary to choose large N >> 2 * B for more accuracy.
    // Return:
    // . rdft_theta_output_xr_bin_arr: rDFT bin polar values for
    //   theta_output_xr(theta_mid).
    let(// First calculate exact theta_output_xr(theta_mid_arr) for N points, so
        // that with rDFT this yields N // 2 + 1 bins, with DC at bin 0.
        // Prepare theta_mid_arr in [0:360>
        n_arr = num_range(0, 1, N - 1),
        theta_mid_arr = [ for (n = n_arr) n / N * 360 ],  // one period of 0 to 360

        // Calculate exact theta_output_xr_arr
        // . can use x_input = 1, x_mid = 1, x_output = 1, because swivel thrust
        //   vector pointing direction is independent of swivel tube lengths.
        thrust_vector_arr = [ for (n = n_arr) SwivelThrustVector(1, 1, 1, alpha_tilt,
                                                                 0, theta_mid_arr[n]) ],
        theta_output_xr_arr = [ for (n = n_arr) toAngle360(fAngleXR(thrust_vector_arr[n])) ],

        // Create sinus like signal from theta_output_xr_arr and negated copy
        N2 = 2 * N,
        theta_output_xr_arr2 = concat(theta_output_xr_arr, -theta_output_xr_arr),

        // Determine harmonics in theta_output_xr_arr2 using rDFT
        // . scale k = 0 DC by 1 / N2
        // . scale k > 0 harmonics by 2 / N2 to get their amplitude
        rdft_theta_output_xr_arr = rdft(theta_output_xr_arr2),
        rdft_theta_output_xr_re_arr = dft_scale_dc_harmonics(rdft_theta_output_xr_arr[0], N2),
        rdft_theta_output_xr_im_arr = dft_scale_dc_harmonics(rdft_theta_output_xr_arr[1], N2),
        rdft_theta_output_xr_ampl_arr = num_complex_abs(rdft_theta_output_xr_re_arr,
                                                        rdft_theta_output_xr_im_arr),
        rdft_theta_output_xr_angle_arr = num_complex_angle(rdft_theta_output_xr_re_arr,
                                                           rdft_theta_output_xr_im_arr, f_eps),
        // . polar bin values
        ampl_arr = num_slice(rdft_theta_output_xr_ampl_arr, 0, B - 1),
        angle_arr = num_slice(rdft_theta_output_xr_angle_arr, 0, B - 1),
        rdft_theta_output_xr_bin_arr = [ampl_arr, angle_arr]
    )
    rdft_theta_output_xr_bin_arr;


function ApproximateThetaOutputXrAsFunctionOfThetaMid(theta_mid, f1_ampl) =
    // Approximate theta_output_xr(theta_mid) using first harmonic from real
    // input DFT.
    //
    // Equation:
    //   theta_output_xr = f1_ampl * abs(sin(theta_mid / 2))
    //   . for theta_mid in [0:360>
    //   . use abs() because XR angle in [0:180>
    //
    // Input:
    // . f1_ampl ~= SwivelTiltXrMax(alpha_tilt)
    // Return:
    // . theta_output_xr_approx_arr: Equation result for theta_mid
    let(theta_output_xr_approx = f1_ampl * abs(sin(theta_mid / 2)))
    theta_output_xr_approx;


// Inverse function of ApproximateThetaOutputXrAsFunctionOfThetaMid()
function ApproximateThetaMidAsFunctionOfThetaOutputXr(theta_output_xr,
                                                      f1_ampl,
                                                      mid_tube_rotation) =
    // Approximate theta_mid(theta_output_xr) using first harmonic from real
    // input DFT.
    //
    // Equation:
    //   theta_mid ~= 2 * arcsin(f1_fraction)
    //
    //   with:
    //   . f1_theta_output_xr_ampl ~= swivel_tilt_xr_max, first harmonic from
    //     rDFT of exact theta_output_xr(theta_mid).
    //   . f1_fraction = theta_output_xr / f1_theta_output_xr_ampl
    //     f1_fraction = 1 if fraction > 1 (so theta_output_xr >
    //                   f1_theta_output_xr_ampl)
    //     f1_fraction in [0:1], because theta_output_xr >= 0
    //   . theta_mid in [0:180], because f1_fraction in [0:1]
    //     theta_mid = c_theta_mid_eps if theta_mid < c_theta_mid_eps ~= 0
    //     theta_mid in <0:180] when mid_tube_rotation is positive
    //     theta_mid = 360 - theta_mid, when mid_tube_rotation is negative, so
    //                 in [180:360>
    //
    // Input:
    // . f1_ampl ~= SwivelTiltXrMax(alpha_tilt)
    // Return:
    // . theta_mid_approx: Equation result for theta_mid
    let(// y = arcsin(x) for -1 <= x <= 1, -90 <= y <= 90, therefore the
        // maximum theta_output_xr = f1_ampl for this approximation.
        f1_fraction = theta_output_xr < f1_ampl ? theta_output_xr / f1_ampl : 1,
        f1_theta_mid_approx = 2 * asin(f1_fraction),
        theta_mid_approx = map_theta_mid_rotation(f1_theta_mid_approx, mid_tube_rotation)
    )
    theta_mid_approx;

function map_theta_mid_rotation(theta_mid_180, mid_tube_rotation) =
    // Assume theta_mid_180 is in [0:180]. Map theta_mid_180 to <0:180] or to
    // [180:360>, dependent on mid_tube_rotation.
    let(// . Replace theta_mid_180 ~= 0 by c_theta_mid_eps to get <0:180]. To
        //   avoid undefined YZ angle for horizontal swivel.
        theta_mid_positive = theta_mid_180 < c_theta_mid_eps
                               ? c_theta_mid_eps
                               : theta_mid_180,
        // . Account for sign of mid_tube_rotation. Keep theta_mid <0:180] when
        //   MidTube rotation is positive else negate theta_mid by
        //   360 - theta_mid, if MidTube rotation direction is negative.
        theta_mid_mapped = mid_tube_rotation == "positive"
                               ? theta_mid_positive
                               : 360 - theta_mid_positive
    )
    theta_mid_mapped;
