################################################################################
# Copyright 2022 E. Kooistra
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################
#
# Author: E. Kooistra, March 2022 created
#                      July 2024 Updated
# Purpose: Library to calculate the orientation of the swivel
# Description:
# . Corresponds to swivel_functions.scad
# Self test:
# > python swivel_functions.py

import numpy as np
import linear_algebra as la

################################################################################
# Swivel constants

# Use theta_mid_arr + c_theta_mid_eps to offset theta_mid = 0 for straight
# swivel, which leads to NaN YZ angle. It appears that for SwivelThrustVector()
# c_theta_mid_eps needs to be ~< 1e-7, so la.f_eps = 1e-10 is too small.
c_theta_mid_eps = 1e-7

# The swivel assembly is constructed with InputTube() long side markers up,
# so with phi_input_yz = 90. However the zero position for the long side
# markers is better defined at phi_input_yz = 0. Therefore use
# c_phi_input_yz_construction to compensate for this construction offset, by
# subracting it from the input phi_input_yz in SwivelOutputPosition() and
# SwivelOutputPosition_Gonio().
c_phi_input_yz_construction = 90


################################################################################
# Swivel position vector

def SwivelOutputPosition(x_input, x_mid, x_output, alpha_tilt, phi_input_yz, theta_mid):
    """Calculate position vector from origin to swivel output.

    Uses matrices."""
    x = np.array([0, 0, 0, 1])
    return la.Rot_yz(phi_input_yz - c_phi_input_yz_construction) @ \
        la.Trans(x_input, 0, 0) @ la.Rot_zx(alpha_tilt) @ la.Rot_yz(theta_mid) @ la.Rot_zx(-alpha_tilt) @ \
        la.Trans(x_mid, 0, 0) @ la.Rot_zx(-alpha_tilt) @ la.Rot_yz(-theta_mid) @ la.Rot_zx(alpha_tilt) @ \
        la.Trans(x_output, 0, 0) @ x


def SwivelOutputPositionsList(x_input, x_mid, x_output, alpha_tilt, phi_input_yz_arr, theta_mid_arr):
    """Calculate position vectors from origin to swivel outputs.

    . For array of InputTube YZ angles phi_input_yz_arr and corresponding array
      of MidTube angles theta_mid_arr.
    """
    pVectorList = []
    [pVectorList.append(SwivelOutputPosition(x_input, x_mid, x_output, alpha_tilt, phi_input_yz, theta_mid))
     for phi_input_yz, theta_mid in zip(phi_input_yz_arr, theta_mid_arr)]
    return pVectorList


def SwivelOutputPosition_Gonio(x_input, x_mid, x_output, alpha_tilt, phi_input_yz, theta_mid):
    """Calculate position vector from origin to swivel output.

    Using gonio equations obtained from elaborating the matrices.
    """
    theta_mid_rad = np.radians(theta_mid)
    alpha_tilt_rad = np.radians(alpha_tilt)
    xo = x_output
    xm = x_mid
    xi = x_input
    ca = np.cos(alpha_tilt_rad)
    sa = np.sin(alpha_tilt_rad)
    ct = np.cos(theta_mid_rad)
    st = np.sin(theta_mid_rad)
    ca2 = ca * ca
    # ca3 = ca * ca2
    ca4 = ca2 * ca2
    sa2 = sa * sa
    # sa3 = sa * sa2
    sa4 = sa2 * sa2
    # ca2sa2 = ca2 * sa2
    ct2 = ct * ct
    st2 = st * st
    phi_input_yz_rad = np.radians(phi_input_yz - c_phi_input_yz_construction)
    cp = np.cos(phi_input_yz_rad)
    sp = np.sin(phi_input_yz_rad)
    Xm = ca2 + sa2 * ct
    Xo = ca4 + sa4 * ct2 + -sa2 * st2 + ca2 * sa2 * (4 * ct - ct2 - 1)
    Ym = sa * -st
    Yo = sa * -st * (ca2 * (2 - ct) + ct * (1 + sa2))
    Zm = ca * sa * (1 - ct)
    Zo = ca * sa * (st2 + (sa2 - ca2) * (2 * ct - 1 - ct2))
    p_vector = np.array([xi + xm * Xm + xo * Xo,
                         xm * (cp * Ym + sp * Zm) + xo * (cp * Yo + sp * Zo),
                         xm * (sp * Ym - cp * Zm) + xo * (sp * Yo - cp * Zo), 1])
    return p_vector


################################################################################
# Swivel thrust_vector

def SwivelThrustVector(x_input, x_mid, x_output, alpha_tilt, phi_input_yz, theta_mid):
    """Calculate thrust vector of swivel output.

    . As function of InputTube YZ angle phi_input_yz and MidTube angle
      theta_mid.
    . Replace theta_mid == 0 by theta_mid = la.f_eps to avoid arctan() = nan
      for YZ angle of the thrust vector when the swivel is straight.
    . Uses matrices.
    """
    return \
        SwivelOutputPosition(x_input, x_mid, x_output, alpha_tilt, phi_input_yz, theta_mid) - \
        SwivelOutputPosition(x_input, x_mid, 0, alpha_tilt, phi_input_yz, theta_mid)


def SwivelThrustVectorsList(x_input, x_mid, x_output, alpha_tilt, phi_input_yz_arr, theta_mid_arr):
    """Calculate thrust vectors of swivel outputs.

    . For array of InputTube YZ angles phi_input_yz_arr and corresponding array
      of MidTube angles theta_mid_arr.
    """
    tVectorList = []
    [tVectorList.append(SwivelThrustVector(x_input, x_mid, x_output, alpha_tilt, phi_input_yz, theta_mid))
     for phi_input_yz, theta_mid in zip(phi_input_yz_arr, theta_mid_arr)]
    return tVectorList


def SwivelThrustVector_Gonio(x_input, x_mid, x_output, alpha_tilt, phi_input_yz, theta_mid):
    """Calculate thrust vector of swivel output.

    . Uses gonio equations obtained from elaborating the matrices.
    . Equivalent to SwivelThrustVector(), so SwivelThrustVector_Gonio() yields
      same thrust vector.
    """
    return \
        SwivelOutputPosition_Gonio(x_input, x_mid, x_output, alpha_tilt, phi_input_yz, theta_mid) - \
        SwivelOutputPosition_Gonio(x_input, x_mid, 0, alpha_tilt, phi_input_yz, theta_mid)


################################################################################
# Swivel angle constants

# Swivel OutputTube YZ angle = 0 when swivel is straight for anaylysis of
# phi_output_yz(theta_mid) function.
def PhiInputYzForAnalysis():
    """InputTube YZ angle to have phi_output_yz_arr[theta_mid = 0] = 0.

    With phi_output_yz = PhiInputYzForAnalysis() the phi_output_yz(theta_mid)
    curve will about linearly increase from 0 : 180 for theta_mid 0 : 360.
    Using PhiInputYzForAnalysis() avoids that the phi_output_yz angle wraps
    around 0 or 360 degrees, so that the curve is suitable for DFT analysis.
    """
    return 270  # = 180 + c_phi_input_yz_construction


def PhiOutputYzForAnalysis():
    """Start OutputTube YZ angle to have phi_output_yz_arr[theta_mid = 0] = 0.

    With phi_input_yz == PhiInputYzForAnalysis() then phi_output_yz(0) = 0.
    """
    return 0


# Swivel phi_input_yz control angle from y to z in YZ plane
def PhiInputYzForInputMarkerUp():
    """phi_input_yz angle in YZ plane for InputTube long side marker up."""
    return 90


def InitPhiInputYzHorizontal(mid_tube_rotation):
    """Initial InputTube YZ angle for horizontal, straight swivel, when
    theta_mid = 0.

    InitPhiInputYzHorizontal() prepares phi_input_yz of horizontal, straight
    swivel, at theta_mid = 0, such that including compensating for the counter
    rotation the swivel phi_output_yz will reach PhiOutputYzVertical() for
    vertical, maximum tilted swivel. The phi_input_yz is then
    PhiInputYzVertical(). The InitPhiInputYzHorizontal() can be found by
    starting with a maximum tilted swivel and then rotating the MidTube to
    theta_mid = 0 and meanwhile compensating the InputTube YZ angle to keep
    the swivel output in the vertical ZX plane.
    """
    if mid_tube_rotation == 'positive':
        return 180
    else:
        return 0


def PhiInputYzVertical(output_tube_pointing):
    """InputTube YZ angle for vertical, maximum tilted swivel, when theta_mid
    = 180.
    """
    phi_output_yz_vertical = PhiOutputYzVertical(output_tube_pointing)
    return la.toAngle360(phi_output_yz_vertical + 180)


# Swivel thrust angle phi_output_yz from y to z in YZ plane
def PhiOutputYzVertical(output_tube_pointing):
    """OutputTube YZ angle for vertical, maximum tilted swivel, when theta_mid
    = 180.
    """
    if output_tube_pointing == "down":
        return 270  # = -90
    else:
        return 90  # up = down - 180


def InitPhiOutputYzHorizontal(mid_tube_rotation):
    """Initial OutputTube YZ angle for horizontal, straight swivel, when
    theta_mid = 0.

    InitPhiOutputYzHorizontal() = InitPhiInputYzHorizontal(). This can be seen
    when OutputTube is not rotated and kept tilted with the MidTube.
    """
    return InitPhiInputYzHorizontal(mid_tube_rotation)


def PhiOutputYzCrosstalkOffset(output_tube_pointing, mid_tube_rotation):
    """OutputTube YZ rotation when theta_mid is rotated from 0 to 180.

    The counter rotation of InputTube YZ angle makes sure that the swivel
    OutputTube YZ angle remains constant to keep in the vertical ZX plane.
    The required counter rotation of the InputTube YZ angle is the opposite of
    the crosstalk offset that is caused by theta_mid on the OutputTube YZ
    angle when theta_mid = 180.
    """
    # if output_tube_pointing == "down":
    #     if mid_tube_rotation == "positive":
    #         return 90
    #     else:
    #         return 270
    # else:
    #     if mid_tube_rotation == "positive":
    #         return 270
    #     else:
    #         return 90
    return la.toAngle360(InitPhiInputYzHorizontal(mid_tube_rotation) -
                         PhiInputYzVertical(output_tube_pointing))


# Swivel theta_output_zx thrust angle from z to x
def SwivelTiltMax(alpha_tilt):
    """Maximum swivel thrust angle."""
    return alpha_tilt * 4


def ThetaOutputZxHorizontal():
    """Horizontal swivel thrust angle theta_output_zx in ZX plane."""
    return 90


def ThetaOutputZxMax(alpha_tilt):
    """Maximum swivel thrust theta_output_zx angle (with swivel output down)."""
    return ThetaOutputZxHorizontal() + SwivelTiltMax(alpha_tilt)


def ThetaOutputZxMin(alpha_tilt):
    """Minimum swivel thrust theta_output_zx angle (with swivel output up)."""
    return ThetaOutputZxHorizontal() - SwivelTiltMax(alpha_tilt)


def ThetaOutputZxVertical(output_tube_pointing):
    """Swivel thrust angle theta_output_zx in ZX plane for vertical down or up.
    """
    if output_tube_pointing == 'down':
        return 180
    else:
        return 0


################################################################################
# Swivel control

def DetermineThetaMidForThetaOutputZx(theta_output_zx_request, theta_output_zx_resolution,
                                      alpha_tilt, output_tube_pointing):
    """Calculate theta_mid that yields requested theta_output_zx.

    Uses binary search and fAngleXR(SwivelThrustVector()) to find theta_mid,
    because the exact inverse function to calculate theta_mid as function of
    theta_output_zx is not available.
    """
    zx_horizontal = ThetaOutputZxHorizontal()
    # Account for output_tube_pointing on theta_output_zx range
    if output_tube_pointing == 'down':
        zx_max = ThetaOutputZxMax(alpha_tilt)
        zx_min = zx_horizontal
        theta_mid_zx_max = 180
        theta_mid_zx_min = 0
    else:
        zx_max = zx_horizontal
        zx_min = ThetaOutputZxMin(alpha_tilt)
        theta_mid_zx_max = 0
        theta_mid_zx_min = 180

    # Fixed theta_mid at zx_min and zx_max ends of theta_output_zx range
    if np.isclose(theta_output_zx_request, zx_horizontal, atol=la.f_eps):
        theta_mid = 0
    elif np.isclose(theta_output_zx_request, zx_max, atol=la.f_eps):
        theta_mid = theta_mid_zx_max
    elif np.isclose(theta_output_zx_request, zx_min, atol=la.f_eps):
        theta_mid = theta_mid_zx_min
    elif theta_output_zx_request > zx_max:
        print('Too large theta_output_zx_request')
        theta_mid = theta_mid_zx_max
    elif theta_output_zx_request < zx_min:
        print('Too small theta_output_zx_request')
        theta_mid = theta_mid_zx_max
    else:

        # Binary search theta_mid
        theta_mid_lo = 0
        theta_mid_hi = 180
        n = 0
        n_max = np.ceil(np.log2((theta_mid_hi - theta_mid_lo) / theta_output_zx_resolution))
        while n < n_max:
            theta_mid = (theta_mid_lo + theta_mid_hi) / 2
            # Determine theta_output_zx using OutputTube XR angle, so
            # independent of phi_input_yz, to avoid having to account for
            # the phi_output(theta_mid) crosstalk. Use phi_input_yz = 0.
            t_vector = SwivelThrustVector(1, 1, 1, alpha_tilt, 0, theta_mid)
            if output_tube_pointing == 'down':
                theta_output_zx = ThetaOutputZxHorizontal() + la.fAngleXR(t_vector)
                if theta_output_zx > theta_output_zx_request + theta_output_zx_resolution:
                    theta_mid_hi = theta_mid
                elif theta_output_zx < theta_output_zx_request - theta_output_zx_resolution:
                    theta_mid_lo = theta_mid
                else:
                    break
            else:
                theta_output_zx = ThetaOutputZxHorizontal() - la.fAngleXR(t_vector)
                if theta_output_zx > theta_output_zx_request + theta_output_zx_resolution:
                    theta_mid_lo = theta_mid
                elif theta_output_zx < theta_output_zx_request - theta_output_zx_resolution:
                    theta_mid_hi = theta_mid
                else:
                    break
            n = n + 1
    return theta_mid


################################################################################
# Swivel harmonic approximation for phi_output_yz(theta_mid)

def rfft_bins_of_phi_output_yz_as_function_of_theta_mid(alpha_tilt, N=16):
    """Calculate DC and first harmonic using real input rDFT of sinus like
    part in exact phi_output_yz(theta_mid).

    See swivel.ipynb for more description and plots.

    Input:
    . N: number of points in theta_mid_arr and therefore of the DFT. The
         theta_mid_arr is over range [0 : 360> degrees. The maximum number
         of frequency bins is K = N // 2 + 1.
         It is not necessary to choose large N >> 16 for more accuracy.
    Return:
    . rfft_phi_output_yz_diff_bin_arr: rFFT bin polar values for
      phi_output_yz(theta_mid).
    """
    print('rfft_bins_of_phi_output_yz_as_function_of_theta_mid()')
    phi_input_yz_for_analysis = PhiInputYzForAnalysis()
    phi_output_yz_for_analysis = PhiOutputYzForAnalysis()
    print('. phi_input_yz_for_analysis = ', phi_input_yz_for_analysis)
    print('. phi_output_yz_for_analysis = ', phi_output_yz_for_analysis)

    # Prepare theta_mid_arr, 0 <= theta_mid < 360 degrees
    N_degrees = 360
    theta_mid_arr = np.linspace(0, N_degrees, N, endpoint=False)

    # Calculate exact phi_output_yz_arr
    # . can use x_input = 1, x_mid = 1, x_output = 1, because swivel thrust
    #   vector pointing direction is independent of swivel tube lengths.
    phi_output_yz_arr = np.zeros(N)
    for T in range(N):
        t_vector = SwivelThrustVector(1, 1, 1, alpha_tilt, phi_input_yz_for_analysis, theta_mid_arr[T])
        phi_output_yz_arr[T] = la.toAngle360(la.fAngleYZ(t_vector))

    # . replace phi_output_yz_arr[theta_mid = 0] = NaN, when OutputTube is
    #   horizontal at x-axis (so z/y = 0/0)
    phi_output_yz_arr[0] = phi_output_yz_for_analysis

    # Determine deviation of phi_output_yz from linear
    # phi_output_yz_for_analysis + theta_mid / 2. Keep DC level of
    # phi_output_yz_for_analysis in phi_output_yz_diff_arr, because that then
    # will appear in the DC bin 0.
    phi_output_yz_diff_arr = phi_output_yz_arr - theta_mid_arr / 2

    # Determine harmonics in phi_output_yz_diff_arr using rFFT
    # . scale k = 0 DC by 1 / N
    # . scale k > 0 harmonics by 2 / N to get their amplitude
    rfft_phi_output_yz_diff_arr = 2 / N * np.fft.rfft(phi_output_yz_diff_arr)
    rfft_phi_output_yz_diff_arr[0] = rfft_phi_output_yz_diff_arr[0] / 2  # DC bin
    ampl_rfft_phi_output_yz_diff_arr = np.abs(rfft_phi_output_yz_diff_arr)
    angle_rfft_phi_output_yz_diff_arr = np.degrees(np.angle(rfft_phi_output_yz_diff_arr))
    # . polar bin values
    rfft_phi_output_yz_diff_bin_arr = (ampl_rfft_phi_output_yz_diff_arr,
                                       angle_rfft_phi_output_yz_diff_arr)

    # Single harmonic approximation for phi_output_yz as function of theta_mid
    # . log f0 DC and f1 harmonic
    f0 = 0
    f0_ampl = ampl_rfft_phi_output_yz_diff_arr[f0]  # = DC = PhiOutputYzForAnalysis()
    f0_angle = angle_rfft_phi_output_yz_diff_arr[f0]
    f1 = 1
    f1_ampl = ampl_rfft_phi_output_yz_diff_arr[f1]
    f1_angle = angle_rfft_phi_output_yz_diff_arr[f1]
    print('. f0 = %d : f0_ampl = %.10f, f0_angle = %.10f, = DC = PhiOutputYzForAnalysis()' %
          (f0, f0_ampl, f0_angle))
    print('. f1 = %d : f1_ampl = %.10f, f1_angle = %.10f' % (f1, f1_ampl, f1_angle))
    # . verify expected f0_ampl == 0
    if not np.isclose(f0_ampl, 0):
        print('. Unexpected f0_ampl = %f' % f1_angle)
        return None
    # . verify expected f1_angle == 90
    if np.isclose(f1_angle, 90):
        print('. Use cos(t + f1_angle) = cos(t + 90) = -sin(t)')
    else:
        print('. Unexpected f1_angle = %f' % f1_angle)
        return None
    # Return polar bin values
    return rfft_phi_output_yz_diff_bin_arr


def approximate_phi_output_yz_as_function_of_theta_mid(theta_mid_arr, f1_ampl):
    """Approximate phi_output_yz(theta_mid) using first harmonic from DFT

    See swivel.ipynb for more description and plots.

    Input:
    . theta_mid_arr: Range of theta_mid angles to evaluate for
         phi_output_yz_approx_arr
    . f1_ampl: first harmonic from rDFT of harmonic part in exact
        phi_output_yz(theta_mid). Few degrees for the small sinus like
        deviation from linear.
    Return:
    . phi_output_yz_approx_arr: approximate phi_output_yz(theta_mid). This
        needs to be compensated via phi_input_yz to keep swivel motion in ZX
        plane.
    """
    print('approximate_phi_output_yz_as_function_of_theta_mid():')
    print('. f1_ampl = %.3f' % f1_ampl)

    # Approximate phi_output_yz using single harmonic approximation
    # . Use cos(t + f1_angle) = cos(t + 90) = -sin(t)
    f1 = 1
    f1_phi_output_yz_arr = -f1_ampl * np.sin(np.radians(f1 * theta_mid_arr))
    phi_output_yz_approx_arr = theta_mid_arr / 2 + f1_phi_output_yz_arr
    return phi_output_yz_approx_arr


################################################################################
# Swivel harmonic approximation for theta_output_zx(theta_mid)

def rfft_bins_of_theta_output_zx_as_function_of_theta_mid(alpha_tilt,
                                                          mid_tube_rotation,
                                                          N=16):
    """Calculate DC and first harmonic using real input rDFT of sinus like
    signal that is created from exact theta_output_zx(theta_mid).

    See swivel.ipynb for more description and plots.

    Input:
    . N: number of points in theta_mid_arr and therefore of the DFT. The
         theta_mid_arr is over range [0 : 360> degrees. The maximum number
         of frequency bins is K = N // 2 + 1.
         It is not necessary to choose large N >> 16 for more accuracy.
    Return:
    . rfft_theta_output_zx_bin_arr2: rFFT bin polar values for
      theta_output_zx(theta_mid).
    """
    print('rfft_bins_of_theta_output_zx_as_function_of_theta_mid()')
    print('. mid_tube_rotation = ', mid_tube_rotation)

    phi_input_yz_horizontal = InitPhiInputYzHorizontal(mid_tube_rotation)
    theta_output_zx_horizontal = ThetaOutputZxHorizontal()
    print('. phi_input_yz_horizontal = ', phi_input_yz_horizontal)
    print('. theta_output_zx_horizontal = ', theta_output_zx_horizontal)

    # Prepare theta_mid_arr, 0 <= theta_mid < 360 degrees
    N_degrees = 360
    theta_mid_arr = np.linspace(0, N_degrees, N, endpoint=False)

    # Calculate exact phi_output_yz_arr
    # . can use x_input = 1, x_mid = 1, x_output = 1, because swivel thrust
    #   vector pointing direction is independent of swivel tube lengths.
    theta_output_zx_arr = np.zeros(N)
    for T in range(N):
        t_vector = SwivelThrustVector(1, 1, 1, alpha_tilt, phi_input_yz_horizontal, theta_mid_arr[T])
        theta_output_zx_arr[T] = theta_output_zx_horizontal + la.toAngle360(la.fAngleXR(t_vector))

    # Create sinus like signal from theta_output_arr and negated copy
    N2 = 2 * N
    theta_output_zx_negated_arr = 2 * theta_output_zx_horizontal - theta_output_zx_arr
    theta_output_zx_arr2 = np.append(theta_output_zx_arr, theta_output_zx_negated_arr)

    # Determine harmonics in theta_output_arr2 using rFFT
    # . scale k = 0 DC by 1 / N2
    # . scale k > 0 harmonics by 2 / N2 to get their amplitude
    rfft_theta_output_zx_arr2 = 2 / N2 * np.fft.rfft(theta_output_zx_arr2)
    rfft_theta_output_zx_arr2[0] = rfft_theta_output_zx_arr2[0] / 2  # DC bin
    rfft_theta_output_zx_ampl_arr2 = np.abs(rfft_theta_output_zx_arr2)
    rfft_theta_output_zx_angle_arr2 = np.degrees(np.angle(rfft_theta_output_zx_arr2))
    # . polar bin values
    rfft_theta_output_zx_bin_arr2 = (rfft_theta_output_zx_ampl_arr2,
                                     rfft_theta_output_zx_angle_arr2)

    # Single harmonic approximation for theta_output_zx as function of theta_mid
    # . log f0 DC and f1 harmonic
    f0 = 0  # DC
    f0_ampl = rfft_theta_output_zx_ampl_arr2[f0]  # = DC = = ThetaOutputZxHorizontal()
    f0_angle = rfft_theta_output_zx_angle_arr2[f0]
    f1 = 1  # first harmonic, 1 period in N2 points
    f1_ampl = rfft_theta_output_zx_ampl_arr2[f1]  # ~= SwivelTiltMax()
    f1_angle = rfft_theta_output_zx_angle_arr2[f1]
    print('. f0 = %2d : f0_ampl = %.10f, f0_angle = %6.1f, = DC = ThetaOutputZxHorizontal()' %
          (f0, f0_ampl, f0_angle))
    print('. f1 = %2d : f1_ampl = %.10f, f1_angle = %6.1f, ~= SwivelTiltMax()' %
          (f1, f1_ampl, f1_angle))

    # . verify expected f0_angle == 0 or 180 for sign +1 or -1
    if np.isclose(np.cos(f0_angle), 1):
        print('. Use +1 * f0_ampl')
    elif np.isclose(np.cos(f0_angle), -1):
        print('. Use -1 * f0_ampl')
    else:
        print('. Unexpected f0_angle = %f' % f0_angle)
        return None
    # . verify expected f1_angle == -90
    if np.isclose(f1_angle, -90):
        print('. Use cos(t + f1_angle) = cos(t - 90) = sin(t)')
    else:
        print('. Unexpected f1_angle = %f' % f1_angle)
        return None

    # Return polar bin values
    return rfft_theta_output_zx_bin_arr2


def approximate_theta_output_zx_as_function_of_theta_mid(theta_mid_arr,
                                                         f0_ampl,
                                                         f1_ampl, f1_angle):
    """Approximate theta_output_zx(theta_mid) using first harmonic from real
    input DFT.

    See swivel.ipynb for more description and plots.

    Input:
    . theta_mid_arr: Range of theta_mid angles to evaluate for
         theta_output_zx_approx_arr
    . f0_ampl: DC from rDFT of exact. DC = +f0_ampl, because f0_angle = 0, so
        cos(f0_angle) = +1, and no need to pass f0_angle on.
      f1_ampl, f1_angle: first harmonics from rDFT of exact
        theta_output_zx(theta_mid).
    Return:
    . theta_output_zx_approx_arr: approximate theta_output_zx(theta_mid)
    """
    print('approximate_theta_output_zx_as_function_of_theta_mid()')
    print('. f0_ampl = %.3f, f1_ampl = %.3f, f1_angle = %.1f' % (f0_ampl, f1_ampl, f1_angle))

    # Approximate theta_output_zx using single harmonic approximation
    # . Use cos(t + f1_angle) = cos(t - 90) = sin(t)
    f1 = 1
    f1_theta_output_zx_arr = f1_ampl * np.sin(np.radians(f1 * theta_mid_arr / 2))
    theta_output_zx_approx_arr = f0_ampl + f1_theta_output_zx_arr
    return theta_output_zx_approx_arr


# Inverse function of approximate_theta_output_zx_as_function_of_theta_mid()
def approximate_theta_mid_as_function_of_theta_output_zx(theta_output_zx_arr,
                                                         f0_ampl,
                                                         f1_ampl, f1_angle,
                                                         mid_tube_rotation):
    """Approximate theta_mid(theta_output_zx) using first harmonic from real
    input DFT.

    Input:
    . f0_ampl = DC = ThetaOutputZxHorizontal()
    . f1_ampl ~= SwivelTiltMax(alpha_tilt)
    Return:
    . theta_mid_approx_arr: approximate theta_mid(theta_output_zx)
    """
    print('approximate_theta_mid_as_function_of_theta_output_zx()')
    print('. f0_ampl = %.3f, f1_ampl = %.3f, f1_angle = %.1f' % (f0_ampl, f1_ampl, f1_angle))
    print('. mid_tube_rotation = %s' % mid_tube_rotation)

    # Approximate theta_mid using single harmonic approximation
    # . Use cos(t + f1_angle) = cos(t - 90) = sin(t)
    # . y = arcsin(x) for -1 <= x <= 1, -90 <= y <= 90, therefore the maximum
    #   theta_output_ac = f1_ampl for this approximation, and f1_ampl ~=
    #   SwivelTiltMax(alpha_tilt).
    # . Offset theta_mid = 0 by c_theta_mid_eps to avoid NaN for YZ angle of
    #   straight swivel.
    theta_output_ac_arr = theta_output_zx_arr - f0_ampl
    f1_fraction_arr = [c_theta_mid_eps if np.isclose(x, 0, atol=c_theta_mid_eps) else
                       x / f1_ampl if np.abs(x) < f1_ampl else
                       1 if x > 0 else
                       -1 for x in theta_output_ac_arr]
    theta_mid_approx_arr = 2 * np.degrees(np.arcsin(f1_fraction_arr))

    # Account for sign of mid_tube_rotation
    if mid_tube_rotation == 'positive':
        return theta_mid_approx_arr
    else:
        return theta_mid_approx_arr * -1


################################################################################
# Local self test functions for __main__

def _verify_position_vector():
    # . echo
    print('>>> Verify position vector:')
    p_vector = SwivelOutputPosition_Gonio(x_input, x_mid, x_output, alpha_tilt_gold, phi_input_yz_gold, theta_mid_gold)
    print('p_vector[0] = ', p_vector[0])
    print('p_vector[1] = ', p_vector[1])
    print('p_vector[2] = ', p_vector[2])
    print('p_vector[3] = ', p_vector[3])
    print('fAngleYZ(p_vector) = ', la.fAngleYZ(p_vector))
    print('fAngleZX(p_vector) = ', la.fAngleZX(p_vector))
    print('fAngleXY(p_vector) = ', la.fAngleXY(p_vector))
    print('fAngleXR(p_vector) = ', la.fAngleXR(p_vector))
    print()
    # . verify
    result = True
    if np.abs(p_vector[0] - 149.20526004015963) > la.f_eps or \
       np.abs(p_vector[1] - -55.94492263756476) > la.f_eps or \
       np.abs(p_vector[2] - -29.273574891694743) > la.f_eps or \
       np.abs(p_vector[3] - 1.0) > la.f_eps:
        print('Wrong p_vector')
        result = False
    if np.abs(la.fAngleYZ(p_vector) - -152.37886965244215) > la.f_eps:
        print('Wrong fAngleYZ(p_vector)')
        result = False
    if np.abs(la.fAngleZX(p_vector) - 101.10024652138387) > la.f_eps:
        print('Wrong fAngleZX(p_vector)')
        result = False
    if np.abs(la.fAngleXY(p_vector) - -20.553671748587778) > la.f_eps:
        print('Wrong fAngleXY(p_vector)')
        result = False
    if np.abs(la.fAngleXR(p_vector) - 22.93718903508899) > la.f_eps:
        print('Wrong fAngleXR(p_vector)')
        result = False
    return result


def _verify_thrust_vector():
    # . echo
    print('>>> Verify thrust vector:')
    t_vector = SwivelThrustVector_Gonio(x_input, x_mid, x_output, alpha_tilt_gold,
                                        phi_input_yz_gold, theta_mid_gold)
    print('t_vector[0] = ', t_vector[0])
    print('t_vector[1] = ', t_vector[1])
    print('t_vector[2] = ', t_vector[2])
    print('t_vector[3] = ', t_vector[3])
    print('fAngleYZ(t_vector) = ', la.fAngleYZ(t_vector))
    print('fAngleZX(t_vector) = ', la.fAngleZX(t_vector))
    print('fAngleXY(t_vector) = ', la.fAngleXY(t_vector))
    print('fAngleXR(t_vector) = ', la.fAngleXR(t_vector))
    print()
    # . verify
    result = True
    if np.abs(t_vector[0] - 26.34950784642885) > la.f_eps or \
       np.abs(t_vector[1] - -26.665070575911415) > la.f_eps or \
       np.abs(t_vector[2] - -13.952686029315188) > la.f_eps or \
       np.abs(t_vector[3] - 0.0) > la.f_eps:
        print('Wrong t_vector')
        result = False
    if np.abs(la.fAngleYZ(t_vector) - -152.37886965244215) > la.f_eps:
        print('Wrong fAngleYZ(t_vector)')
        result = False
    if np.abs(la.fAngleZX(t_vector) - 117.90227534520515) > la.f_eps:
        print('Wrong fAngleZX(t_vector)')
        result = False
    if np.abs(la.fAngleXY(t_vector) - -45.341042022386276) > la.f_eps:
        print('Wrong fAngleXY(t_vector)')
        result = False
    if np.abs(la.fAngleXR(t_vector) - 48.796326761795235) > la.f_eps:
        print('Wrong fAngleXR(t_vector)')
        result = False
    return result


def _verify_SwivelOutputPosition_Gonio_equals_SwivelOutputPosition():
    print('>>> Verify SwivelOutputPosition_Gonio == SwivelOutputPosition:')
    result = True
    step = 30
    for theta_mid in range(0, 360, step):
        for phi_input_yz in range(0, 360, step):
            position_vector = SwivelOutputPosition(x_input, x_mid, x_output, alpha_tilt,
                                                   phi_input_yz, theta_mid)
            position_vector_gonio = SwivelOutputPosition_Gonio(x_input, x_mid, x_output, alpha_tilt,
                                                               phi_input_yz, theta_mid)
            if np.abs(position_vector[0] - position_vector_gonio[0]) > la.f_eps:
                print('position vector x:', position_vector[0], '!=', position_vector_gonio[0])
                result = False
            if np.abs(position_vector[1] - position_vector_gonio[1]) > la.f_eps:
                print('position vector y:', position_vector[1], '!=', position_vector_gonio[1])
                result = False
            if np.abs(position_vector[2] - position_vector_gonio[2]) > la.f_eps:
                print('position vector z:', position_vector[2], '!=', position_vector_gonio[2])
                result = False
    if result:
        print('SwivelOutputPosition() = SwivelOutputPosition_Gonio()')
        print()
    return result


def _verify_phi_output_yz_and_theta_output_zx():
    print('>>> Verify phi_output_yz and theta_output_zx:')
    # . Use arbitrary phi_input_yz and theta_mid
    phi_input_yz = 15  # = 0 for swivel movement in ZX plane
    theta_mid = 35
    p_vector = SwivelOutputPosition(1, 1, 1, alpha_tilt, phi_input_yz, theta_mid)
    t_vector = SwivelThrustVector(1, 1, 1, alpha_tilt, phi_input_yz, theta_mid)

    # . Determine phi_output_yz of the position vector and thrust vector
    #   The position vector and thrust vector should have same angle YZ around
    #   the x-axis, because the theta_mid rotates the InputTube and the
    #   OutputTube by the same angle in opposite directions.
    p_phi_output_yz = la.toAngle360(la.fAngleYZ(p_vector))
    t_phi_output_yz = la.toAngle360(la.fAngleYZ(t_vector))

    # . Determine t_theta_output_xr of the thrust vector, with swivel movement
    #   in plane defined by phi_input_yz.
    #   The thrust vector angle t_theta_output_xr with respect to the X-axis
    #   only depends on theta_mid of the MidTube, and is therefore independent
    #   of the phi_input_yz angle of the InputTube.
    t_theta_output_xr = la.fAngleXR(t_vector)
    t_theta_output_xr_zx = ThetaOutputZxHorizontal() + t_theta_output_xr

    # . Determine t_zx_phi_input_yz and theta_output_zx for swivel movement in
    #   the ZX plane.
    #   Compensate t_zx_phi_input_yz for phi_input_yz and for impact of
    #   theta_mid on phi_output_yz, to get thrust vector t_zx_vector in the ZX
    #   plane. Then for t_zx_vector expect:
    #   . t_zx_phi_output_yz == PhiOutputYzVertical('down'),
    #   . t_zx_theta_output_zx == t_theta_output_xr.
    t_zx_phi_input_yz = PhiOutputYzVertical('down') + phi_input_yz - t_phi_output_yz
    t_zx_vector = SwivelThrustVector(1, 1, 1, alpha_tilt, t_zx_phi_input_yz, theta_mid)
    t_zx_phi_output_yz = la.toAngle360(la.fAngleYZ(t_zx_vector))
    t_zx_theta_output_zx = la.fAngleZX(t_zx_vector)

    # . verify
    result = True
    if np.abs(p_phi_output_yz - t_phi_output_yz) > la.f_eps:
        print('Wrong: phi_output_yz using p_vector and t_vector differ')
        print(p_phi_output_yz, t_phi_output_yz)
        result = False
    if np.abs(t_zx_phi_output_yz - PhiOutputYzVertical('down')) > la.f_eps:
        print('Wrong: t_zx_phi_output_yz not in ZX plane')
        print(t_zx_phi_output_yz)
        result = False
    if np.abs(t_theta_output_xr_zx - t_zx_theta_output_zx) > la.f_eps:
        print('Wrong: theta_output_zx using fAngleZX and fAngleXR differ')
        print(t_zx_theta_output_zx, t_theta_output_xr)
        result = False

    if result:
        # The phi_output_yz can be calculated using p_vector or t_vector
        print('p_phi_output_yz = t_phi_output_yz =', t_phi_output_yz)

        # The phi_output_yz in ZX plane is -90 degrees for vertical down
        print('t_zx_phi_output_yz = PhiOutputYzVertical("down") =', t_zx_phi_output_yz)

        # The t_zx_theta_output_zx can be calculated in:
        # . ZX plane using fAngleZX(t_vector),
        # . any plane using ThetaOutputZxHorizontal() + la.fAngleXR(t_vector)
        print('t_zx_theta_output_zx = t_theta_output_xr =', t_zx_theta_output_zx)
        print()
    return result


def _verify_DetermineThetaMidForThetaOutputZx():
    print('>>> Verify DetermineThetaMidForThetaOutputZx():')
    result = True
    output_tube_pointing = 'down'
    theta_output_zx_resolution = 0.01
    theta_output_zx_lo = ThetaOutputZxHorizontal()
    theta_output_zx_hi = ThetaOutputZxMax(alpha_tilt)
    step_size = 5
    nof_steps = np.round((theta_output_zx_hi - theta_output_zx_lo) / step_size) + 1
    nof_steps = nof_steps.astype(int)
    theta_output_zx_arr = np.linspace(theta_output_zx_lo, theta_output_zx_hi, nof_steps)
    print('theta_output_zx_request  theta_output_zx_result  theta_mid')
    for theta_output_zx_request in theta_output_zx_arr:
        theta_mid = DetermineThetaMidForThetaOutputZx(theta_output_zx_request, theta_output_zx_resolution,
                                                      alpha_tilt, output_tube_pointing)
        t_vector = SwivelThrustVector(1, 1, 1, alpha_tilt, 0, theta_mid)
        theta_output_zx_result = ThetaOutputZxHorizontal() + la.fAngleXR(t_vector)
        print('                    %3.0f                 %7.3f    %7.3f' %
              (theta_output_zx_request, theta_output_zx_result, theta_mid))
        if np.abs(theta_output_zx_request - theta_output_zx_result) > theta_output_zx_resolution:
            print('Wrong theta_mid for theta_output_zx_request')
            result = False
    print()
    return result


if __name__ == '__main__':

    test_passed = True

    # Use same values as in swivel_functions.scad to check that they yield the same result

    # Swivel tube segment parameters
    # . Fixed values
    alpha_tilt_gold = 25
    theta_mid_gold = 60     # Swivel off center angle [degrees]
    phi_input_yz_gold = c_phi_input_yz_construction

    x_input = 50       # Length of input segment
    x_mid = 80         # Length of middle segment
    x_output = 40      # Length of output segment

    # . Arbitrary values
    alpha_tilt = 27.5    # Segment angle [degrees]

    # Run self tests
    test_passed &= _verify_position_vector()
    test_passed &= _verify_thrust_vector()
    test_passed &= _verify_SwivelOutputPosition_Gonio_equals_SwivelOutputPosition()
    test_passed &= _verify_phi_output_yz_and_theta_output_zx()
    test_passed &= _verify_DetermineThetaMidForThetaOutputZx()

    # Echo test result
    if test_passed:
        print('PASSED all tests')
    else:
        print('FAILED one or more tests')
