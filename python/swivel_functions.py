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
# . See swivel.ipynb for usage, more description and plots.
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

# The swivel assembly is constructed with InputTube long side markers up,
# so with phi_input_yz = 90. However the zero position for the long side
# markers is better defined at phi_input_yz = 0. Therefore use
# c_phi_input_yz_construction to compensate for this construction offset, by
# subracting it from the input phi_input_yz in SwivelOutputPosition() and
# SwivelOutputPosition_Gonio().
c_phi_input_yz_construction = 90


################################################################################
# Swivel angles class
class SwivelAngles:
    """Swivel angles.

    Constant angles and angles that depend on the input parameters.

    Input:
    . alpha_tilt: MidTube construction angle with InputTube and with OutputTube.
    . output_tube_pointing: OutputTube YZ pointing 'down' or 'up', changing
      the pointing to opposite maximum tilt requires a 180 degree YZ rotation
      of the InputTube, because the MidTube rotation can only change the
      pointing between horizontal (= straight) and maximum tilt, not to
      -maximum tilt.
    . mid_tube_rotation: MidTube rotation to tilt the swivel output between
      straight and maximum tilt.
    """
    pass

    def __init__(self, alpha_tilt=25, output_tube_pointing='down', mid_tube_rotation='positive'):
        # Input parameters
        self.alpha_tilt = alpha_tilt
        self.output_tube_pointing = output_tube_pointing
        self.mid_tube_rotation = mid_tube_rotation

        # Constants
        self.swivel_tilt_xr_vertical = self.SwivelTiltXrVertical()
        self.phi_input_yz_for_marker_up = self.PhiInputYzForInputMarkerUp()
        self.phi_input_yz_for_analysis = self.PhiInputYzForAnalysis()
        self.phi_output_yz_for_analysis = self.PhiOutputYzForAnalysis()
        self.theta_mid_to_phi_output_yz_crosstalk_max = self.ThetaMidToPhiOutputYzCrosstalkMax()
        self.theta_output_zx_horizontal = self.ThetaOutputZxHorizontal()

        # Derived from alpha_tilt
        self.swivel_tilt_xr_max = self.SwivelTiltXrMax(alpha_tilt)
        self.theta_output_zx_max = self.ThetaOutputZxMax(alpha_tilt)
        self.theta_output_zx_min = self.ThetaOutputZxMin(alpha_tilt)

        # Derived from output_tube_pointing and/or mid_tube_rotation
        self.phi_output_yz_vertical = self.PhiOutputYzVertical(output_tube_pointing)
        self.phi_input_yz_vertical = self.PhiInputYzVertical(output_tube_pointing)
        self.phi_input_yz_horizontal = self.InitPhiInputYzHorizontal(output_tube_pointing, mid_tube_rotation)
        self.phi_output_yz_horizontal = self.InitPhiOutputYzHorizontal(output_tube_pointing, mid_tube_rotation)

        self.theta_output_zx_vertical = self.ThetaOutputZxVertical(output_tube_pointing)

        # Other
        self.control_orientations = [('down', 'positive'),
                                     ('down', 'negative'),
                                     ('up', 'positive'),
                                     ('up', 'negative')]

    def LogSwivelAngles(self):
        """Log swivel angles"""
        print('Input parameters:')
        print('. alpha_tilt                 = %.0f' % self.alpha_tilt)
        print('. output_tube_pointing       = %s' % self.output_tube_pointing)
        print('. mid_tube_rotation          = %s' % self.mid_tube_rotation)
        print('Constant swivel angles:')
        print('. swivel_tilt_xr_vertical                  = %.0f' % self.swivel_tilt_xr_vertical)
        print('. phi_input_yz_for_marker_up               = %.0f' % self.phi_input_yz_for_marker_up)
        print('. phi_input_yz_for_analysis                = %.0f' % self.phi_input_yz_for_analysis)
        print('. phi_output_yz_for_analysis               = %.0f' % self.phi_output_yz_for_analysis)
        print('. theta_mid_to_phi_output_yz_crosstalk_max = %.0f' % self.theta_mid_to_phi_output_yz_crosstalk_max)
        print('. theta_output_zx_horizontal               = %.0f' % self.theta_output_zx_horizontal)
        print('. theta_output_zx_horizontal               = %.0f' % self.theta_output_zx_horizontal)
        print('Derived from alpha_tilt:')
        print('. swivel_tilt_xr_max         = %.0f' % self.swivel_tilt_xr_max)
        print('. theta_output_zx_max        = %.0f' % self.theta_output_zx_max)
        print('. theta_output_zx_min        = %.0f' % self.theta_output_zx_min)
        print('Derived from output_tube_pointing:')
        print('. phi_output_yz_vertical     = %.0f' % self.phi_output_yz_vertical)
        print('. phi_input_yz_vertical      = %.0f' % self.phi_input_yz_vertical)
        print('. theta_output_zx_vertical   = %.0f' % self.theta_output_zx_vertical)
        print('Derived from output_tube_pointing and mid_tube_rotation:')
        print('. phi_input_yz_horizontal    = %.0f' % self.phi_input_yz_horizontal)
        print('. phi_output_yz_horizontal   = %.0f' % self.phi_output_yz_horizontal)

    def LogYzAnglesTable(self):
        """Show InputTube YZ and OutputTube YZ angles.

        Dependend on MidTube angle, output_tube_pointing and mid_tube_rotation.
        """
        print('                     phi_output_yz  phi_input_yz  phi_input_yz  phi_output_yz')
        print('                          vertical      vertical    horizontal     horizontal')
        print('           theta_mid:          180           180             0              0')
        print('swivel')
        print('output    mid_tube')
        print('pointing  rotation')
        for pointing, sign in self.control_orientations:
            print('  %-8s  %-8s: %12d  %12d  %12d  %13d' % (pointing,
                                                            sign,
                                                            self.PhiOutputYzVertical(pointing),
                                                            self.PhiInputYzVertical(pointing),
                                                            self.InitPhiInputYzHorizontal(pointing, sign),
                                                            self.InitPhiOutputYzHorizontal(pointing, sign)))

    # Swivel OutputTube YZ angle = 0 when swivel is straight for analysis of
    # phi_output_yz(theta_mid) function.
    def PhiInputYzForAnalysis(self):
        """InputTube YZ angle to have phi_output_yz_arr[theta_mid = 0] = 0.

        With phi_output_yz = PhiInputYzForAnalysis(), then the
        phi_output_yz(theta_mid) curve will about linearly increase from
        0 : 180 for theta_mid 0 : 360. Using PhiInputYzForAnalysis() avoids
        that the phi_output_yz angle wraps around 0 or 360 degrees, so that the
        curve is suitable for DFT analysis.
        """
        return 270  # = -ThetaMidToPhiOutputYzCrosstalkMax()

    def PhiOutputYzForAnalysis(self):
        """Start OutputTube YZ angle to have phi_output_yz_arr[theta_mid = 0]
        = 0.

        With phi_input_yz == PhiInputYzForAnalysis() then phi_output_yz(0) = 0.
        """
        return 0

    def ThetaMidToPhiOutputYzCrosstalkMax(self):
        """Maximum crosstalk between MidTube rotation and OutputTube YZ angle

        The crosstalk is 0 when swivel is straight and 90 when maximum tilted.
        The OutputTube YZ angle rotates +90 degrees when the MidTube rotates
        positive from 0 to 180 degrees, and -90 degrees when the MidTube rotates
        negative from 0 to -180 degrees.
        """
        return 90

    # Swivel phi_input_yz control angle from y to z in YZ plane
    def PhiInputYzForInputMarkerUp(self):
        """phi_input_yz angle in YZ plane for InputTube long side marker up."""
        return 90

    def InitPhiInputYzHorizontal(self, output_tube_pointing, mid_tube_rotation):
        """Initial InputTube YZ angle for horizontal, straight swivel, when
        theta_mid = 0, to prepare for vertical swivel when theta_mid > 0.

        InitPhiInputYzHorizontal() prepares phi_input_yz of horizontal, straight
        swivel, at theta_mid = 0, such that including compensating for the
        counter rotation the swivel phi_output_yz will reach
        PhiOutputYzVertical() for vertical, maximum tilted swivel. The
        phi_input_yz is then PhiInputYzVertical().

        The InitPhiInputYzHorizontal() can be found by starting with a maximum
        tilted swivel and then rotating the MidTube back to theta_mid = 0 and
        meanwhile compensating the InputTube YZ angle to keep the swivel output
        in the vertical ZX plane.

        Note that mid_tube_rotation is defined positive for theta_mid from 0 to
        180 downto 0 and defined negative for theta_mid from 360 downto 180 to
        360 (= 0 downto -180 to 0).
        """
        if output_tube_pointing == "down":
            if mid_tube_rotation == "positive":
                return 180
            else:
                return 0
        else:
            if mid_tube_rotation == "positive":
                return 0
            else:
                return 180

    def PhiInputYzVertical(self, output_tube_pointing):
        """InputTube YZ angle for vertical, maximum tilted swivel, when
        theta_mid = 180.

        The PhiInputYzVertical() follows directly from putting the maximum
        tilted swivel in down or up pointing and then taking the InputTube YZ
        angle.
        """
        phi_output_yz_vertical = self.PhiOutputYzVertical(output_tube_pointing)
        return la.toAngle360(phi_output_yz_vertical + 180)

    # Swivel thrust output angle phi_output_yz from y to z in YZ plane
    def PhiOutputYzVertical(self, output_tube_pointing):
        """OutputTube YZ angle for vertical, maximum tilted swivel, when
        theta_mid = 180.
        """
        if output_tube_pointing == "down":
            return 270  # = -90
        else:
            return 90  # up = down - 180

    def InitPhiOutputYzHorizontal(self, output_tube_pointing, mid_tube_rotation):
        """Initial OutputTube YZ angle for horizontal, straight swivel, when
        theta_mid = 0.

        InitPhiOutputYzHorizontal() = InitPhiInputYzHorizontal(). This can be
        seen when OutputTube is not rotated and kept tilted with the MidTube.
        """
        return self.InitPhiInputYzHorizontal(output_tube_pointing, mid_tube_rotation)

    # Swivel theta_output_xr thrust angle with respect to x-axis
    def SwivelTiltXrMax(self, alpha_tilt):
        """Maximum swivel thrust XR angle."""
        return alpha_tilt * 4

    def SwivelTiltXrVertical(self):
        """XR angle for vertical swivel."""
        return 90

    # Swivel theta_output_zx thrust angle from z to x
    def ThetaOutputZxHorizontal(self):
        """Horizontal swivel thrust angle theta_output_zx in ZX plane."""
        return 90

    def ThetaOutputZxAngle(self, theta_output_xr, output_tube_pointing):
        """Swivel thrust theta_output_zx from theta_output_xr."""
        if output_tube_pointing == "down":
            theta_output_zx = self.ThetaOutputZxHorizontal() + theta_output_xr
        else:
            theta_output_zx = self.ThetaOutputZxHorizontal() - theta_output_xr
        return theta_output_zx

    def ThetaOutputZxMax(self, alpha_tilt):
        """Maximum swivel thrust theta_output_zx angle (with swivel output down)."""
        return self.ThetaOutputZxHorizontal() + self.SwivelTiltXrMax(alpha_tilt)

    def ThetaOutputZxMin(self, alpha_tilt):
        """Minimum swivel thrust theta_output_zx angle (with swivel output up)."""
        return self.ThetaOutputZxHorizontal() - self.SwivelTiltXrMax(self.alpha_tilt)

    def ThetaOutputZxVertical(self, output_tube_pointing):
        """Swivel thrust angle theta_output_zx in ZX plane for vertical down or up.
        """
        if self.output_tube_pointing == 'down':
            return 180
        else:
            return 0


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

def SwivelThrustVector(alpha_tilt, phi_input_yz, theta_mid):
    """Calculate thrust vector of swivel output.

    . As function of InputTube YZ angle phi_input_yz and MidTube angle
      theta_mid.
    . Replace theta_mid == 0 by theta_mid = la.f_eps to avoid arctan() = nan
      for YZ angle of the thrust vector when the swivel is straight.
    . Uses matrices.
    . The thrust vector pointing does not depend on x_input, x_mid, x_output,
      so use fixed 1, 1, 1 and 1, 1, 0 to derive the pointing.
    """
    return \
        SwivelOutputPosition(1, 1, 1, alpha_tilt, phi_input_yz, theta_mid) - \
        SwivelOutputPosition(1, 1, 0, alpha_tilt, phi_input_yz, theta_mid)


def SwivelThrustVectorsList(alpha_tilt, phi_input_yz_arr, theta_mid_arr):
    """Calculate thrust vectors of swivel outputs.

    . For array of InputTube YZ angles phi_input_yz_arr and corresponding array
      of MidTube angles theta_mid_arr.
    """
    tVectorList = []
    [tVectorList.append(SwivelThrustVector(alpha_tilt, phi_input_yz, theta_mid))
     for phi_input_yz, theta_mid in zip(phi_input_yz_arr, theta_mid_arr)]
    return tVectorList


def SwivelThrustVector_Gonio(alpha_tilt, phi_input_yz, theta_mid):
    """Calculate thrust vector of swivel output.

    . Uses gonio equations obtained from elaborating the matrices.
    . Equivalent to SwivelThrustVector(), so SwivelThrustVector_Gonio() yields
      same thrust vector.
    """
    return \
        SwivelOutputPosition_Gonio(1, 1, 1, alpha_tilt, phi_input_yz, theta_mid) - \
        SwivelOutputPosition_Gonio(1, 1, 0, alpha_tilt, phi_input_yz, theta_mid)


################################################################################
# Swivel control

def FindThetaMidForThetaOutputXr(theta_output_xr_request, theta_output_xr_resolution,
                                 alpha_tilt, mid_tube_rotation):
    """Find theta_mid that yields requested theta_output_xr.

    Uses binary search and fAngleXR(SwivelThrustVector()) to find theta_mid,
    because the exact inverse function to calculate theta_mid as function of
    theta_output_xr is not available.

    Return:
    . theta_mid: MidTube angle in <0:360> to achieve theta_output_xr_request.
      . in [c_theta_mid_eps:180] when mid_tube_rotation is positive
      . in [180:360 - c_theta_mid_eps] when mid_tube_rotation is negative
    """
    sa = SwivelAngles(alpha_tilt=alpha_tilt,
                      mid_tube_rotation=mid_tube_rotation)
    # Fixed theta_mid at xr_min and xr_max ends of theta_output_xr range
    xr_min = 0
    xr_max = sa.SwivelTiltXrMax(alpha_tilt)
    if np.isclose(theta_output_xr_request, 0, atol=la.f_eps):
        theta_mid = 0
    elif np.isclose(theta_output_xr_request, xr_max, atol=la.f_eps):
        theta_mid = 180
    elif theta_output_xr_request > xr_max:
        print('Too large theta_output_xr_request')
        theta_mid = 180
    elif theta_output_xr_request < xr_min:
        print('Too small theta_output_xr_request')
        theta_mid = 0
    else:
        # Binary search theta_mid in [0:180]
        theta_mid_lo = 0
        theta_mid_hi = 180
        n = 0
        n_max = np.ceil(np.log2((theta_mid_hi - theta_mid_lo) / theta_output_xr_resolution))
        while n < n_max:
            theta_mid = (theta_mid_lo + theta_mid_hi) / 2
            t_vector = SwivelThrustVector(alpha_tilt, 0, theta_mid)
            theta_output_xr = la.fAngleXR(t_vector)
            if theta_output_xr > theta_output_xr_request + theta_output_xr_resolution:
                theta_mid_hi = theta_mid
            elif theta_output_xr < theta_output_xr_request - theta_output_xr_resolution:
                theta_mid_lo = theta_mid
            else:
                break
            n = n + 1
    # Map MidTube angle theta_mid in [0:180] to <0:180] or to [180:360>
    # dependent on the mid_tube_rotation.
    theta_mid_arr = map_theta_mid_rotation([theta_mid], mid_tube_rotation)
    return theta_mid_arr[0]


def map_theta_mid_rotation(theta_mid_180_arr, mid_tube_rotation):
    """Map theta_mid in [0:180] to <0:180] or to [180:360>, dependent on
    mid_tube_rotation.

    . Assume all theta_mid in theta_mid_180_arr are in [0:180]
    . Replace theta_mid ~= 0 by c_theta_mid_eps to get <0:180]. To avoid
      undefined YZ angle for horizontal swivel.
    . Keep theta_mid <0:180] when MidTube rotation is positive, else negate
      theta_mid by 360 - theta_mid, if MidTube rotation direction is negative.
    """
    theta_mid_positive_arr = np.array(
        [c_theta_mid_eps if np.isclose(tm, 0, atol=c_theta_mid_eps) else
         tm for tm in theta_mid_180_arr])
    if mid_tube_rotation == 'positive':
        return theta_mid_positive_arr  # theta_mid in <0:180]
    else:
        return 360 - theta_mid_positive_arr  # theta_mid in [180:360>


################################################################################
# Swivel harmonic approximation for phi_output_yz(theta_mid)

def rfft_bins_of_phi_output_yz_as_function_of_theta_mid(alpha_tilt, N=16):
    """Calculate DC and first harmonic using real input rDFT of sinus like
    part in exact phi_output_yz(theta_mid).

    Input:
    . N: number of points in theta_mid_arr and therefore of the DFT. The
         theta_mid_arr is over range [0 : 360> degrees. The maximum number
         of frequency bins is K = N // 2 + 1.
         It is not necessary to choose large N >> 16 for more accuracy.
    Return:
    . rfft_phi_output_yz_diff_bin_arr: rFFT bin polar values for
      phi_output_yz(theta_mid).
    """
    sa = SwivelAngles(alpha_tilt=alpha_tilt)
    print('rfft_bins_of_phi_output_yz_as_function_of_theta_mid()')
    phi_input_yz_for_analysis = sa.PhiInputYzForAnalysis()
    phi_output_yz_for_analysis = sa.PhiOutputYzForAnalysis()
    print('. phi_input_yz_for_analysis = ', phi_input_yz_for_analysis)
    print('. phi_output_yz_for_analysis = ', phi_output_yz_for_analysis)

    # Prepare theta_mid_arr, 0 <= theta_mid < 360 degrees
    N_degrees = 360
    theta_mid_arr = np.linspace(0, N_degrees, N, endpoint=False)

    # Calculate exact phi_output_yz_arr
    phi_output_yz_arr = np.zeros(N)
    for T in range(N):
        t_vector = SwivelThrustVector(alpha_tilt, phi_input_yz_for_analysis, theta_mid_arr[T])
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


def approximate_phi_output_yz_crosstalk(theta_mid_arr, f1_ampl):
    """Approximate crosstalk phi_output_yz(theta_mid) using the first harmonic
    from DFT.

    Equation:
      phi_output_yz_crosstalk = theta_mid / 2 - f1_ampl * sin(theta_mid)
      . for theta_mid in [0:360>

    Input:
    . theta_mid_arr: MidTube angles in [0:360> to evalute in the equation.
    . f1_ampl: first harmonic from rDFT of harmonic part in exact
        phi_output_yz(theta_mid). Few degrees for the small sinus like
        deviation from linear.
    Return:
    . phi_output_yz_crosstalk_arr: Equation result for theta_mid_arr.
    """
    # Crosstalk linear term in approximate phi_output_yz(theta_mid)
    phi_output_yz_crosstalk_arr = la.toAngleArr360(theta_mid_arr / 2)

    # Improve approximate phi_output_yz(theta_mid) using single harmonic
    # approximation
    # . Use cos(t + f1_angle) = cos(t + 90) = -sin(t)
    f1_phi_output_yz_arr = -f1_ampl * np.sin(np.radians(theta_mid_arr))
    phi_output_yz_crosstalk_arr += f1_phi_output_yz_arr
    return phi_output_yz_crosstalk_arr


def approximate_phi_output_yz_as_function_of_theta_mid(theta_mid_arr, f1_ampl,
                                                       verbosity=1):
    """Approximate phi_output_yz(theta_mid) using the first harmonic from DFT.

    Equation:
      phi_output_yz(theta_mid) ~=
         -phi_input_yz_for_analysis + approximate_phi_output_yz_crosstalk(theta_mid_arr, f1_ampl)

         with: approximate_phi_output_yz_crosstalk(theta_mid_arr, f1_ampl) =
                   theta_mid / 2 - f1_ampl * sin(theta_mid)
         for theta_mid in [0:360>

    Input:
    . theta_mid_arr: MidTube angles in [0:360> to evaluate for
         phi_output_yz_approx_arr.
    . f1_ampl: first harmonic from rDFT of harmonic part in exact
        phi_output_yz(theta_mid). Few degrees for the small sinus like
        deviation from linear. Assume DFT harmonic f1_ampl is calculated with
        rfft_bins_of_phi_output_yz_as_function_of_theta_mid().
    Return:
    . phi_output_yz_approx_arr: approximate phi_output_yz(theta_mid). This
        needs to be compensated via phi_input_yz to keep swivel motion in ZX
        plane.
    """
    sa = SwivelAngles()
    if verbosity:
        print('approximate_phi_output_yz_as_function_of_theta_mid():')
        print('. f1_ampl              = %.3f' % f1_ampl)

    # Anaysis offset term
    phi_input_yz_for_analysis = sa.PhiInputYzForAnalysis()
    if verbosity:
        print('. phi_input_yz_for_analysis  = %d' % phi_input_yz_for_analysis)

    # Crosstalk linear term and single harmonic term
    phi_output_yz_crosstalk_arr = approximate_phi_output_yz_crosstalk(theta_mid_arr, f1_ampl)

    # Combine terms
    phi_output_yz_approx_arr = -phi_input_yz_for_analysis + phi_output_yz_crosstalk_arr
    return la.toAngleArr360(phi_output_yz_approx_arr)


def log_and_verify_input_tube_control_near_horizontal(f1_phi_output_yz_ampl=0):
    """Log and verify InputTube control near horizontal for vertical swivel
    OutputTube.

    The MidtTube angle theta_mid is used to tilt the swivel, but it also causes
    crosstalk to the OutputTube YZ angle. The equation yields the InputTube YZ
    angle for a requested OutputTube YZ angle and MidTube angle.

    Equation:
        phi_input_yz_control ~= phi_output_yz_request - approximate_phi_output_yz_theta_mid

        with: approximate_phi_output_yz_theta_mid
                  ~= -phi_input_yz_for_analysis + theta_mid / 2 - f1_ampl * sin(theta_mid)
                  for theta_mid in [0:360>
    """
    sa = SwivelAngles()
    print('>>> Verify log_and_verify_input_tube_control_near_horizontal():')
    print('sa.phi_input_yz_for_analysis =', sa.phi_input_yz_for_analysis)
    print('sa.phi_output_yz_for_analysis =', sa.phi_output_yz_for_analysis)
    print('f1_phi_output_yz_ampl =', f1_phi_output_yz_ampl)
    print('')
    print('swivel                           phi       phi        phi                        f1_phi')
    print('output    mid_tube               input_yz  output_yz  input_yz                   output_yz_ampl *')
    print('pointing  rotation    theta_mid  control   vertical   for_analysis  theta_mid/2  sin(theta_mid)')
    result = True
    for pointing, sign in sa.control_orientations:
        phi_input_yz_horizontal = sa.InitPhiInputYzHorizontal(pointing, sign)
        phi_output_yz_vertical = sa.PhiOutputYzVertical(pointing)
        if sign == 'positive':
            theta_mid_zero = c_theta_mid_eps
            theta_mid_str = '+0'
        else:
            theta_mid_zero = 360 - c_theta_mid_eps
            theta_mid_str = '-0'
        approximate_phi_output_yz_theta_mid_arr = approximate_phi_output_yz_as_function_of_theta_mid(
                                                      np.array([theta_mid_zero]),
                                                      f1_phi_output_yz_ampl,
                                                      verbosity=0)
        approximate_phi_output_yz_theta_mid = approximate_phi_output_yz_theta_mid_arr[0]
        phi_input_yz_control = la.toAngle360(np.round(phi_output_yz_vertical - approximate_phi_output_yz_theta_mid))
        # Log InputTube YZ control near theta_mid = 0 for vertical swivel OutputTube YZ
        print('  %-8s  %-8s:  %8s  %6.f  = %5.f    + %6.f      - %7.f    + %4.f' %
              (pointing, sign,
               theta_mid_str,
               phi_input_yz_control,
               phi_output_yz_vertical,
               sa.phi_input_yz_for_analysis,
               np.round(theta_mid_zero / 2),
               np.round(f1_phi_output_yz_ampl * np.sin(np.radians(theta_mid_zero)))))
        # Verify InputTube YZ control for vertical swivel OutputTube YZ near theta_mid = 0, is horizontal
        if phi_input_yz_control != phi_input_yz_horizontal:
            result = False
    print('')
    return result


################################################################################
# Swivel harmonic approximation for theta_output_xr(theta_mid)

def rfft_bins_of_theta_output_xr_as_function_of_theta_mid(alpha_tilt, N=16):
    """Calculate DC and first harmonic using real input rDFT of sinus like
    signal that is created from exact theta_output_xr(theta_mid).

    Input:
    . N: number of points in theta_mid_arr and therefore of the DFT. The
         theta_mid_arr is over range [0 : 360> degrees. The maximum number
         of frequency bins is K = N // 2 + 1.
         It is not necessary to choose large N >> 16 for more accuracy.
    Return:
    . rfft_theta_output_xr_bin_arr: rFFT bin polar values for
      theta_output_xr(theta_mid).
    """
    print('rfft_bins_of_theta_output_xr_as_function_of_theta_mid()')

    # Prepare theta_mid_arr, 0 <= theta_mid < 360 degrees
    N_degrees = 360
    theta_mid_arr = np.linspace(0, N_degrees, N, endpoint=False)

    # Calculate exact theta_output_xr_arr
    # . can use x_input = 1, x_mid = 1, x_output = 1, because swivel thrust
    #   vector pointing direction is independent of swivel tube lengths.
    theta_output_xr_arr = np.zeros(N)
    for T in range(N):
        t_vector = SwivelThrustVector(alpha_tilt, 0, theta_mid_arr[T])
        theta_output_xr_arr[T] = la.fAngleXR(t_vector)

    # Create sinus like signal from theta_output_arr and negated copy
    N2 = 2 * N
    theta_output_xr_arr2 = np.append(theta_output_xr_arr, -theta_output_xr_arr)

    # Determine harmonics in theta_output_arr2 using rFFT
    # . scale k = 0 DC by 1 / N2
    # . scale k > 0 harmonics by 2 / N2 to get their amplitude
    rfft_theta_output_xr_arr = 2 / N2 * np.fft.rfft(theta_output_xr_arr2)
    rfft_theta_output_xr_arr[0] = rfft_theta_output_xr_arr[0] / 2  # DC bin
    rfft_theta_output_xr_ampl_arr = np.abs(rfft_theta_output_xr_arr)
    rfft_theta_output_xr_angle_arr = np.degrees(np.angle(rfft_theta_output_xr_arr))
    # . polar bin values
    rfft_theta_output_xr_bin_arr = (rfft_theta_output_xr_ampl_arr,
                                    rfft_theta_output_xr_angle_arr)

    # Single harmonic approximation for theta_output_xr as function of theta_mid
    f0 = 0  # DC
    f0_ampl = rfft_theta_output_xr_ampl_arr[f0]
    f1 = 1  # first harmonic, 1 period in N2 points
    f1_ampl = rfft_theta_output_xr_ampl_arr[f1]  # ~= SwivelTiltXrMax()
    f1_angle = rfft_theta_output_xr_angle_arr[f1]
    print('. f1 = %2d : f1_ampl = %.10f, f1_angle = %6.1f' % (f1, f1_ampl, f1_angle))

    # . verify expected f0_ampl is DC = 0
    if not np.isclose(f0_ampl, 0):
        print('. Unexpected f0_ampl = %f' % f0_ampl)
        return None
    # . verify expected f1_angle is -90
    if np.isclose(f1_angle, -90):
        print('. Use cos(t + f1_angle) = cos(t - 90) = sin(t)')
    else:
        print('. Unexpected f1_angle = %f' % f1_angle)
        return None

    # Return polar bin values
    return rfft_theta_output_xr_bin_arr


def approximate_theta_output_xr_as_function_of_theta_mid(theta_mid_arr, f1_ampl):
    """Approximate theta_output_xr(theta_mid) using first harmonic from real
    input DFT.

    Equation:
      theta_output_xr = f1_ampl * abs(sin(theta_mid / 2))
      . for theta_mid in [0:360>
      . use abs() because XR angle in [0:180>

    Input:
    . theta_mid_arr: MidTube angles in [0:360> to evaluate in the equation
      f1_ampl: first harmonic from rDFT of exact theta_output_xr(theta_mid).
    Return:
    . theta_output_xr_approx_arr: Equation result for theta_mid_arr.
    """
    print('approximate_theta_output_xr_as_function_of_theta_mid()')
    print('. f1_ampl = %.3f' % f1_ampl)

    # Approximate theta_output_xr using single harmonic approximation
    # . Use cos(t + f1_angle) = cos(t - 90) = sin(t)
    theta_output_xr_approx_arr = f1_ampl * np.abs(np.sin(np.radians(theta_mid_arr / 2)))
    return theta_output_xr_approx_arr


# Inverse function of approximate_theta_output_xr_as_function_of_theta_mid()
def approximate_theta_mid_as_function_of_theta_output_xr(theta_output_xr_arr,
                                                         f1_ampl,
                                                         mid_tube_rotation):
    """Approximate theta_mid(theta_output_xr) using first harmonic from real
    input DFT.

    The swivel is horizontal when theta_mid = 0 and maximum tilted when
    theta_mid = 180. It is not possible to tilt the swivel in the -maximum
    tilted direction controling only theta_mid, because to change between
    swivel tilting up or down requires a 180 jump in InputTube YZ. Therefore
    theta_output_xr_arr must only contain angles for swivel tilt relative
    to the x-axis, so theta_output_xr >= 0. If theta_output_xr > f1_ampl then
    limit it to f1_ampl.

    Equation:
      theta_mid ~= 2 * arcsin(f1_fraction)

      with:
      . f1_theta_output_xr_ampl ~= swivel_tilt_xr_max, first harmonic from rDFT
        of exact theta_output_xr(theta_mid).
      . f1_fraction = theta_output_xr / f1_theta_output_xr_ampl
        f1_fraction = 1 if f1_fraction > 1 (so theta_output_xr >
                      f1_theta_output_xr_ampl)
        f1_fraction in [0:1], because theta_output_xr >= 0
      . theta_mid in [0:180], because f1_fraction in [0:1]
        theta_mid = c_theta_mid_eps if theta_mid < c_theta_mid_eps ~= 0
        theta_mid in <0:180] when mid_tube_rotation is positive
        theta_mid = 360 - theta_mid, when mid_tube_rotation is negative, so in
                    [180:360>

    Input:
    . theta_output_xr_arr: OutputTube XR angles to evaluate in the equation.
    . f1_ampl ~= SwivelTiltXrMax(alpha_tilt)
    . mid_tube_rotation: defines sign of theta_mid_approx_arr
    Return:
    . theta_mid_approx_arr: Equation result for theta_output_xr_arr, with
        theta_mid in <0:360>
    """
    print('approximate_theta_mid_as_function_of_theta_output_xr()')
    print('. f1_ampl = %.3f' % f1_ampl)
    print('. mid_tube_rotation = %s' % mid_tube_rotation)

    # Approximate theta_mid using single harmonic approximation
    # . Use cos(t + f1_angle) = cos(t - 90) = sin(t)
    # . y = arcsin(x) for -1 <= x <= 1, -90 <= y <= 90, therefore the maximum
    #   theta_output_xr = f1_ampl for this approximation, and f1_ampl ~=
    #   SwivelTiltXrMax(alpha_tilt).
    # . Offset theta_mid = 0 by c_theta_mid_eps to avoid NaN for YZ angle of
    #   straight swivel when theta_output_xr ~= 0.
    f1_fraction_arr = [xr / f1_ampl if xr < f1_ampl else
                       1 for xr in theta_output_xr_arr]
    # . MidTube angle in [0:180]
    f1_theta_mid_approx_arr = 2 * np.degrees(np.arcsin(f1_fraction_arr))

    # Map MidTube angles in [0:180] to <0:180] or to [180:360> dependent on the
    # mid_tube_rotation.
    theta_mid_approx_arr = map_theta_mid_rotation(f1_theta_mid_approx_arr, mid_tube_rotation)
    return theta_mid_approx_arr


################################################################################
# Other

def log_result(result):
    print('')
    if result:
        print('PASSED')
    else:
        print('FAILED')


def snr(requested_arr, error_arr):
    """Signal to noise ratio (SNR) of requested values and error values.

    . assume error_arr = requested_arr - result_arr
    . SNR = power of requested_arr / power of error_arr, and 10 log10() to have
            value in dB
    """
    if len(requested_arr) == len(error_arr):
        return 10 * np.log10(np.sum(requested_arr**2) / np.sum(error_arr**2))
    else:
        return False


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
    t_vector = SwivelThrustVector_Gonio(alpha_tilt_gold,
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
    if np.abs(t_vector[0] - 0.6587376961607214) > la.f_eps or \
       np.abs(t_vector[1] - -0.6666267643977856) > la.f_eps or \
       np.abs(t_vector[2] - -0.34881715073287967) > la.f_eps or \
       np.abs(t_vector[3] - 0.0) > la.f_eps:
        print('Wrong t_vector')
        result = False
    if np.abs(la.fAngleYZ(t_vector) - -152.37886965244215) > la.f_eps:
        print('Wrong fAngleYZ(t_vector)')
        result = False
    if np.abs(la.fAngleZX(t_vector) - 117.90227534520513) > la.f_eps:
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
    sa = SwivelAngles()
    print('>>> Verify phi_output_yz and theta_output_zx:')
    # . Use arbitrary phi_input_yz and theta_mid
    phi_input_yz = 15  # = 0 for swivel movement in ZX plane
    theta_mid = 35
    p_vector = SwivelOutputPosition(x_input, x_mid, x_output, alpha_tilt, phi_input_yz, theta_mid)
    t_vector = SwivelThrustVector(alpha_tilt, phi_input_yz, theta_mid)

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
    t_theta_output_xr_zx = sa.ThetaOutputZxHorizontal() + t_theta_output_xr

    # . Determine t_zx_phi_input_yz and theta_output_zx for swivel movement in
    #   the ZX plane.
    #   Compensate t_zx_phi_input_yz for phi_input_yz and for impact of
    #   theta_mid on phi_output_yz, to get thrust vector t_zx_vector in the ZX
    #   plane. Then for t_zx_vector expect:
    #   . t_zx_phi_output_yz == PhiOutputYzVertical('down'),
    #   . t_zx_theta_output_zx == t_theta_output_xr.
    t_zx_phi_input_yz = sa.PhiOutputYzVertical('down') + phi_input_yz - t_phi_output_yz
    t_zx_vector = SwivelThrustVector(alpha_tilt, t_zx_phi_input_yz, theta_mid)
    t_zx_phi_output_yz = la.toAngle360(la.fAngleYZ(t_zx_vector))
    t_zx_theta_output_zx = la.fAngleZX(t_zx_vector)

    # . verify
    result = True
    if np.abs(p_phi_output_yz - t_phi_output_yz) > la.f_eps:
        print('Wrong: phi_output_yz using p_vector and t_vector differ')
        print(p_phi_output_yz, t_phi_output_yz)
        result = False
    if np.abs(t_zx_phi_output_yz - sa.PhiOutputYzVertical('down')) > la.f_eps:
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


def _verify_FindThetaMidForThetaOutputXr():
    sa = SwivelAngles()
    print('>>> Verify FindThetaMidForThetaOutputXr():')
    result = True
    mid_tube_rotation = 'positive'
    theta_output_xr_resolution = 0.01
    swivel_tilt_xr_max = sa.SwivelTiltXrMax(alpha_tilt)
    step_size = 5
    nof_steps = np.round(swivel_tilt_xr_max / step_size) + 1
    nof_steps = nof_steps.astype(int)
    theta_output_xr_arr = np.linspace(0, swivel_tilt_xr_max, nof_steps)
    print('theta_output_xr_request  theta_output_xr_result  theta_mid')
    for theta_output_xr_request in theta_output_xr_arr:
        theta_mid = FindThetaMidForThetaOutputXr(theta_output_xr_request, theta_output_xr_resolution,
                                                 alpha_tilt, mid_tube_rotation)
        t_vector = SwivelThrustVector(alpha_tilt, 0, theta_mid)
        theta_output_xr_result = la.fAngleXR(t_vector)
        print('                    %3.0f                 %7.3f    %7.3f' %
              (theta_output_xr_request, theta_output_xr_result, theta_mid))
        if np.abs(theta_output_xr_request - theta_output_xr_result) > theta_output_xr_resolution:
            print('Wrong theta_mid for theta_output_xr_request')
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
    test_passed &= _verify_FindThetaMidForThetaOutputXr()
    test_passed &= log_and_verify_input_tube_control_near_horizontal()

    # Echo test result
    if test_passed:
        print('PASSED all tests')
    else:
        print('FAILED one or more tests')
