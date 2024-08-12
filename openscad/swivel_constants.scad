//------------------------------------------------------------------------------
// Copyright 2024 E. Kooistra
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
// Author: E. Kooistra, March 2024
// Purpose: Library of swivel constants to include
// Description:

//------------------------------------------------------------------------------
// General
//------------------------------------------------------------------------------

// Use theta_mid_arr + c_theta_mid_eps to offset theta_mid = 0 for straight
// swivel, which leads to NaN YZ angle. It appears that for SwivelThrustVector()
// c_theta_mid_eps needs to be ~< 1e-7, so la.f_eps = 1e-10 is too small.
c_theta_mid_eps = 1e-7;

// The swivel assembly is constructed with InputTube() long side markers up,
// so with phi_input_yz = 90. However the zero position for the long side 
// markers is better defined at phi_input_yz = 0. Therefore use
// c_phi_input_yz_construction to compensate for this construction offset.
c_phi_input_yz_construction = 90;
