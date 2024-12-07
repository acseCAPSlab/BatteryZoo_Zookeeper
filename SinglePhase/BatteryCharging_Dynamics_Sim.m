function dx = BatteryCharging_Dynamics_Sim(x,u,p,t,vdat)
%Double Integrator Dynamics for Simulation
%
% Syntax:  
%          [dx] = Dynamics(x,u,p,t,vdat)
% 
% Inputs:
%    x  - state vector
%    u  - input
%    p  - parameter
%    t  - time
%    vdat - structured variable containing the values of additional data used inside
%          the function%      
% Output:
%    dx - time derivative of x
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk
%
%------------- BEGIN CODE --------------

SOC = x(:,1);V_OC = x(:,2);Temp_batt=x(:,3);current_bat = u(:,1); 

dx(:,1) = current_bat./vdat.Q;

dx(:,2) = -V_OC./(vdat.R1*vdat.C1)+current_bat./vdat.C1;

dx(:,3) = 1./(vdat.batt_m*vdat.batt_Cp).*(current_bat.^2*vdat.R0-vdat.batt_h*vdat.batt_A*(Temp_batt-vdat.TempAmb));
%------------- END OF CODE --------------