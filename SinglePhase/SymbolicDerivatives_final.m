
% Cart Pole Swing-up Problem Dynamics - Simulation (Exact Model)
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

clear all; close all;clc
syms x1 x2 x3 u1 Q R0 R1 C1 batt_m batt_Cp batt_h batt_A TempAmb p1 p2 p3 p4
  
vdat.Q=Q;
vdat.R0=R0
vdat.R1=R1;
vdat.C1=C1;
vdat.batt_m=batt_m;
vdat.batt_Cp=batt_Cp;
vdat.batt_h=batt_h;
vdat.batt_A=batt_A;
vdat.TempAmb=TempAmb;
% vdat.OCVLUT(SOC)=OCVLUT(SOC);
vdat.p1=p1;
vdat.p2=p2;
vdat.p3=p3;
vdat.p4=p4;
f(:,1) = u1./vdat.Q;

f(:,2) = -x2./(vdat.R1*vdat.C1)+u1./vdat.C1;

f(:,3) = 1./(vdat.batt_m*vdat.batt_Cp).*(u1.^2*vdat.R0-vdat.batt_h*vdat.batt_A*(x3-vdat.TempAmb));

g_neq = vdat.p4+vdat.p3.*x1+vdat.p2.*(x1.^2)+vdat.p1.*(x1.^3) + x2 + vdat.R0 * u1;

% voltage=3.64+0.55.*x1-0.72.*(x1.^2)+0.75.*(x1.^3)+x2+vdat.R0.*u1;
% power=-voltage.*u1;
% costL=-x1^2;
costL=0;

x=[x1 x2 x3];
u=[u1];
z=[x u];

Jac_x=jacobian(f,x)
Jac_u=jacobian(f,u)
Jac=jacobian(f,z)
Jacg=jacobian(g_neq,z)
Jac_gx=jacobian(g_neq,x)
Jac_gu=jacobian(g_neq,u)

grad_g_neq = jacobian(g_neq, z)
Hes_x1=jacobian(Jac(:,1),z(1:end))
Hes_x2=jacobian(Jac(:,2),z(2:end))
Hes_x3=jacobian(Jac(:,3),z(3:end))
Hes_u1=jacobian(Jac(:,4),z(4:end))
Hesg_x1=jacobian(Jacg(:,1),z(1:end))
Hesg_x2=jacobian(Jacg(:,2),z(2:end))
Hesg_x3=jacobian(Jacg(:,3),z(3:end))
Hesg_u1=jacobian(Jacg(:,4),z(4:end))
gradCost=jacobian(costL,z)

%------------- END OF CODE --------------