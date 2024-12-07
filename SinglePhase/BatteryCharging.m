function [problem,guess] = BatteryCharging(BattData,batt_idx)
%DoubleIntergratorTracking - Double Integrator Tracking Problem
%
% Syntax:  [problem,guess] = DoubleIntergratorTracking
%
% Outputs:
%    problem - Structure with information on the optimal control problem
%    guess   - Guess for state, control and multipliers.
%
% Other m-files required: none
% MAT-files required: none
%
% Copyright (C) 2019 Yuanbo Nie, Omar Faqir, and Eric Kerrigan. All Rights Reserved.
% The contribution of Paola Falugi, Eric Kerrigan and Eugene van Wyk for the work on ICLOCS Version 1 (2010) is kindly acknowledged.
% This code is published under the MIT License.
% Department of Aeronautics and Department of Electrical and Electronic Engineering,
% Imperial College London London  England, UK 
% ICLOCS (Imperial College London Optimal Control) Version 2.5 
% 1 Aug 2019
% iclocs@imperial.ac.uk

%------------- BEGIN CODE --------------
Q = BattData(batt_idx).Q; 
R0 = BattData(batt_idx).R0; 
R1 = BattData(batt_idx).R1; 
C1 = BattData(batt_idx).C1; 
batt_m = BattData(batt_idx).mass / 1000; 
ocv_points = table2array(BattData.OCV_points);


InternalDynamics=@BatteryCharging_Dynamics_Internal;
SimDynamics=@BatteryCharging_Dynamics_Sim;




% Analytic derivative files (optional)
problem.analyticDeriv.gradCost=@gradCost_final;
problem.analyticDeriv.hessianLagrangian=@hessianLagrangian_final;
problem.analyticDeriv.jacConst=@jacConst_final;

% Settings file
problem.settings=@settings_BatteryCharging;


%Initial Time. t0<tf
problem.time.t0_min=0;
problem.time.t0_max=0;
guess.t0=0;
Vmin=ocv_points(1,2); % setting Vmin to the OCV at zero SOC
Vmax=4.7;
TempMax=30;
TempMin=10;
TempAmb=15;

% Final time. Let tf_min=tf_max if tf is fixed.
problem.time.tf_min=0;     
problem.time.tf_max=60*60*60; 
guess.tf=10*60;

% Parameters bounds. pl=< p <=pu
problem.parameters.pl=[];
problem.parameters.pu=[];
guess.parameters=[];

% Initial conditions for system.
problem.states.x0=[];

% Initial conditions for system. Bounds if x0 is free s.t. x0l=< x0 <=x0u
problem.states.x0l=[0 0 TempAmb]; 
problem.states.x0u=[0 0.25 TempAmb]; 

% State bounds. xl=< x <=xu
problem.states.xl=[0 0 TempMin];
problem.states.xu=[1 0.25 TempMax];

% State error bounds
problem.states.xErrorTol_local=[1e-6 1e-6 1e-6];
problem.states.xErrorTol_integral=[1e-6 1e-6 1e-6];


% State constraint error bounds
problem.states.xConstraintTol=[1e-4 1e-4 1e-4];

% Terminal state bounds. xfl=< xf <=xfu
problem.states.xfl=[1 0 TempMin];
problem.states.xfu=[1 0.25 TempMax];

% Guess the state trajectories with [x0 xf]
guess.states(:,1)=[0 1];
guess.states(:,2)=[0.25 0.25];
guess.states(:,3)=[TempAmb TempMax];


% Number of control actions N 
% Set problem.inputs.N=0 if N is equal to the number of integration steps.  
% Note that the number of integration steps defined in settings.m has to be divisible 
% by the  number of control actions N whenever it is not zero.
problem.inputs.N=0;       
      
% Input bounds
problem.inputs.ul=0;
problem.inputs.uu=10*15;

% Bounds on the first control action
problem.inputs.u0l=0;
problem.inputs.u0u=10*15;

% Input constraint error bounds
problem.inputs.uConstraintTol=[0.1];

% Guess the input sequences with [u0 uf]
guess.inputs(:,1)=[10*15 0];



% Choose the set-points if required
problem.setpoints.states=[];
problem.setpoints.inputs=[];

% Bounds for path constraint function gl =< g(x,u,p,t) =< gu
problem.constraints.ng_eq=0;
problem.constraints.gTol_eq=[];

problem.constraints.gl=[Vmin];
problem.constraints.gu=[Vmax];
problem.constraints.gTol_neq=[0.01];

% Bounds for boundary constraints bl =< b(x0,xf,u0,uf,p,t0,tf) =< bu
problem.constraints.bl=[];
problem.constraints.bu=[];
problem.constraints.bTol=[];

% Store auxilary data
problem.data.Q=Q;
problem.data.R0=R0;
problem.data.R1=R1;
problem.data.C1=C1; 
problem.data.batt_m=batt_m;
problem.data.batt_Cp=2025.737;
problem.data.batt_h=43.061;
problem.data.batt_A=3.714e-03;
problem.data.TempAmb=TempAmb;
problem.data.OCV_coeff = BattData(batt_idx).OCV_coeff;
problem.data.Poly = BattData(batt_idx).OCVPieceWiseFit.Poly{1,1};
problem.data.PolyDerivative = BattData(batt_idx).OCVPieceWiseFit.PolyDerivative{1,1};
problem.data.PolySecondDerivative = BattData(batt_idx).OCVPieceWiseFit.PolySecondDerivative;

% Get function handles and return to Main.m
problem.data.InternalDynamics=InternalDynamics;
problem.data.functionfg=@fg;
problem.data.plantmodel = func2str(InternalDynamics);
problem.functions={@L,@E,@f,@g,@avrc,@b};
problem.sim.functions=SimDynamics;
problem.sim.inputX=[];
problem.sim.inputU=1:length(problem.inputs.ul);
problem.functions_unscaled={@L_unscaled,@E_unscaled,@f_unscaled,@g_unscaled,@avrc,@b_unscaled};
problem.data.functions_unscaled=problem.functions_unscaled;
problem.data.ng_eq=problem.constraints.ng_eq;
problem.constraintErrorTol=[problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.constraints.gTol_eq,problem.constraints.gTol_neq,problem.states.xConstraintTol,problem.states.xConstraintTol,problem.inputs.uConstraintTol,problem.inputs.uConstraintTol];

%------------- END OF CODE --------------

function stageCost=L_unscaled(x,xr,u,ur,p,t,vdat)

% L_unscaled - Returns the stage cost.
% The function must be vectorized and
% xi, ui are column vectors taken as x(:,i) and u(:,i) (i denotes the i-th
% variable)
% 
% Syntax:  stageCost = L(x,xr,u,ur,p,t,data)
%
% Inputs:
%    x  - state vector
%    xr - state reference
%    u  - input
%    ur - input reference
%    p  - parameter
%    t  - time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    stageCost - Scalar or vectorized stage cost
%
%  Remark: If the stagecost does not depend on variables it is necessary to multiply
%          the assigned value by t in order to have right vector dimesion when called for the optimization. 
%          Example: stageCost = 0*t;

%------------- BEGIN CODE --------------



stageCost = 0*t;

%------------- END OF CODE --------------


function boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 

% E_unscaled - Returns the boundary value cost
%
% Syntax:  boundaryCost=E_unscaled(x0,xf,u0,uf,p,t0,tf,data) 
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
% Output:
%    boundaryCost - Scalar boundary cost
%
%------------- BEGIN CODE --------------

boundaryCost=tf;

%------------- END OF CODE --------------


function bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)

% b_unscaled - Returns a column vector containing the evaluation of the boundary constraints: bl =< bf(x0,xf,u0,uf,p,t0,tf) =< bu
%
% Syntax:  bc=b_unscaled(x0,xf,u0,uf,p,t0,tf,vdat,varargin)
%
% Inputs:
%    x0  - state at t=0
%    xf  - state at t=tf
%    u0  - input at t=0
%    uf  - input at t=tf
%    p   - parameter
%    tf  - final time
%    data- structured variable containing the values of additional data used inside
%          the function
%
%          
% Output:
%    bc - column vector containing the evaluation of the boundary function 
%
%------------- BEGIN CODE --------------
varargin=varargin{1};
bc=[];
%------------- END OF CODE --------------
% When adpative time interval add constraint on time
%------------- BEGIN CODE --------------
if length(varargin)==2
    options=varargin{1};
    t_segment=varargin{2};
    if ((strcmp(options.discretization,'hpLGR')) || (strcmp(options.discretization,'globalLGR')))  && options.adaptseg==1 
        if size(t_segment,1)>size(t_segment,2)
            bc=[bc;diff(t_segment)];
        else
            bc=[bc,diff(t_segment)];
        end
    end
end

%------------- END OF CODE --------------
