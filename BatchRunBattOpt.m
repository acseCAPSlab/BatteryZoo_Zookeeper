clear all; close all; clc;

addpath('SinglePhase\')
load("SinglePhase/data/BattDataWithDerivatives_d_dU.mat")


% Scenario Settings
Vmax=4.7;
TempMax=30;
TempMin=10;
TempAmb=15;

% Select Battery Cell from Data
indexlist=[1,3,4,5,6];

maxtf=0;
for k=1:length(indexlist)
    i=indexlist(k);

    [problem,guess]=BatteryCharging(BattData,i);          % Fetch the problem definition
    options= problem.settings(100);                  % Get options and solver settings 
    [solution,MRHistory]=solveMyProblem( problem,guess,options);
    BattData(i).FastChargeSolution.Time=solution.T;
    BattData(i).FastChargeSolution.SOC=solution.X(:,1);
    BattData(i).FastChargeSolution.VRC=solution.X(:,2);
    BattData(i).FastChargeSolution.TempBatt=solution.X(:,3);
    BattData(i).FastChargeSolution.I=solution.U(:,1);
    BattData(i).FastChargeSolution.V=ppval(problem.data.Poly, BattData(i).FastChargeSolution.SOC) +BattData(i).FastChargeSolution.VRC +problem.data.R0*BattData(i).FastChargeSolution.I;
    maxtf=max(maxtf,solution.tf);

end
  
%%

figure(1)
t = tiledlayout('flow','TileSpacing','compact');

nexttile
hold on
for k=1:length(indexlist)
    i=indexlist(k);
    plot(BattData(i).FastChargeSolution.Time,BattData(i).FastChargeSolution.SOC ,'LineWidth',2)
end
plot([solution.T(1,1); maxtf],[problem.states.xl(1), problem.states.xl(1)],'r-' )
plot([solution.T(1,1); maxtf],[problem.states.xu(1), problem.states.xu(1)],'r-' )
xlabel('Time [s]','FontSize',12)
ylabel('States: State-of-Charge','FontSize',12)
xlim([0 maxtf])
grid on

nexttile
hold on
for k=1:length(indexlist)
    i=indexlist(k);
    plot(BattData(i).FastChargeSolution.Time,BattData(i).FastChargeSolution.VRC ,'LineWidth',2)
end
plot([solution.T(1,1); maxtf],[problem.states.xl(2), problem.states.xl(2)],'r-' )
plot([solution.T(1,1); maxtf],[problem.states.xu(2), problem.states.xu(2)],'r-' )
xlim([0 maxtf])
xlabel('Time [s]','FontSize',12)
ylabel('States: RC Voltage [V]','FontSize',12)
grid on

nexttile
hold on
for k=1:length(indexlist)
    i=indexlist(k);
    plot(BattData(i).FastChargeSolution.Time,BattData(i).FastChargeSolution.TempBatt ,'LineWidth',2)
end

plot([solution.T(1,1); maxtf],[problem.states.xl(3), problem.states.xl(3)],'r-' )
plot([solution.T(1,1); maxtf],[problem.states.xu(3), problem.states.xu(3)],'r-' )
xlim([0 maxtf])
xlabel('Time [s]','FontSize',12)
ylabel('States: Battery Temperature [C]','FontSize',12)
grid on


nexttile
hold on
for k=1:length(indexlist)
    i=indexlist(k);
    plot(BattData(i).FastChargeSolution.Time,BattData(i).FastChargeSolution.I ,'LineWidth',2)
end
plot([solution.T(1,1); maxtf],[problem.inputs.ul, problem.inputs.ul],'r-' )
xlim([0 maxtf])
xlabel('Time [s]','FontSize',12)
grid on
ylabel('Control Input: Current [A]','FontSize',12)

nexttile
hold on
for k=1:length(indexlist)
    i=indexlist(k);
    plot(BattData(i).FastChargeSolution.Time,BattData(i).FastChargeSolution.V ,'LineWidth',2)
end
plot([solution.T(1,1); maxtf],[Vmax, Vmax],'r-' )
xlim([0 maxtf])
xlabel('Time [s]','FontSize',12)
ylabel('Output: voltage [V]','FontSize',12)
grid on


lgd = legend(BattData(indexlist).model,'FontSize',12);
lgd.Layout.Tile = 6;