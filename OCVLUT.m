function [VOC] = OCVLUT(problem,SOC)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
VOC=problem.data.p4+problem.data.p3*SOC+problem.data.p2*SOC.^2+problem.data.p1*SOC.^3;
end