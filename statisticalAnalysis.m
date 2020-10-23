%%
clc
close all
workbookFile = 'Deep.xlsx';
[Case,cut1,Threshold,Region1,Region2,Region3,Region4,minRadius,OverallDens,estRadius,formfactor,roundness,extent,convexity,...
    lsratio,solidity,area,irreg1,irreg2,scale,state,side,rectReg1,rectReg2,rectReg3,rectReg4,...
    innerDens1,innerDens2,innerDens3,innerDens4,OtsuThresh] = importfile(workbookFile);
variables = whos
[group] = findgroups(state);
%%
figure
boxplot(scale, group )
%%
figure
boxplot((Region1), group )
%%
figure
boxplot((Region2), group )
%%
figure
boxplot((Region3), group )
%%
figure
boxplot((Region4), group )
%%
figure
boxplot((estRadius), group )
%%
figure
boxplot((minRadius), group )
%%
figure
boxplot((OverallDens), group )
%%
figure
boxplot((roundness), group )
%%
figure
boxplot((solidity), group )
%%
figure
boxplot((area), group )
%%
figure
boxplot((convexity), group )
%%
figure
boxplot((formfactor), group )
%%
figure
boxplot((extent), group )
%%
figure
boxplot((irreg1), group )
%%
figure
boxplot((irreg2), group )
%%
figure
boxplot((lsratio), group )
%%
figure
boxplot((innerDens1), group )
%%
figure
boxplot((innerDens2), group )
%%
figure
boxplot((innerDens3), group )
%%
figure
boxplot((innerDens4), group )
%%
figure
boxplot((OtsuThresh), group )
%%
figure
boxplot((rectReg1), group )
%%
figure
boxplot((rectReg2), group )
%%
figure
boxplot((rectReg3), group )
%%
figure
boxplot((rectReg4), group )

