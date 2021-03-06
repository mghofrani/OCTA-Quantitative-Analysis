function [Case,cut1,Threshold,Region1,Region2,Region3,Region4,minRadius,OverallDens,estRadius,formfactor,roundness,extent,convexity,lsratio,solidity,areaCORRECT,irreg1,irreg2,scaleCORRECT,state,side,rectReg1,rectReg2,rectReg3,rectReg4,innerDens1,innerDens2,innerDens3,innerDens4,OtsuThresh] = importfile(workbookFile,sheetName,startRow,endRow)
%IMPORTFILE1 Import data from a spreadsheet
%   [Case,cut1,Threshold,Region1,Region2,Region3,Region4,minRadius,OverallDens,estRadius,formfactor,roundness,extent,convexity,lsratio,solidity,areaCORRECT,irreg1,irreg2,scaleCORRECT,state,side,rectReg1,rectReg2,rectReg3,rectReg4,innerDens1,innerDens2,innerDens3,innerDens4,OtsuThresh]
%   = IMPORTFILE1(FILE) reads data from the first worksheet in the
%   Microsoft Excel spreadsheet file named FILE and returns the data as
%   column vectors.
%
%   [Case,cut1,Threshold,Region1,Region2,Region3,Region4,minRadius,OverallDens,estRadius,formfactor,roundness,extent,convexity,lsratio,solidity,areaCORRECT,irreg1,irreg2,scaleCORRECT,state,side,rectReg1,rectReg2,rectReg3,rectReg4,innerDens1,innerDens2,innerDens3,innerDens4,OtsuThresh]
%   = IMPORTFILE1(FILE,SHEET) reads from the specified worksheet.
%
%   [Case,cut1,Threshold,Region1,Region2,Region3,Region4,minRadius,OverallDens,estRadius,formfactor,roundness,extent,convexity,lsratio,solidity,areaCORRECT,irreg1,irreg2,scaleCORRECT,state,side,rectReg1,rectReg2,rectReg3,rectReg4,innerDens1,innerDens2,innerDens3,innerDens4,OtsuThresh]
%   = IMPORTFILE1(FILE,SHEET,STARTROW,ENDROW) reads from the specified
%   worksheet for the specified row interval(s). Specify STARTROW and
%   ENDROW as a pair of scalars or vectors of matching size for
%   dis-contiguous row intervals. To read to the end of the file specify an
%   ENDROW of inf.
%
%	Non-numeric cells are replaced with: NaN
%
% Example:
%   [Case,cut1,Threshold,Region1,Region2,Region3,Region4,minRadius,OverallDens,estRadius,formfactor,roundness,extent,convexity,lsratio,solidity,areaCORRECT,irreg1,irreg2,scaleCORRECT,state,side,rectReg1,rectReg2,rectReg3,rectReg4,innerDens1,innerDens2,innerDens3,innerDens4,OtsuThresh] = importfile1('Deep.xlsx','Sheet1',2,75);
%
%   See also XLSREAD.

% Auto-generated by MATLAB on 2018/12/18 23:00:05

%% Input handling

% If no sheet is specified, read first sheet
if nargin == 1 || isempty(sheetName)
    sheetName = 1;
end

% If row start and end points are not specified, define defaults
if nargin <= 3
    startRow = 2;
    endRow = 75;
end

%% Import the data
[~, ~, raw] = xlsread(workbookFile, sheetName, sprintf('A%d:AE%d',startRow(1),endRow(1)));
for block=2:length(startRow)
    [~, ~, tmpRawBlock] = xlsread(workbookFile, sheetName, sprintf('A%d:AE%d',startRow(block),endRow(block)));
    raw = [raw;tmpRawBlock]; %#ok<AGROW>
end
raw(cellfun(@(x) ~isempty(x) && isnumeric(x) && isnan(x),raw)) = {''};
cellVectors = raw(:,[1,2]);
raw = raw(:,[3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31]);

%% Replace non-numeric cells with NaN
R = cellfun(@(x) ~isnumeric(x) && ~islogical(x),raw); % Find non-numeric cells
raw(R) = {NaN}; % Replace non-numeric cells

%% Create output variable
I = cellfun(@(x) ischar(x), raw);
raw(I) = {NaN};
data = reshape([raw{:}],size(raw));

%% Allocate imported array to column variable names
Case = cellVectors(:,1);
cut1 = cellVectors(:,2);
Threshold = data(:,1);
Region1 = data(:,2);
Region2 = data(:,3);
Region3 = data(:,4);
Region4 = data(:,5);
minRadius = data(:,6);
OverallDens = data(:,7);
estRadius = data(:,8);
formfactor = data(:,9);
roundness = data(:,10);
extent = data(:,11);
convexity = data(:,12);
lsratio = data(:,13);
solidity = data(:,14);
areaCORRECT = data(:,15);
irreg1 = data(:,16);
irreg2 = data(:,17);
scaleCORRECT = data(:,18);
state = data(:,19);
side = data(:,20);
rectReg1 = data(:,21);
rectReg2 = data(:,22);
rectReg3 = data(:,23);
rectReg4 = data(:,24);
innerDens1 = data(:,25);
innerDens2 = data(:,26);
innerDens3 = data(:,27);
innerDens4 = data(:,28);
OtsuThresh = data(:,29);

