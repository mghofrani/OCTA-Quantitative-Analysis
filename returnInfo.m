function [scale, state , side ] = returnInfo(caseName,Names,DiabetesHealthy,scaleOS,scaleOD)
idx = find(ismember(strtrim(Names),caseName));
if ~isempty(strfind(caseName, 'os'))
    side = 'OS';
    scale = scaleOS(idx);
else
    side = 'OD';
    scale = scaleOD(idx);
end
state = DiabetesHealthy(idx);
end