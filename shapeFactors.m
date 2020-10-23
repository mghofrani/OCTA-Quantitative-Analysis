function [form_factor,roundness,extent,convexity,ls_ratio, Solidity , area2] = shapeFactors(maskInitial)
bw = double(maskInitial{end});
%% Area
area2 = regionprops(bw==1, 'area');
area2 = cat(1, area2.Area);
%% Perimeter 
perimeter = regionprops(bw, 'Perimeter');
perimeter = perimeter.Perimeter;
%% Form Factor
form_factor=4*pi*area2/(perimeter)^2;
%% Solidity         
%Solidity=area/convexarea;
Solidity = regionprops(bw==1, 'Solidity');
Solidity = Solidity.Solidity ;
%% Roundness  Roundness=4area/pi*maxdiameter^2
max_diameter = regionprops(bw==1, 'MajorAxisLength');
max_diameter = cat(1, max_diameter.MajorAxisLength);
roundness =(4*area2)/(pi*(max_diameter)^2);
%% Extend  Etend=area/ boundingrectangle
extent = regionprops(bw==1, 'Extent');
extent = cat(1, extent.Extent);
% bounding_rectangle =bounding_rectangle(1,3)*bounding_rectangle(1,4);
% extent = area2/(bounding_rectangle)
%% convexity   convexity=convexprimeter/primeter
% [y,x] = find(bw);
% cf=convhull(x,y);
% convex_perimeter=length(cf);
convexImage = regionprops(bw == 1, 'ConvexImage');
convex_perimeter = regionprops(convexImage.ConvexImage , 'Perimeter');
convexity = convex_perimeter.Perimeter/perimeter;
%% L:S ratio   long axis/short axis
min_diameter = regionprops(bw==1, 'MinorAxisLength');
min_diameter = cat(1, min_diameter. MinorAxisLength);
ls_ratio = max_diameter/ min_diameter;
end