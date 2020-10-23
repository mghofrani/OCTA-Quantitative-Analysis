function vascDensGUI()
% Revision, 28 Mehr 96
% Yalda Amirmoezzi Jahromi, Mohsen Ghofrani Jahromi
clc
close all
format short
Image_Initial=[]; Image_Raw = []; Image_Processed=[]; Image_Thresh=[];
y_center=[]; x_center= []; radiusMacula=[]; radiusMacula2 =[];
Lcorner = []; % left corner, default value for Superficial analysis
Tcorner = []; Width = []; Height = Width;
scale = []; state = [];  side = []; OtsuThresh = [];
t = linspace(0,2*pi);
h1=[];h2=[];h3=[];h4=[];h5=[];h6=[]; h7=[];h8=[]; imageHandleP=[]; hOVM1=[]; hOVM2=[]; maskInitial={};
hCross = [];
mkdir('results\images')
[Names,DiabetesHealthy,scaleOS,scaleOD] = getPatientsData('patientsInfo.xlsx');
nameExt = clock;
nameExt([1, end]) = [];
fileID = fopen([pwd '\results\' mat2str(nameExt) '.csv'],'w');
fprintf(fileID,'%10s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%8s, %8s,%8s,%8s,%8s, %8s  \n',...
    'Case', 'Threshold' , 'Region1', 'Region2', 'Region3', 'Region4', 'min Radius', 'Overall Dens', 'est Radius',...
    'form factor','roundness','extent','convexity','ls ratio','solidity', 'area',...
    'irreg1','irreg2','cut','scale' , 'state' , 'side', 'rectReg1', 'rectReg2', 'rectReg3','rectReg4',...
    'innerDens1', 'innerDens2', 'innerDens3','innerDens4','Otsu Thresh');

%% Initializations
fh1=figure('Units','Normalized');
maxfig(fh1,1)

uicontrol(fh1,'Style','pushbutton','String','Open Image','Units','normalized',...
    'Position',[.01 .04 .07 .03],'Callback',@importImage);

uicontrol(fh1,'Style','pushbutton','String','Superficial','Units','normalized',...
    'Tag','Superficial','Position',[.1 .04 .07 .03],'Callback',@analyze);

uicontrol(fh1,'Style','pushbutton','String','Deep','Units','normalized',...
    'Tag','Deep','Position',[.19 .04 .07 .03],'Callback',@analyze);

uicontrol(fh1,'Style','pushbutton','String','Crop','Units','normalized',...
    'Position',[.19 .068 .07 .03],'Callback',@Crop);

uicontrol(fh1,'Style','pushbutton','String','Modify Center','Units','normalized',...
    'Position',[.31 .04 .08 .03],'Callback',@modifyCenter);

uicontrol(fh1,'Style','pushbutton','String','Realign ','Units','normalized',...
    'Position',[.41 .04 .08 .03],'Callback',@realignCircles);

uicontrol(fh1,'Style','pushbutton','String','Grow Region','Units','normalized',...
    'Position',[.51 .04 .08 .03],'Callback',@growRegion);

uicontrol(fh1,'Style','pushbutton','String','Shrink Region','Units','normalized',...
    'Position',[.61 .04 .08 .03],'Callback',@removeRegion);

growAnnotation = uicontrol(fh1,'Style','Text','FontWeight','bold',...
    'Units','normalized','Position',[.42 .01 .16 .03]);

cutAnnotation = uicontrol(fh1,'Style','Text','FontWeight','bold',...
    'String','Superficial','Units','normalized','Position',[.01 .01 .08 .03]);

uicontrol(fh1,'Style','pushbutton','String','Reset Values','Units','normalized',...
    'Position',[.75 .04 .08 .03],'Callback',@resetValues);

uicontrol(fh1,'Style','pushbutton','String','Magnify','Units','normalized',...
    'Position',[.85 .04 .08 .03],'Callback',@magnify);

uicontrol(fh1,'Style','pushbutton','String','Save Results','Units','normalized',...
    'Position',[.01 .068 .07 .03],'Callback',@saveResults);

uicontrol(fh1,'Style','pushbutton','String','Show Results','Units','normalized',...
    'Position',[.1 .068 .07 .03],'Callback',@showResults);

axInitial= axes('Units','normalized','Position', [.01 .1 0.25 0.354],'xtick',[],'ytick',[],'box','on');
axProcessed = axes('Units','normalized','Position', [.01 .46 0.25 0.49],'xtick',[],'ytick',[],'box','on');
axMain= axes('Units','normalized','Position', [.3 .1 0.4 0.85],'xtick',[],'ytick',[],'box','on');
axCut= axes('Units','normalized','Position', [0.75 0.59 .2 .36],'xtick',[],'ytick',[],'box','on');

sliderThresh= javax.swing.JSlider(0,256);
set(sliderThresh, 'Value',5, 'MajorTickSpacing',40, 'PaintLabels',true);
[~, container] = javacomponent(sliderThresh);
set(container,'Units', 'normalized', 'Position', [0.75 0.46 .2 .1]);
set(sliderThresh, 'StateChangedCallback', @applyThresh);  %alternative: MouseReleasedCallback

thresholdAnnotation= uicontrol(fh1,'Style','Text','String',['Threshold: ' num2str(get(sliderThresh,'Value'))],...
    'Units','normalized','Position',[0.8 0.56 .16 .02]);

uicontrol(fh1,'Style','pushbutton','String','Manual','Units','normalized',...
    'Position',[.75 .56 .08 .03],'Callback',@manualBoundary);

sliderRadius= com.jidesoft.swing.RangeSlider(0,256,0,256);  % min,max,low,high
set(sliderRadius,'MajorTickSpacing',40, 'PaintLabels',true);
[~, container] = javacomponent(sliderRadius);
set(container,'Units', 'normalized', 'Position', [0.75 0.36 .2 .1]);
set(sliderRadius, 'StateChangedCallback', @drawCircles);  %alternative

radiAnnotation= uicontrol(fh1,'Style','Text','String',{['Lower Radius: ' num2str(getLowValue(sliderRadius))],...
    ['Higher Radius: ' num2str(getHighValue(sliderRadius))]},...
    'Units','normalized','Position',[0.75 0.31 .2 .05]);

radiiTable = uitable('Parent',fh1,'Units','normalized','Position', [.75 .2 .19 .1], ...
    'data',{'Minimum Macula Radius (pixels)';'Overall Density';'Estimated Macula Radius'},...
    'BackgroundColor',[ 0.92 0.76 0.55; .65 .92 .71],'RowName',[]...
    ,'ColumnName',[],'FontSize',10,'ColumnWidth',{200,56});

densitiesTable = uitable('Parent',fh1,'Units','normalized','Position', [.75 .1 .19 .07], ...
    'Data',{'R1','R2';'R4','R3'},'BackgroundColor',[ 0.92 0.76 0.55; .65 .92 .71],...
    'RowName',[],'ColumnName',[],'FontSize',10, 'ColumnWidth',{128 128});

%% Running the Algorithm
importImage();

%% Functions Defenitions
    function importImage(~,~)
        h1=[];h2=[];h3=[];h4=[];h5=[];h7=[]; imageHandleP=[]; hOVM1=[]; hOVM2=[]; maskInitial={};
        hCross = [];
        Lcorner=128;; % left corner, default value for Deep analysis
        Tcorner = 896;
        Width = 512;
        Height = Width;
        [FileName,PathName] = uigetfile('*.*',...61
            'Select a File ') % get the main image
        enntry = strfind(PathName,'\');    
        enntry = PathName(enntry(end-1)+1:enntry(end)-1)       
        [scale, state , side] = returnInfo(enntry,Names,DiabetesHealthy,scaleOS,scaleOD)        
        set(fh1,'Name',FileName)
        Image_Initial=imread([PathName FileName]);
        Image_Initial=rgb2gray(Image_Initial);
%         Image_Initial=uint8(((double(Image_Initial)).^2)/(2^16-1)*(2^8-1));
        Image_Initial=uint8(double(Image_Initial));
        ProcessImage()
    end

    function ProcessImage()       
        set(fh1, 'currentaxes', axInitial);
        cla
        imshow(Image_Initial);
        hold on
        rectangle('Position',[Lcorner, Tcorner,Width,Height], 'EdgeColor','b','LineWidth',3,'LineStyle',':')
        drawnow
        Image_Raw = Image_Initial(Tcorner : Tcorner+Height , Lcorner : Lcorner+Width);
        set(fh1, 'currentaxes', axCut);
        imshow(Image_Raw)
        %imwrite(Image_Raw, 'cropped.jpg')
        [m,n]=size(Image_Raw);
        y_center=round(m/2);
        x_center=round(n/2);
        setHighValue(sliderRadius, round(1e3/scale))
        setLowValue(sliderRadius, round(5e2/scale))
        % Motion Artifact Removal
        %         hdint = vision.Deinterlacer;
        %         y = step(hdint,Image_Raw);
        %  Wiener Filtering
        %  Image_wiener = wiener2(y,[1 1]);
        %         Image_Processed = imsharpen(ordfilt2(y,2,[0 1 0; 1 1 1; 0 1 0]));
        %         Image_Processed = ordfilt2(y,2,[0 1 0; 1 1 1; 0 1 0]);
         
        bw = Image_Raw;
        d = 1:1:180;
        bw2 = zeros(size(bw,1),size(bw,2),15*numel(d));
        for i = 1 : numel(d)
            se = strel('line',3,d(i));
            bw2(:,:,i) = imopen(bw,se);
        end
        bw2 = imsharpen(mean(bw2,3));
        bw2 = 255 * bw2/max(bw2(:));
        Image_Processed = uint8(bw2);
        OtsuThresh = round(255*graythresh(Image_Raw));
        set(fh1, 'currentaxes', axProcessed);
        cla
        imshow(Image_Processed,[]);
        title('Processed Image')                
        
        applyThresh();        
        findMinimumRadi()
    end

    function applyThresh(~,~)
        thresh = get(sliderThresh,'Value');
        
        set(thresholdAnnotation, 'String',['Threshold: ' num2str(thresh) ' Otsu:' num2str(OtsuThresh) ])
        Image_Thresh=zeros(size(Image_Raw));
        Image_Thresh(Image_Processed>=thresh)=1;
%         Image_Thresh(Image_Raw>=thresh)=1;
        
        set(fh1, 'currentaxes', axMain);
        cla
        imageHandleP = imshow(Image_Thresh);
        y = ylim;
        x = xlim;
        hCross(1) = line([x_center, x_center],[y(1) y(2)]);
        set(hCross(1), 'LineWidth' , 2)
        hCross(2) = line([x(1), x(2)],[y_center y_center]);
        set(hCross(2), 'LineWidth' , 2)%, 'Color','r')
        
        text(x(1)+10,y(2)-15,'R4','Color','w','BackGroundColor','b')
        text(x(2)-15,y(2)-15,'R3','Color','w','BackGroundColor','b')
        text(x(2)-15,y(1)+10,'R2','Color','b','BackGroundColor','w')
        text(x(1)+10,y(1)+10,'R1','Color','b','BackGroundColor','w')
        drawnow
        drawCircles();
        %
        % imshow(uint8(not(Image_Thresh)).*Image_Processed)
        % title('Removed Pixels')                
    end

    function drawCircles(~,~)       
        set(radiAnnotation,'String',{['Lower Radius: ' num2str(1e-3*scale*getLowValue(sliderRadius))],...
            ['Higher Radius: ' num2str(1e-3*scale*getHighValue(sliderRadius))]})
        set(fh1, 'currentaxes', axMain);
        delete([h1,h2,h3,h4,h5])
        hold on
        h1 = plot(x_center+getLowValue(sliderRadius)*cos(t),...
            y_center+getLowValue(sliderRadius)*sin(t),'LineWidth',2, 'Color',[.929 .694 .125]);
        h2 = plot(x_center+getHighValue(sliderRadius)*cos(t),...
            y_center+getHighValue(sliderRadius)*sin(t),'LineWidth',2,'Color',[.494 .184 .556]);
        set(fh1, 'currentaxes', axProcessed);
        hold on
        h3 = plot(x_center,y_center, '+' , 'MarkerSize', 10, 'color', [.85 .325 .098]);
        h4 = plot(x_center+getLowValue(sliderRadius)*cos(t),...
            y_center+getLowValue(sliderRadius)*sin(t),'LineWidth',1, 'Color',[.929 .694 .125]);
        h5 = plot(x_center+getHighValue(sliderRadius)*cos(t),...
            y_center+getHighValue(sliderRadius)*sin(t),'LineWidth',2,'Color',[.494 .184 .556]);
        
        applyCircularMask();
    end

    function findMinimumRadi()
        whitePixels=find(Image_Thresh);
        [indY, indX] = ind2sub(size(Image_Thresh),whitePixels);
        pixelsCoordinates=[indY, indX];
        N=length(whitePixels);
        distances=sqrt(sum(((repmat([y_center x_center],N,1)-pixelsCoordinates).^2),2));
        [radiusMacula, nearestPoint] = min(distances);
        [height,width] = size(Image_Raw);
        [W,H] = meshgrid(1:width,1:height);
        maskInitial = {};
        maskInitial {1} = (W-x_center).^2 + (H-y_center).^2 - radiusMacula^2<1 ;
        [nearestPointY, nearestPointX] = ind2sub(size(Image_Thresh),whitePixels(nearestPoint));
        set(fh1, 'currentaxes', axProcessed);
        delete([h6,h7,h8])
        h8 = plot(nearestPointX,nearestPointY, 's' , 'MarkerSize', 5, ...
            'MarkerFaceColor', [0 .447 .741],'MarkerEdgeColor',[.301 .745 .933]);
        set(fh1, 'currentaxes', axProcessed);
        h6=plot(x_center+radiusMacula*cos(t),y_center+radiusMacula*sin(t),...
            'LineStyle','--','LineWidth',2,'Color',[.466 .674 .188]);
        set(fh1, 'currentaxes', axMain);
        h7=plot(x_center+radiusMacula*cos(t),y_center+radiusMacula*sin(t),...
            'LineStyle','--','LineWidth',2,'Color',[.466 .674 .188]);
        sortedDistances=sort(distances);
        lowerDistances=sortedDistances(1:floor(0.0005 * length(distances)),:);
        radiusMacula2=mean(lowerDistances);
        disp(['Minimum Macula Radius (pixels): ',num2str(radiusMacula,'%2.1f')])
        disp(['Minimum Macula Radius : ',num2str(radiusMacula,'%3.2f')])
        disp(['Estimated Macula Radius: ',num2str(radiusMacula2,'%3.2f')])
        disp(['Standard Deviation of Macula Radius: ',num2str(std(lowerDistances),'%3.2f')])
        disp('- - - - - - - - - - - - - - - - - - - - - - - - -')
        set(radiiTable,'data',{'Minimum Macula Radius (pixels)',num2str(radiusMacula,'%3.2f');...
            'Overall Density',num2str(nnz(Image_Thresh)/numel(Image_Thresh),'%3.2f');...
            'Estimated Macula Radius',num2str(radiusMacula2,'%3.2f')})
    end

    function applyCircularMask()
        [height,width] = size(Image_Raw);
        radius1 = getHighValue(sliderRadius);
        radius2 = getLowValue(sliderRadius);
        [W,H] = meshgrid(1:width,1:height);
        mask = and(((W-x_center).^2 + (H-y_center).^2) - radius1^2<1 , ((W-x_center).^2 + (H-y_center).^2) - radius2^2>1);
        %         set(fh1, 'currentaxes', axThresh);
        set(imageHandleP, 'AlphaData', .9 + .1*mask);
        maskedImage = Image_Thresh.*mask;                                
        R1 = nnz(maskedImage(1:y_center,1:x_center))/nnz(mask(1:y_center,1:x_center));
        R2 = nnz(maskedImage(1:y_center,x_center+1:end))/nnz(mask(1:y_center,x_center+1:end));
        R3 = nnz(maskedImage(y_center+1:end,x_center+1:end))/nnz(mask(y_center+1:end,x_center+1:end));
        R4 = nnz(maskedImage(y_center+1:end,1:x_center))/nnz(mask(y_center+1:end,1:x_center));
%         disp(['Density in Region1: ',num2str(region1,'%1.3f')])                        
        set(densitiesTable,'data',{['R1    ' num2str(R1,'%1.3f')],['R2    ' num2str(R2,'%1.3f')];...
            ['R4    ' num2str(R4,'%1.3f')],['R3    ' num2str(R3,'%1.3f')]})                
    end

    function [RectReg1, RectReg2, RectReg3, RectReg4] = rectangularDensity()
        RectReg1 = nnz(Image_Thresh(1:y_center,1:x_center)) / (y_center * x_center);
        RectReg2 = nnz(Image_Thresh(1:y_center,x_center+1:end)) / (y_center * (size(Image_Thresh,2)-x_center));
        RectReg3 = nnz(Image_Thresh(y_center+1:end,x_center+1:end)) / ((size(Image_Thresh,1)-y_center) * (size(Image_Thresh,2)-x_center));
        RectReg4 = nnz(Image_Thresh(y_center+1:end,1:x_center))/ ((size(Image_Thresh,1)-y_center) * x_center);
    end

    function growRegion(~,~)
        set(0, 'currentfigure', fh1);
        set(fh1, 'currentaxes', axMain);
        area = scale^2* nnz(maskInitial{end})*1e-6;
        set(growAnnotation,'String',['Wait!   ' num2str(area,'%4.2f') ' mm2'])
        drawnow
        mask= activecontour(Image_Thresh, maskInitial{end}, 60, 'Chan-Vese','SmoothFactor',2.);
        mask = imfill(mask, 'holes');
        
        maskInitial{end+1} = mask;
        set(fh1, 'currentaxes', axMain);
        hOVM1(end+1) = alphamask(mask, [.4 .6 .1], .75, axMain);
        
        set(fh1, 'currentaxes', axProcessed);
        hOVM2(end+1) = alphamask(mask, [.4 .6 .1], .75, axProcessed);
        set(fh1, 'currentaxes', axCut);
        plotBoundary(mask);
        area = scale^2* nnz(maskInitial{end})*1e-6;
        set(growAnnotation,'String',['Area:   ' num2str(area,'%4.2f') ' mm2'])
    end

    function removeRegion(~,~)
        set(0, 'currentfigure', fh1);
        set(fh1, 'currentaxes', axMain);
        if length(maskInitial)>1,
            maskInitial(end) = [];
        end
        
        area = scale^2*nnz(maskInitial{end})*1e-6;
        set(growAnnotation,'String',['Area:  ' num2str(area,'%4.2f')])
        
        if exist('hOVM1') && ~isempty(hOVM1)
            delete(hOVM1(end))
            hOVM1(end)=[];
        end
        
        if exist('hOVM2') && ~isempty(hOVM2)
            delete(hOVM2(end))
            hOVM2(end)=[];
        end
        set(fh1, 'currentaxes', axCut);
        plotBoundary(maskInitial{end});
    end

    function modifyCenter(~,~)
        [x_center,y_center] = ginput(1);
        x_center=round(x_center);y_center=round(y_center);
        if ishandle(hOVM1),
            delete(hOVM1),
        end
        hOVM1= [];
        
        if ishandle(hCross),
            delete(hCross),
        end
        hCross= [];
        
        if any(ishandle(hOVM2)),
            set(fh1, 'currentaxes', axProcessed);
            delete(hOVM2),
        end
        hOVM2= [];
        
        applyThresh();        
        findMinimumRadi()
        y = ylim;
        x = xlim;
        hCross(1) = line([x_center, x_center],[y(1) y(2)]);
        set(hCross(1), 'LineWidth' , 2)
        hCross(2) = line([x(1), x(2)],[y_center y_center]);
        set(hCross(2), 'LineWidth' , 2)%, 'Color','r')
    end

    function realignCircles(~,~)
        [y,x] = find(maskInitial{end});
        k = boundary(x,y,1);
        x_center = round(mean(x(k)));
        y_center = round(mean(y(k)));
        drawCircles();
    end

    function resetValues(~,~)
%         [m,n]=size(Image_Raw);
%         y_center=round(m/2);
%         x_center=round(n/2);
        set(sliderThresh, 'Value',30)
        drawnow
        setHighValue(sliderRadius, round(1e3/scale))
        drawnow
        setLowValue(sliderRadius, round(5e2/scale))
        drawnow
%         if any(ishandle(hOVM1)),
%             delete(hOVM1),
%             hOVM1= [];
%         end
%         if any(ishandle(hOVM2)),
%             set(fh1, 'currentaxes', axProcessed);
%             delete(hOVM2),
%             hOVM2= [];
%         end
        applyThresh();
                 set(radiiTable,'data',{'Minimum Macula Radius (pixels)',num2str(radiusMacula,'%3.2f');...
            'Overall Density',num2str(nnz(Image_Thresh)/numel(Image_Thresh),'%3.2f');...
            'Estimated Macula Radius',num2str(radiusMacula2,'%3.2f')})
    end

    function analyze(source,~)
        set(cutAnnotation,'String',source.Tag)
        switch source.Tag
            case 'Superficial'
                Lcorner=128;
                set(sliderThresh, 'Value', 5)
            case 'Deep'
                Lcorner=896;
                set(sliderThresh, 'Value', 30)
                setHighValue(sliderRadius, round(1e3/scale))
        drawnow
        setLowValue(sliderRadius, round(5e2/scale))
        drawnow
        end
        ProcessImage()
    end

    function magnify (~,~)
        figure(2)
        ax(1)=subplot(121);
        imshow(uint8(Image_Thresh).*Image_Raw)
        title('After Thresholding')
        
        ax(2)=subplot(122);
        imshow(not(Image_Thresh))
        title('Removed Pixels')
        colormap('summer')
        linkaxes(ax)
    end

    function [boundaryLocation ,boundaryHandle] = plotBoundary(mask)
        cla
        imshow(Image_Raw)
        hold on
        title('FAZ Boundary')
        [y,x] = find(mask);
        k = boundary(x,y,1);
        boundaryLocation=[x(k),y(k)];
        boundaryHandle = plot(boundaryLocation(:,1), boundaryLocation(:,2));
    end

    function manualBoundary(~,~)
        fh3 = figure(3);
        [boundaryLocation , boundaryHandle] = plotBoundary((maskInitial{end}));
        uicontrol(fh3,'Style','pushbutton','String','Move Points','Units','normalized',...
            'Position',[.35 .05 .1 .03],'Callback',@movePointsFunction);
        uicontrol(fh3,'Style','togglebutton','String','Show/Hide','Units','normalized',...
            'Position',[.55 .05 .1 .03],'Callback',@show_Hide);
        maxfig(fh3,1)
        
        function movePointsFunction(~,~)
            nMovedPoints = 10;
            while 1
                [xPos,yPos, button] = ginput(1);
                if button ~=1 , break, end
                distancesToPoint = pdist2(boundaryLocation,[xPos,yPos]);
                [distancesToPoint,I] = sort(distancesToPoint);
                normalizationFactor = repmat(distancesToPoint(1:nMovedPoints)/distancesToPoint(1), 1, 2);
                boundaryLocation(I(1:nMovedPoints),:) = boundaryLocation(I(1:nMovedPoints),:)+ ...
                    .9 * (1./normalizationFactor) .* (repmat([xPos,yPos],nMovedPoints,1)-boundaryLocation(I(1:nMovedPoints),:));
                maskNew = poly2mask(boundaryLocation(:,1),boundaryLocation(:,2),size(Image_Processed,1),size(Image_Processed,2));
                maskInitial{end+1} = maskNew;
                
                set(0, 'currentfigure', fh1);
                set(fh1, 'currentaxes', axMain);
                
                area = scale^2*nnz(maskInitial{end})*1e-6
                set(growAnnotation,'String',['Area:  ' num2str(area,'%4.2f') ' mm2'])
                hOVM1(end+1) = alphamask(maskInitial{end}, [.4 .6 .1], .75, axMain);
                set(fh1, 'currentaxes', axProcessed);
                hOVM2(end+1) = alphamask(maskInitial{end}, [.4 .6 .1], .75, axProcessed);
                set(fh1, 'currentaxes', axCut);
                plotBoundary(maskInitial{end});
                
                set(0, 'currentfigure', fh3);
                delete (boundaryHandle)
                [boundaryLocation ,boundaryHandle] = plotBoundary(maskInitial{end});
            end
        end
        
        function show_Hide(~,~)
            switch(strcmp(get(boundaryHandle,'Visible'),'off'))
                case 0
                    set(boundaryHandle,'Visible','off')
                case 1
                    set(boundaryHandle,'Visible','on')
            end
            %             h = impoly(gca, boundaryLocation);
            %             setColor(h,'yellow');
            %             fcn = makeConstrainToRectFcn('impoly',get(gca,'XLim'),...
            %                 get(gca,'YLim'));
            %             setPositionConstraintFcn(h,fcn);
            %             boundaryLocation = h.getPosition();
        end
    end

    function saveResults(~,~)        
        [form_factor,roundness,extent,convexity,ls_ratio,solidity ,area] = shapeFactors(maskInitial);
        area = scale^2* area *1e-6;
        [ir1,ir2] = IrregularityIndex(maskInitial{end},[]);
        [RectReg1, RectReg2, RectReg3, RectReg4] = rectangularDensity();
        fprintf(fileID,'%10s,%f,%8s,%8s,%8s,%8s,%8s,%8s,%8s,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.3f,%2.2f,%2.3f,%10.2f,%10s,%2.2f,%1d,%4s,%2.2f,%2.2f,%2.2f,%2.2f,',...
            get(fh1,'Name'),get(sliderThresh,'Value'),  ...
            cell2mat(densitiesTable.Data(1)),cell2mat(densitiesTable.Data(3)),...
            cell2mat(densitiesTable.Data(4)),cell2mat(densitiesTable.Data(2)),...
            cell2mat(radiiTable.Data(1,2)),...
            cell2mat(radiiTable.Data(2,2)),...
            cell2mat(radiiTable.Data(3,2)),...
            form_factor,roundness,extent,convexity,ls_ratio, solidity ,area,ir1,ir2,get(cutAnnotation,'String'), ...
            scale , state , side, RectReg1, RectReg2, RectReg3, RectReg4);        
        
        setHighValue(sliderRadius, round(5e2/scale));
        setLowValue(sliderRadius, round(0/scale));
        drawnow;
        fprintf(fileID, '%8s,%8s,%8s,%8s,', ...
            cell2mat(densitiesTable.Data(1)),cell2mat(densitiesTable.Data(3)),...
            cell2mat(densitiesTable.Data(4)),cell2mat(densitiesTable.Data(2)));
        fprintf(fileID,'%f \n',OtsuThresh);
        imwrite(Image_Raw ,['results\images\' get(fh1,'Name') get(cutAnnotation,'String') mat2str(nameExt) '.png' ])
        imwrite(maskInitial{end},['results\images\' get(fh1,'Name') get(cutAnnotation,'String') '_mask_' mat2str(nameExt) '.png'])
        
    end

    function showResults(~,~)
        d1 = dialog('Position',[100 100 200 250],'Name','Shape Factors');
        closeBtn = uicontrol('Parent',d1, 'Position',[65 10 70 15],...
            'String','Close',...
            'Callback','delete(gcf)');
        growBtn = uicontrol('Parent',d1,'Position',[60 40 80 20],...
            'String','Grow',...
            'Callback',@grow);
        shrinkBtn = uicontrol('Parent',d1,'Position',[60 70 80 20],...
            'String','Shrink',...
            'Callback',@shrink);
        
        calculateFactors;
        
        function calculateFactors(~,~)
            [form_factor,roundness,extent,convexity,ls_ratio,solidity , area] = shapeFactors(maskInitial);
            area = scale^2* area *1e-6 ;
            [ir1,ir2] = IrregularityIndex(maskInitial{end},d1);
%             IrregularityIndex(maskInitial{end},d1);
            set(growAnnotation,'String',['Area:  ' num2str(area,'%4.2f') ' mm2'])
            txt = uicontrol('Parent',d1,...
                'Style','text',...
                'Position',[60 100 150 140],...
                'HorizontalAlignment', 'left',...
                'String',{...
                ['Area        ' num2str(area,'%4.2f')],...
                ['FormFactor  ' num2str(form_factor,'%2.2f')],...
                ['Roundness   ' num2str(roundness,'%2.2f')],...
                ['Extent      ' num2str(extent,'%2.2f')],...
                ['Convexity   ' num2str(convexity,'%2.2f')],...
                ['Solidity    ' num2str(solidity,'%2.2f')],...
                ['LS Ratio    ' num2str(ls_ratio,'%2.2f')],...
                ['Irreg1      ' num2str(ir1,'%2.2f')],...
                ['Irreg2      ' num2str(ir2,'%10.2f')]});
        end
        
        function grow(~,~)
            growRegion;
            calculateFactors;
        end
        
        function shrink(~,~)
            removeRegion;
            calculateFactors;
        end
    end

    function Crop(~,~)
        f = figure('units','Normalized');
        imshow(Image_Initial)
        maxfig(f,1)
        title('Select Top-left corner and Right-Bottom corners')
        uicontrol('Parent',f,'Units','normalized' ,'Position',[.55 .01 .1 .05],...
            'String','Click Corners','Callback',@getCorners);
        cuts = {'Superficial' ,'Deep'};
        cutHandle = uicontrol('Style','popupmenu','Parent',f,'Units','normalized' ,'Position',[.3 .01 .15 .05],...
            'String',cuts,'Callback',@getCorners);
        function getCorners(~,~)
            [x, y]=ginput(2);
            Tcorner = round(y(1));
            Lcorner = round(x(1));
            Width = round(diff(x));
            Height = round(diff(y));
%             set(0, 'currentfigure', fh1);
            set(cutAnnotation,'String', cuts{get(cutHandle,'value')})
            close(f)
            ProcessImage()
        end
    end
end