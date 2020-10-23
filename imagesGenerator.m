clc
clear
close all
Lcorner = 128; % left corner, default value for Deep analysis
Tcorner = 896;
Width = 512;
Height = Width;
[num,txt,raw] = xlsread('patientsInfo');
%%
age = num(: , end);
state = num(:,1);
sex = txt(2:end,6);
G = findgroups(state,sex);
mkdir('photos')

%%
for i = 1: length(sex)
    switch G(i)
        case 1
            ind = i + find (G(i+1:end) == 3 , 1 , 'first');
        case 2
            ind = i + find (G(i+1:end) == 4 , 1 , 'first');        
        otherwise
            continue
    end            
    if isempty(ind), continue, end
    Healthyname = txt{i+1,1}
    folder = [pwd '\mohsenipour1\' strtrim(Healthyname) '\'];
    files = ls(folder);
    files (1:2, :) = [];
    close all
    for j = 1: 2
        figure(j);
        subplot(121)
        fileName = strtrim(files(j , :));
        I = imread([folder fileName]);
        I = I(Tcorner : Tcorner+Height , Lcorner : Lcorner+Width);
        imshow(I)       
        title([cell2mat(sex(i)) '-' num2str(age(i)) '-' num2str(state(i))])
        xlabel(fileName)
        drawnow
    end   
    
    DiabeticsName = txt{ind+1,1}
    folder = [pwd '\mohsenipour1\' strtrim(DiabeticsName) '\'];
    files = ls(folder);
    files (1:2, :) = [];
    for j = 1: 2
        figure(j);
        subplot(122)
        fileName = strtrim(files(j , :));
        I = imread([folder fileName]);
        I = I(Tcorner : Tcorner+Height , Lcorner : Lcorner+Width);
        imshow(I)
        title([cell2mat(sex(ind)) '-' num2str(age(ind)) '-' num2str(state(ind))])
        xlabel(fileName)
        drawnow
    end
    saveas(figure(1), [pwd '\photos\' cell2mat(sex(ind)) '-' num2str(age(ind)) ' A ' num2str(i) '.jpg'])   
    saveas(figure(2), [pwd '\photos\' cell2mat(sex(ind)) '-' num2str(age(ind)) ' B ' num2str(i) '.jpg'])     
end