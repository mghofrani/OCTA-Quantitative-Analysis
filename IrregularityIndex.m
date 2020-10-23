function [ir1,ir2] = IrregularityIndex(binaryImage,d)
showGraph = nargout == 0;
[y,x] = find(binaryImage);
k = boundary(x,y,1);
% k(1) = [];
L = length(k);
x = x(k);
y = y(k);
F = (fft(x+1i*y,L));

if showGraph
    close(d)
    figure
    maxfig(gcf,1)
    sliderFreq= com.jidesoft.swing.RangeSlider(1,L,1,L);  % min,max,low,high
    [~, container] = javacomponent(sliderFreq);
    set(container,'Units', 'normalized', 'Position', [0.1 0.01 .8 .05]);
    set(sliderFreq, 'StateChangedCallback', @selectFreq);  %alternatives: StateChangedCallback  MouseReleasedCallback
end
selectFreq;
%%
    function selectFreq(~,~)
        F_filtered = F;
        if showGraph
            subplot(212)
            cla
            stem(log(abs(F)),'.')
            axis tight
            hold on
            % F_filtered([1: getLowValue(sliderFreq), getHighValue(sliderFreq) : L])= 0;
            F_filtered(getLowValue(sliderFreq) : getHighValue(sliderFreq))= 0;
            stem(log(abs(F_filtered)))
            title(num2str([find(F_filtered,1,'First'),find(F_filtered,1,'Last')]))
        else
            F_filtered(round(0.2*L: 0.8*L))= 0;
        end
        iF = ifft((F_filtered),L);
        x2 = real(iF);
        y2 = imag(iF);
        centroid(1) = mean(x2);
        centroid(2) = mean(y2);
        centDist = pdist2([x2 y2], centroid);
        if showGraph
            subplot(211)
            cla
            imshow(binaryImage)
            xlabel(num2str(L))
            hold on
            plot(x,y , '.','Markersize',4)
            plot(x2,y2,'.')
            plot(centroid(1),centroid(2),'g+','Markersize',8)
        end
        ir1 = std(centDist)/mean(centDist);
        ir2 = mean((abs(F_filtered(3:end)).^2)/L); % First Element of FFT: DC component , Second Element: Circular Pattern
        if showGraph
            title(num2str([ir1, ir2],'%10.2f'))
        end
    end
end
