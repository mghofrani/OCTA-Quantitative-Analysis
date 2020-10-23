Image_Raw = ans;
%%
figure
Image_Processed = Image_Raw;
Image_Thresh =  (Image_Processed>8).*true(size(Image_Raw));
subplot(121)
imshow(Image_Processed)
subplot(122)
imshow(Image_Thresh);
%%
figure
Image_Processed = adapthisteq(Image_Raw);
Image_Thresh =  (Image_Processed>.3).*true(size(Image_Raw));

subplot(121)
imshow(Image_Processed)
subplot(122)
imshow(Image_Thresh);

%%
figure
Image_Processed = wiener2(Image_Raw);
Image_Thresh =  (Image_Processed>8).*true(size(Image_Raw));
subplot(121)
imshow(Image_Processed)
subplot(122)
imshow(Image_Thresh);

%%
bw = Image_Raw;
d = 0:15:180;
bw2 = zeros(size(bw,1),size(bw,2),15*numel(d));
for i = 1 : numel(d)
    se = strel('line',7,d(i));
    bw2(:,:,i) = imopen(bw,se);   
end
figure
imshow(Image_Raw)
figure
bw2 = mean(bw2,3);
imshow(bw2)
%%
figure
Image_Processed = imsharpen(bw2);
% Image_Thresh =  (Image_Processed>.3).*true(size(Image_Raw));
% subplot(121)
imshow(Image_Processed)
% subplot(122)
% imshow(Image_Thresh);