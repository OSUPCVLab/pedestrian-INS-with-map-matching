clear all;
close all;
clc;

load('kernelsForCaldwell.mat');
% prior pdf map information Caldwell Lab.
img = uint8(imread('caldwell_2nd_floor_rotated.png'));
imgOriginal = img;
nRows = size(imgOriginal, 1); nCols = size(imgOriginal,2);
imgMapPrior = uint8(255*ones(nRows, nCols));
w = 27; % width

P1 = [387, 1082]; P2 = [872, 1082]; P3 = [872, 711];
P4 = [1021, 711]; P5 = [1021, 370]; P6 = [387, 370];

% node between P1 and P2
img(P1(2)-((w-1)/2):P1(2)+((w-1)/2),P1(1)-((w-1)/2):P2(1)+((w-1)/2)) = zeros(w, P2(1)-P1(1)+w);
% node between P2 and P3
img(P3(2):P2(2)+((w-1)/2),P2(1)-((w-1)/2):P2(1)+((w-1)/2)) = zeros(P2(2)-P3(2)+1+((w-1)/2),w);
% node between P3 and P4
img(P3(2)-((w-1)/2):P3(2)+((w-1)/2),P3(1)-((w-1)/2):P4(1)+((w-1)/2)) = zeros(w, P4(1)-P3(1)+w);
% node between P4 and P5
img(P5(2):P4(2)+((w-1)/2),P4(1)-((w-1)/2):P4(1)+((w-1)/2)) = zeros(P4(2)-P5(2)+1+((w-1)/2),w);
% node between P5 and P6
img(P5(2)-((w-1)/2):P5(2)+((w-1)/2),P6(1)-((w-1)/2):P5(1)+((w-1)/2)) = zeros(w, P5(1)-P6(1)+w);
% node between P6 and P1
img(P6(2):P1(2)+((w-1)/2),P6(1)-((w-1)/2):P6(1)+((w-1)/2)) = zeros(P1(2)-P6(2)+1+((w-1)/2),w);

% node between P1 and P2
imgMapPrior(P1(2)-((w-1)/2):P1(2)+((w-1)/2),P1(1)-((w-1)/2):P2(1)+((w-1)/2)) = zeros(w, P2(1)-P1(1)+w);
% node between P2 and P3
imgMapPrior(P3(2):P2(2)+((w-1)/2),P2(1)-((w-1)/2):P2(1)+((w-1)/2)) = zeros(P2(2)-P3(2)+1+((w-1)/2),w);
% node between P3 and P4
imgMapPrior(P3(2)-((w-1)/2):P3(2)+((w-1)/2),P3(1)-((w-1)/2):P4(1)+((w-1)/2)) = zeros(w, P4(1)-P3(1)+w);
% node between P4 and P5
imgMapPrior(P5(2):P4(2)+((w-1)/2),P4(1)-((w-1)/2):P4(1)+((w-1)/2)) = zeros(P4(2)-P5(2)+1+((w-1)/2),w);
% node between P5 and P6
imgMapPrior(P5(2)-((w-1)/2):P5(2)+((w-1)/2),P6(1)-((w-1)/2):P5(1)+((w-1)/2)) = zeros(w, P5(1)-P6(1)+w);
% node between P6 and P1
imgMapPrior(P6(2):P1(2)+((w-1)/2),P6(1)-((w-1)/2):P6(1)+((w-1)/2)) = zeros(P1(2)-P6(2)+1+((w-1)/2),w);

H = fspecial('gaussian',[31 31],3);
imgMapPrior = imfilter(imgMapPrior,H,'replicate');
H = fspecial('average',[33 33]);
imgMapPrior = imfilter(imgMapPrior,H,'replicate');

% superimpose prior pdf on floor plan
r = size(img,1); c = size(img,2);
for i=1:r
    for j=1:c
        if (imgMapPrior(i,j) ~= 255)
            img(i,j,1) = 255 - imgMapPrior(i,j);
            img(i,j,2) = 255 - imgMapPrior(i,j);
            img(i,j,3) = 255 - imgMapPrior(i,j);
        end
    end
end

figure(1);
subplot(121);
imshow(img);
subplot(122);
imshow(imgMapPrior);

%%
figure(2); set(gcf,'position',[13 126 1563 672]); s = 1.01;
subplot(131);
imshow(imgOriginal);
set(gca,'position',[0.0100 0.1100 s*0.3034 0.8150]);
subplot(132);
imshow(img);
set(gca,'position',[0.0100+s*0.3044 0.1100 s*0.3034 0.8150]);


finalImg = 255 - imgMapPrior;
finalImg2 = zeros(size(finalImg));
for i=1:r
    finalImg2(i,:) = finalImg(r-i+1,:);
end
finalImg2 = (255/max(max(finalImg2)))*finalImg2;

subplot(133);
[cv,ch] = contourf(finalImg2,30);
%shading flat
set(ch,'edgecolor','none');
colormap(jet);
set(gca,'xtick',[],'ytick',[],'position',[0.0100+2*s*0.3044 0.1700 s*0.3034 0.6950]);
%%
finalImg3 = zeros(size(finalImg));
for i=1:r
    finalImg3(i,:) = ch.ZData(r-i+1,:);
end
caldwellPDF = uint8((255/max(max(finalImg3)))*finalImg3);

for i=1:r
    for j=1:c
        if (caldwellPDF(i,j) > 219)
            caldwellPDF(i,j) = 219;
        end
    end
end

% linear mapping of intensity values to [1-255] region
caldwellPDF = im2double(caldwellPDF);
maxIntensity = max(max(caldwellPDF));
for i=1:r
    for j=1:c
        caldwellPDF(i,j) = ( (255-1) / maxIntensity ) * caldwellPDF(i,j) + 1;
    end
end
caldwellPDF = uint8(caldwellPDF);

% door updates
for i=951-41:951+41
    for j=895-22:895+22
        caldwellPDF(i,j) = kernel2T(i-951+41+1,j-895+22+1);
    end
end

for i=847-41:847+41
    for j=364-22:364+22
        caldwellPDF(i,j) = kernelT(i-847+41+1,j-364+22+1);
    end
end

for i=1059-22:1059+22
    for j=598-41:598+41
        caldwellPDF(i,j) = kernel(i-1059+22+1,j-598+41+1);
    end
end

kernel3 = caldwellPDF(916:987,913:917);
for i=916:987
    for j=913:917
        caldwellPDF(i,j+4) = kernel3(i-915,j-912);
    end
end
for i=916:987
    for j=918:922
        caldwellPDF(i,j) = kernel3(i-915,j-917);
    end
end

figure(3); set(gcf,'position',[371   151   535   422]);
imshow(caldwellPDF,'colormap',jet(255));
colormap jet;
set(gca, 'YDir', 'reverse');

figure(4); set(gcf,'position',[371   151   535   422]);
imshow(imgOriginal);

figure(5); set(gcf, 'position', [911   152   560   420]);
surf(caldwellPDF); shading flat; colormap jet;
set(gca, 'YDir', 'reverse');
axis tight;

imgTemp = imgOriginal(750:1200,200:1100,:);
figure(6);
set(gcf,'position',[371   151   535   422]);
imshow(imgTemp);

figure(7);
set(gcf,'position',[371   151   535   422]);
imshow(caldwellPDF(750:1200,200:1100),'colormap',jet(255));