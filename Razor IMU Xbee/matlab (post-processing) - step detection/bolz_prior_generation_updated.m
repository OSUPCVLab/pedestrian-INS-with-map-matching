clear all;
close all;
clc;

% prior pdf map information Bolz Hall.
img = uint8(imread('bolz_hall_2nd_floor_rotated.jpg'));
imgOriginal = img;
nRows = size(imgOriginal, 1); nCols = size(imgOriginal,2);
imgMapPrior = uint8(255*ones(nRows, nCols));
w1 = 27; % larger width
w2 = 27; % width of the corridor around PCV Lab.

P1 = [1364, 735]; P2 = [274, 735]; P3 = [274, 522];
P4 = [366, 522]; P5 = [366, 179]; P6 = [179, 179]; P7 = [179, 522];

% node between P1 and P2
img(P1(2)-((w1-1)/2):P1(2)+((w1-1)/2),P2(1)-((w1-1)/2):P1(1)+((w1-1)/2)) = zeros(w1, P1(1)-P2(1)+w1);
% node between P2 and P3
img(P3(2):P2(2)+((w1-1)/2),P2(1)-((w1-1)/2):P2(1)+((w1-1)/2)) = zeros(P2(2)-P3(2)+1+((w1-1)/2),w1);
% node between P3 and P4
img(P3(2)-((w2-1)/2):P3(2)+((w2-1)/2),P3(1)-((w2-1)/2):P4(1)+((w2-1)/2)) = zeros(w2, P4(1)-P3(1)+w2);
% node between P4 and P5
img(P5(2):P4(2)+((w2-1)/2),P4(1)-((w2-1)/2):P4(1)+((w2-1)/2)) = zeros(P4(2)-P5(2)+1+((w2-1)/2),w2);
% node between P5 and P6
img(P5(2)-((w2-1)/2):P5(2)+((w2-1)/2),P6(1)-((w2-1)/2):P5(1)+((w2-1)/2)) = zeros(w2, P5(1)-P6(1)+w2);
% node between P6 and P7
img(P6(2):P7(2)+((w2-1)/2),P6(1)-((w2-1)/2):P6(1)+((w2-1)/2)) = zeros(P7(2)-P6(2)+1+((w2-1)/2),w2);
% node between P7 and P3
img(P7(2)-((w2-1)/2):P7(2)+((w2-1)/2),P7(1)-((w2-1)/2):P3(1)+((w2-1)/2)) = zeros(w2, P3(1)-P7(1)+w2);

for i=691-20:1:721
    for j=1209-5:1225+5
        img(i,j,1) = 0;
    end
end

% node between P1 and P2
imgMapPrior(P1(2)-((w1-1)/2):P1(2)+((w1-1)/2),P2(1)-((w1-1)/2):P1(1)+((w1-1)/2)) = zeros(w1, P1(1)-P2(1)+w1);
% node between P2 and P3
imgMapPrior(P3(2):P2(2)+((w1-1)/2),P2(1)-((w1-1)/2):P2(1)+((w1-1)/2)) = zeros(P2(2)-P3(2)+1+((w1-1)/2),w1);
% node between P3 and P4
imgMapPrior(P3(2)-((w2-1)/2):P3(2)+((w2-1)/2),P3(1)-((w2-1)/2):P4(1)+((w2-1)/2)) = zeros(w2, P4(1)-P3(1)+w2);
% node between P4 and P5
imgMapPrior(P5(2):P4(2)+((w2-1)/2),P4(1)-((w2-1)/2):P4(1)+((w2-1)/2)) = zeros(P4(2)-P5(2)+1+((w2-1)/2),w2);
% node between P5 and P6
imgMapPrior(P5(2)-((w2-1)/2):P5(2)+((w2-1)/2),P6(1)-((w2-1)/2):P5(1)+((w2-1)/2)) = zeros(w2, P5(1)-P6(1)+w2);
% node between P6 and P1
imgMapPrior(P6(2):P7(2)+((w2-1)/2),P6(1)-((w2-1)/2):P6(1)+((w2-1)/2)) = zeros(P7(2)-P6(2)+1+((w2-1)/2),w2);
% node between P7 and P3
imgMapPrior(P7(2)-((w2-1)/2):P7(2)+((w2-1)/2),P7(1)-((w2-1)/2):P3(1)+((w2-1)/2)) = zeros(w2, P3(1)-P7(1)+w2);
for i=691-20:1:721
    for j=1209-5:1225+5
        imgMapPrior(i,j) = 0;
    end
end

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
figure(2); set(gcf,'position',[13 126 1563 518]); s = 1.01;
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
bolzPDF = uint8((255/max(max(finalImg3)))*finalImg3);

for i=1:r
    for j=1:c
        if (bolzPDF(i,j) > 219)
            bolzPDF(i,j) = 219;
        end
    end
end

% linear mapping of intensity values to [1-255] region
bolzPDF = im2double(bolzPDF);
maxIntensity = max(max(bolzPDF));
for i=1:r
    for j=1:c
        bolzPDF(i,j) = ( (255-1) / maxIntensity ) * bolzPDF(i,j) + 1;
    end
end
bolzPDF = uint8(bolzPDF);

% crop entrance of rooms so that trajectory can enter the rooms
for i=650:690
    for j=1176:1256
        bolzPDF(i,j) = 1;
    end
end
bolzPDF = uint8(bolzPDF);

% copy paste
kernel = uint8(zeros(abs(691-735)+1,abs(1176-1258)+1));
for i=691:735
    for j=1176:1258
        kernel(i-690,j-1175) = bolzPDF(i,j);
    end
end

kernel2 = imrotate(kernel,180);
for i=757-22:757+22
    for j=998-41:998+41
        bolzPDF(i,j) = kernel2(i-757+22+1,j-998+41+1);
    end
end

for i=713-22:713+22
    for j=660-41:660+41
        bolzPDF(i,j) = kernel(i-713+22+1,j-660+41+1);
    end
end

for i=500-22:500+22
    for j=274-41:274+41
        bolzPDF(i,j) = kernel(i-500+22+1,j-274+41+1);
    end
end

for i=757-22:757+22
    for j=790-41:790+41
        bolzPDF(i,j) = kernel2(i-757+22+1,j-790+41+1);
    end
end

figure(3); set(gcf,'position',[371   151   535   422]);
imshow(bolzPDF,'colormap',jet(255));
colormap jet;
set(gca, 'YDir', 'reverse');

figure(4); set(gcf,'position',[371   151   535   422]);
imshow(imgOriginal);

figure(5); set(gcf, 'position', [911   152   560   420]);
surf(bolzPDF); shading flat; colormap jet;
set(gca, 'YDir', 'reverse');

imgTemp = imgOriginal(600:end-150,600:end,:);
figure(6);
set(gcf,'position',[371   151   535   422]);
imshow(imgTemp);

figure(7);
set(gcf,'position',[371   151   535   422]);
imshow(bolzPDF(600:end-150,600:end),'colormap',jet(255));