clear all;
close all;
clc;

% prior pdf map information Bolz Hall.
img = uint8(imread('bolz_hall_2nd_floor_rotated.jpg'));
imgOriginal = img;
nRows = size(imgOriginal, 1); nCols = size(imgOriginal,2);
imgMapPrior = uint8(255*ones(nRows, nCols));
w1 = 27; % larger width
w2 = 13; % width of the corridor around PCV Lab.

P1 = [1364, 735]; P2 = [274+2, 735]; P3 = [274, 523];
P4 = [367, 523]; P5 = [367, 181]; P6 = [181, 181]; P7 = [181, 523];
% corrections
P1 = [1390, 735];


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

% for i=691-20:1:721
%     for j=1209-5+1:1225+5+1
%         img(i,j,1) = 0;
%     end
% end

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

% for i=691-20:1:721
%     for j=1209-5+1:1225+5+1
%         imgMapPrior(i,j) = 0;
%     end
% end

imgMapPriorBackUp = imgMapPrior;
imgMapPrior = imgMapPrior(1:534,:);
imgBackUp = img;
img = img(1:534,:,:);
% imgMapPriorBottom = imgMapPrior(535:end,:);
% imgMapPriorTop = uint8(255*ones(nRows, nCols));
% imgMapPriorBottom = uint8(255*ones(nRows, nCols));

H = fspecial('gaussian',[5 5],3);
imgMapPrior = imfilter(imgMapPrior,H,'replicate');
H = fspecial('average',[7 7]);
imgMapPrior = imfilter(imgMapPrior,H,'replicate');

% H = fspecial('gaussian',[31 31],3);
% imgMapPriorBottom = imfilter(imgMapPriorBottom,H,'replicate');
% H = fspecial('average',[33 33]);
% imgMapPriorBottom = imfilter(imgMapPriorBottom,H,'replicate');
% imgMapPrior(1:534,:) = imgMapPriorTop(1:534,:);
% imgMapPrior(535:end,:) = ones(524,1410);
% imgMapPrior(535:end,:) = imgMapPriorBottom(535:end,:);
% imgMapPriorBackUp = imgMapPrior;
% imgMapPrior = zeros(1058,1410);
% imgMapPrior(1:534,:) = imgMapPriorTop;
% imgMapPrior(535:end,:) = imgMapPriorBottom;
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
bolzPDFtop = uint8((255/max(max(finalImg3)))*finalImg3);
% bolzPDF = bolzPDF + 50 + 23;

% for i=1:r
%     for j=1:c
%         if (bolzPDF(i,j) > 219)
%             bolzPDF(i,j) = 219;
%         end
%     end
% end

% linear mapping of intensity values to [1-255] region
% bolzPDF = bolzPDF + (255-max(max(bolzPDF)));
% bolzPDF = double(bolzPDF);
% maxIntensity = max(max(bolzPDF));
% for i=1:r
%     for j=1:c
%         bolzPDF(i,j) = ( (255-1) / maxIntensity ) * bolzPDF(i,j) + 1;
%     end
% end
% bolzPDF = (bolzPDF - 73)*(254/182) + 1;
% bolzPDF = uint8(bolzPDF);

figure(20);
imshow(imgOriginal(138:223,100:260));
figure(21);
imshow(bolzPDFtop(138:223,100:260),'colormap',jet(255));
set(gca, 'YDir', 'reverse');

figure(22);
imshow(imgOriginal(480:534,200:400,:));
figure(23);
imshow(bolzPDFtop(480:end,200:400),'colormap',jet(255));
set(gca, 'YDir', 'reverse');

%%
imgMapPrior = imgMapPriorBackUp(535:end,:);
img = imgBackUp(535:end,:,:);
% imgMapPriorBottom = imgMapPrior(535:end,:);
% imgMapPriorTop = uint8(255*ones(nRows, nCols));
% imgMapPriorBottom = uint8(255*ones(nRows, nCols));

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

figure(24);
subplot(121);
imshow(img);
subplot(122);
imshow(imgMapPrior);

%%
figure(25); set(gcf,'position',[13 126 1563 518]); s = 1.01;
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
bolzPDFbottom = uint8((255/max(max(finalImg3)))*finalImg3);
bolzPDFbottom = double(bolzPDFbottom + 31);
bolzPDFbottom = uint8((bolzPDFbottom - 31).*254./(255-31)+1); 

figure(26);
imshow(imgOriginal(120+534:280+534,180:420));
figure(27);
imshow(bolzPDFbottom(120:280,180:420),'colormap',jet(255));
set(gca, 'YDir', 'reverse');

bolzPDF = uint8(zeros(1058,1410));
bolzPDF(1:534,:) = bolzPDFtop;
bolzPDF(535:end,:) = bolzPDFbottom;

figure(28);
imshow(imgOriginal);
figure(29);
imshow(bolzPDF,'colormap',jet(255));
set(gca, 'YDir', 'reverse');

% place the door models before blurring for PCV lab door region
load doorKernels.mat; load doorKernels2.mat;
% doorKernelShrunkPCV2 = doorKernelShrunkPCV(1:38,:);
% doorKernelShrunk = imresize(doorKernel,[134 30]);
% doorKernel2 = uint8(((double(doorKernel2) - 31) / (255-31))*254 + 1);
bolzPDF(660-2:793-2,1203:1232,:) = doorKernelShrunk2;
% doorKernelShrunkRotated2 = imrotate(doorKernelShrunk2,180);
bolzPDF(677+2:810+2,984:1013,:) = doorKernelShrunkRotated2;
bolzPDF(660-2:793-2,645:674,:) = doorKernelShrunk2;
bolzPDF(735:760,601:701) = bolzPDF(735:760,500:600);
bolzPDF(690:735,950:1050) = bolzPDF(690:735,850:950);
% bolzPDF(735:760,901:1001) = bolzPDF(735:760,500:600);
bolzPDF(735:760,1101:1301) = bolzPDF(735:760,400:600);
% bolzPDF(504:523,276-30:276+29,:) = doorKernelShrunkPCV2;


% corrections of pcv lab door area
bolzPDF(532:534,245:307) = bolzPDF(535:537,245:307);
H = fspecial('gaussian',[5 5],3);
bolzPDF = imfilter(bolzPDF,H,'replicate');
H = fspecial('average',[7 7]);
bolzPDF = imfilter(bolzPDF,H,'replicate');
bolzPDF = double(bolzPDF + 13);
bolzPDF = uint8((bolzPDF - 14).*254./(255-14)+1); 
bolzPDFbackup = bolzPDF;

mirrorPCVdoor = bolzPDF(523:548,234:315);
mirrorPCVdoor = imrotate(mirrorPCVdoor,180);
mirrorPCVdoor = imresize(mirrorPCVdoor,[26,41]);
for i=1:26
    bolzPDF(523-26+1:523,234+21:315-20) = mirrorPCVdoor;
end

for i=1200:1235
    for j=650:686
        bolzPDF(j,i) = 1;
    end
end
for i=635:680
    for j=650:686
        bolzPDF(j,i) = 1;
    end
end

for i=983:1015
    for j=782:830
        bolzPDF(j,i) = 1;
    end
end

fprintf('stop.\n');



% kernel2 = imrotate(kernel,180);
% for i=757-22:757+22
%     for j=998-41:998+41
%         bolzPDF(i,j) = kernel2(i-757+22+1,j-998+41+1);
%     end
% end
% 
% for i=713-22:713+22
%     for j=660-41:660+41
%         bolzPDF(i,j) = kernel(i-713+22+1,j-660+41+1);
%     end
% end
% 
% for i=500-22:500+22
%     for j=274-41:274+41
%         bolzPDF(i,j) = kernel(i-500+22+1,j-274+41+1);
%     end
% end

% short parallel corridor
% for i=757-22:757+22
%     for j=790-41:790+41
%         bolzPDF(i,j) = kernel2(i-757+22+1,j-790+41+1);
%     end
% end

% figure(3); set(gcf,'position',[371   151   535   422]);
% imshow(bolzPDF,'colormap',jet(255));
% colormap jet;
% set(gca, 'YDir', 'reverse');
% 
% figure(4); set(gcf,'position',[371   151   535   422]);
% imshow(imgOriginal);
% 
% figure(5); set(gcf, 'position', [911   152   560   420]);
% surf(bolzPDF); shading flat; colormap jet;
% set(gca, 'YDir', 'reverse');
% 
% imgTemp = imgOriginal(600:end-150,600:end,:);
% figure(6);
% set(gcf,'position',[371   151   535   422]);
% imshow(imgTemp);
% figure(7);
% set(gcf,'position',[371   151   535   422]);
% imshow(bolzPDF(600:end-150,600:end),'colormap',jet(255));
% 
% %%
% imgTemp = imgOriginal(660:793,1170:1265,:);
% figure(8); clf; set(gcf,'position',[679    94   794   638]);
% set(gcf,'position',[371   151   535   422]);
% imshow(imgTemp);
% % doorKernel = bolzPDF(660:793,1170:1265,:);
% figure(9); clf; set(gcf,'position',[679    94   794   638]);
% set(gcf,'position',[371   151   535   422]);
% imshow(bolzPDF(660:793,1170:1265),'colormap',jet(255));
% 
% imgTemp = imgOriginal(677-10:810+10,983-40:1012+40,:);
% figure(10); clf; set(gcf,'position',[679    94   794   638]);
% set(gcf,'position',[371   151   535   422]);
% imshow(imgTemp);
% % doorKernel = bolzPDF(660:793,1170:1265,:);
% figure(11); clf; set(gcf,'position',[679    94   794   638]);
% set(gcf,'position',[371   151   535   422]);
% imshow(bolzPDF(677-10:810+10,983-40:1012+40),'colormap',jet(255));
% 
% imgTemp = imgOriginal(660:793,645:674,:);
% figure(12); clf; set(gcf,'position',[679    94   794   638]);
% set(gcf,'position',[371   151   535   422]);
% imshow(imgTemp);
% % doorKernel = bolzPDF(660:793,1170:1265,:);
% figure(13); clf; set(gcf,'position',[679    94   794   638]);
% set(gcf,'position',[371   151   535   422]);
% imshow(bolzPDF(660:793,645:674),'colormap',jet(255));
% % doorKernelShrunk = imresize(doorKernel,[134 30]);
% % figure(10); imshow(doorKernelShrunk,'colormap',jet(255));
% % doorKernelShrunkRotated = imrotate(doorKernelShrunk,180);
% % figure(11); imshow(doorKernelShrunkRotated,'colormap',jet(255));
% 
% % save('doorKernels.mat','doorKernel','doorKernelShrunk',...
% %     'doorKernelShrunkRotated');
save('bolz_hall_pdf_updated3.mat','bolzPDF');
