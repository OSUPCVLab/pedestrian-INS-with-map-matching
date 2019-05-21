function croppedRegion = imcrop_taha_subpixel_accuracy(choice,mu,nx,px,ny,py)

global staticFloorPDF

x = 1:1:size(staticFloorPDF{choice},2);
y = 1:1:size(staticFloorPDF{choice},1);
[X,Y] = meshgrid(x,y);
xq = (mu(1)-nx) : 1 : (mu(1)+px);
yq = (mu(2)-ny) : 1 : (mu(2)+py);
[Xq,Yq] = meshgrid(xq,yq);

croppedRegion = interp2(X,Y,staticFloorPDF{choice},Xq,Yq);