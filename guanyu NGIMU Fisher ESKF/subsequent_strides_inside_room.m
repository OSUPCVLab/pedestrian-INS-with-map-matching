function lastFigNumber = subsequent_strides_inside_room(img,maxColors,...
    finalEstimateRotatedTranslated,strideNumber,...
    room1enteringStride,subsequentStrides,color,hypNumber,...
    finalEstimateRotated,choice,figNumber)

global staticFloorPDF

tempHypNumber = 0;
if (hypNumber <= 8)
    tempHypNumber = hypNumber;
else
    tempHypNumber = 8;
end
showRotated = false;
if (ceil(hypNumber/8) >= 1)
figNumber = figNumber+1;
figure(figNumber); set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% iStart = 8*ceil(rooms.room1.numberOfHyp/8)-7;
for i=1:tempHypNumber % rooms.room1.numberOfHyp
    tempNumber = mod(i,maxColors);
    if (tempNumber == 0)
        tempNumber = 8;
    end
    plot(finalEstimateRotatedTranslated(1,strideNumber - ...
        (strideNumber - room1enteringStride - i):strideNumber - ...
        (strideNumber - room1enteringStride - i) + ...
        subsequentStrides,i), finalEstimateRotatedTranslated(2,strideNumber-...
        (strideNumber - room1enteringStride - i):strideNumber-...
        (strideNumber - room1enteringStride - i)+subsequentStrides,i),...
        'g-x','linewidth',1.5,'color',...
        color(tempNumber,:));
end
if (hypNumber < 9)
    showRotated = 1; %hypNumber = 8; % hypNumber can be 1 to 8
end
if (choice == 1 || choice == 2)
    if (showRotated)
        % automated plotting
        plot(finalEstimateRotated(1,:,hypNumber),...
             finalEstimateRotated(2,:,hypNumber),'r-x','linewidth',1.5);
        % (strideNumber - room1enteringStride - 1) = 350
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),'ks',...
            'linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),'ko',...
            'linewidth',2,'markerfacecolor','y','markersize',16);
%         % manual plotting
%         plot(finalEstimateRotatedhk8(1,1:end),finalEstimateRotatedhk8(2,1:end),'r-x','linewidth',1.5);
%         % 350 is h1, 349 is h2, ...
%         plot(finalEstimateRotatedhk8(1,strideNumber-343),finalEstimateRotatedhk8(2,strideNumber-343),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
%         plot(finalEstimateRotatedhk8(1,strideNumber-344),finalEstimateRotatedhk8(2,strideNumber-344),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
hold off;
legend('$H_1$','$H_2$','$H_3$','$H_4$','$H_5$','$H_6$','$H_7$','$H_8$');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
end


if (hypNumber <= 16)
    tempHypNumber = hypNumber;
else
    tempHypNumber = 16;
end
if (ceil(hypNumber/8) >= 2)
figNumber = figNumber+1;
figure(figNumber); set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% iStart = 8*ceil(rooms.room1.numberOfHyp/8)-7;
for i=9:tempHypNumber
    tempNumber = mod(i,maxColors);
    if (tempNumber == 0)
        tempNumber = 8;
    end
    plot(finalEstimateRotatedTranslated(1,strideNumber - ...
        (strideNumber - room1enteringStride - i):strideNumber - ...
        (strideNumber - room1enteringStride - i) + ...
        subsequentStrides,i), finalEstimateRotatedTranslated(2,strideNumber-...
        (strideNumber - room1enteringStride - i):strideNumber-...
        (strideNumber - room1enteringStride - i)+subsequentStrides,i),...
        'g-x','linewidth',1.5,'color',...
        color(tempNumber,:));
end
if (hypNumber < 17)
    showRotated = 1; %hypNumber = 8; % hypNumber can be 1 to 8
end
% showRotated = 1; %hypNumber = rooms.room1.numberOfHyp; % hypNumber can be 9 to 16
if (choice == 1 || choice == 2)
    if (showRotated)
        % automated plotting
        plot(finalEstimateRotated(1,:,hypNumber),...
             finalEstimateRotated(2,:,hypNumber),'r-x','linewidth',1.5);
        % (strideNumber - room1enteringStride - 1) = 350
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),'ks',...
            'linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),'ko',...
            'linewidth',2,'markerfacecolor','y','markersize',16);
%         % manual plotting
%         plot(finalEstimateRotatedhk8(1,1:end),finalEstimateRotatedhk8(2,1:end),'r-x','linewidth',1.5);
%         % 350 is h1, 349 is h2, ...
%         plot(finalEstimateRotatedhk8(1,strideNumber-343),finalEstimateRotatedhk8(2,strideNumber-343),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
%         plot(finalEstimateRotatedhk8(1,strideNumber-344),finalEstimateRotatedhk8(2,strideNumber-344),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
hold off;
legend('$H_9$','$H_{10}$','$H_{11}$','$H_{12}$','$H_{13}$','$H_{14}$',...
    '$H_{15}$','$H_{16}$');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
end

if (hypNumber <= 24)
    tempHypNumber = hypNumber;
else
    tempHypNumber = 24;
end
if (ceil(hypNumber/8) >= 3)
figNumber = figNumber+1;
figure(figNumber); set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% iStart = 8*ceil(rooms.room1.numberOfHyp/8)-7;
for i=17:tempHypNumber
    tempNumber = mod(i,maxColors);
    if (tempNumber == 0)
        tempNumber = 8;
    end
    plot(finalEstimateRotatedTranslated(1,strideNumber - ...
        (strideNumber - room1enteringStride - i):strideNumber - ...
        (strideNumber - room1enteringStride - i) + ...
        subsequentStrides,i), finalEstimateRotatedTranslated(2,strideNumber-...
        (strideNumber - room1enteringStride - i):strideNumber-...
        (strideNumber - room1enteringStride - i)+subsequentStrides,i),...
        'g-x','linewidth',1.5,'color',...
        color(tempNumber,:));
end

if (hypNumber < 25)
    showRotated = 1; %hypNumber = 8; % hypNumber can be 1 to 8
end
if (choice == 1 || choice == 2)
    if (showRotated)
        % automated plotting
        plot(finalEstimateRotated(1,:,hypNumber),...
             finalEstimateRotated(2,:,hypNumber),'r-x','linewidth',1.5);
        % (strideNumber - room1enteringStride - 1) = 350
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),'ks',...
            'linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),'ko',...
            'linewidth',2,'markerfacecolor','y','markersize',12);
%         % manual plotting
%         plot(finalEstimateRotatedhk8(1,1:end),finalEstimateRotatedhk8(2,1:end),'r-x','linewidth',1.5);
%         % 350 is h1, 349 is h2, ...
%         plot(finalEstimateRotatedhk8(1,strideNumber-343),finalEstimateRotatedhk8(2,strideNumber-343),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
%         plot(finalEstimateRotatedhk8(1,strideNumber-344),finalEstimateRotatedhk8(2,strideNumber-344),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
hold off;
legend('$H_{17}$','$H_{18}$','$H_{19}$','$H_{20}$','$H_{21}$',...
    '$H_{22}$','$H_{23}$','$H_{24}$');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
end

if (hypNumber <= 32)
    tempHypNumber = hypNumber;
else
    tempHypNumber = 32;
end
if (ceil(hypNumber/8) >= 4)
figNumber = figNumber+1;
figure(figNumber); set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% iStart = 8*ceil(rooms.room1.numberOfHyp/8)-7;
for i=25:tempHypNumber
    tempNumber = mod(i,maxColors);
    if (tempNumber == 0)
        tempNumber = 8;
    end
    plot(finalEstimateRotatedTranslated(1,strideNumber - ...
        (strideNumber - room1enteringStride - i):strideNumber - ...
        (strideNumber - room1enteringStride - i) + ...
        subsequentStrides,i), finalEstimateRotatedTranslated(2,strideNumber-...
        (strideNumber - room1enteringStride - i):strideNumber-...
        (strideNumber - room1enteringStride - i)+subsequentStrides,i),...
        'g-x','linewidth',1.5,'color',...
        color(tempNumber,:));
end
if (hypNumber < 33)
    showRotated = true;
end
if (choice == 1 || choice == 2)
    if (showRotated)
        % automated plotting
        plot(finalEstimateRotated(1,:,hypNumber),...
             finalEstimateRotated(2,:,hypNumber),'r-x','linewidth',1.5);
        % (strideNumber - room1enteringStride - 1) = 350
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),'ks',...
            'linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),'ko',...
            'linewidth',2,'markerfacecolor','y','markersize',12);
%         % manual plotting
%         plot(finalEstimateRotatedhk8(1,1:end),finalEstimateRotatedhk8(2,1:end),'r-x','linewidth',1.5);
%         % 350 is h1, 349 is h2, ...
%         plot(finalEstimateRotatedhk8(1,strideNumber-343),finalEstimateRotatedhk8(2,strideNumber-343),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
%         plot(finalEstimateRotatedhk8(1,strideNumber-344),finalEstimateRotatedhk8(2,strideNumber-344),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
hold off;
legend('$H_{25}$','$H_{26}$','$H_{27}$','$H_{28}$','$H_{29}$','$H_{30}$',...
    '$H_{31}$','$H_{32}$');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
end

if (hypNumber <= 40)
    tempHypNumber = hypNumber;
else
    tempHypNumber = 40;
end
if (ceil(hypNumber/8) >= 5)
figNumber = figNumber+1;
figure(figNumber); set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% iStart = 8*ceil(rooms.room1.numberOfHyp/8)-7;
for i=33:tempHypNumber
    tempNumber = mod(i,maxColors);
    if (tempNumber == 0)
        tempNumber = 8;
    end
    plot(finalEstimateRotatedTranslated(1,strideNumber - ...
        (strideNumber - room1enteringStride - i):strideNumber - ...
        (strideNumber - room1enteringStride - i) + ...
        subsequentStrides,i), finalEstimateRotatedTranslated(2,strideNumber-...
        (strideNumber - room1enteringStride - i):strideNumber-...
        (strideNumber - room1enteringStride - i)+subsequentStrides,i),...
        'g-x','linewidth',1.5,'color',...
        color(tempNumber,:));
end
if (hypNumber < 41)
    showRotated = 1; %hypNumber = rooms.room1.numberOfHyp; % hypNumber can be 9 to 16
end
if (choice == 1 || choice == 2)
    if (showRotated)
        % automated plotting
        plot(finalEstimateRotated(1,:,hypNumber),...
             finalEstimateRotated(2,:,hypNumber),'r-x','linewidth',1.5);
        % (strideNumber - room1enteringStride - 1) = 350
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),'ks',...
            'linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),'ko',...
            'linewidth',2,'markerfacecolor','y','markersize',12);
%         % manual plotting
%         plot(finalEstimateRotatedhk8(1,1:end),finalEstimateRotatedhk8(2,1:end),'r-x','linewidth',1.5);
%         % 350 is h1, 349 is h2, ...
%         plot(finalEstimateRotatedhk8(1,strideNumber-343),finalEstimateRotatedhk8(2,strideNumber-343),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
%         plot(finalEstimateRotatedhk8(1,strideNumber-344),finalEstimateRotatedhk8(2,strideNumber-344),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
hold off;
legend('$H_{33}$','$H_{34}$','$H_{35}$','$H_{36}$','$H_{37}$',...
       '$H_{38}$','$H_{39}$','$H_{40}$');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
end

if (hypNumber <= 48)
    tempHypNumber = hypNumber;
else
    tempHypNumber = 48;
end
if (ceil(hypNumber/8) >= 6)
figNumber = figNumber+1;
figure(figNumber); set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% iStart = 8*ceil(rooms.room1.numberOfHyp/8)-7;
for i=41:tempHypNumber
    tempNumber = mod(i,maxColors);
    if (tempNumber == 0)
        tempNumber = 8;
    end
    plot(finalEstimateRotatedTranslated(1,strideNumber - ...
        (strideNumber - room1enteringStride - i):strideNumber - ...
        (strideNumber - room1enteringStride - i) + ...
        subsequentStrides,i), finalEstimateRotatedTranslated(2,strideNumber-...
        (strideNumber - room1enteringStride - i):strideNumber-...
        (strideNumber - room1enteringStride - i)+subsequentStrides,i),...
        'g-x','linewidth',1.5,'color',...
        color(tempNumber,:));
end
if (hypNumber < 49)
    showRotated = 1; %hypNumber = rooms.room1.numberOfHyp; % hypNumber can be 9 to 16
end
if (choice == 1 || choice == 2)
    if (showRotated)
        % automated plotting
        plot(finalEstimateRotated(1,:,hypNumber),...
             finalEstimateRotated(2,:,hypNumber),'r-x','linewidth',1.5);
        % (strideNumber - room1enteringStride - 1) = 350
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),'ks',...
            'linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),'ko',...
            'linewidth',2,'markerfacecolor','y','markersize',12);
%         % manual plotting
%         plot(finalEstimateRotatedhk8(1,1:end),finalEstimateRotatedhk8(2,1:end),'r-x','linewidth',1.5);
%         % 350 is h1, 349 is h2, ...
%         plot(finalEstimateRotatedhk8(1,strideNumber-343),finalEstimateRotatedhk8(2,strideNumber-343),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
%         plot(finalEstimateRotatedhk8(1,strideNumber-344),finalEstimateRotatedhk8(2,strideNumber-344),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
hold off;
legend('$H_{41}$','$H_{42}$','$H_{43}$','$H_{44}$','$H_{45}$',...
    '$H_{46}$','$H_{47}$','$H_{48}$');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
end

if (hypNumber <= 56)
    tempHypNumber = hypNumber;
else
    tempHypNumber = 56;
end
if (ceil(hypNumber/8) >= 7)
figNumber = figNumber+1;
figure(figNumber); set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% iStart = 8*ceil(rooms.room1.numberOfHyp/8)-7;
for i=49:tempHypNumber
    tempNumber = mod(i,maxColors);
    if (tempNumber == 0)
        tempNumber = 8;
    end
    plot(finalEstimateRotatedTranslated(1,strideNumber - ...
        (strideNumber - room1enteringStride - i):strideNumber - ...
        (strideNumber - room1enteringStride - i) + ...
        subsequentStrides,i), finalEstimateRotatedTranslated(2,strideNumber-...
        (strideNumber - room1enteringStride - i):strideNumber-...
        (strideNumber - room1enteringStride - i)+subsequentStrides,i),...
        'g-x','linewidth',1.5,'color',...
        color(tempNumber,:));
end
if (hypNumber < 57)
    showRotated = 1; %hypNumber = rooms.room1.numberOfHyp; % hypNumber can be 9 to 16
end
if (choice == 1 || choice == 2)
    if (showRotated)
        % automated plotting
        plot(finalEstimateRotated(1,:,hypNumber),...
             finalEstimateRotated(2,:,hypNumber),'r-x','linewidth',1.5);
        % (strideNumber - room1enteringStride - 1) = 350
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),'ks',...
            'linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),'ko',...
            'linewidth',2,'markerfacecolor','y','markersize',12);
%         % manual plotting
%         plot(finalEstimateRotatedhk8(1,1:end),finalEstimateRotatedhk8(2,1:end),'r-x','linewidth',1.5);
%         % 350 is h1, 349 is h2, ...
%         plot(finalEstimateRotatedhk8(1,strideNumber-343),finalEstimateRotatedhk8(2,strideNumber-343),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
%         plot(finalEstimateRotatedhk8(1,strideNumber-344),finalEstimateRotatedhk8(2,strideNumber-344),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
hold off;
legend('$H_{49}$','$H_{50}$','$H_{51}$','$H_{52}$','$H_{53}$',...
       '$H_{54}$','$H_{55}$','$H_{56}$');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
end

if (hypNumber <= 64)
    tempHypNumber = hypNumber;
else
    tempHypNumber = 64;
end
if (ceil(hypNumber/8) >= 8)
figNumber = figNumber+1;
figure(figNumber); set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% iStart = 8*ceil(rooms.room1.numberOfHyp/8)-7;
for i=57:tempHypNumber
    tempNumber = mod(i,maxColors);
    if (tempNumber == 0)
        tempNumber = 8;
    end
    plot(finalEstimateRotatedTranslated(1,strideNumber - ...
        (strideNumber - room1enteringStride - i):strideNumber - ...
        (strideNumber - room1enteringStride - i) + ...
        subsequentStrides,i), finalEstimateRotatedTranslated(2,strideNumber-...
        (strideNumber - room1enteringStride - i):strideNumber-...
        (strideNumber - room1enteringStride - i)+subsequentStrides,i),...
        'g-x','linewidth',1.5,'color',...
        color(tempNumber,:));
end
if (hypNumber < 65)
    showRotated = 1; %hypNumber = rooms.room1.numberOfHyp; % hypNumber can be 9 to 16
end
if (choice == 1 || choice == 2)
    if (showRotated)
        % automated plotting
        plot(finalEstimateRotated(1,:,hypNumber),...
             finalEstimateRotated(2,:,hypNumber),'r-x','linewidth',1.5);
        % (strideNumber - room1enteringStride - 1) = 350
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber),hypNumber),'ks',...
            'linewidth',2,'markerfacecolor','y','markersize',16);
        plot(finalEstimateRotated(1,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),...
            finalEstimateRotated(2,strideNumber-(strideNumber - ...
            room1enteringStride - hypNumber + 1),hypNumber),'ko',...
            'linewidth',2,'markerfacecolor','y','markersize',12);
%         % manual plotting
%         plot(finalEstimateRotatedhk8(1,1:end),finalEstimateRotatedhk8(2,1:end),'r-x','linewidth',1.5);
%         % 350 is h1, 349 is h2, ...
%         plot(finalEstimateRotatedhk8(1,strideNumber-343),finalEstimateRotatedhk8(2,strideNumber-343),'ks','linewidth',2,'markerfacecolor','y','markersize',16);
%         plot(finalEstimateRotatedhk8(1,strideNumber-344),finalEstimateRotatedhk8(2,strideNumber-344),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
        % we can take stride k-2 to calculate the heading of the current hypothesis
        %plot(finalEstimateRotatedhkplus3(1,strideNumber-16),finalEstimateRotatedhkplus3(2,strideNumber-16),'ko','linewidth',2,'markerfacecolor','y','markersize',16);
    end
else (choice == 3)
%plot(finalEstimateRotated(1,1:end),finalEstimateRotated(2,1:end),'r-x','linewidth',1.5);
end
hold off;
legend('$H_{57}$','$H_{58}$','$H_{59}$','$H_{60}$','$H_{61}$',...
       '$H_{62}$','$H_{63}$','$H_{64}$');
set(legend,'fontsize',16,'interpreter','latex','location','northwest');
end

figNumber = figNumber + 1;
% rasterized map score control
figure(figNumber); set(gcf,'position',[309    60   943   669]);
imshow(staticFloorPDF{choice},'colormap',jet(255));
colormap jet;
set(gca, 'YDir', 'reverse');
hold on;
% for i=9:rooms.room1.numberOfHyp % i starts from 1 or 9 or 17 ...
tempNumber = mod(hypNumber,maxColors);
if (tempNumber == 0)
    tempNumber = 8;
end
plot(finalEstimateRotatedTranslated(1,strideNumber - ...
    (strideNumber - room1enteringStride - hypNumber):strideNumber - ...
    (strideNumber - room1enteringStride - hypNumber) + ...
    subsequentStrides,hypNumber), finalEstimateRotatedTranslated(2,strideNumber-...
    (strideNumber - room1enteringStride - hypNumber):strideNumber-...
    (strideNumber - room1enteringStride - hypNumber)+...
    subsequentStrides,hypNumber),'g-x','linewidth',1.5,'color',...
    color(tempNumber,:));
% end
legend(['$h_{',num2str(hypNumber),'}$']);
set(legend,'interpreter','latex','fontsize',20,'location','northeast');
hold off;

clear img maxColors finalEstimateRotatedTranslated strideNumber
clear room1enteringStride subsequentStrides color hypNumber
clear finalEstimateRotated choice

lastFigNumber = figNumber;