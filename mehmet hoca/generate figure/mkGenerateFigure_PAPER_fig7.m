%%%bolz 30 pixel initial hypotheses
load('img.mat')
load('1_58_30pixel_probabilities.mat')
%load('mk60QuantBolz20px.mat')
load('mk60QuantBolz30px.mat')
%load('mk60QuantBolz40px.mat')
%load('mk60QuantBolz60px.mat')
figure(1); set(figure(1),'position',[1 59 1114 671],'papertype','a3');
set(gcf,'position',[564 61 927 670]);
imshow(img); hold on;
% plot(data(1,:), abs(data(2,:)), '.')
plot(data(1,1),data(2,1),'ro','markersize',10,'linewidth',2);

j=30; %hangi stride icin olasilik degerlerine bakilacaksa...
probabiliets_at_stride = [];
normalized_prob = [];

%%% j stride daki butun hipotez olas?l?k degerlerini normalize etme
for i=1:length(quantize)
    probabiliets_at_stride(i,1) = quantize(i).log(1,j);    
end
normalized_prob2 = normalize_var(probabiliets_at_stride,1,10);


%%%olasilik degerine gore circle cizme---normalize
r=10;
for i=1:1:length(data)
    viscircles([data(1,i),abs(data(2,i))],normalized_prob2(i,1))
end
% legend('normalized hypothesis probabilities at stride 30');
% set(legend,'fontsize',22,'interpreter','latex','location','northeast');
hold off;
print('bolz30pixelstride50','-dpdf','-opengl');