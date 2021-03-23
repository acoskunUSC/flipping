clc;
fileID = fopen('results.txt','r');
formatSpec = '%f';
results = fscanf(fileID, formatSpec);

n = 4; % 4 cases (5 10 15 20)
numberOfInstances = 20;


count = zeros(n);

for j=1:n  
  for i=1:numberOfInstances      
    if (results(2+(i-1)*13+(j-1)*13*numberOfInstances) != -1 && results(6+(i-1)*13+(j-1)*13*numberOfInstances) != -1 && results(10+(i-1)*13+(j-1)*13*numberOfInstances) != -1)      
        count(j)++;                
    endif         
  end  
end

numberOfSuccessfulInstances = numberOfInstances; # numberOfSuccessfulInstances is the minimum of successful runs among all cases
for  k=1:n
  if (count(k) <= numberOfSuccessfulInstances) 
    numberOfSuccessfulInstances = count(k);
  endif
end

meanOfTravelTimes_RMTRACK = (1:n)';
meanOfTravelTimes_TFF = (1:n)';
meanOfTravelTimes_TFA = (1:n)';

meanOfCompTimes_TFF = (1:n)';
meanOfCompTimes_TFA = (1:n)';

meanOfNumberOfFlips_TFF = (1:n)';
meanOfNumberOfFlips_TFA = (1:n)';

successRate_RMTRACK = (1:n)';
successRate_TFF = (1:n)';
successRate_TFA = (1:n)';

for j=1:n
  
  sumOfTravelTimes_RMTRACK = 0;
  sumOfTravelTimes_TFF = 0;
  sumOfTravelTimes_TFA = 0;
  
  sumOfCompTimes_TFF = 0;
  sumOfCompTimes_TFA = 0;
  
  sumOfNumberOfFlips_TFF = 0;
  sumOfNumberOfFlips_TFA = 0;
  
  numberOfDeadlocks_RMTRACK = 0;
  numberOfDeadlocks_TFF = 0;
  numberOfDeadlocks_TFA = 0;
  
  count = 0;
  
  for i=1:numberOfInstances  
    
    if (results(2+(i-1)*13+(j-1)*13*numberOfInstances) == -1)
      numberOfDeadlocks_RMTRACK++;
    endif
    
    if (results(6+(i-1)*13+(j-1)*13*numberOfInstances) == -1) 
      numberOfDeadlocks_TFF++;
    endif
    
    if (results(10+(i-1)*13+(j-1)*13*numberOfInstances) == -1) 
      numberOfDeadlocks_TFA++;
    endif
    
    if (results(2+(i-1)*13+(j-1)*13*numberOfInstances) != -1 && results(6+(i-1)*13+(j-1)*13*numberOfInstances) != -1 && results(10+(i-1)*13+(j-1)*13*numberOfInstances) != -1 && count < numberOfSuccessfulInstances)
      
      count++;          
      
      sumOfTravelTimes_RMTRACK += results(2+(i-1)*13+(j-1)*13*numberOfInstances);
      
      sumOfTravelTimes_TFF += results(6+(i-1)*13+(j-1)*13*numberOfInstances);
      sumOfNumberOfFlips_TFF += results(8+(i-1)*13+(j-1)*13*numberOfInstances);
      sumOfCompTimes_TFF += results(9+(i-1)*13+(j-1)*13*numberOfInstances);          
      
      sumOfTravelTimes_TFA += results(10+(i-1)*13+(j-1)*13*numberOfInstances);
      sumOfNumberOfFlips_TFA += results(12+(i-1)*13+(j-1)*13*numberOfInstances);
      sumOfCompTimes_TFA += results(13+(i-1)*13+(j-1)*13*numberOfInstances);
      
    endif       
    
  end
  
  meanOfTravelTimes_RMTRACK(j) = sumOfTravelTimes_RMTRACK/count;
  meanOfTravelTimes_TFF(j) = sumOfTravelTimes_TFF/count;
  meanOfTravelTimes_TFA(j) = sumOfTravelTimes_TFA/count;
  
  meanOfCompTimes_TFF(j) = sumOfCompTimes_TFF/count;
  meanOfCompTimes_TFA(j) = sumOfCompTimes_TFA/count;
  
  meanOfNumberOfFlips_TFF(j) = sumOfNumberOfFlips_TFF/count;
  meanOfNumberOfFlips_TFA(j) = sumOfNumberOfFlips_TFA/count;
  
  successRate_RMTRACK(j) = (numberOfInstances - numberOfDeadlocks_RMTRACK) * (100 / numberOfInstances);
  successRate_TFF(j) = (numberOfInstances - numberOfDeadlocks_TFF) * (100 / numberOfInstances);
  successRate_TFA(j) = (numberOfInstances - numberOfDeadlocks_TFA) * (100 / numberOfInstances);
    
end

varianceOfTravelTimes_RMTRACK = (1:n);
varianceOfTravelTimes_TFF = (1:n);
varianceOfTravelTimes_TFA = (1:n);

for j=1:n
  
  sumOfDifferenceOfMeanOfTravelTimes_RMTRACK = 0;
  sumOfDifferenceOfMeanOfTravelTimes_TFF = 0;
  sumOfDifferenceOfMeanOfTravelTimes_TFA = 0;
  
  count = 0;
  
  for i=1:numberOfInstances  
        
    if (results(2+(i-1)*13+(j-1)*13*numberOfInstances) != -1 && results(6+(i-1)*13+(j-1)*13*numberOfInstances) != -1 && results(10+(i-1)*13+(j-1)*13*numberOfInstances) != -1 && count < numberOfSuccessfulInstances)      
      count++;      
      sumOfDifferenceOfMeanOfTravelTimes_RMTRACK += (meanOfTravelTimes_RMTRACK(j) - results(2+(i-1)*13+(j-1)*13*numberOfInstances)) ^ 2;
      sumOfDifferenceOfMeanOfTravelTimes_TFF += (meanOfTravelTimes_TFF(j) - results(6+(i-1)*13+(j-1)*13*numberOfInstances)) ^ 2;
      sumOfDifferenceOfMeanOfTravelTimes_TFA += (meanOfTravelTimes_TFA(j)- results(10+(i-1)*13+(j-1)*13*numberOfInstances)) ^ 2;      
    endif
    
  end
  
  varianceOfTravelTimes_RMTRACK(j) = sumOfDifferenceOfMeanOfTravelTimes_RMTRACK / (count - 1);
  varianceOfTravelTimes_TFF(j) = sumOfDifferenceOfMeanOfTravelTimes_TFF / (count - 1);
  varianceOfTravelTimes_TFA(j) = sumOfDifferenceOfMeanOfTravelTimes_TFA / (count - 1);
  
end


grid on;
fontSize = 24;
xlim([5 n*5]); 
xticks([5 10 15 20]);
D = (5:5:n*5)';
hold on;
p1 = plot(D,meanOfTravelTimes_RMTRACK);
p2 = plot(D,meanOfTravelTimes_TFF);
p3 = plot(D,meanOfTravelTimes_TFA);
set(p1, 'LineWidth', 6, 'Color', [255, 0, 0]/255);
set(p2, 'LineWidth', 6, 'Color', [0, 0, 255]/255, 'Linestyle', '--');
set(p3, 'LineWidth', 6, 'Color', [0, 255, 0]/255, 'Linestyle', ':');
set(gca, "linewidth", 4, "fontsize", 24);
xlabel('number of robots', "fontsize", 32);
ylabel('average travel time (sec)', "fontsize", 30);
title('average travel time (sec)', "fontsize", 36, "fontweight", "normal");
lgd=legend('RMTRACK', 'RMTRACK+TFF', 'RMTRACK+TFA');
legend (lgd, "location", "northwest");
set(lgd, "fontSize", 28);
print -depsc plots/plot1.eps;

figure;
grid on;
hold on;
xlim([5 5*n]); 
xticks([5 10 15 20]);
p4 = plot(D,meanOfCompTimes_TFF);
p5 = plot(D,meanOfCompTimes_TFA);
set(p4, 'LineWidth', 6, 'Color', [0, 0, 255]/255, 'Linestyle', '--');
set(p5, 'LineWidth', 6, 'Color', [0, 255, 0]/255, 'Linestyle', ':');
set(gca, "linewidth", 4, "fontsize", 24);
xlabel('number of robots', "fontsize", 32);
ylabel('average comp. time for flipping (sec)', "fontsize", 20);
title('average comp. time for flipping (sec)', "fontsize", 34, "fontweight", "normal");
lgd=legend('RMTRACK+TFF','RMTRACK+TFA');
legend (lgd, "location", "northwest");
set(lgd, "fontSize", 28);
print -depsc plots/plot2.eps;

figure;
grid on;
hold on;
xlim([5 5*n]); 
xticks([5 10 15 20]);
p6 = plot(D,meanOfNumberOfFlips_TFF);
p7 = plot(D,meanOfNumberOfFlips_TFA);
set(p6, 'LineWidth', 6, 'Color', [0, 0, 255]/255, 'Linestyle', '--');
set(p7, 'LineWidth', 6, 'Color', [0, 255, 0]/255, 'Linestyle', ':');
set(gca, "linewidth", 4, "fontsize", 24);
xlabel('number of robots', "fontsize", 32);
ylabel('average number of flips executed', "fontsize", 24);
title('average number of flips executed', "fontsize", 34, "fontweight", "normal");
lgd=legend('RMTRACK+TFF','RMTRACK+TFA');
legend (lgd, "location", "northwest");
set(lgd, "fontSize", 28);
print -depsc plots/plot3.eps;

figure;
grid on;
hold on;
xlim([5 5*n]); 
xticks([5 10 15 20]);
ylim([0 100]);
p8 = plot(D,successRate_RMTRACK);
p9 = plot(D,successRate_TFF);
p10 = plot(D,successRate_TFA);
set(p8, 'LineWidth', 8, 'Color', [255, 0, 0]/255);
set(p9, 'LineWidth', 8, 'Color', [0, 0, 255]/255, 'Linestyle', '--');
set(p10, 'LineWidth', 8, 'Color', [0, 255, 0]/255, 'Linestyle', ':');
set(gca, "linewidth", 4, "fontsize", 24);
xlabel('number of robots', "fontsize", 32);
ylabel('success rate [%]', "fontsize", 30);
title('success rate [%]', "fontsize", 36, "fontweight", "normal");
lgd=legend('RMTRACK', 'RMTRACK+TFF', 'RMTRACK+TFA');
legend (lgd, "location", "southwest");
set(lgd, "fontSize", 28);
print -depsc plots/plot4.eps;

figure;
grid on;
hold on;
xlim([5 5*n]); 
xticks([5 10 15 20]);
p11 = plot(D, sqrt(varianceOfTravelTimes_RMTRACK));
p12 = plot(D, sqrt(varianceOfTravelTimes_TFF));
p13 = plot(D, sqrt(varianceOfTravelTimes_TFA));
set(p11, 'LineWidth', 6, 'Color', [255, 0, 0]/255);
set(p12, 'LineWidth', 6, 'Color', [0, 0, 255]/255, 'Linestyle', '--');
set(p13, 'LineWidth', 6, 'Color', [0, 255, 0]/255, 'Linestyle', ':');
set(gca, "linewidth", 4, "fontsize", 24);
xlabel('number of robots','FontSize', 32);
ylabel('standard deviation of travel time (sec)','FontSize', 20);
title('standard deviation of travel time (sec)','FontSize',32,'FontWeight','normal');
lgd=legend('RMTRACK', 'RMTRACK+TFF', 'RMTRACK+TFA');
legend (lgd, "location", "northwest");
set(lgd, "fontSize", 28);
print -depsc plots/plot5.eps;

figure;
grid on;
hold on;
xlim([0 5*(n+1)]); 
xticks([5 10 15 20]);
h = zeros(1,4);
h(1) = errorbar(D,meanOfTravelTimes_RMTRACK,sqrt(varianceOfTravelTimes_RMTRACK), '.k');
h(2) = errorbar(D,meanOfTravelTimes_TFF,sqrt(varianceOfTravelTimes_TFF), '.k');
set(h(1), 'LineWidth', 2);
set(h(2), 'LineWidth', 2);
h(3) = plot(D,meanOfTravelTimes_RMTRACK);
h(4) = plot(D,meanOfTravelTimes_TFF);
set(h(3), 'LineWidth', 6, 'Color', [255, 0, 0]/255);
set(h(4), 'LineWidth', 6, 'Color', [0, 0, 255]/255);
set(gca, "linewidth", 4, "fontsize", 24);
xlabel('number of robots','FontSize', 32);
ylabel('mean and standard deviation of travel time','FontSize', 20);
title('mean and standard deviation of travel time (sec)','FontSize',30,'FontWeight','normal');
lgd=legend(h(3:4),'RMTRACK', 'RMTRACK+TFF');
legend (lgd, "location", "northwest");
set(lgd, "fontSize", 28);
print -depsc plots/plot6.eps;

figure;
grid on;
hold on;
xlim([0 5*(n+1)]); 
xticks([5 10 15 20]);
h = zeros(1,4);
h(1) = errorbar(D,meanOfTravelTimes_RMTRACK,sqrt(varianceOfTravelTimes_RMTRACK), '.k');
h(2) = errorbar(D,meanOfTravelTimes_TFA,sqrt(varianceOfTravelTimes_TFF), '.k');
set(h(1), 'LineWidth', 2);
set(h(2), 'LineWidth', 2);
h(3) = plot(D,meanOfTravelTimes_RMTRACK);
h(4) = plot(D,meanOfTravelTimes_TFA);
set(h(3), 'LineWidth', 6, 'Color', [255, 0, 0]/255);
set(h(4), 'LineWidth', 6, 'Color', [0, 255, 0]/255);
set(gca, "linewidth", 4, "fontsize", 24);
xlabel('number of robots','FontSize', 32);
ylabel('mean and standard deviation of travel time','FontSize', 20);
title('mean and standard deviation of travel time (sec)','FontSize',30,'FontWeight','normal');
lgd=legend(h(3:4),'RMTRACK', 'RMTRACK+TFA');
legend (lgd, "location", "northwest");
set(lgd, "fontSize", 28);
print -depsc plots/plot7.eps;
