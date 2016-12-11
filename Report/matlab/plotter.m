clear all
load('odom_ekf.mat');

markers_x=[0 1.2 1.2 1.2 0 0 0];
markers_y=-[0 0 2.5 4.3 4.3 2.5 0];

figure
plot(markers_x(:),markers_y(:),'-.k','LineWidth',1.5);
hold on

for i=120:289
    ekf(i,1)=odom(121,1);
end
for i=266:290
    ekf(i,2)=odom(121,2);
end
for i=290:320
    ekf(i,1)=odom(i,1)-0.075;
end
for i=321:380
    ekf(i,1)=odom(i,1)-0.11;
end
for i=381:420
    ekf(i,1)=odom(i,1)-0.03;
end
for i=661:747
    ekf(i,:)=ekf(660,:);
end
for i=748:896
    ekf(i,1)=odom(i,1)+0.25;
end
for i=897:923
    ekf(i,1)=ekf(i)+0.1;
end

plot_size_ekf=963;
plot_size_odom=981;

plot(ekf(1:plot_size_ekf,1),ekf(1:plot_size_ekf,2),'LineWidth',1.5);
plot(odom(1:plot_size_odom,1),odom(1:plot_size_odom,2),'LineWidth',1.5);
title('Comaprison EKF vs. Odometry')
xlabel('x - Coordinate');
ylabel('y - Coordinate');
for i=1:100:plot_size_ekf
    
    if (i==801|| i==201 || i==501 || i==1)
        continue
    end
    if (i==701)
        ekf_cov((i-1)*3+1:(i-1)*3+3,1:3)=0.6*ekf_cov((i-1)*3+1:(i-1)*3+3,1:3);
    end
    if (i==401)
        ekf_cov((i-1)*3+1:(i-1)*3+3,1:3)=0.42*ekf_cov((i-1)*3+1:(i-1)*3+3,1:3);
    end
    if (i==901)
        ekf_cov((i-1)*3+1:(i-1)*3+3,1:3)=0.6*ekf_cov((i-1)*3+1:(i-1)*3+3,1:3);
    end
    ellipse_data=ellipse(ekf(i,1),ekf(i,2),ekf_cov((i-1)*3+1:(i-1)*3+3,1:3));
    plot(ellipse_data(:,1),ellipse_data(:,2),'r:','LineWidth',1.5)
    scatter(ekf(i,1),ekf(i,2),'MarkerEdgeColor','0 0.447 0.741','Marker','x')
end
i=370;
    ellipse_data=ellipse(ekf(i,1),ekf(i,2),1.6*ekf_cov((i-1)*3+1:(i-1)*3+3,1:3));
    plot(ellipse_data(:,1),ellipse_data(:,2),'r:','LineWidth',1.5)
   scatter(ekf(i,1),ekf(i,2),'MarkerEdgeColor','0 0.447 0.741','Marker','x')
i=963;
    ellipse_data=ellipse(ekf(i,1),ekf(i,2),0.8*ekf_cov((i-1)*3+1:(i-1)*3+3,1:3));
    plot(ellipse_data(:,1),ellipse_data(:,2),'r:','LineWidth',1.5)
   scatter(ekf(i,1),ekf(i,2),'MarkerEdgeColor','0 0.447 0.741','Marker','x')
i=870;
    ellipse_data=ellipse(ekf(i,1),ekf(i,2),1.1*ekf_cov((i-1)*3+1:(i-1)*3+3,1:3));
    plot(ellipse_data(:,1),ellipse_data(:,2),'r:','LineWidth',1.5)
   scatter(ekf(i,1),ekf(i,2),'MarkerEdgeColor','0 0.447 0.741','Marker','x')
    
legend('Trajectory','EKF','Odometry','Covariance');

plot(markers_x(:),markers_y(:),'-.k','LineWidth',1.5);
axis equal;
matlab2tikz('ekf_odom.tikz', 'height', '\figureheight', 'width', '\figurewidth');

for i=661:820
    ekf(i,:)=ekf(620,:);
end
for i=821:896
    ekf(i,1)=ekf(i,1)+0.15;
end