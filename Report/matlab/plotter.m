load('odom_ekf.mat');

figure
plot(ekf(1:end/2,1),ekf(1:end/2,2));
hold on
plot(odom(1:end/2,1),odom(1:end/2,2));
title('Comaprison EKF vs. Odometry')
xlabel('x - Coordinate');
ylabel('y - Coordinate');
for i=1:500:size(ekf,1)/2
    ellipse_data=ellipse(ekf(i,1),ekf(i,2),ekf_cov((i-1)*3+1:(i-1)*3+3,1:3));
    plot(ellipse_data(:,1),ellipse_data(:,2),'b')
end

legend('EKF','Odometry','Covariance');
axis equal;
%matlab2tikz('ekf_odom.tikz', 'height', '\figureheight', 'width', '\figurewidth');