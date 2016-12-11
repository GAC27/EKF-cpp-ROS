function [ellipse]=ellipse(x,y,cov)
covariance = cov(1:2,1:2);
[eigenvec, eigenval ] = eig(covariance);
[largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);
largest_eigenval = max(max(eigenval));
if(largest_eigenvec_ind_c == 1)
    smallest_eigenval = max(eigenval(:,2));
else
    smallest_eigenval = max(eigenval(:,1));
end
angle = atan2(largest_eigenvec(2), largest_eigenvec(1));
if(angle < 0)
    angle = angle + 2*pi;
end

chisquare_val = 2.4477;
theta_grid = linspace(0,2*pi);
phi = angle;
X0=x;
Y0=y;
a=chisquare_val*sqrt(largest_eigenval);
b=chisquare_val*sqrt(smallest_eigenval); 
ellipse_x_r  = a*cos( theta_grid );
ellipse_y_r  = b*sin( theta_grid );
R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;
ellipse(:,1)=r_ellipse(:,1) + X0;
ellipse(:,2)=r_ellipse(:,2) + Y0;