function [scan,robot,wall] = raycast(map, x, y , theta)
    % Parameters
    ang_incr = 1 * pi/180;
    scan_ang = 180 * pi/180;
    resolution=0.01;
    max_range=0.25;
   
    
    number_rays=ceil(scan_ang/ang_incr)+1;
    scan=NaN(1,number_rays);
    
    ang_start = theta + scan_ang/2;
    ang_end = theta - scan_ang/2;
    robot=[ceil(round(x,2)*100),ceil(round(y,2)*100)];
    wall(1,2)=NaN;
    
    for i=1:number_rays
        dist=resolution;
        ang=ang_start+(i-1)*ang_incr;
      %  if (size(wall,1)==112)
      %     debug=1;
      %  end
        while (true)
            ind_x=round((x+dist*cos(ang))*100);
            ind_y=round((y+dist*sin(ang))*100);
            if (dist > max_range)
                scan(i)=NaN;
                break;
            end
            if (map(ceil(ind_x),ceil(ind_y))==1)
                scan(i)=dist;
                wall(end+1,1)=ceil(ind_x);
                wall(end,2)=ceil(ind_y);
                break;
            end
            dist=dist+resolution;
        end
    end
end