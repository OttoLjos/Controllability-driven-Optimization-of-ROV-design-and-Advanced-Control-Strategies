function T = Thrust_config_matrix(Thrusters, COB, plot_tot, plot_Dekomp, plot_ga)
COG = COB;
T=zeros(6,length(Thrusters));
    for i = 1:length(Thrusters)
    Thruster = Thrusters(i);
    Thruster = Thruster{1};
    t = Thrust_mixing_vect(Thruster(1:3), Thruster(4:6), COG, plot_Dekomp, plot_ga);
    T(:,i)=t;
    end

    if plot_tot
        scale = 2/sum(sum(abs(T(1:3, :))));
        t1 = quiver3(COG(1), COG(2), COG(3), sum(abs(T(1, :)))*scale, 0, 0,'r', 'LineWidth', 2);
        t2 = quiver3(COG(1), COG(2), COG(3), 0, -sum(abs(T(2, :)))*scale, 0,'b', 'LineWidth', 2);
        t3 = quiver3(COG(1), COG(2), COG(3), 0, 0, -sum(abs(T(3, :)))*scale,'g', 'LineWidth', 2);
        legend([t1, t2, t3], 'Surge', 'Sway', 'Heave');
    end
end    

function t = Thrust_mixing_vect(position, orientation, COG, plot_Dekomp, plot_ga)
    x0=position(1)-COG(1);
    y0=position(2)-COG(2);
    z0=position(3)-COG(3);
    dx=orientation(1)/norm(orientation);
    dy=orientation(2)/norm(orientation);
    dz=orientation(3)/norm(orientation);

    F_t_roll = [0, dy, dz];
    Position_roll = [0, y0, z0];

    if norm(Position_roll) == 0
        Force_roll = 0;
        orth_roll = [0,0,0];
    else
        proj_roll = (dot(F_t_roll, Position_roll) / norm(Position_roll)^2) * Position_roll;
        orth_roll = F_t_roll - proj_roll;
        Force_roll = y0*orth_roll(3)-z0*orth_roll(2);
    end
  
    F_t_stamp = [dx, 0, dz];
    Position_stamp = [x0, 0, z0];

    if norm(Position_stamp) == 0
        Force_stamp = 0;
        orth_stamp = [0,0,0];
    else
        proj_stamp = (dot(F_t_stamp, Position_stamp) / norm(Position_stamp)^2) * Position_stamp;
        orth_stamp = F_t_stamp - proj_stamp;
        Force_stamp = x0*orth_stamp(3)-z0*orth_stamp(1);
    end

    % Calculate the rotational force yaw
    F_t_yaw = [dx, dy, 0];
    Position_yaw = [x0, y0, 0];

    if norm(Position_yaw) == 0
        Force_yaw = 0;
        orth_yaw = [0,0,0];
    else
        proj_yaw = (dot(F_t_yaw, Position_yaw) / norm(Position_yaw)^2) * Position_yaw;
        orth_yaw = F_t_yaw - proj_yaw;
        Force_yaw = y0*orth_yaw(1)-x0*orth_yaw(2);
    end
    
    Force_roll = y0*dz-z0*dy;
    Force_stamp = x0*dz-z0*dx;
    Force_yaw = y0*dx-x0*dy;

    if plot_Dekomp
        h1 = quiver3(position(1), position(2), position(3),orth_roll(1), orth_roll(2), orth_roll(3),'g', 'LineWidth', 4);
        h2 = quiver3(position(1), position(2), position(3),orth_stamp(1), orth_stamp(2), orth_stamp(3),'b', 'LineWidth', 3);
        h3 = quiver3(position(1), position(2), position(3),orth_yaw(1), orth_yaw(2), orth_yaw(3),'r', 'LineWidth', 2);
        legend([h1, h2, h3], 'Roll', 'Pitch', 'Yaw');
        quiver3(position(1), position(2), position(3), dx, dy, dz,'k', 'LineWidth', 1)
    end

    if plot_ga
        quiver3(position(1), position(2), position(3), dx/8, dy/8, dz/8,'k', 'LineWidth', 3,'MarkerSize', 4)
        quiver3(position(1), position(2), position(3), -dx/8, -dy/8, -dz/8,'k', 'LineWidth', 3,'MarkerSize', 4)
        plot3(position(1), position(2), position(3), 'go', 'MarkerSize', 2,'LineWidth', 2);
    end
    
    t = [dx; -dy; -dz; Force_roll; Force_stamp; Force_yaw];
end