% car time planning on mesh
close all
clear all

%% basic node structure
prob.arc_xs = [110 9 -9 -50; % car 1
              -50 -6 0  0]'; % car 2

prob.arc_ys = [0  0 0 0;  % car 1
               3 3 -3 -50]'; % car 2

prob.arc_hdgs = [pi pi pi pi; % ACW from +x
                0 0 -pi/2 -pi/2]';

%% extract sizes
prob.n_cars = size(prob.arc_xs,2);
prob.n_arcs = size(prob.arc_xs,1)-1;

%% speeds

% max throughout
prob.v_max = 8.94*ones(prob.n_arcs,prob.n_cars); % 20mph

% initial
prob.v_init = 0.95*prob.v_max(1,:);

% lateral accel
prob.lat_acc = 0.25*9.81;

%% arc geometry
prob = getGeometry(prob);

%% initial guess
prob = formInitGuess(prob);

%% linear constraints

% default linear constraints
prob.Aeq = [];
prob.beq = [];
prob.A = [];
prob.b = [];

%% conflict avoidance

% constraints for 2 to go after 1
prob.A_after = [1 1 1 1 0 0 0 0 -1 -1 -1 0 0 0 0 0];
prob.b_after = [0];

% constraints for 2 to go before 1
prob.A_before = [-1 -1 -1  0 0 0 0 0 1 1 1 1 0 0 0 0];
prob.b_before = [-2];
           
%% solve

options = optimoptions('fmincon');
options = optimoptions(options,'GradObj','on');

tic
[x_opt,j_opt,flag] = fmincon(@(x)car_cost(x,prob),prob.x0,[],[],prob.Aeq,prob.beq,prob.x_lb,prob.x_ub,@(x)car_cons(x,prob),options)
%[x_opt,j_opt,flag] = fmincon(@(x)car_cost(x,prob),prob.x0,prob.A_before,prob.b_before,prob.Aeq,prob.beq,prob.x_lb,prob.x_ub,@(x)car_cons(x,prob),options)
if prob.A_before*x_opt>prob.b_before,
    [x_opt,j_opt,flag] = fmincon(@(x)car_cost(x,prob),prob.x0,prob.A_after,prob.b_after,prob.Aeq,prob.beq,prob.x_lb,prob.x_ub,@(x)car_cons(x,prob),options)
end
t_solve = toc

% convert back to times and speeds
[dts_opt,vs_opt] = get_vars(x_opt,prob);
% and to displacements
[ell_opt,t_opt,vss_opt,ss_opt] = get_dists(dts_opt,vs_opt,prob);

%% animation
figure

% animate
t_max = max(max(t_opt));
t_anim = linspace(0.001,0.999*t_max,100);
dt = t_max/100;
traj_store = [];
for tt=t_anim,
    for cc=1:prob.n_cars,
        [t_int,i_int,~] = unique(t_opt(:,cc));
        ss_int = ss_opt(i_int,cc);
        s = interp1(t_int,ss_int,tt);
        % catch off-the-end NaN case
        if isnan(s),
            s = 0.9999*prob.n_arcs;
        end
        %arc = floor(s);
        %s = s - arc;
        %x(cc) = prob.arc_xs(arc+1,cc)*(1-s) + prob.arc_xs(arc+2,cc)*(s);
        %y(cc) = prob.arc_ys(arc+1,cc)*(1-s) + prob.arc_ys(arc+2,cc)*(s);
        % try interpolating again on to the finer geometry
        [s_int,i_int,~] = unique(prob.arc_sp(:,cc));
        sp_int = prob.arc_sp(i_int,cc);
        xp_int = prob.arc_xp(i_int,cc);
        yp_int = prob.arc_yp(i_int,cc);
        hp_int = prob.arc_hp(i_int,cc);
        x(cc) = interp1(sp_int,xp_int,s);
        y(cc) = interp1(sp_int,yp_int,s);
        h(cc) = interp1(sp_int,hp_int,s);
    end
    plot(prob.arc_xs,prob.arc_ys,'x',prob.arc_xp,prob.arc_yp,'--',x,y,'s')
    title(sprintf('Time: %.2f',tt))
    axis equal
    pause(0.1*dt)
    traj_store = [traj_store; [tt x(1) y(1) 0 h(1) x(2) y(2) 0 h(2)]];
end

%% save for use in ROS
file_name = 'turnin.csv';
fid = fopen(file_name,'w');

% make header row
fprintf(fid,'time_from_start,robot1_move_x,robot1_move_y,robot1_move_z,robot1_turn_z,robot2_move_x,robot2_move_y,robot2_move_z,robot2_turn_z\n');

% store the trajectory
fprintf(fid,'%f,%f,%f,%f,%f,%f,%f,%f,%f\n',traj_store');

fclose(fid);

%% plotting

figure
subplot 311
plot(t_opt,ell_opt,'x-')
grid on
subplot 312
plot(t_opt,ss_opt,'x-')
grid on
subplot 313
plot(t_opt,vss_opt,'x-',t_opt,0*t_opt+prob.v_max(1),'r')
grid on

