% car time planning on mesh
close all
clear all

tic

%% basic node structure
prob.arc_xs = [40 9 -9 -50; % car 1
    0  0 -9 -50]'; % car 2

prob.arc_ys = [0  0 0 0;  % car 1
    -40 -9 0 0]'; % car 2

prob.arc_hdgs = [pi pi pi pi; % ACW from +x
    pi/2 pi/2 pi pi]';

%% crossing
prob.crossings = [1 2 2 2]; % arc 2 for car 1 cross arc 2 for car 2

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

%% AMPL data file

c = 0;
fid=fopen('car.dat','w');

c = c + AMPLscalarint(fid,'Ncars',prob.n_cars);
c = c + AMPLscalarint(fid,'Narcs',prob.n_arcs);

c = c + AMPLvector(fid,'Vinit',prob.v_init);
c = c + AMPLmatrix(fid,'Vmax',prob.v_max');
c = c + AMPLmatrix(fid,'Larcs',prob.ell_arcs');

c = c + AMPLscalarint(fid,'Ncross',size(prob.crossings,1));
c = c + AMPLmatrixint(fid,'Dcross',prob.crossings);

% completed writing file
fclose(fid);
sprintf('%d bytes written',c)

%% solve
!ampl car.run
toc

%% load
dts_opt = load('Ts.dat');
vs_opt = load('Vs.dat');
% and shape
dts_opt = reshape(dts_opt',2*prob.n_arcs,prob.n_cars);
vs_opt = reshape(vs_opt,prob.n_arcs+1,prob.n_cars);

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
file_name = 'turnout.csv';
fid = fopen(file_name,'w');

% make header row
fprintf(fid,'time_from_start,robot1_move_x,robot1_move_y,robot1_move_z,robot1_turn_z,robot2_move_x,robot2_move_y,robot2_move_z,robot2_turn_z,robot3_move_x,robot3_move_y,robot3_move_z,robot3_turn_z\n');

% store the trajectory
fprintf(fid,'%f,%f,%f,%f,%f,%f,%f,%f,%f,3,-9,0,-1.57\n',traj_store');

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

