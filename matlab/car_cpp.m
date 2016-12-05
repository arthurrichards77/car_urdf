% car time planning on mesh
close all
clear all

tic

%% basic node structure
prob.arc_xs = [60 9 -9 -50; % car 1
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

% settings for speed constraint generation
prob.num_spd_pts = 7;

%% arc geometry
prob = getGeometry(prob);

%% initial guess
prob = formInitGuess(prob);

% constraints for 2 to go after 1
prob.A = [1 1 1 1 0 0 0 0 -1 -1 -1 0 0 0 0 0];
prob.b = [0];
           
%% use init guess to form bounds vectors

[dts0,vs0]=get_vars(prob.x0,prob);

% speeds
[v,vhi,vlo] = car_spds(prob.x0,prob);

% and continuities
c_eq0 = car_eqs(prob.x0,prob);

% form bounds on g
g_L = [vlo;0*c_eq0;0*prob.b-2e19];
g_H = [vhi;0*c_eq0;prob.b];
x_L = 0*prob.x0;
x_H = x_L + max(max(prob.v_max));

%% build and make

ipoptgen(@(x)car_cost(x,prob),@(x)car_g(x,prob),prob.x0,x_L,x_H,g_L,g_H)

cd ipoptgen
!make
copyfile solveit.exe ..
cd ..

%% solve and load
tic
!solveit
toc
x_opt = load('XOPT.dat')

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

