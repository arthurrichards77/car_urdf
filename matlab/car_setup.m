% car time planning on mesh

%% default geometry
prob.arc_xs = [-30 -9 9 30; % car 1
    0  0 0 0]'; % car 2

prob.arc_ys = [0  0 0 0;  % car 1
    -10 -3 3 10]'; % car 2

%% extract sizes
prob.n_cars = size(prob.arc_xs,2);
prob.n_arcs = size(prob.arc_xs,1)-1;

prob.v_max = 8.94*ones(prob.n_arcs,prob.n_cars); % 20mph

%% alternative - bit like turn OUT
% prob.arc_xs = [30 9 -9 -30; % car 1
%     0  0 -9 -24]'; % car 2
% 
% prob.arc_ys = [0  0 0 0;  % car 1
%     -30 -9 0 0]'; % car 2
% 
% prob.arc_hdgs = [pi pi pi pi; % ACW from +x
%     pi/2 pi/2 pi pi]';
% 
% prob.v_max(2,2) = 0.5*prob.v_max(2,2); % reduced on turn arc

%% alternative - bit like turn IN
prob.arc_xs = [30 9 -9 -30; % car 1
              -30 -6 0  0]'; % car 2

prob.arc_ys = [0  0 0 0;  % car 1
               3 3 -3 -30]'; % car 2

prob.arc_hdgs = [pi pi pi pi; % ACW from +x
                0 0 -pi/2 -pi/2]';

prob.v_max(2,2) = 0.5*prob.v_max(2,2); % reduced on turn arc

%% alternative - following
% prob.arc_xs = [10 3 -3 -10; % car 1
%                30  3 -3 -10]'; % car 2
%
% prob.arc_ys = [0  0 0 0;  % car 1
%                0 0 0 0]'; % car 2
%
% prob.v_max = 8.94*ones(prob.n_arcs,prob.n_cars); % 20mph
% prob.v_max(:,1) = 0.75*prob.v_max(:,1);

%% arc geometry
prob.ell_arcs = sqrt(diff(prob.arc_xs).^2 + diff(prob.arc_ys).^2);

% number of points for plotting
prob.n_arc_p = 50;

% get detailed model of each arc
for cc=1:prob.n_cars,
    for ii=1:prob.n_arcs,
        % indices for storing
        inds = (ii-1)*prob.n_arc_p + (1:prob.n_arc_p);
        % start and end points
        xs = prob.arc_xs(ii,cc);
        ys = prob.arc_ys(ii,cc);
        xf = prob.arc_xs(ii+1,cc);
        yf = prob.arc_ys(ii+1,cc);
        hs = prob.arc_hdgs(ii,cc);
        hf = prob.arc_hdgs(ii+1,cc);
        % baseline direction of arc, i.e. between endpoints
        theta = atan2(yf-ys,xf-xs);
        % heading offsets from baseline at either end
        dhs = hs - theta;
        dhf = hf - theta;
        % linear distance for interp
        prob.arc_sp(inds,cc) = (ii-1)+linspace(0,1,prob.n_arc_p);
        % heading always linear in both cases
        prob.arc_hp(inds,cc) = linspace(hs,hf,prob.n_arc_p);
        if (dhs==0)&&(dhf==0),
            % straight line case
            prob.arc_xp(inds,cc) = linspace(xs,xf,prob.n_arc_p);
            prob.arc_yp(inds,cc) = linspace(ys,yf,prob.n_arc_p);
        elseif dhs==-dhf,
            % circular arc case
            dhs
            dhf
            % radius of turn
            R = -0.5*prob.ell_arcs(ii,cc)/sin(dhs);
            % centre of turn
            xc = xs - R*sin(hs);
            yc = ys + R*cos(hs);
            % reconstruct segment
            prob.arc_xp(inds,cc) = xc + R*sin(prob.arc_hp(inds,cc));
            prob.arc_yp(inds,cc) = yc - R*cos(prob.arc_hp(inds,cc));
            % overwrite arc length
            prob.ell_arcs(ii,cc) = abs(R*(hf-hs));
        else
%             [xs ys hs]
%             [xf yf hf]
%             theta
%             dhs
%             dhf
            error('Neither line nor arc');
        end
    end
end


%% initial guess
prob.v_init = 0.75*prob.v_max(1,:);

prob.dts0 = zeros(2*prob.n_arcs,prob.n_cars);
prob.dts0(2:2:end,:) = 1.5*prob.ell_arcs./prob.v_max;
prob.vs0 = ones(prob.n_arcs-1,prob.n_cars)*min(min(prob.v_max));

prob.dts_lb = 0*prob.dts0;
prob.dts_ub = prob.dts0 + inf;

prob.vs_lb = 0*prob.vs0;
prob.vs_ub = min(prob.v_max(2:end,:),prob.v_max(1:(end-1),:));

% reshape them to vector form
prob.x0 = reshape([prob.dts0; prob.vs0],(2*prob.n_arcs + (prob.n_arcs-1))*prob.n_cars,1);
prob.x_lb = reshape([prob.dts_lb; prob.vs_lb],(2*prob.n_arcs + (prob.n_arcs-1))*prob.n_cars,1);
prob.x_ub = reshape([prob.dts_ub; prob.vs_ub],(2*prob.n_arcs + (prob.n_arcs-1))*prob.n_cars,1);

% default linear constraints
prob.Aeq = [];
prob.beq = [];
prob.A = [];
prob.b = [];

%% conflict avoidance
prob.A = [1 1 1 1 0 0 0 0 -1 -1 -1 0 0 0 0 0];
prob.b = [-0.5];

%% stopping for give way
% prob.Aeq = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0];
% prob.beq = 0;
% 
% prob.x0(15)=0;

% try adding a wait manually to guess
%prob.x0(11)=3; % doesn't work - always keep IG value

%% solve

options = optimoptions('fmincon');
options = optimoptions(options,'GradObj','on');

tic
[x_opt,j_opt,flag] = fmincon(@(x)car_cost(x,prob),prob.x0,prob.A,prob.b,prob.Aeq,prob.beq,prob.x_lb,prob.x_ub,@(x)car_cons(x,prob),options)
t_solve = toc

% convert back to times and speeds
[dts_opt,vs_opt] = get_vars(x_opt,prob);
% and to displacements
[ell_opt,t_opt,vss_opt,ss_opt] = get_dists(dts_opt,vs_opt,prob);

%% plotting
close all

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

%% animation
figure
plot(prob.arc_xs,prob.arc_ys,'x',prob.arc_xp,prob.arc_yp,'--')

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
    pause(0.8*dt)
    traj_store = [traj_store; [tt x(1) y(1) 0 h(1) x(2) y(2) 0 h(2)]];
end

%% save for use in ROS

% make header file
!echo time_from_start,robot1_move_x,robot1_move_y,robot1_move_z,robot1_turn_z,robot2_move_x,robot2_move_y,robot2_move_z,robot2_turn_z, > headers.csv

% store the trajectory
csvwrite('trajdata.csv',traj_store)

% combine the two files
!type headers.csv trajdata.csv > mytraj.csv