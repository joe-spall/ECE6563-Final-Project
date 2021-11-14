
% Same script as leader_follower.m but with additional plotting routines.
% In this experiment edges between robots will be plotted as well as robot
% labels and leader goal locations.

% Sean Wilson and Joseph Spall
% 07/2019

clear;close all;clc
init

%% Experiment Constants

%Run the simulation for a specific number of iterations
iterations = 5000;

visibility_types = {'uniform', 'wedge'};
visibility_cur_type = cell2mat(visibility_types(2));

%Number of agents
N = 6;
%Number of leaders
Nl = 2;
robot_diameter = 0.11;
boundaries = [-1.6,1.6,-1,1];

% These are gains for our formation control algorithm
formation_control_gain = 5;
desired_distance = 0.3;

% Visibility
visibility_dist_meter = 0.5;
visibility_line_width = 3;
% Vision angle (only for non-uniform)
visibility_angle = pi/3;

% Leader
leader_const_velocity = 0.2;
leader_waypoint_dist = 0.05;
leader_color = 'r';

% Follower
follower_color = 'b';

% Connection
connection_line = gobjects(N,N);
connection_color = 'k';

% Captions
leader_caption = cell(Nl);
follower_caption = cell(N-Nl);

% Labels
leader_labels = gobjects(Nl,1);
follower_labels = gobjects(N-Nl,1);

%waypoints
waypoint_plot = gobjects(Nl,1);

%% Set up the Robotarium object

initial_positions = generate_initial_conditions(N, ...
    'Width', boundaries(2)-boundaries(1)-robot_diameter, ...
    'Height', boundaries(4)-boundaries(3)-robot_diameter, ...
    'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions',initial_positions);

%Leader boundary
leader_boundary = r.robot_diameter;

%Initialize velocity vector
dxi = zeros(2, N);

%Tracks previous theta for rotation tracking
old_theta = zeros(1,N);

%States
L_states = 1:Nl; %Leaders
F_states = Nl+1:N; %Followers

%Waypoints
random_waypoint = zeros(2,Nl);

% Visibility plots
visibility_plot = gobjects(1,N);


%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
% Single-integrator position controller
leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.08);

%% Plotting Setup

% Color Vector for Plotting
% Note the Robotarium MATLAB instance runs in a docker container which will 
% produce the same rng value every time unless seeded by the user.
CM = ['k','b','r','g'];

%Marker, font, and line sizes
marker_size_robot = determine_robot_marker_size(r,visibility_dist_meter);
marker_size_goal = determine_marker_size(r, 0.2);
font_size = determine_font_size(r, 0.05);
line_width = 3;

% Plot graph connections
%Need location of robots
x=r.get_poses();

% Setup agents connections to each other
axes = get(gca, 'Position');
xlims = get(gca, 'xlim');
ylims = get(gca, 'ylim');

for i = 1:N
    for j = 1:N
%            [x_i_out, y_i_out] = norm_coord(x(1,i), x(2,i), get(gca, 'Position'), xlims, ylims)
%            [x_j_out, y_j_out] = norm_coord(x(1,j), x(2,j), get(gca, 'Position'), xlims, ylims)

%            connection_line(i,j) = annotation('arrow',[x_i_out,x_j_out],...
%                [y_i_out,y_j_out], ...
%                'LineWidth', line_width, 'Color', connection_color,...
%                'Units','points');
        connection_line(i,j) = line([x(1,i), x(1,j)],[x(2,i), x(2,j)],...
            'LineWidth', line_width, 'Color', connection_color,'LineStyle','none');
    end 
end

% Caption/Label plot setup
for j = 1:N    
    if any(j==L_states)
        % Text color
        color = leader_color;
        % Text for robot identification
        leader_caption{j} = sprintf('L%d', j);
        % Plot the robot label text 
        leader_labels(j) = text(500, 500, leader_caption{j}, 'FontSize', font_size, 'FontWeight', 'bold');
    else
        % Text color
        color = follower_color;
        % Text for robot identification
        follower_caption{j-Nl} = sprintf('F%d', j-Nl);
        % Plot the robot label text 
        follower_labels(j-Nl) = text(500, 500, follower_caption{j-Nl}, 'FontSize', font_size, 'FontWeight', 'bold');
    end
    
    % Visibility Plots
    if strcmp(visibility_cur_type,'uniform')
        visibility_plot(j) = plot(x(1,j),x(2,j),'o','MarkerSize', ...
            marker_size_robot,'LineWidth',visibility_line_width,'Color',color);
    elseif strcmp(visibility_cur_type,'wedge')
        visibility_plot(j) = arcpatch(x(1,j),x(2,j),visibility_dist_meter,...
            [rad2deg(-visibility_angle/2)+rad2deg(x(3,j))...
            ,rad2deg(visibility_angle/2)+rad2deg(x(3,j))], color);
    end  
    
    % Waypoint plot
    if any(j==L_states)
        waypoint_plot(j) = plot(random_waypoint(1,j), random_waypoint(2,j),'s','MarkerSize',marker_size_goal,'LineWidth',2,'Color','r');
    end
end
r.step();

for t = 1:iterations
    old_pose = x;
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Check for Connections
    A = zeros(N);
    for i = 1:N
        for j = 1:N
            if i ~= j
                % Polar coordinate conversion
                radius = norm([x(1,i), x(2,i)] - [x(1,j), x(2,j)]);
                angle = atan2(x(2,j) - x(2,i), x(1,j) - x(1,i));
                angle_start = (visibility_angle/2)+x(3,i);
                angle_end = (-visibility_angle/2)+x(3,i);
                if radius < visibility_dist_meter && ...
                   angle > angle_end && angle < angle_start
                    A(i,j) = 1;
                end
            end
        end 
    end
    
    %% Follower Controller
    
%     for i = F_states
%         
%         %Zero velocity and get the topological neighbors of agent i
%         dxi(:, i) = [0 ; 0];
%         
%         neighbors = topological_neighbors(L, i);
%         
%         for j = neighbors
%             dxi(:, i) = dxi(:, i) + ...
%                 formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*(x(1:2, j) - x(1:2, i));
%         end
%     end
%     
    %% Leader Controller
    %% Make the leader travel between waypoints
    % TODO -- multiple leaders...state variable now array L_states
%     waypoint = waypoints(:, state);
%     
%     switch state        
%         case 1
%             dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
%             if(norm(x(1:2, 1) - waypoint) < close_enough)
%                 state = 2;
%             end
%         case 2
%             dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
%             if(norm(x(1:2, 1) - waypoint) < close_enough)
%                 state = 3;
%             end
%         case 3
%             dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
%             if(norm(x(1:2, 1) - waypoint) < close_enough)
%                 state = 4;
%             end
%         case 4
%             dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
%             if(norm(x(1:2, 1) - waypoint) < close_enough)
%                 state = 1;
%             end
%     end
    %% Random Leader Paths
    
    % Random Direction
%     leader_const_velocity = 0.2;
%     if t == 1
%         phi_random = x(3,1);
%     elseif mod(t,20) == 0
%         
%         phi_random = x(3,1) + rand*pi/2;
%     end
%     
%     rotation_random = [cos(phi_random) -sin(phi_random);
%                        sin(phi_random) cos(phi_random)];
%     dxi(:,1) = rotation_random*leader_const_velocity*[1; 0];
%     
    % Random Position
    for n = L_states
        if t == 1 || norm(([x(1,n); x(2,n)]-random_waypoint(:,n))) < leader_waypoint_dist
            random_x = (r.boundaries(2)-leader_boundary)*2*(rand-0.5);
            random_y = (r.boundaries(4)-leader_boundary)*2*(rand-0.5);
            random_waypoint(:,n) = [random_x;random_y];
        end
        dxi(:, n) = leader_controller(x(1:2, n), random_waypoint(:,n));
    end

    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    to_thresh(1) = 0;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert(dxu, x);
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:N, dxu);
    
    %% Update Plot Handles
    
    %Update Waypoints
    marker_size_goal = determine_marker_size(r, 0.2);
    for j = 1:Nl
        waypoint_plot(j).MarkerSize = marker_size_goal;
        waypoint_plot(j).XData = random_waypoint(1,j);
        waypoint_plot(j).YData = random_waypoint(2,j);
    end
    
    %Update position of labels
    font_size = determine_font_size(r, 0.05);
    for j = 1:N
        if any(j==L_states)
            leader_labels(j).FontSize = font_size;
            leader_labels(j).Position = x(1:2, j) + [-0.15;0.15];
        else
            follower_labels(j-Nl).FontSize = font_size;
            follower_labels(j-Nl).Position = x(1:2, j) + [-0.15;0.15];  
        end
    end
    
    % Connection lines showing if in visibility region
    for i = 1:N
        for j = 1:N
            if A(i,j) == 1
                connection_line(i,j).XData = [x(1,i), x(1,j)];
                connection_line(i,j).YData = [x(2,i), x(2,j)];
                connection_line(i,j).LineStyle = ':';
            else
                connection_line(i,j).LineStyle = 'none';
            end
        end 
    end

    % Visibility Plots
    marker_size_robot = determine_robot_marker_size(r,visibility_dist_meter);
    for n = 1:N
        % Resize Marker Sizes (In case user changes simulated figure window
        % size, this is unnecessary in submission as the figure window 
        % does not change size).
        visibility_plot(n).MarkerSize = marker_size_robot;
        
        % Circle plot position updates
        if strcmp(visibility_cur_type,'uniform')
            visibility_plot(n).XData = x(1,n);
            visibility_plot(n).YData = x(2,n);
        elseif strcmp(visibility_cur_type,'wedge')
            cur_vertices = visibility_plot(n).Vertices;
            % Subtract to make center of robot origin
            
            cur_vertices(:,1:2) = cur_vertices(:,1:2) - ...
                [x(1,n)*ones(size(cur_vertices,1),1), x(2,n)*ones(size(cur_vertices,1),1)];
            % Get in homogenous coordinate form
            cur_vertices(:,3) = 1;
            d_pos = x(1:2,n)-old_pose(1:2,n);
            d_theta = x(3,n)-old_pose(3,n);
            homog = [cos(d_theta) -sin(d_theta) d_pos(1);
                    sin(d_theta)  cos(d_theta)  d_pos(2);
                    0             0             1];
            % Apply homogenous coordinate rot and translation
            for v = 1:size(cur_vertices,1)
                cur_vertices(v,:) = (homog*cur_vertices(v,:).').';
            end
            % Undo homogenous coordinate
            cur_vertices(:,3) = 0;
            % Add to make global coordinates
            cur_vertices(:,1:2) = cur_vertices(:,1:2) + ...
                [x(1,n)*ones(size(cur_vertices,1),1), x(2,n)*ones(size(cur_vertices,1),1)];
            visibility_plot(n).Vertices = cur_vertices;
        end
    end
    
    %Iterate experiment
    r.step();
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();

%% Helper Functions
function [x_out, y_out] = norm_coord(x, y, axes, xlims, ylims)
    x_out = ((x-xlims(1))/(xlims(2) - xlims(1)))*axes(3);
    y_out = ((y-ylims(1))/(ylims(2) - ylims(1)))*axes(4);
    x_out = axes(1) + x_out;
    y_out = axes(2) + y_out;
end       



% Marker Size Helper Function to scale size with figure window
% Input: robotarium instance, desired size of the marker in meters
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Points');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
marker_ratio = (marker_size_meters)/(robotarium_instance.boundaries(2) -...
    robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * marker_ratio;

end

% Font Size Helper Function to scale size with figure window
% Input: robotarium instance, desired height of the font in meters
function font_size = determine_font_size(robotarium_instance, font_height_meters)

% Get the size of the robotarium figure window in point units
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the font height to the y-axis
font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
    robotarium_instance.boundaries(3));

% Determine the font size in points so it fits the window. cursize(4) is
% the hight of the figure window in points.
font_size = cursize(4) * font_ratio;

end

% Marker Size Helper Function to scale size of markers for robots with figure window
% Input: robotarium class instance
function marker_size = determine_robot_marker_size(robotarium_instance,robot_vision_distance_meters)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
robot_ratio = (robot_vision_distance_meters)/...
    (robotarium_instance.boundaries(2) - robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * robot_ratio;

end


