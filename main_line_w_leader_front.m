
% Same script as leader_follower.m but with additional plotting routines.
% In this experiment edges between robots will be plotted as well as robot
% labels and leader goal locations.

% Sean Wilson, Joseph Spall, Juan Elizondo
% 07/2019

function [connected,num_iterations] = main_line_w_leader_front(varargin) 
    %% Input Parsing
    parser = inputParser;
            
    addParameter(parser,'NumberOfRobots', 2);
    addParameter(parser,'NumberOfLeaders', 1);
    addParameter(parser,'MaxIterations',5000);
    addParameter(parser,'VisibilityAngle', 2*pi);
    addParameter(parser,'VisibilityDist', 0.5);
    addParameter(parser,'ShowFigure', true);
    addParameter(parser,'InitialConditions', []);
    addParameter(parser,'RoboDebug', false);
                        
    parse(parser, varargin{:})

    %% Experiment Constants
    % RNG Seed
    % Note the Robotarium MATLAB instance runs in a docker container which will 
    % produce the same rng value every time unless seeded by the user.
    rng('shuffle')
    
    show_figure = parser.Results.ShowFigure;
    
    %Run the simulation for a specific number of iterations
    iterations = parser.Results.MaxIterations;

    %Number of total agents
    N = parser.Results.NumberOfRobots;
    %Number of leaders
    N_L = parser.Results.NumberOfLeaders;
    robot_diameter = 0.11;

    % Visibility
    visibility_dist = parser.Results.VisibilityDist;
    visibility_angle = parser.Results.VisibilityAngle;

    % Graph connection termination
    % 'weak' or 'strong'
    graph_connection = 'weak';


    % These are gains for our formation control algorithm
    gain = 0.5;

    % Leader
    leader_waypoint_dist = robot_diameter;
    leader_color = 'r';

    % Follower
    follower_color = 'b';



    % Initial Conditions
    initial_conditions = parser.Results.InitialConditions;    
    if(isempty(initial_conditions))
        initial_conditions = generate_initial_conditions(N,'Spacing', visibility_dist+0.1);
    end
    
    % Set up the Robotarium object
    r = Robotarium('NumberOfRobots', N, 'ShowFigure', show_figure, ...
        'InitialConditions',initial_conditions);

    % Leader boundary
    leader_boundary = r.robot_diameter;

    % Initialize velocity vector
    dxi = zeros(2, N);

    % States
    L_states = 1:N_L; %Leaders
    F_states = N_L+1:N; %Followers

    % Waypoints
    random_waypoint = zeros(2,N_L);

    % Grab tools we need to convert from single-integrator to unicycle dynamics
    % Single-integrator -> unicycle dynamics mapping
    si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
    % Single-integrator barrier certificates
    uni_barrier_cert = create_uni_barrier_certificate_with_boundary('BoundaryPoints',r.boundaries);
    % Single-integrator position controller
    leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.08);

    %% Plotting Setup
    if show_figure
        % Connection
        connection_line = gobjects(N,N);
        connection_color = 'k';

        % Captions
        leader_caption = cell(N_L);
        follower_caption = cell(N-N_L);

        % Labels
        leader_labels = gobjects(N_L,1);
        follower_labels = gobjects(N-N_L,1);

        % Waypoints
        waypoint_plot = gobjects(N_L,1);
        
        % Visibility plots
        visibility_plot = gobjects(1,N);
    
        % Marker, font, and line sizes
        marker_size_goal = determine_marker_size(r, 0.2);
        font_size = determine_font_size(r, 0.05);
        line_width = 3;

        % Plot graph connections
        %Need location of robots
        x=r.get_poses();

        % Caption/Label plot setup
        for i = 1:N    
            if any(i==L_states)
                % Text color
                color = leader_color;
                % Text for robot identification
                leader_caption{i} = sprintf('L%d', i);
                % Plot the robot label text 
                leader_labels(i) = text(500, 500, leader_caption{i}, 'FontSize', font_size, 'FontWeight', 'bold');
            else
                % Text color
                color = follower_color;
                % Text for robot identification
                follower_caption{i-N_L} = sprintf('F%d', i-N_L);
                % Plot the robot label text 
                follower_labels(i-N_L) = text(500, 500, follower_caption{i-N_L}, 'FontSize', font_size, 'FontWeight', 'bold');
            end

            % Visibility Plots
            visibility_plot(i) = arcpatch(x(1,i),x(2,i),visibility_dist,...
                    [rad2deg(-visibility_angle/2)+rad2deg(x(3,i)),...
                    rad2deg(visibility_angle/2)+rad2deg(x(3,i))], color);
            
            % Setup agents connections to each other
            for j = 1:N
                connection_line(i,j) = line([x(1,i), x(1,j)],[x(2,i), x(2,j)],...
                    'LineWidth', line_width, 'Color', connection_color,'LineStyle','none');
            end 
            
            % Waypoint Plot
            if any(i==L_states)
                waypoint_plot(i) = plot(random_waypoint(1,i), random_waypoint(2,i),'s','MarkerSize',marker_size_goal,'LineWidth',2,'Color','r');
            end
        end
        
        % Updates plot
        r.step();
    end


    %% Experiment

    % Tracks how many iterations before connected
    num_iterations = iterations;
    connected = false;
    for t = 1:iterations
        if show_figure
            % Captures the previous pose for use in plotting translation
            old_pose = x;
        end
        
        % Retrieve the most recent poses from the Robotarium.  The time delay is
        % approximately 0.033 seconds
        x = r.get_poses();

        % Check for Connections
        A = get_adjacency(x, visibility_angle, visibility_dist);

        %% Follower Controller
                
        % Line Init
        if t == 1
            
            % Init cell size, line for each leader
            control_lines = cell(1,size(L_states,2));
            
            % Each cell has leader number at the beginning
            for leader = L_states
                control_lines{leader} = leader;
            end
        end
        
        % Single-File Line Addition
        for line_list_index = L_states
            current_line = control_lines{line_list_index};
            line_end_x1 = x(:,current_line(end));
            for index_x2 = F_states
                x2 = x(:,index_x2);
                % Add to line if 
                % 1. they are in the vision of each other
                % 2. they aren't in the current list
                if  are_points_connected(line_end_x1, x2, ...
                    visibility_angle, visibility_dist) && ...
                    ~any(current_line == index_x2)
 
                    control_lines{line_list_index} = ...
                        [control_lines{line_list_index} index_x2];
                end
            end
        end
        
        % Single-File Removal
        for line_list_index = L_states
            current_line = control_lines{line_list_index};
            % Won't run if only leader
            for n = 1:size(current_line,2)-1
                m = n + 1;
                xi = x(:,current_line(n));
                xj = x(:,current_line(m));
                if ~are_points_connected(xi, xj, ...
                    visibility_angle, visibility_dist)
                    % Removes disconnected element to the end
                    current_line(m:end) = [];
                    control_lines{line_list_index} = current_line;
                    break;
                end
                
            end
            
        end
        
        for line_list_index = L_states
            current_line = control_lines{line_list_index};
            % Adjacent line formation from behind
            for n = size(current_line,2):-1:2
                m = n-1;
                i = current_line(n);
                j = current_line(m);

                radius = norm(x(1:2,i) - x(1:2,j));
                % Difference in index matches graph linear distance
                add_full_line = 4*(radius^2 - gain^2) ... 
                    *(x(1:2, i) - x(1:2, j));

                dxi(:, i) = dxi(:, i) + add_full_line;
            end                 
        end
        
        % Set to zero velocity if not in the line
        for i = F_states
            in_line = false;
            for line_list_index = L_states
                current_line = control_lines{line_list_index};
                if any(current_line)
                    in_line = true;
                end
            end
            
            if ~in_line
                dxi(:, i) = [0;0];
            end
        end
        

        %% Leader Controller
        
        % Random Leader Paths

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
        
        %% Controller Conversion
        % Avoid actuator errors

        % To avoid errors, we need to threshold dxi
        norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
        threshold = 3/4*r.max_linear_velocity;
        to_thresh = norms > threshold;
        dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);

        % Use barrier certificate and convert to unicycle dynamics
        dxu = si_to_uni_dyn(dxi, x);
        dxu = uni_barrier_cert(dxu, x);

        %% Send velocities to agents

        %Set velocities
        r.set_velocities(1:N, dxu);

        %% Update Plot Handles
        if show_figure
            %Update Waypoints
            marker_size_goal = determine_marker_size(r, 0.2);
            for j = 1:N_L
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
                    follower_labels(j-N_L).FontSize = font_size;
                    follower_labels(j-N_L).Position = x(1:2, j) + [-0.15;0.15];  
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
            marker_size_robot = determine_robot_marker_size(r,visibility_dist);
            for n = 1:N
                % Resize Marker Sizes (In case user changes simulated figure window
                % size, this is unnecessary in submission as the figure window 
                % does not change size).
                visibility_plot(n).MarkerSize = marker_size_robot;

                % Wedge plot position updates
                cur_vertices = visibility_plot(n).Vertices';
                for v = 1:size(cur_vertices,2)
                    % Robot coordinates
                    cur_vertices(1:2,v) = global_to_robot(old_pose(:,n),cur_vertices(1:2,v));
                    % Move with robot and global coordinates
                    cur_vertices(1:2,v) = robot_to_global(x(:,n),cur_vertices(1:2,v));
                end
                visibility_plot(n).Vertices = cur_vertices';
            end
        end

        %% Termination if graph is connected 
        if determine_connected(A,graph_connection)
            connected = true;
            num_iterations = t;
            break;
        end

        % Iterate experiment
        r.step();
    end
   
    % We can call this function to debug our experiment!  Fix all the errors
    % before submitting to maximize the chance that your experiment runs
    % successfully.
    robo_debug = parser.Results.RoboDebug;
    if robo_debug
        r.debug();
    end
end

%% Helper Functions
function status = are_points_connected(x1,x2, viz_angle, viz_dist)
    status = 0;
    % Polar coordinate conversion
    x_j = global_to_robot(x1,x2(1:2));
    radius = norm(x_j);
    if radius <= viz_dist
        angle = atan2(x_j(2),x_j(1));
        angle_start = viz_angle/2;
        angle_end = -viz_angle/2;
        if angle >= angle_end && angle <= angle_start
            status = 1;
        end
    end
    
end

function A = get_adjacency(x, viz_angle, viz_dist)
    N = size(x,2);
    A = zeros(N);
    for i = 1:N
        for j = 1:N
            if i ~= j
                A(i,j) = are_points_connected(x(:,i),x(:,j),viz_angle,viz_dist);
            end
        end 
    end
end

function status = determine_connected(A, connection_type)
    cur_graph = digraph(A);
    [~,binsize] = conncomp(cur_graph,'Type',connection_type);

    if binsize >= size(A,1)
       status = 1;
    else
       status = 0;
    end
end

function x_out = global_to_robot(x_robot, x_in)
    T = [cos(x_robot(3)),-sin(x_robot(3)),x_robot(1);
         sin(x_robot(3)),cos(x_robot(3)),x_robot(2);
         0              ,0              ,1];
    x_new = T\[x_in;1];
    x_out = x_new(1:2);
end

function x_out = robot_to_global(x_robot, x_in)
    T = [cos(x_robot(3)),-sin(x_robot(3)),x_robot(1);
         sin(x_robot(3)),cos(x_robot(3)),x_robot(2);
         0              ,0              ,1];
    x_new = T*[x_in;1];
    x_out = x_new(1:2);
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


