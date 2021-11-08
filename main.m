% Same script as leader_follower.m but with additional plotting routines.
% In this experiment edges between robots will be plotted as well as robot
% labels and leader goal locations.

% Sean Wilson and Joseph Spall
% 07/2019

%% Experiment Constants

%Run the simulation for a specific number of iterations
iterations = 5000;

%% Set up the Robotarium object

N = 6;
robot_diameter = 0.11;
boundaries = [-1.6,1.6,-1,1];
initial_positions = generate_initial_conditions(N, ...
    'Width', boundaries(2)-boundaries(1)-robot_diameter, ...
    'Height', boundaries(4)-boundaries(3)-robot_diameter, ...
    'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions',initial_positions);

%% Create the desired Laplacian

%Graph laplacian
followers = -completeGL(N-1);
L = zeros(N, N);
L(2:N, 2:N) = followers;
L(2, 2) = L(2, 2) + 1;
L(2, 1) = -1;

%Initialize velocity vector
dxi = zeros(2, N);

%State for leader
state = 1;

% These are gains for our formation control algorithm
formation_control_gain = 5;
desired_distance = 0.3;

% All agent
visibilty_dist_meter = 0.5;


% Leader
leader_const_velocity = 0.2;
leader_waypoint_dist = 0.05;
leader_boundary = r.robot_diameter;

% Follower

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
marker_size_robot = determine_robot_marker_size(r,visibilty_dist_meter);
marker_size_goal = determine_marker_size(r, 0.2);
font_size = determine_font_size(r, 0.05);
line_width = 3;

% Plot graph connections
%Need location of robots
x=r.get_poses();

% Follower connections to each other
[rows, cols] = find(L == 1);

%Only considering half due to symmetric nature
for i = 1:N
    for j = 1:N
        if i ~= j
            connection_line(i,j) = line([x(1,rows(k)), x(1,cols(k))],[x(2,rows(k)), x(2,cols(k))], 'LineWidth', line_width, 'Color', 'b','LineStyle','none'); 
        end
    end 
end

% Leader connection assuming only connection between first and second
% robot.
%ll3 = line([x(1,1), x(1,2)],[x(2,1), x(2,2)], 'LineWidth', line_width, 'Color', 'r'); 

% Follower plot setup
for j = 1:N-1    
    % Text for robot identification
    follower_caption{j} = sprintf('F%d', j);
    % Plot the robot label text 
    follower_labels{j} = text(500, 500, follower_caption{j}, 'FontSize', font_size, 'FontWeight', 'bold');

end

% Color circle plots
for i = 1:N
    color = 'b';
    if i == 1
        color = 'r';
    end
    circle_plot(i) = plot(x(1,i),x(2,i),'o','MarkerSize', marker_size_robot,'LineWidth',5,'Color',color);
end

%Leader plot setup
random_waypoint = [0;0];
leader_label = text(500, 500, 'L1', 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');
leader_plot = plot(random_waypoint(1), random_waypoint(2),'s','MarkerSize',marker_size_goal,'LineWidth',2,'Color','r');
r.step();

for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Algorithm
    
%     for i = 2:N
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
    if t == 1 || norm(([x(1,1); x(2,1)]-random_waypoint)) < leader_waypoint_dist
        random_x = (r.boundaries(2)-leader_boundary)*2*(rand-0.5);
        random_y = (r.boundaries(4)-leader_boundary)*2*(rand-0.5);
        random_waypoint = [random_x;random_y];
    end
    dxi(:, 1) = leader_controller(x(1:2, 1), random_waypoint);

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
    
    marker_size_goal = num2cell(ones(1,1)*determine_marker_size(r, 0.2));
    [leader_plot.MarkerSize] = marker_size_goal{:};
    leader_plot.XData = random_waypoint(1);
    leader_plot.YData = random_waypoint(2);
    
    
    %Update position of labels for followers
    for q = 1:N-1
        follower_labels{q}.Position = x(1:2, q+1) + [-0.15;0.15];    
    end
    
    % Connection lines showing if in visibility region
    for i = 1:N
        for j = 1:N
            if i ~= j 
                if norm([x(1,i), x(2,i)] - [x(1,j), x(2,j)]) < visibilty_dist_meter
                    connection_line(i,j).XData = [x(1,i), x(1,j)];
                    connection_line(i,j).YData = [x(2,i), x(2,j)];
                    connection_line(i,j).LineStyle = ':'; 
                else
                    connection_line(i,j).LineStyle = 'none';
                end
            end
        end 
    end
    
    %Update position of label and graph connection for leader
    leader_label.Position = x(1:2, 1) + [-0.15;0.15];
    %ll3.XData = [x(1,1), x(1,2)];
    %ll3.YData = [x(2,1), x(2,2)];
    
    % Resize Marker Sizes (In case user changes simulated figure window
    % size, this is unnecessary in submission as the figure window 
    % does not change size).

    marker_size_robot = num2cell(ones(1,N)*determine_robot_marker_size(r,visibilty_dist_meter));
    [circle_plot.MarkerSize] = marker_size_robot{:};
    font_size = determine_font_size(r, 0.05);
    leader_label.FontSize = font_size;
    
    for n = 1:N
        % Circle plot position updates
        circle_plot(n).XData = x(1,n);
        circle_plot(n).YData = x(2,n);
        % Have to update font in loop for some conversion reasons.
        % Again this is unnecessary when submitting as the figure
        % window does not change size when deployed on the Robotarium.
        follower_labels{n}.FontSize = font_size;
    end
    
    %Iterate experiment
    r.step();
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();

%% Helper Functions

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

