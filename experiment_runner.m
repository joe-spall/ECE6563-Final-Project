clear;close all;clc
init

fprintf("Line Front 1 \n")
num_robots = 10;
num_leaders = 1;
num_loops = 50;
max_iterations = 5000;
iterations = NaN(size(num_robots,2),num_loops);
for n = 1:size(num_robots,2)
    a = num_robots(n);
    fprintf('Starting with %d leaders and %d total robots \n', num_leaders, a);

    success_count = 0;
    total_iterations = 0;
    fprintf('Running %d tests \n', num_loops);
    parfor i = 1:num_loops
        %fprintf('Running #%d ',i);
        [connected,num_iterations] = main_line_w_leader_front('NumberOfRobots',a,     ...
                                          'NumberOfLeaders',num_leaders,    ...
                                          'MaxIterations',max_iterations,   ...
                                          'VisibilityAngle',2*pi, ...
                                          'VisibilityDist', 0.5,  ...
                                          'ShowFigure', true,    ...
                                          'InitialConditions', [], ...
                                          'RoboDebug', false); 

        iterations(n,i) = num_iterations;
        if connected
            success_count = success_count+1;
            total_iterations = total_iterations+num_iterations;
            %fprintf('Success! \n');
        else
            %fprintf('Fail! \n');
        end
    end
    iterations(iterations == max_iterations) = NaN;
    
    fprintf('Done \n');
    fprintf('Finished with %d leaders and %d total robots \n', num_leaders, a);

    fprintf('Success Percentage: %.2f%% \n',round(success_count/ num_loops*100,2));
    fprintf('Average Success Duration: %.2f \n',round(total_iterations/ success_count,2));
    fprintf('Std Success Duration: %.2f \n \n',round(std(iterations(n,:),'omitnan'),2));
end
fprintf("\n")
% 
% fprintf("Line Front 2 \n")
% num_robots = 3:10;
% num_leaders = 2;
% num_loops = 50;
% max_iterations = 5000;
% iterations = NaN(size(num_robots,2),num_loops);
% for n = 1:size(num_robots,2)
%     a = num_robots(n);
%     fprintf('Starting with %d leaders and %d total robots \n', num_leaders, a);
% 
%     success_count = 0;
%     total_iterations = 0;
%     fprintf('Running %d tests \n', num_loops);
%     parfor i = 1:num_loops
%         %fprintf('Running #%d ',i);
%         [connected,num_iterations] = main_line_w_leader_front('NumberOfRobots',a,     ...
%                                           'NumberOfLeaders',num_leaders,    ...
%                                           'MaxIterations',max_iterations,   ...
%                                           'VisibilityAngle',2*pi, ...
%                                           'VisibilityDist', 0.5,  ...
%                                           'ShowFigure', false,    ...
%                                           'InitialConditions', [], ...
%                                           'RoboDebug', false); 
% 
%         iterations(n,i) = num_iterations;
%         if connected
%             success_count = success_count+1;
%             total_iterations = total_iterations+num_iterations;
%             %fprintf('Success! \n');
%         else
%             %fprintf('Fail! \n');
%         end
%     end
%     iterations(iterations == max_iterations) = NaN;
%     
%     fprintf('Done \n');
%     fprintf('Finished with %d leaders and %d total robots \n', num_leaders, a);
% 
%     fprintf('Success Percentage: %.2f%% \n',round(success_count/ num_loops*100,2));
%     fprintf('Average Success Duration: %.2f \n',round(total_iterations/ success_count,2));
%     fprintf('Std Success Duration: %.2f \n \n',round(std(iterations(n,:),'omitnan'),2));
% end
% fprintf("\n")
% 
% fprintf("Line Angle 1 \n")
% num_robots = 2:10;
% num_leaders = 1;
% num_loops = 50;
% max_iterations = 5000;
% iterations = NaN(size(num_robots,2),num_loops);
% for n = 1:size(num_robots,2)
%     a = num_robots(n);
%     fprintf('Starting with %d leaders and %d total robots \n', num_leaders, a);
% 
%     success_count = 0;
%     total_iterations = 0;
%     fprintf('Running %d tests \n', num_loops);
%     parfor i = 1:num_loops
%         %fprintf('Running #%d ',i);
%         [connected,num_iterations] = main_line_angle_formation('NumberOfRobots',a,     ...
%                                           'NumberOfLeaders',num_leaders,    ...
%                                           'MaxIterations',max_iterations,   ...
%                                           'VisibilityAngle',2*pi, ...
%                                           'VisibilityDist', 0.5,  ...
%                                           'ShowFigure', false,    ...
%                                           'InitialConditions', [], ...
%                                           'RoboDebug', false); 
% 
%         iterations(n,i) = num_iterations;
%         if connected
%             success_count = success_count+1;
%             total_iterations = total_iterations+num_iterations;
%             %fprintf('Success! \n');
%         else
%             %fprintf('Fail! \n');
%         end
%     end
%     iterations(iterations == max_iterations) = NaN;
%     
%     fprintf('Done \n');
%     fprintf('Finished with %d leaders and %d total robots \n', num_leaders, a);
% 
%     fprintf('Success Percentage: %.2f%% \n',round(success_count/ num_loops*100,2));
%     fprintf('Average Success Duration: %.2f \n',round(total_iterations/ success_count,2));
%     fprintf('Std Success Duration: %.2f \n \n',round(std(iterations(n,:),'omitnan'),2));
% end
% fprintf("\n")
% 
% fprintf("Line Angle 2 \n")
% num_robots = 3:10;
% num_leaders = 2;
% num_loops = 50;
% max_iterations = 5000;
% iterations = NaN(size(num_robots,2),num_loops);
% for n = 1:size(num_robots,2)
%     a = num_robots(n);
%     fprintf('Starting with %d leaders and %d total robots \n', num_leaders, a);
% 
%     success_count = 0;
%     total_iterations = 0;
%     fprintf('Running %d tests \n', num_loops);
%     parfor i = 1:num_loops
%         %fprintf('Running #%d ',i);
%         [connected,num_iterations] = main_line_angle_formation('NumberOfRobots',a,     ...
%                                           'NumberOfLeaders',num_leaders,    ...
%                                           'MaxIterations',max_iterations,   ...
%                                           'VisibilityAngle',2*pi, ...
%                                           'VisibilityDist', 0.5,  ...
%                                           'ShowFigure', false,    ...
%                                           'InitialConditions', [], ...
%                                           'RoboDebug', false); 
% 
%         iterations(n,i) = num_iterations;
%         if connected
%             success_count = success_count+1;
%             total_iterations = total_iterations+num_iterations;
%             %fprintf('Success! \n');
%         else
%             %fprintf('Fail! \n');
%         end
%     end
%     iterations(iterations == max_iterations) = NaN;
%     
%     fprintf('Done \n');
%     fprintf('Finished with %d leaders and %d total robots \n', num_leaders, a);
% 
%     fprintf('Success Percentage: %.2f%% \n',round(success_count/ num_loops*100,2));
%     fprintf('Average Success Duration: %.2f \n',round(total_iterations/ success_count,2));
%     fprintf('Std Success Duration: %.2f \n \n',round(std(iterations(n,:),'omitnan'),2));
% end
% fprintf("\n")
% 
% fprintf("Flock 1 \n")
% num_robots = 2:10;
% num_leaders = 1;
% num_loops = 50;
% max_iterations = 5000;
% iterations = NaN(size(num_robots,2),num_loops);
% for n = 1:size(num_robots,2)
%     a = num_robots(n);
%     fprintf('Starting with %d leaders and %d total robots \n', num_leaders, a);
% 
%     success_count = 0;
%     total_iterations = 0;
%     fprintf('Running %d tests \n', num_loops);
%     parfor i = 1:num_loops
%         %fprintf('Running #%d ',i);
%         [connected,num_iterations] = main_flock_formation('NumberOfRobots',a,     ...
%                                           'NumberOfLeaders',num_leaders,    ...
%                                           'MaxIterations',max_iterations,   ...
%                                           'VisibilityAngle',2*pi, ...
%                                           'VisibilityDist', 0.5,  ...
%                                           'ShowFigure', false,    ...
%                                           'InitialConditions', [], ...
%                                           'RoboDebug', false); 
% 
%         iterations(n,i) = num_iterations;
%         if connected
%             success_count = success_count+1;
%             total_iterations = total_iterations+num_iterations;
%             %fprintf('Success! \n');
%         else
%             %fprintf('Fail! \n');
%         end
%     end
%     iterations(iterations == max_iterations) = NaN;
%     
%     fprintf('Done \n');
%     fprintf('Finished with %d leaders and %d total robots \n', num_leaders, a);
% 
%     fprintf('Success Percentage: %.2f%% \n',round(success_count/ num_loops*100,2));
%     fprintf('Average Success Duration: %.2f \n',round(total_iterations/ success_count,2));
%     fprintf('Std Success Duration: %.2f \n \n',round(std(iterations(n,:),'omitnan'),2));
% end
% fprintf("\n")
% 
% fprintf("Flock 2 \n")
% num_robots = 3:10;
% num_leaders = 2;
% num_loops = 50;
% max_iterations = 5000;
% iterations = NaN(size(num_robots,2),num_loops);
% for n = 1:size(num_robots,2)
%     a = num_robots(n);
%     fprintf('Starting with %d leaders and %d total robots \n', num_leaders, a);
% 
%     success_count = 0;
%     total_iterations = 0;
%     fprintf('Running %d tests \n', num_loops);
%     parfor i = 1:num_loops
%         %fprintf('Running #%d ',i);
%         [connected,num_iterations] = main_flock_formation('NumberOfRobots',a,     ...
%                                           'NumberOfLeaders',num_leaders,    ...
%                                           'MaxIterations',max_iterations,   ...
%                                           'VisibilityAngle',2*pi, ...
%                                           'VisibilityDist', 0.5,  ...
%                                           'ShowFigure', false,    ...
%                                           'InitialConditions', [], ...
%                                           'RoboDebug', false); 
% 
%         iterations(n,i) = num_iterations;
%         if connected
%             success_count = success_count+1;
%             total_iterations = total_iterations+num_iterations;
%             %fprintf('Success! \n');
%         else
%             %fprintf('Fail! \n');
%         end
%     end
%     iterations(iterations == max_iterations) = NaN;
%     
%     fprintf('Done \n');
%     fprintf('Finished with %d leaders and %d total robots \n', num_leaders, a);
% 
%     fprintf('Success Percentage: %.2f%% \n',round(success_count/ num_loops*100,2));
%     fprintf('Average Success Duration: %.2f \n',round(total_iterations/ success_count,2));
%     fprintf('Std Success Duration: %.2f \n \n',round(std(iterations(n,:),'omitnan'),2));
% end
% fprintf("\n")
% 
% fprintf("MinMax 1 \n")
% num_robots = 2:10;
% num_leaders = 1;
% num_loops = 50;
% max_iterations = 5000;
% iterations = NaN(size(num_robots,2),num_loops);
% for n = 1:size(num_robots,2)
%     a = num_robots(n);
%     fprintf('Starting with %d leaders and %d total robots \n', num_leaders, a);
% 
%     success_count = 0;
%     total_iterations = 0;
%     fprintf('Running %d tests \n', num_loops);
%     parfor i = 1:num_loops
%         %fprintf('Running #%d ',i);
%         [connected,num_iterations] = main_min_max_dist_attract('NumberOfRobots',a,     ...
%                                           'NumberOfLeaders',num_leaders,    ...
%                                           'MaxIterations',max_iterations,   ...
%                                           'VisibilityAngle',2*pi, ...
%                                           'VisibilityDist', 0.5,  ...
%                                           'ShowFigure', false,    ...
%                                           'InitialConditions', [], ...
%                                           'RoboDebug', false); 
% 
%         iterations(n,i) = num_iterations;
%         if connected
%             success_count = success_count+1;
%             total_iterations = total_iterations+num_iterations;
%             %fprintf('Success! \n');
%         else
%             %fprintf('Fail! \n');
%         end
%     end
%     iterations(iterations == max_iterations) = NaN;
%     
%     fprintf('Done \n');
%     fprintf('Finished with %d leaders and %d total robots \n', num_leaders, a);
% 
%     fprintf('Success Percentage: %.2f%% \n',round(success_count/ num_loops*100,2));
%     fprintf('Average Success Duration: %.2f \n',round(total_iterations/ success_count,2));
%     fprintf('Std Success Duration: %.2f \n \n',round(std(iterations(n,:),'omitnan'),2));
% end
% fprintf("\n")
% 
% fprintf("MinMax 2 \n")
% num_robots = 3:10;
% num_leaders = 2;
% num_loops = 50;
% max_iterations = 5000;
% iterations = NaN(size(num_robots,2),num_loops);
% for n = 1:size(num_robots,2)
%     a = num_robots(n);
%     fprintf('Starting with %d leaders and %d total robots \n', num_leaders, a);
% 
%     success_count = 0;
%     total_iterations = 0;
%     fprintf('Running %d tests \n', num_loops);
%     parfor i = 1:num_loops
%         %fprintf('Running #%d ',i);
%         [connected,num_iterations] = main_min_max_dist_attract('NumberOfRobots',a,     ...
%                                           'NumberOfLeaders',num_leaders,    ...
%                                           'MaxIterations',max_iterations,   ...
%                                           'VisibilityAngle',2*pi, ...
%                                           'VisibilityDist', 0.5,  ...
%                                           'ShowFigure', false,    ...
%                                           'InitialConditions', [], ...
%                                           'RoboDebug', false); 
% 
%         iterations(n,i) = num_iterations;
%         if connected
%             success_count = success_count+1;
%             total_iterations = total_iterations+num_iterations;
%             %fprintf('Success! \n');
%         else
%             %fprintf('Fail! \n');
%         end
%     end
%     iterations(iterations == max_iterations) = NaN;
%     
%     fprintf('Done \n');
%     fprintf('Finished with %d leaders and %d total robots \n', num_leaders, a);
% 
%     fprintf('Success Percentage: %.2f%% \n',round(success_count/ num_loops*100,2));
%     fprintf('Average Success Duration: %.2f \n',round(total_iterations/ success_count,2));
%     fprintf('Std Success Duration: %.2f \n \n',round(std(iterations(n,:),'omitnan'),2));
% end
% fprintf("\n")
