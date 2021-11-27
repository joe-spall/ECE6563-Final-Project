clear;close all;clc
init

num_loops = 30;
success_count = 0;
total_iterations = 0;
fprintf('Running %d tests \n', num_loops);
for i = 1:num_loops
    fprintf('Running #%d ',i);
    [connected,num_iterations] = main('NumberOfRobots',6,     ...
                                      'NumberOfLeaders',1,    ...
                                      'MaxIterations',5000,   ...
                                      'VisibilityAngle',2*pi, ...
                                      'VisibilityDist', 0.5,  ...
                                      'ShowFigure', true,    ...
                                      'InitialConditions', [], ...
                                      'RoboDebug', false); 
    if connected
        success_count = success_count+1;
        total_iterations = total_iterations+num_iterations;
        fprintf('Success! \n');
    else
    	fprintf('Fail! \n');
    end

end
fprintf('Done \n');
%% 
fprintf('Success Percentage: %.2f%% \n',round(success_count/ num_loops*100,2));
fprintf('Average Success Duration: %.2f \n',round(total_iterations/ success_count,2));

