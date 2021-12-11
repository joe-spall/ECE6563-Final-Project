clear
clc
close all
A = readmatrix('Results.csv','OutputType','string');

% figure
% hold on
% legend_contents = [];
% for i = 1:9:36
%     legend_contents = [legend_contents ;A(i,1)];
%     x = [0];
%     y = [0];
%     for j = 1:9
%         x = [x str2num(A(i+j-1,3))];
%         y = [y str2double(A(i+j-1,4))];
%     end
%     x = x(2:end);
%     y = y(2:end);
%     plot(x,y,'.-','MarkerSize',20)
% end
% legend_contents = legend_contents.';
% legend(legend_contents,'Location','southeast')
% xlim([0.5,9.5])
% ylabel("Ave Successful Runs")
% xlabel("# of Followers")
% title("Ave Successful Runs for 1 Leader vs. Num of Followers")
% 
% figure
% hold on
% legend_contents = [];
% for i = 1:9:36
%     legend_contents = [legend_contents ;A(i,1)];
%     x = [0];
%     y = [0];
%     for j = 1:9
%         x = [x str2num(A(i+j-1,3))];
%         y = [y str2double(A(i+j-1,6))*100];
%     end
%     x = x(2:end);
%     y = y(2:end);
%     plot(x,y,'.-','MarkerSize',20)
% end
% legend_contents = legend_contents.';
% legend(legend_contents,'Location','southeast')
% xlim([0.5,9.5])
% xlabel("# of Followers")
% ylabel("% of Success")
% title("Percentage of Sucess for 1 Leader vs. Num of Followers")
% 
% figure
% hold on
% legend_contents = [];
% for i = 1:9:36
%     legend_contents = [legend_contents ;A(i,1)];
%     x = [0];
%     y = [0];
%     for j = 1:9
%         x = [x str2num(A(i+j-1,3))];
%         y = [y str2double(A(i+j-1,7))];
%     end
%     x = x(2:end);
%     y = y(2:end);
%     plot(x,y,'.-','MarkerSize',20)
% end
% legend_contents = legend_contents.';
% legend(legend_contents,'Location','southeast')
% xlim([0.5,9.5])
% xlabel("# of Followers")
% ylabel("Ave Runs")
% title("Ave Runs for 1 Leader vs. Num of Followers")
% 
% 
% % 2 Leaders
% 
% figure
% hold on
% legend_contents = [];
% for i = 37:8:68
%     legend_contents = [legend_contents ;A(i,1)];
%     x = [0];
%     y = [0];
%     for j = 1:8
%         x = [x str2num(A(i+j-1,3))];
%         y = [y str2double(A(i+j-1,4))];
%     end
%     x = x(2:end);
%     y = y(2:end);
%     plot(x,y,'.-','MarkerSize',20)
% end
% legend_contents = legend_contents.';
% legend(legend_contents,'Location','southeast')
% xlim([0.5,9.5])
% ylabel("Ave Successful Runs")
% xlabel("# of Followers")
% title("Ave Successful Runs for 2 Leaders vs. Num of Followers")
% 
% figure
% hold on
% legend_contents = [];
% for i = 37:8:68
%     legend_contents = [legend_contents ;A(i,1)];
%     x = [0];
%     y = [0];
%     for j = 1:8
%         x = [x str2num(A(i+j-1,3))];
%         y = [y str2double(A(i+j-1,6))*100];
%     end
%     x = x(2:end);
%     y = y(2:end);
%     plot(x,y,'.-','MarkerSize',20)
% end
% legend_contents = legend_contents.';
% legend(legend_contents,'Location','southeast')
% xlim([0.5,9.5])
% xlabel("# of Followers")
% ylabel("% of Success")
% title("Percentage of Sucess for 2 Leaders vs. Num of Followers")
% 
% figure
% hold on
% legend_contents = [];
% for i = 37:8:68
%     legend_contents = [legend_contents ;A(i,1)];
%     x = [0];
%     y = [0];
%     for j = 1:8
%         x = [x str2num(A(i+j-1,3))];
%         y = [y str2double(A(i+j-1,7))];
%     end
%     x = x(2:end);
%     y = y(2:end);
%     plot(x,y,'.-','MarkerSize',20)
% end
% legend_contents = legend_contents.';
% legend(legend_contents,'Location','southeast')
% xlim([0.5,9.5])
% xlabel("# of Followers")
% ylabel("Ave Runs")
% title("Ave Runs for 2 Leader vs. Num of Followers")
% 
% % Total Ave Runs
% 
% figure
% hold on
% legend_contents = [];
% for i = 1:9:36
%     legend_contents = [legend_contents ;A(i,1) + " 1"];
%     x = [0];
%     y = [0];
%     for j = 1:9
%         x = [x str2num(A(i+j-1,3))];
%         y = [y str2double(A(i+j-1,7))];
%     end
%     x = x(2:end);
%     y = y(2:end);
%     plot(x,y,'.-','MarkerSize',20)
% end
% for i = 37:8:68
%     legend_contents = [legend_contents ;A(i,1) + " 2"];
%     x = [0];
%     y = [0];
%     for j = 1:8
%         x = [x str2num(A(i+j-1,3))];
%         y = [y str2double(A(i+j-1,7))];
%     end
%     x = x(2:end);
%     y = y(2:end);
%     plot(x,y,'.-','MarkerSize',20)
% end
% legend_contents = legend_contents.';
% legend(legend_contents,'Location','southeast')
% xlim([0.5,9.5])
% xlabel("# of Followers")
% ylabel("Ave Runs")
% title("Ave Runs vs. Num of Followers")

% Subplots Ave Runs
f = figure
t = tiledlayout(1,2,'TileSpacing','none');
title(t, "Ave Number of Iterations vs. Num of Followers")
ax1 = nexttile;
hold on
legend_contents = [];
for i = 1:9:36
    legend_contents = [legend_contents ;A(i,1)];
    x = [0];
    y = [0];
    for j = 1:8
        x = [x str2num(A(i+j-1,3))];
        y = [y str2double(A(i+j-1,7))];
    end
    x = x(2:end);
    y = y(2:end);
    plot(x,y,'LineStyle', '--', 'Marker','.','MarkerSize',20)
end
legend_contents = legend_contents.';
%legend(legend_contents,'Location','southeast')
xlim([0.5,8.5])
xlabel("# Followers")
ylabel("# Ave Iterations")
title("1 Leader(s)")

ax2 = nexttile;
hold on
legend_contents = [];
for i = 37:8:68
    legend_contents = [legend_contents ;A(i,1)];
    x = [0];
    y = [0];
    for j = 1:8
        x = [x str2num(A(i+j-1,3))];
        y = [y str2double(A(i+j-1,7))];
    end
    x = x(2:end);
    y = y(2:end);
    plot(x,y,'LineStyle', '--', 'Marker','.','MarkerSize',20)
end

legend_contents = legend_contents.';
legend(legend_contents,'Location','southeast')
xlim([0.5,8.5])
set(ax2,'yticklabel',[])
linkaxes([ax1, ax2], 'y');
xlabel("# Followers")
title("2 Leader(s)")
f.Position = [100 100 800 400];


% Subplots Success Rate
f = figure
t = tiledlayout(1,2,'TileSpacing','none');
title(t, "Success Rate vs. Num of Followers")
ax1 = nexttile;
hold on
legend_contents = [];
for i = 1:9:36
    legend_contents = [legend_contents ;A(i,1)];
    x = [0];
    y = [0];
    for j = 1:8
        x = [x str2num(A(i+j-1,3))];
        y = [y str2double(A(i+j-1,6))*100];
    end
    x = x(2:end);
    y = y(2:end);
    plot(x,y,'LineStyle', '--', 'Marker','.','MarkerSize',20)
end
legend_contents = legend_contents.';
legend(legend_contents,'Location','southwest')
xlim([0.5,8.5])
xlabel("# Followers")
ylabel("% Success Rate")
title("1 Leader(s)")

ax2 = nexttile;
hold on
legend_contents = [];
for i = 37:8:68
    legend_contents = [legend_contents ;A(i,1)];
    x = [0];
    y = [0];
    for j = 1:8
        x = [x str2num(A(i+j-1,3))];
        y = [y str2double(A(i+j-1,6))*100];
    end
    x = x(2:end);
    y = y(2:end);
    plot(x,y,'LineStyle', '--', 'Marker','.','MarkerSize',20)
end

legend_contents = legend_contents.';
%legend(legend_contents,'Location','southwest')
xlim([0.5,8.5])
set(ax2,'yticklabel',[])
linkaxes([ax1, ax2], 'y');
xlabel("# Followers")
title("2 Leader(s)")
f.Position = [200 200 800 450];
