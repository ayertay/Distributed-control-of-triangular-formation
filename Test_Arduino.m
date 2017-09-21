close all
X_data = load('Test.txt');
X = X_data(:, 1);
Y = X_data(:, 2);
IDX = X_data(:,3);
XY = [X Y];
PlotClusterinResult(XY, IDX);
title(['DBSCAN Clustering (\epsilon = ' num2str(epsilon) ', MinPts = ' num2str(MinPts) ')']);