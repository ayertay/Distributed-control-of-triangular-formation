%
% Copyright (c) 2015, Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "license.txt" for license terms.
%
% Project Code: YPML110
% Project Title: Implementation of DBSCAN Clustering in MATLAB
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: S. Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

clc;
clear;
close all;


%% Load Data

% data=load('mydata');
% X=data.X;

Sweep = load('Sweep.txt');
first_run = Sweep;
Angle = first_run(:, 1);
Distance = first_run(:, 2);



X = [cosd(Angle).*Distance, sind(Angle).*Distance ];

%%%%%%%%%%%% Filters %%%%%%%%%%%%%%%%%%%

%Max distance = 200 cm
for i = 1:1:length(X)
    if sqrt(X(i, 1)^2 + X(i, 2)^2) > 300
        X(i,1) = 0;
        X(i,2) = 0;
    end
end

% X = sortrows(X, 1);
% X = flipud(X);
% j_pos = 0;
% j_neg = 0;
% for (i=1:1:length(X))
%     if (X(i) ~= 0 && X(i) > 0)
%         j_pos = j_pos + 1;
%     end
%     if (X(i) ~= 0 && X(i) < 0)
%         j_neg = j_neg + 1;
%     end
% end
% j = j_pos + j_neg;
% j_z = length(X) - j;
% for (i=1:1:j_pos)
%     B(i,1) = X(i,1);
%     B(i,2) = X(i,2);
% end
% for (i=(j_pos+1):1:j)
%     B(i,1) = X(j_z + i,1);
%     B(i,2) = X(j_z + i,2);
% end
%B = sortrows(X,2);
%% Run DBSCAN Clustering Algorithm

epsilon=10;
MinPts=2;
[IDX, isnoise, D] =DBSCAN(X,epsilon,MinPts);
% IDX = kmeans(B,4,'Distance','cityblock')
% Taking out clusters with more than 9 points
avg_dist = zeros(max(IDX), 1);
X_avg = zeros(max(IDX), 1);
Y_avg = zeros(max(IDX), 1);
    for ctr = max(IDX):-1:1
        k = 0;
        for j = 1:1:length(IDX)
            if (IDX(j) == ctr)
                k=k+1;
                X_avg(ctr) = X(j,1) + X_avg(ctr);
                Y_avg(ctr) = X(j,2) + Y_avg(ctr);
                avg_dist(ctr) = sqrt(X(j, 1)^2 + X(j, 2)^2) + avg_dist(ctr);
            end
        end
        X_avg(ctr) = X_avg(ctr)/k;
        Y_avg(ctr) = Y_avg(ctr)/k;
        avg_dist(ctr) = avg_dist(ctr)/k;
        % k - number of points in cluster, 9 is max at closest distance,
        % 800 is 200 cm times 4 cluster points at 200 distance (measure
        % again)
        if ((k > 9) || (k*avg_dist(ctr) > 800))
            X_avg(ctr) = 0;
            Y_avg(ctr) = 0;
            avg_dist(ctr) = 0;
            for j = 1:1:length(IDX)
                if (IDX(j) == ctr)
                    IDX(j) = 0;
                end
            end
        end
    end

%% Plot Results

PlotClusterinResult(X, IDX);
title(['DBSCAN Clustering (\epsilon = ' num2str(epsilon) ', MinPts = ' num2str(MinPts) ')']);
