

%%
tic
%%% Load the Point Cloud Data
disp('Project Title: Object Detection in Point Cloud Data')
disp(' ------------------------------------------------- ')
disp('Loading Cross-Road Point Cloud Data')
ptCloud1 = pcread('E:\IITH BOOKS\LIDAR\PCD11\1317022804.762056112.pcd');
disp('Done !!! ')
disp(' ------------------------------------------------- ')
pause(1)

%% Downsampling using Voxel Grid Filter and k-NN algorithm with weighted average.
% gridStep = 0.08721834218;
gridStep = input('Enter the Voxel Grid Filter - Grid_Size: ');
disp('Downsampling')
ptCloud2 = pcdownsample(ptCloud1,'gridAverage',gridStep);
disp('Done !!! ')
disp(' ------------------------------------------------- ')
pause(1)
%% Noise removal or Filtering to remove 'Shadow Points'
disp('Shadow Points Filtering')
% numNeighs = 50;
numNeighs = input('Enter the min-neighbours: ');
ptCloud3 = pcdenoise(ptCloud2,'NumNeighbors',numNeighs);
disp('Done !!! ')
disp(' ------------------------------------------------- ')
pause(1)

%% PCD  size
disp('Size of LiDAR (raw/input) point cloud data')
length(ptCloud1.Location(:,1))
disp('Size of Downsampled pcd')
length(ptCloud2.Location(:,1))
disp('Size of Filtered pcd')
length(ptCloud3.Location(:,1))
disp(' ------------------------------------------------- ')
pause(1)

%% Finding points of interest in the filtered point cloud data
%%% Clustering with Ground
disp('Finding points of interest in the filtered point cloud data')

width_x = ptCloud1.XLimits(2) - ptCloud1.XLimits(1);
width_y = ptCloud1.YLimits(2) - ptCloud1.YLimits(1);
distThreshold = max(width_x/abs(max(ptCloud1.XLimits(2),ptCloud1.XLimits(1))),...
    width_y/abs(max(ptCloud3.YLimits(2),ptCloud3.YLimits(1))));

%%% Clustering based on min-Euclidean Distance
[labels,numClusters] = pcsegdist(ptCloud3,distThreshold);
labelColorIndex = labels;

max_len = 0;
max_idx = [];

%%% Finding cluster with largest number of points
for i = 1:numClusters
    idx_len = find(labels == i);
    if length(idx_len) >= max_len
        max_len = length(idx_len);
        max_idx = idx_len;
    end  
end

ptCloud4 = select(ptCloud3,max_idx);
disp('Done !!! ')
disp(' ------------------------------------------------- ')
disp('Size of pcd with points within ROI')
length(ptCloud4.Location(:,1))
disp(' ------------------------------------------------- ')
pause(1)

%% Ground Removal using plane-fitting (RANSAC algorithm)
disp('Ground Removal using plane-fitting (RANSAC algorithm)')

%%% Plane fitting over an area of bin size 5X5
binsize_x = 5; 
binsize_y = 5; 
width_x = ptCloud4.XLimits(2) - ptCloud4.XLimits(1);
width_y = ptCloud4.YLimits(2) - ptCloud4.YLimits(1);
Nbins_x = width_x/binsize_x;
Nbins_y = width_y/binsize_y;
figure(6)
hist = histogram2(ptCloud4.Location(:,1),ptCloud4.Location(:,2),sum(round([Nbins_x Nbins_y])));
xlabel('x')
ylabel('y')

index = [];
% maxDistance = 0.05;
% maxAngularDistance = 8;
maxDistance = input('Enter the maxDistance threshold: ');
maxAngularDistance = input('Enter the maxAngularDistance threshold: ');

referenceVector = [0,0,1];

for i = 1:size(hist.Values,1)
    %%% Finding points with x-coordinate within the bin.
    idx_x = find(hist.XBinEdges(i) <= ptCloud4.Location(:,1) & ptCloud4.Location(:,1) <= hist.XBinEdges(i+1));
    
    for j = 1:size(hist.Values,2)
        %%% Finding points with y-coordinate within the bin.
        idx_y = find(hist.YBinEdges(j) <= ptCloud4.Location(:,2) & ptCloud4.Location(:,2) <= hist.YBinEdges(j+1));
        idx = intersect(idx_x,idx_y);
        
        if ~isempty(idx)
            ptCloud5_temp = select(ptCloud4,idx);
            %%% Bin Ground removal
            [~,inlierIndices,outlierIndices] = pcfitplane(ptCloud5_temp,...
                        maxDistance,referenceVector,maxAngularDistance);
            index = [index; ptCloud5_temp.Location(outlierIndices,:)];
        end
    end
end
%
ptCloud5 = pointCloud(index);
disp('Done !!! ')
disp(' ------------------------------------------------- ')
%%% Noise removal after removing ground
ptCloud6 = pcdenoise(ptCloud5,'NumNeighbors',10);
pause(1)
%% Unwanted Cluster removal
disp('Unwanted Cluster removal')
disp(' -------- Before removing -------- ')

%% Clustering based on min-Euclidean Distance
[labels_f,numClusters_f] = pcsegdist(ptCloud6,0.3);
labelColorIndex_f = labels_f;

disp(['Number of Clusters: ', num2str(numClusters_f)])

binc = 1:numClusters_f;
counts = histcounts(labels_f,numClusters_f);
result = [binc; counts];
rem_cluster = find(result(2,:) < 51);

idx = [];
for i = 1:length(rem_cluster)
    idx = [idx; find(labels_f == rem_cluster(i))];
end

ptCloud7 = select(ptCloud6,setdiff(1:length(ptCloud6.Location(:,1)),idx));

%% Clustering based on min-Euclidean Distance
[labels_ff,numClusters_ff] = pcsegdist(ptCloud7,0.5);
labelColorIndex_ff = labels_ff;

disp(' -------- After removing -------- ')
disp(['Number of Clusters: ', num2str(numClusters_ff)])

disp('Done !!! ')
disp(' ------------------------------------------------- ')
pause(1)

%% CAR-Feature Labelling
%%%% single car

%%% Finding centroid of the feature and average distance of data points
%%% from the centroid.

% idx_x = find(4 <= ptCloud7.Location(:,1) & ptCloud7.Location(:,1) <= 8);
% idx_y = find(16 <= ptCloud7.Location(:,2) & ptCloud7.Location(:,2) <= 18);
% idx = intersect(idx_x,idx_y);
% ptCloud8_temp = select(ptCloud7,idx);
% pcshow(ptCloud8_temp)
% [~,~,~,D] = kmeans(ptCloud8_temp.Location,1);
% param = sum(D)/length(D);


%% Plot the results..
figure(1)
pcshow(ptCloud1)
xlabel('x')
ylabel('y')
zlabel('z')
title('Input Point Cloud Data')
figure(2)
pcshow(ptCloud2)
xlabel('x')
ylabel('y')
zlabel('z')
title('Downsampled Point Cloud Data')
figure(3)
pcshow(ptCloud3)
xlabel('x')
ylabel('y')
zlabel('z')
title('Filtered Point Cloud Data')
figure(4)
pcshow(ptCloud3.Location,labelColorIndex)
colormap(lines(numClusters))
xlabel('x')
ylabel('y')
zlabel('z')
%title('Clustered Point Cloud with Ground')
figure(5)
pcshow(ptCloud4)
xlabel('x')
ylabel('y')
zlabel('z')
%title('PCD with point within ROI')
figure(7)
pcshow(ptCloud5)
xlabel('x')
ylabel('y')
zlabel('z')
title('PCD with Ground Removal')
figure(8)
pcshow(ptCloud6)
xlabel('x')
ylabel('y')
zlabel('z')
title('Ground Removed Data after filtering')
figure(9)
pcshow(ptCloud6.Location,labelColorIndex_f)
colormap(lines(numClusters_f))
xlabel('x')
ylabel('y')
zlabel('z')
title('Clustered Point Cloud without Ground')
figure(10)
pcshow(ptCloud7)
xlabel('x')
ylabel('y')
zlabel('z')
title('Point Cloud without Ground after removing unwanted clusters')
% figure(11)
% pcshow(ptCloud7.Location,labelColorIndex_ff)
% colormap(lines(numClusters_ff))
% xlabel('x')
% ylabel('y')
% zlabel('z')
% title('Clustered Point Cloud without Ground')

%% Detecting Cars
param = 1.3641;
disp('Detecting Cars')
figure(12)
for i = 1:numClusters_ff
    idx = find(labels_ff == i);
    clust = ptCloud7.Location(idx,:);
    [~,~,~,D_temp] = kmeans(clust,1);
    param_temp = sum(D_temp)/length(D_temp);
    if (param_temp < param + 0.4) && (param_temp > param - 0.4)
        pcshow(clust,'c');
        hold on;
    end
end
disp('Done !!! ')
disp(' ------------------------------------------------- ')

toc
 
