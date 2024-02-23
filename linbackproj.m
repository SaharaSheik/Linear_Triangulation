function [pntcloud, worldPoints, CloudSNR] = linbackproj(im1,im2,cam1,cam2,baseline,width,height)
% This function computes matching points between a pair of images and uses
% the input camera parameters to linealy back-project the matching points
% into the 3D space to construct a 3D point cloud.
% 
% im1 and im2 are the input stereo images
% cam1 and cam2 are the intrinsic parameters of the camera corresponding to
% im1 and im2
% baseline is the stereo baseline (physical distance between the two camera
% centers)
% width and height are the width and height of the images.
%
% The function returns:
%   pntcloud, which is a 3D point cloud computed using the linear 
%   backprojection mehod described in Hartley-Zisserman
%
%   worldPoints, which is a 3D point cloud computed using also the linear
%   backprojection method but implemented by a built-in Matlab function
%   called triangulate
%
%   CloudSNR, which is the signal-to-noise ratio of computed pntcloud with
%   respect to the reference point cloud worldPoints
% 
%   Author: Dr. Hassan Foroosh, 2023
%

%% Keep the original images 
%colorim1=im1;
%colorim2=im2;


colorim1=im1;
colorim2=im2;

%% Make grayscale copies of images, if they are color images
if size(im1,3)>0
    im1=rgb2gray(im1);
end
if size(im2,3)>0
     im2=rgb2gray(im2);
end

%% Detect feature points
% points1 = detectHarrisFeatures(im1);
% points2 = detectHarrisFeatures(im2);
% points1 = detectSURFFeatures(im1);
% points2 = detectSURFFeatures(im2);
points1 = detectORBFeatures(im1);
points2 = detectORBFeatures(im2);
[features1,valid_points1] = extractFeatures(im1,points1);
[features2,valid_points2] = extractFeatures(im2,points2);
indexPairs = matchFeatures(features1,features2);
%% Match feature points
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);
% Disply matching points
figure; showMatchedFeatures(colorim1,colorim2,matchedPoints1,matchedPoints2,"montag");
title("Matching Points");

Pnts1=matchedPoints1.Location;
Pnts2=matchedPoints2.Location;

%% Find the camera intrinsic and extrinsic parameters
% Assume fixed intrinsic parameters
K=(cam1+cam2)/2;

% Estimate the fundamental matrix
F = estimateFundamentalMatrix(Pnts1,Pnts2);
% Matlab uses a special structure for intrinsics
intrinsics = cameraIntrinsics([K(1,1) K(2,2)],[K(1,3) K(2,3)],[height,width]);
% Estimate the extrinsic parameters (Rotation matrix and translation up to scale)
[R,t] = relativeCameraPose(F,intrinsics,Pnts1,Pnts2);

%% Build the two camera matrices with world coordinate frame attached to the first camera 
P1=K*[eye(3),zeros(3,1)];
% Translation is scaled by the baseline distance to correct for the unknown scale 
P2=K*[R',t'*baseline];

N=size(Pnts1,1);
color=zeros(3,N);
for n = 1:N
    color(:,n)=double(colorim1(round(Pnts1(n,2)),round(Pnts1(n,1)),:));
end

pntcloud = backproject(P1,P2,Pnts1,Pnts2);

% Matlab's pcshow refused to display the colors - not sure why! Or, maybe
% they are there but difficult to see because of sparsity of point clouds.
% It seems for the globe data set the colors are there.
figure; pcshow(pntcloud',color'); axis vis3d
title("Pointcloud from My Implementation");
xlabel("X");
ylabel("Y");
zlabel("Z");

%% Matlab's built-in implementation to compare with
worldPoints = triangulate(matchedPoints1,matchedPoints2,P1',P2');
figure; pcshow(worldPoints,color'); axis vis3d
title("Pointcloud from Matlab's Built-in Function");
xlabel("X");
ylabel("Y");
zlabel("Z");

%% Compute the signal-to-noise ratio w.r.t. Matlab's built-in implementation
% snr and psnr are reserved words in matlab, so I used cloudsnr
err=pntcloud'-worldPoints;
CloudSNR=10*log10(var(pntcloud(:))/var(err(:)));
disp(num2str(CloudSNR))

end

function pntcloud = backproject(P1,P2,Pnts1,Pnts2)

N=size(Pnts1,1);
pntcloud=zeros(3,N);

% c
for i = 1:length(Pnts1)

%create a 4x4 matrix for A

    A = zeros(4, 4);
%create each row of the 4x4 matrix based on HW 
    A(:,:) = [
        Pnts1(i, 1)*P1(3, :) - P1(1, :);
        Pnts1(i, 2)*P1(3, :) - P1(2, :);
        Pnts2(i, 1)*P2(3, :) - P2(1, :);
        Pnts2(i, 2)*P2(3, :) - P2(2, :)
    ];
   
 %Singular Value Decomposition of matrix A
    [~, ~, V] = svd(A);
    M_homogeneous_i = V(:, end);
 %Create a 3D point cloud
    pntcloud(:, i) = M_homogeneous_i(1:3) / M_homogeneous_i(4);

end

% Step 5: Singular Value Decomposition of matrix A
%[~, ~, V] = svd(A);
%disp(size(A))
%disp(size(V))
% Step 6: Create a 3D point cloud

%for i = 1:length(Pnts1)
   % M_homogeneous_i = V(:, end);
    %disp((M_homogeneous_i))
    %pntcloud(i, :) = M_homogeneous_i(1:3) / M_homogeneous_i(4);
%end

%pntcloud = pntcloud'
%% This needs to be implemented
end

