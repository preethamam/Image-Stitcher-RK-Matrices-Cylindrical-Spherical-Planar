%//%*****************************************************************************%
%//%*                 Automatic Panorama Image Stitcher                         *%
%//%*   Stitches panorama image given the Rotation and camera intrinsic         *%
%//%*                 matrices after the bundle adjustment                      *%
%//%*                    Name: Dr. Preetham Manjunatha                          *%
%//%*               GitHub: https://github.com/preethamam	                    *%
%//%*             Repo Name: AutoPanoStitch (auxiliary function)                *%
%//%*                    Written Date: 01/08/2025                               *%
%%**************************************************************************    *%
%* Citation 1: Automatic Panoramic Image Stitching using Invariant Features.    *% 
%* M. Brown and D. Lowe. International Journal of Computer Vision. 74(1),       *%
%* pages 59-73, 2007                                                            *%
%*                                                                              *%
%* Citation 2: Recognising Panoramas. M. Brown and D. G. Lowe.                  *%
%* International Conference on Computer Vision (ICCV2003). pages 1218-1225,     *%
%* Nice, France, 2003.                                                          *%
%*                                                                              *%
%* Please refer to the GitHub repo:                                             *%
%* https://github.com/preethamam/AutomaticPanoramicImageStitching-AutoPanoStitch*%
%* for the full Automatic Panorama Image Stitcher (AutoPanoStitch).             *%
%********************************************************************************%

%% Start
%--------------------------------------------------------------------------
clear; close all; clc;
clcwaitbarz = findall(0,'type','figure','tag','TMWWaitbar');
delete(clcwaitbarz);
warning('off','all');
Start = tic;

%% Get inputs
%--------------------------------------------------------------------------
% Inputs file
%--------------------------------------------------------------------------
% 0 - No blend | 1 - Feathering | 2 - Gain compensation + multiband blending
blendFeather = 2;  
imagesFolder = 'images';
inputs;

%--------------------------------------------------------------------------
% Load files
%--------------------------------------------------------------------------
if strcmp(imagesFolder, 'images')
    % For images folder
    load cameras.mat 
else
    % For images 2 folder
    load cameras2.mat %#ok<*UNRCH> 
end

%% Load images
loadimagestic = tic;
[keypoints, allDescriptors, images, imageSizes, imageNames, numImg] = loadImages(input, imagesFolder);
fprintf('Loading images (+ features): %f seconds\n', toc(loadimagestic));

%% Get feature matrices and keypoints    
featureMatchtic = tic;
matches = featureMatching(input, allDescriptors, numImg);
fprintf('Matching features : %f seconds\n', toc(featureMatchtic));

%% Find matches        
imageMatchtic = tic;
[allMatches, numMatches, initialTforms] = imageMatching(input, numImg, keypoints, matches, images);
fprintf('Matching images: %f seconds\n', toc(imageMatchtic));               

%% Get gain images neighbors
imNtic = tic;
imageNeighbors = gainImageNeighbors(input, numMatches);

fprintf('Image neighbors: %f seconds\n', toc(imNtic));

%% Automatic panorama straightening
straightentic = tic;
bundlerTforms = {cameras};
finalPanoramaTforms = straightening(bundlerTforms);
fprintf('Automatic panorama straightening: %f seconds\n', toc(straightentic));

%% Stitch images
cameras = finalPanoramaTforms{1};

% Loop through all type of motion model and warp/projection types
% (demostration purpose)
transformationtype = {'projective', 'affine', 'similarity', 'rigid', 'translation'};
warpType = {'planar', 'cylindrical', 'spherical'};

panorama =  cell(1, length(warpType));
croppedPanorama =  cell(1, length(warpType));

for i = 1:length(transformationtype)
    fh = figure(1);    
    fh.WindowState = 'maximized';
    tiledlayout(3, 2, TileSpacing="tight", Padding="tight");
    for j = 1:length(warpType)
        % Get transformation type and warp/projection type
        input.Transformationtype = transformationtype{i};
        input.warpType = warpType{j};

        customrender = tic;
        % Render panorama
        panorama{j} = stitchImagesCylSphRK(input, images, cameras, imageNeighbors{1}, blendFeather);
        fprintf('Custom render: %f seconds\n', toc(customrender));

        % Crop image
        croppedPanorama{j}  = panoramaCropper(input, panorama{j});        
    end

    % Show panorama
    word = transformationtype{i};
    transTitle = strcat(upper(word(1)), lower(word(2:end)));

    nexttile; imshow(panorama{1}); title([transTitle ' ' warpType{1}], 'fontsize', 25)        
    nexttile; imshow(croppedPanorama{1}); title([transTitle ' ' warpType{1} ' ' 'cropped'], 'fontsize', 25)

    nexttile; imshow(panorama{2}); title([transTitle ' ' warpType{2}], 'fontsize', 25)     
    nexttile; imshow(croppedPanorama{2}); title([transTitle ' ' warpType{2} ' ' 'cropped'], 'fontsize', 25)

    nexttile; imshow(panorama{3}); title([transTitle ' ' warpType{3}], 'fontsize', 25)
    nexttile; imshow(croppedPanorama{3}); title([transTitle ' ' warpType{3} ' ' 'cropped'], 'fontsize', 25)
    
    % Export graphics
    ax = gcf;
    exportgraphics(ax, fullfile('assets', [transformationtype{i} '_panorama.png']))

    % Refresh figure
    clf
end

%% End parameters
%--------------------------------------------------------------------------
clcwaitbarz = findall(0,'type','figure','tag','TMWWaitbar');
delete(clcwaitbarz);
statusFclose = fclose('all');
if(statusFclose == 0)
    disp('All files are closed.')
end
Runtime = toc(Start);
fprintf('Total runtime : %f seconds\n', Runtime);
currtime = datetime('now');
display(currtime)