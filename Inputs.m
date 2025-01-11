
%% Inputs 2
%--------------------------------------------------------------------------
% Parallel workers
input.numCores = str2double(getenv('NUMBER_OF_PROCESSORS'));    % Number of cores for parallel processing
input.poolType = 'numcores';     % 'numcores' | 'Threads'

%% Inputs 3
% Warping
input.warpType = 'spherical';   % 'planar' | 'cylindrical' | 'spherical'

% Lens distortion coefficients [k1, k2, k3, p1, p2]
input.distCoeffs = [0, 0, 0, 0, 0];

% Feature matching
input.detector = 'SIFT';                % 'HARRIS' | 'SIFT' | 'vl_SIFT' | 'FAST' | 'SURF' | 'BRISK' | 'ORB' | 'KAZE'
input.Matchingthreshold = 3.5;          % 10.0 or 1.0 (default) | percent value in the range (0, 100] | depends on 
                                        % binary and non-binary features (3.5)
input.Ratiothreshold = 0.6;             % ratio in the range (0,1]
input.Sigma = 1.6;                      % Sigma of the Gaussian (1.4142135623)
input.NumLayersInOctave = 4;            % Number of layers in each octave -- SIFT only
input.ContrastThreshold = 0.00133;      % Contrast threshold for selecting the strongest features, 
                                        % specified as a non-negative scalar in the range [0,1]. 
                                        % The threshold is used to filter out weak features in 
                                        % low-contrast regions of the image. -- SIFT only
input.EdgeThreshold = 6;                % Edge threshold, specified as a non-negative scalar greater than or equal to 1. 
                                        % The threshold is used to filter out unstable edge-like features  -- SIFT only  
input.nearestFeaturesNum = 5;           % Nearest images minimum number of features to filter
                                        % distant image matches (filter gain overlap images to reduce time complexity)

% Image matching (RANSAC)
input.Matchingmethod = 'Approximate';   %'Exhaustive' (default) | 'Approximate'
input.Inliersconfidence = 99.9;         % Inlier confidence [0,100]
input.maxIter = 500;                    % RANSAC maximum iterations
input.Transformationtype = 'projective';     % Motion model:  'projective' | 'affine' | 'similarity' | 'rigid' | 'translation'
input.MaxDistance = 1.50;               % Maximum distance (pixels) 1.5

% Image blending and panorama
input.gainDerivation = 1;           % Two types of gain matrix derivations 1 or 2 (both leads to same results with some roundoffs)
input.sigmaN = 10;                  % Standard deviations of the normalised intensity error
input.sigmag = 0.1;                 % Standard deviations of the gain
input.resizeImage = 0;              % Resize input images
input.resizeStitchedImage = 0;      % Resize stitched image
input.maxPanoramaArea = 3e6;        % Maximum panorama area
input.blending = 'multiband';       % 'multiband' | 'linear' | 'none'
input.bands = 6;                    % bands
input.MBBsigma = 5;                 % Multi-band Gaussian sigma
input.filtSize = [5,5];             % Gaussian kernel Filter size
input.parforSummation = true;       % Gain diagonal elements summation by parfor
input.showPanoramaImgsNums = 0;     % display the panorama images with numbers after tranform 0 or 1

% Post-processing
input.canvas_color = 'black';       % Panorama canvas color 'black' | 'white'
input.showCropBoundingBox = 0;      % Display cropping bounding box 0 | 1
input.blackRange = 0;               % Minimum dark pixel value to crop panaroma
input.whiteRange = 250;             % Minimum bright pixel value to crop panaroma
input.showKeypointsPlot  = 0;       % Display keypoints plot (parfor suppresses this flag, so no use)
input.displayPanoramas = 0;         % Display panoramas in figure
