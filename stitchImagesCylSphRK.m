function stitched = stitchImagesCylSphRK(input, images, cameras, imageNeighbors, ...
                                             blendFeather, ref_idx)
    % STITCHIMAGES_TIGHTROI Stitch multiple images into a panorama.
    %   stitched = STITCHIMAGES_TIGHTROI(images, cameras, ref_idx, blendFeather)
    %   stitches the input images into a single panorama using the provided
    %   camera parameters and reference image index with an automatic bounds 
    %   detected using the homographies.
    %
    %   Faster than the STITCHIMAGES_LARGEROI ~ 0.7000 seconds for the given
    %   images
    %
    %   Inputs:
    %       input - input struture (struct)
    %       images - Cell array containing the input images.
    %       cameras - Struct array containing camera parameters (K and R).
    %       blendFeather - Flag to enable/disable feather blending (0 or 1).
    %       ref_idx - Index of the reference image.
    %
    %   Outputs:
    %       stitched - The resulting stitched panorama image.
    
    % Find the identity matrix index
    if nargin < 6
        cameras_R = {cameras.R};
        ref_idx = find(cellfun(@(x) sum(sum(x - eye(size(x,1)))), cameras_R) == 0);        
        if isempty(ref_idx)
            ref_idx = 1;
        end
    end

    % Get reference camera R and K
    camRefR = cameras(ref_idx).R;
    camRefK = cameras(ref_idx).K;    
    
    % Detect if the panorama is 1D or 2D
    [is1D, panorama1D2DParams] = detectPanoramaType(cameras, ref_idx);

    % Get the panorama limits for the planar, cylindrical and spherical
    % projections and motion models 
    [T, output_view, output_h, output_w] = getPanoramaLimits(input, cameras, images, ...
                                                    camRefR, camRefK, ref_idx);

    % Initialize arrays for warped images and masks
    warped_images = cell(length(images), 1);
    masks = cell(length(images), 1);
    warpedWeights = cell(length(images), 1);

    % First pass: warp all images
    parfor i = 1:length(images)        
        % Project images onto cylindrical or spherical surface
        switch input.warpType
            case 'cylindrical'
                image_projected = image2cylindrical(images{i}, cameras(i).K, input.distCoeffs);
                
                % Compute the transform for the cylindrical projected image
                tform = computeTransform(cameras(i).K, cameras(i).R, ...
                                camRefK, camRefR, input.warpType, ...
                                input.Transformationtype, panorama1D2DParams);

                % Apply translation to ensure all coordinates are positive
                tform = T * tform;
                tform = tform';

            case 'spherical'
                image_projected = image2spherical(images{i}, cameras(i).K, input.distCoeffs);
                
                % Compute the transform for the spherical projected image
                tform = computeTransform(cameras(i).K, cameras(i).R, ...
                                camRefK, camRefR, input.warpType, ...
                                input.Transformationtype, panorama1D2DParams);

                % Apply translation to ensure all coordinates are positive
                tform = T * tform;
                tform = tform';
                
            otherwise
                image_projected = images{i};

                % Compute homography from reference to current view
                tform = computeTransform(cameras(i).K, cameras(i).R, ...
                                camRefK, camRefR, input.warpType, ...
                                input.Transformationtype, panorama1D2DParams);
                
                % Warp image
                tform = T * tform;
                tform = tform';
        end
        
        % Warp image
        warped = imWarp(image_projected, tform, output_view);
        
        % Weights
        if blendFeather == 2
            weight = getWeight(size(images{i}));
            warpedWeight = imWarp(weight, tform, output_view);
            warpedWeights{i} = warpedWeight;  
        end
        
        % Create mask
        mask = any(warped > 0, 3);
        
        % Save warped images and masks
        warped_images{i} = warped;
        masks{i} = double(mask);
    end        

    % Simple feathering at boundaries    
    switch blendFeather
        case 0
            % Blend images directly without feathering
            accumulated = zeros(output_h, output_w, 3, 'double');
            weight_sum = zeros(output_h, output_w, 'double');
            for i = 1:length(images)
                curr_mask = repmat(masks{i}, [1 1 3]);
                accumulated = accumulated + double(warped_images{i}) .* curr_mask;
                weight_sum = weight_sum + masks{i};
            end
            
            % Normalize
            weight_sum(weight_sum < eps) = 1;
            weight_sum = repmat(weight_sum, [1 1 3]);
            stitched = uint8(accumulated ./ weight_sum);
        case 1
            % Blend images directly with feathering
            kernel_size = 3;
            kernel = fspecial('gaussian', [kernel_size kernel_size], kernel_size/6);
            
            for i = 1:length(masks)
                masks{i} = imfilter(masks{i}, kernel, 'replicate');
            end
            
            % Blend images
            accumulated = zeros(output_h, output_w, 3, 'double');
            weight_sum = zeros(output_h, output_w, 'double');
            
            for i = 1:length(images)
                curr_mask = repmat(masks{i}, [1 1 3]);
                accumulated = accumulated + double(warped_images{i}) .* curr_mask;
                weight_sum = weight_sum + masks{i};
            end
            
            % Normalize
            weight_sum(weight_sum < eps) = 1;
            weight_sum = repmat(weight_sum, [1 1 3]);
            stitched = uint8(accumulated ./ weight_sum);    
        case 2
            tic
            % Gain compensation
            if sum(cellfun(@length, imageNeighbors)) == 0 || sum(cellfun(@length, imageNeighbors)) < length(warped_images)
                    gainImages = warped_images;
                    gainpanorama = []; %#ok<*NASGU>
                    gainRGB = []; 
            else  
                [gainpanorama, gainImages, gainRGB] = gainCompensation(input, warped_images, imageNeighbors); %#ok<*ASGLU>
            end
            fprintf('Gain compensation: %f seconds\n', toc);

            mbb = tic;
            stitched = multiBandBlending(input, gainImages, warpedWeights);
            fprintf('Multiband blending: %f seconds\n', toc(mbb));
    end
    
    % Crop black borders
    [rows, cols] = find(rgb2gray(stitched) > 0);
    if ~isempty(rows) && ~isempty(cols)
        min_row = max(1, min(rows) - 10);
        max_row = min(output_h, max(rows) + 10);
        min_col = max(1, min(cols) - 10);
        max_col = min(output_w, max(cols) + 10);
        stitched = stitched(min_row:max_row, min_col:max_col, :);
    end
end

function tform = computeTransform(curr_K, curr_R, ref_K, ref_R, warpType, tranformType, params)
    % COMPUTETRANSFORM Compute transformation matrix based on motion model and projection type
    %   tform = computeTransform(curr_K, curr_R, ref_K, ref_R, warpType, projType, params)
    %
    % Inputs:
    %   curr_K, curr_R - Camera intrinsics and rotation for current image
    %   ref_K, ref_R   - Camera intrinsics and rotation for reference image
    %   warpType       - Motion model: 'translation', 'rigid', 'similarity', 'affine', 'homography'
    %   projType       - Projection type: 'planar', 'cylindrical', 'spherical'
    %   params         - Struct with optional parameters:
    %       .is1DPanorama        - Boolean, true for 1D panorama (default: true)
    %       .usePitchCompensation - Boolean, use pitch compensation for 2D (default: false)
    %       .pitchScale          - Scale factor for pitch compensation (default: 1.0)
    
    % Set default parameters if not provided
    if nargin < 7 || isempty(params)
        params = struct();
    end
    if ~isfield(params, 'is1DPanorama')
        params.is1DPanorama = true;
    end
    if ~isfield(params, 'usePitchCompensation')
        params.usePitchCompensation = false;
    end
    if ~isfield(params, 'pitchScale')
        params.pitchScale = 1.0;
    end

    % Calculate relative rotation to reference view
    R_rel = curr_R / ref_R;
    
    % Get focal lengths
    f_curr = curr_K(1,1);
    f_ref = ref_K(1,1);
    
    % Use average focal length for consistent scale
    f = sqrt(f_curr * f_ref);
    
    % First handle projection type
    switch warpType
        case 'cylindrical'
            % Get yaw angle for cylindrical projection
            yaw = atan2(R_rel(1,3), R_rel(3,3));
            
            % Base transform for cylindrical projection
            base_tform = [1  0  -f*yaw;
                         0  1   0;
                         0  0   1];
                         
        case 'spherical'
            % Get both yaw and pitch angles for spherical projection
            yaw = atan2(R_rel(1,3), R_rel(3,3));
            pitch = atan2(-R_rel(2,3), sqrt(R_rel(1,3)^2 + R_rel(3,3)^2));
            
            % Determine pitch compensation based on parameters
            pitch_compensation = 0;  % Default no compensation
            
            if ~params.is1DPanorama && params.usePitchCompensation
                % Apply pitch compensation with scaling factor
                pitch_compensation = -f * pitch * params.pitchScale;
            end
            
            % Create transformation matrix
            base_tform = [1  0  -f*yaw;
                         0  1  pitch_compensation;
                         0  0   1];
            
        case 'planar'
            % Base homography for planar projection
            base_tform = (ref_K * ref_R * curr_R') / curr_K;
    end
    
    % Then apply motion model constraints
    switch tranformType
        case 'translation'
            % Only keep translation components
            tform = [1  0  base_tform(1,3);
                    0  1  base_tform(2,3);
                    0  0  1];
                    
        case 'rigid'
            % Extract and normalize rotation, keep translation
            R = base_tform(1:2, 1:2);
            t = base_tform(1:2, 3);
            [U, ~, V] = svd(R);
            R_normalized = U * V';
            tform = [R_normalized, t;
                    0 0 1];
                    
        case 'similarity'
            % Allow uniform scaling with rotation and translation
            R = base_tform(1:2, 1:2);
            t = base_tform(1:2, 3);
            [U, S, V] = svd(R);
            scale = mean(diag(S));
            R_normalized = U * V';
            tform = [scale * R_normalized, t;
                    0 0 1];
                    
        case 'affine'
            % Keep all affine components, drop perspective
            tform = [base_tform(1:2, 1:2), base_tform(1:2, 3);
                    0 0 1];
                    
        case 'projective'
            % Use full homography
            tform = base_tform;
            
        otherwise
            error('Unknown motion model: %s', tranformType);
    end
end

function [is1D, params] = detectPanoramaType(cameras, ref_idx)
    % DETECTPANORAMATYPE Automatically detect if panorama is 1D or 2D
    %   [is1D, params] = detectPanoramaType(cameras, ref_idx)
    %
    % Inputs:
    %   cameras - Array of camera structs containing R (rotation matrices)
    %   ref_idx - Index of reference camera (optional)
    %
    % Outputs:
    %   is1D    - Boolean indicating if panorama is 1D (true) or 2D (false)
    %   params  - Struct containing parameters for transform computation
    
    % If ref_idx not provided, find camera with identity rotation
    if nargin < 2
        cameras_R = {cameras.R};
        ref_idx = find(cellfun(@(x) sum(sum(x - eye(size(x,1)))), cameras_R) == 0);
        if isempty(ref_idx)
            ref_idx = 1;
        end
    end
    
    % Get reference rotation
    ref_R = cameras(ref_idx).R;
    
    % Calculate pitch angles relative to reference camera
    pitch_angles = zeros(length(cameras), 1);
    for i = 1:length(cameras)
        % Calculate relative rotation
        R_rel = cameras(i).R / ref_R;
        
        % Extract pitch angle
        pitch = atan2(-R_rel(2,3), sqrt(R_rel(1,3)^2 + R_rel(3,3)^2));
        pitch_angles(i) = pitch;
    end
    
    % Calculate statistics of pitch variations
    pitch_std = std(pitch_angles);
    pitch_range = range(pitch_angles);
    
    % Define thresholds for 2D detection
    % These can be adjusted based on your specific needs
    PITCH_STD_THRESHOLD = 0.05;    % ~3 degrees
    PITCH_RANGE_THRESHOLD = 0.15;   % ~8.6 degrees
    
    % Determine if 2D based on pitch variation
    is2D = (pitch_std > PITCH_STD_THRESHOLD) || ...
           (pitch_range > PITCH_RANGE_THRESHOLD);
    
    is1D = ~is2D;
    
    % Set up parameters struct
    params = struct();
    params.is1DPanorama = is1D;
    
    if is2D
        % For 2D panoramas, enable pitch compensation if variation is significant
        params.usePitchCompensation = true;
        
        % Scale pitch compensation based on the amount of variation
        % Less aggressive compensation for smaller variations
        if pitch_range < 2 * PITCH_RANGE_THRESHOLD
            params.pitchScale = 0.7;  % Reduced compensation for moderate variation
        else
            params.pitchScale = 1.0;  % Full compensation for large variation
        end
    else
        % For 1D panoramas, disable pitch compensation
        params.usePitchCompensation = false;
        params.pitchScale = 0;
    end
    
    % Print detection results
    if is1D
        fprintf('Detected 1D panorama (horizontal)\n');
    else
        fprintf('Detected 2D panorama (pitch std: %.3f, range: %.3f)\n', ...
                pitch_std, pitch_range);
        fprintf('Using pitch scale: %.1f\n', params.pitchScale);
    end
end

function [T, output_view, output_h, output_w] = getPanoramaLimits(input, cameras, images, ...
                                                                  camRefR, camRefK, ref_idx)
    % GETPANORAMALIMITS Computes the panorama limits and transformation matrix
    %   [T, output_view, output_h, output_w] = GETPANORAMALIMITS(input, cameras, images, ref_idx)
    %   calculates the transformation matrix T, the output view size, output height,
    %   and output width for the panorama stitching process based on the input
    %   parameters, camera parameters, and images.
    %
    %   Inputs:
    %       input - Structure containing warp type information
    %       cameras - Array of camera parameters
    %       images - Cell array of images
    %       camRefR - Reference camera rotation (Ri)
    %       camRefK - Reference camera intrinsics matrix (Ki)
    %       ref_idx - Index of the reference image
    %
    %   Outputs:
    %       T - Transformation matrix
    %       output_view - Size of the output view [height, width]
    %       output_h - Output height
    %       output_w - Output width
    % Get reference image dimensions
    [h, w, ~] = size(images{ref_idx});    
    
    % At the start, right after getting ref camera params
    if strcmp(input.warpType, 'cylindrical')
        % Sample points just like planar case
        [X, Y] = meshgrid([1 w/4 w/2 3*w/4 w], [1 h/4 h/2 3*h/4 h]);
        points = [X(:)'; Y(:)'; ones(1, numel(X))];
        
        % Initialize bounds
        min_x = inf; max_x = -inf;
        min_y = inf; max_y = -inf;
        
        % Project points using cylindrical transformations
        parfor i = 1:length(images)
            R_rel = cameras(i).R * camRefR';
            theta = atan2(R_rel(1,3), R_rel(3,3));
            f = cameras(i).K(1,1);
            
            % Calculate x positions with larger range
            x_proj = f * theta * 2 * ones(1, size(points, 2));  % Multiply by 2 to increase range
            y_proj = points(2,:);
            
            min_x = min(min_x, min(x_proj));
            max_x = max(max_x, max(x_proj));
            min_y = min(min_y, min(y_proj));
            max_y = max(max_y, max(y_proj));
        end
        
        % Add larger margin
        margin = 25;  % Increased margin
        output_w = ceil(max_x - min_x) + 2*margin;
        output_h = ceil(max_y - min_y) + 2*margin;
        output_view = [output_h, output_w];
        
        % Create T matrix with larger coverage
        T = [1    0    max_x * 0.6 + margin;    % Double margin for more space
             0    1    min_y + margin;
             0    0    1];

    elseif strcmp(input.warpType, 'spherical')
        % Sample points just like planar case
        [X, Y] = meshgrid([1 w/4 w/2 3*w/4 w], [1 h/4 h/2 3*h/4 h]);
        points = [X(:)'; Y(:)'; ones(1, numel(X))];
        
        % Initialize bounds
        min_x = inf; max_x = -inf;
        min_y = inf; max_y = -inf;
        
        % Project points using spherical transformations
        parfor i = 1:length(images)
            R_rel = cameras(i).R * camRefR';
            
            % Get both longitude (theta) and latitude (phi) angles
            theta = atan2(R_rel(1,3), R_rel(3,3));  % yaw
            phi = atan2(-R_rel(2,3), sqrt(R_rel(1,3)^2 + R_rel(3,3)^2));  % pitch
            
            f = cameras(i).K(1,1);
            
            % Calculate spherical projection coordinates
            x_proj = f * theta * 2 * ones(1, size(points, 2));
            y_proj = points(2,:); %f * phi * 5 * ones(1, size(points, 2));
            
            min_x = min(min_x, min(x_proj));
            max_x = max(max_x, max(x_proj));
            min_y = min(min_y, min(y_proj));
            max_y = max(max_y, max(y_proj));
        end
        
        % Add small margin
        margin = 25;
        output_w = ceil(max_x - min_x) + 2*margin;
        output_h = ceil(max_y - min_y) + 2*margin;
        output_view = [output_h, output_w];
        
        % Create translation matrix using max_x like planar case
        T = [1  0  max_x * 0.6 + margin;
             0  1  min_y + margin;
             0  0  1];
    else
        % Sample more points along the image boundaries for better bound estimation
        [X, Y] = meshgrid([1 w/4 w/2 3*w/4 w], [1 h/4 h/2 3*h/4 h]);
        points = [X(:)'; Y(:)'; ones(1, numel(X))];
        
        % Initialize bounds
        min_x = inf; max_x = -inf;
        min_y = inf; max_y = -inf;
        
        % Project points from all images
        parfor i = 1:length(images)
            H = (camRefK * camRefR * cameras(i).R') / cameras(i).K;
            projected = H * points;
            projected = bsxfun(@rdivide, projected, projected(3,:));
            
            min_x = min(min_x, min(projected(1,:)));
            max_x = max(max_x, max(projected(1,:)));
            min_y = min(min_y, min(projected(2,:)));
            max_y = max(max_y, max(projected(2,:)));
        end
        
        % Add small margin
        margin = 25;
        output_w = ceil(max_x - min_x) + 2*margin;
        output_h = ceil(max_y - min_y) + 2*margin;
        output_view = [output_h, output_w];
        
        % Create translation matrix to ensure all points are positive
        T = [1 0 -min_x+margin;
             0 1 -min_y+margin;
             0 0 1];
    end
end

function warped = imWarp(image, tform, output_view, options)
    % Highly vectorized implementation of image warping
    % Args:
    %   img: Input image (HxWx3 uint8)
    %   tform: projective2d transform object
    %   output_view: imref2d object defining output limits
    %   options: struct with fields:
    %     - method: 'nearest', 'bilinear', or 'bicubic'
    %     - fill_value: value for outside pixels (default: 0)
    
    % Default options
    if nargin < 4, options = struct(); end
    if ~isfield(options, 'method'), options.method = 'bilinear'; end
    if ~isfield(options, 'fill_value'), options.fill_value = 0; end
    
    % Get dimensions
    [out_height, out_width] = deal(output_view(1), output_view(2));
    [in_height, in_width, num_channels] = size(image);
    total_pixels = out_height * out_width;
    
    % Pre-compute source coordinates
    [X, Y] = meshgrid(single(1:out_width), single(1:out_height));
    src_coords = reshape(tform' \ [X(:)'; Y(:)'; ones(1, total_pixels, 'single')], 3, []);
    src_x = reshape(src_coords(1,:) ./ src_coords(3,:), out_height, out_width);
    src_y = reshape(src_coords(2,:) ./ src_coords(3,:), out_height, out_width);
    
    % Initialize output
    warped = zeros([out_height, out_width, num_channels], class(image)) + options.fill_value;
    
    switch lower(options.method)
        case 'nearest'
            % Round coordinates and create mask
            x = round(src_x);
            y = round(src_y);
            valid = x >= 1 & x <= in_width & y >= 1 & y <= in_height;
            
            % Convert to linear indices
            indices = sub2ind([in_height, in_width], y(valid), x(valid));
            valid_linear = valid;
            
            % Process all channels at once using reshaping
            img_reshaped = reshape(image, [], num_channels);
            warped_reshaped = reshape(warped, [], num_channels);
            warped_reshaped(valid_linear, :) = img_reshaped(indices, :);
            warped = reshape(warped_reshaped, out_height, out_width, num_channels);
            
        case 'bilinear'
            % Floor coordinates and compute weights
            x1 = floor(src_x);
            y1 = floor(src_y);
            x2 = x1 + 1;
            y2 = y1 + 1;
            
            wx = src_x - x1;
            wy = src_y - y1;
            
            % Find valid coordinates
            valid = x1 >= 1 & x2 <= in_width & y1 >= 1 & y2 <= in_height;
            valid_linear = find(valid);
            
            if ~isempty(valid_linear)
                % Get corner indices for valid pixels
                i11 = sub2ind([in_height, in_width], y1(valid), x1(valid));
                i12 = sub2ind([in_height, in_width], y2(valid), x1(valid));
                i21 = sub2ind([in_height, in_width], y1(valid), x2(valid));
                i22 = sub2ind([in_height, in_width], y2(valid), x2(valid));
                
                % Extract weights for valid pixels
                wx = wx(valid);
                wy = wy(valid);
                
                % Compute weights
                w11 = (1-wx).*(1-wy);
                w12 = (1-wx).*wy;
                w21 = wx.*(1-wy);
                w22 = wx.*wy;
                
                % Process all channels simultaneously using matrix operations
                img_reshaped = reshape(double(image), [], num_channels);
                warped_reshaped = reshape(warped, [], num_channels);
                
                % Vectorized interpolation for all channels
                interp_vals = w11 .* img_reshaped(i11,:) + ...
                             w12 .* img_reshaped(i12,:) + ...
                             w21 .* img_reshaped(i21,:) + ...
                             w22 .* img_reshaped(i22,:);
                
                warped_reshaped(valid_linear, :) = interp_vals;
                warped = reshape(warped_reshaped, out_height, out_width, num_channels);
            end
            
        case 'bicubic'
            % Floor coordinates
            x = floor(src_x);
            y = floor(src_y);
            dx = src_x - x;
            dy = src_y - y;
            
            % Find valid coordinates (need one extra pixel on each side for bicubic)
            valid = x >= 2 & x <= in_width-2 & y >= 2 & y <= in_height-2;
            valid_linear = find(valid);
            
            if ~isempty(valid_linear)
                % Extract valid coordinates
                x_valid = x(valid);
                y_valid = y(valid);
                dx_valid = dx(valid);
                dy_valid = dy(valid);
                
                % Pre-compute weights for x and y directions
                x_weights = zeros(length(valid_linear), 4, 'single');
                y_weights = zeros(length(valid_linear), 4, 'single');
                
                % Compute x weights vectorized
                for i = -1:2
                    x_weights(:,i+2) = bicubic_kernel(i - dx_valid);
                    y_weights(:,i+2) = bicubic_kernel(i - dy_valid);
                end
                
                % Create indices matrix for all 16 sample points
                indices = zeros(length(valid_linear), 16, 'single');
                idx = 1;
                for dy = -1:2
                    for dx = -1:2
                        indices(:,idx) = sub2ind([in_height, in_width], ...
                            y_valid+dy, x_valid+dx);
                        idx = idx + 1;
                    end
                end
                
                % Process all channels simultaneously
                if isinteger(image)
                    img_class = class(image);
                    max_val = double(intmax(img_class));
                    img_reshaped = reshape(double(image), [], num_channels); % Use double for better precision
                else
                    img_reshaped = reshape(double(image), [], num_channels);
                    max_val = 1.0;
                end
                
                warped_reshaped = reshape(warped, [], num_channels);
                
                % Get all sample points for all channels
                samples = img_reshaped(indices, :);
                samples = reshape(samples, [], 4, 4, num_channels);
                
                % Apply bicubic interpolation using matrix operations
                interp_vals = zeros(size(samples, 1), num_channels, 'double');
                for c = 1:num_channels
                    temp = squeeze(samples(:,:,:,c));
                    % Reshape temp to handle all points correctly
                    temp_reshaped = reshape(temp, [], 4, 4);
                    
                    % First interpolate in x direction
                    x_interp = zeros(size(temp_reshaped, 1), 4, 'double');
                    for i = 1:4
                        for j = 1:4
                            x_interp(:,j) = x_interp(:,j) + temp_reshaped(:,i,j) .* x_weights(:,i);
                        end
                    end
                    
                    % Then interpolate in y direction
                    interp_vals(:,c) = sum(x_interp .* y_weights, 2);
                end
                
                % Handle output conversion
                if isinteger(image)
                    % For integer types, clamp to valid range
                    interp_vals = max(0, min(max_val, round(interp_vals)));
                end
                
                warped_reshaped(valid_linear, :) = cast(interp_vals, class(image));
                warped = reshape(warped_reshaped, out_height, out_width, num_channels);
            end
    end

    function w = bicubic_kernel(x)
        % Vectorized bicubic kernel
        absx = abs(x);
        w = zeros(size(x), 'single');
        
        mask1 = absx <= 1;
        mask2 = absx <= 2 & ~mask1;
        
        absx2 = absx.^2;
        absx3 = absx.^3;
        
        w(mask1) = 1.5*absx3(mask1) - 2.5*absx2(mask1) + 1;
        w(mask2) = -0.5*absx3(mask2) + 2.5*absx2(mask2) - 4*absx(mask2) + 2;
    end
end

function [weight] = getWeight(size)
    % GETWEIGHT Computes the weight matrix for blending
    %   [weight] = GETWEIGHT(size) calculates the weight matrix used for
    %   blending images based on the input size.
    %
    %   Inputs:
    %       size - A vector containing the dimensions of the image [height, width, channels]
    %
    %   Outputs:
    %       weight - The computed weight matrix for blending

    h = size(1);
    w = size(2);
    c = size(3);

    % Compute horizontal weights
    wx = ones(1, w);
    wx(1:ceil(w/2)) = linspace(0, 1, ceil(w/2));
    wx(floor(w/2 + 1):w) = linspace(1, 0, w - floor(w/2));
    wx = repmat(wx, h, 1, c);

    % Compute vertical weights
    wy = ones(h, 1);
    wy(1:ceil(h/2)) = linspace(0, 1, ceil(h/2));
    wy(floor(h/2 + 1):h) = linspace(1, 0, h - floor(h/2));
    wy = repmat(wy, 1, w, c);

    % Combine horizontal and vertical weights
    weight = wx .* wy;
end

function stitched = simpleBlendImages(warped_images, masks, options)
    % Blend warped images using masks with optional feathering
    % Args:
    %   warped_images: Cell array of warped images
    %   masks: Cell array of binary masks
    %   options: Struct with fields:
    %     - feather: boolean (enable/disable feathering)
    %     - kernel_size: size of Gaussian kernel (default: 31)
    %     - sigma: Gaussian sigma (default: kernel_size/6)
    %     - crop: boolean (enable/disable border cropping)
    %     - crop_padding: padding for cropping (default: 10)
    %
    % Example usage:   
    % Without feathering
    % options_no_feather = struct('feather', false, 'crop', true);
    % stitched = simpleBlendImages(warped_images, masks, options_no_feather);
    
    % % With feathering
    % options_feather = struct('feather', true, ...
    %                         'kernel_size', 31, ...
    %                         'sigma', 5, ...
    %                         'crop', true);
    % stitched = simpleBlendImages(warped_images, masks, options_feather);
    % 
    % % With custom parameters
    % options_custom = struct('feather', true, ...
    %                        'kernel_size', 51, ...
    %                        'sigma', 10, ...
    %                        'crop', true, ...
    %                        'crop_padding', 20);
    % stitched = simpleBlendImages(warped_images, masks, options_custom);

    % Default options
    if nargin < 3, options = struct(); end
    if ~isfield(options, 'feather'), options.feather = true; end
    if ~isfield(options, 'kernel_size'), options.kernel_size = 31; end
    if ~isfield(options, 'sigma'), options.sigma = options.kernel_size/6; end
    if ~isfield(options, 'crop'), options.crop = true; end
    if ~isfield(options, 'crop_padding'), options.crop_padding = 10; end
    
    % Get dimensions
    [output_h, output_w, ~] = size(warped_images{1});
    num_images = length(warped_images);
    
    % Convert all images to double for computation
    warped_double = cellfun(@double, warped_images, 'UniformOutput', false);
    
    if options.feather
        % Create Gaussian kernel
        kernel = fspecial('gaussian', ...
                         [options.kernel_size options.kernel_size], ...
                         options.sigma);
        
        % Apply feathering to all masks at once
        masks = cellfun(@(m) imfilter(m, kernel, 'replicate'), ...
                       masks, 'UniformOutput', false);
    end
    
    % Stack all masks into a 3D array for vectorized operations
    mask_stack = cat(3, masks{:});
    
    % Initialize accumulators
    accumulated = zeros(output_h, output_w, 3, 'double');
    weight_sum = sum(mask_stack, 3);
    
    % Blend all images at once
    for i = 1:num_images
        curr_mask = repmat(masks{i}, [1 1 3]);
        accumulated = accumulated + warped_double{i} .* curr_mask;
    end
    
    % Normalize
    weight_sum(weight_sum < eps) = 1;
    weight_sum_3d = repmat(weight_sum, [1 1 3]);
    stitched = uint8(accumulated ./ weight_sum_3d);
    
    % Crop black borders if requested
    if options.crop
        gray_img = rgb2gray(stitched);
        [rows, cols] = find(gray_img > 0);
        
        if ~isempty(rows) && ~isempty(cols)
            min_row = max(1, min(rows) - options.crop_padding);
            max_row = min(output_h, max(rows) + options.crop_padding);
            min_col = max(1, min(cols) - options.crop_padding);
            max_col = min(output_w, max(cols) + options.crop_padding);
            stitched = stitched(min_row:max_row, min_col:max_col, :);
        end
    end
end