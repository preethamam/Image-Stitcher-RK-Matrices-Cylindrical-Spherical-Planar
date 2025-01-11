function straightenedTforms = straightening(ccbundlerTforms)
    
    % Initialize the empty straightened Tforms for each connected component
    straightenedTforms = cell(1,length(ccbundlerTforms));

    for cc = 1:length(ccbundlerTforms)        
        % Each connected component Tforms (R)
        bundlerTforms = ccbundlerTforms{cc};                

        % Loop over each camera in the array to get the X vector of R
        v = cell2mat(arrayfun(@(c) c.R(1,1:3)', bundlerTforms, 'UniformOutput', false));
        
        % Compute the outer products and sum them up
        cov = v * v';
    
        % Compute the SVD of the covariance matrix
        [~, ~, V] = svd(cov); 
        normY = V(:, 3); % Corrected y-vector (last column of V)
           
        % Compute the average of the z-components from each camera's R
        vz = sum(cell2mat(arrayfun(@(c) c.R(3,:)', bundlerTforms, 'UniformOutput', false)),2);
    
        % Compute the cross product to get normX
        normX = cross(normY, vz);
        normX = normX / norm(normX); % Normalize normX
    
        % Compute normZ as the cross product of normX and normY
        normZ = cross(normX, normY);
         
        % Compute the scaling factor and adjust if necessary (dot product
        % for each vector and sum the results)
        s = sum(normX' * cell2mat(arrayfun(@(c) c.R(1,1:3)', bundlerTforms, 'UniformOutput', false)));
        
        % If s < 0, invert the direction of normX and normY
        if s < 0
            normX = -normX;
            normY = -normY;
        end
    
        % Construct the straightened rotation matrix Rs (3x3)
        straightenedR = zeros(3, 3);
        straightenedR(:, 1) = normX;
        straightenedR(:, 2) = normY;
        straightenedR(:, 3) = normZ;
    
        % Apply the homography matrix to each camera's R matrix
        % Update camera's R matrix and tore the updated camera back
        bundlerTforms = arrayfun(@(t) setfield(t, 'R', t.R * straightenedR), bundlerTforms);
        
        % Save the straightened Tforms for each connected component
        straightenedTforms{cc} = bundlerTforms;
    end
end
