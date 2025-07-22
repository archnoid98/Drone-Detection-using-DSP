clc; clear; close all;

% Parameters
window_size = 15;          % Neighborhood window size
half_window = floor(window_size/2);
step = 10;                 % Quiver sampling step for visualization
scale = 7;                 % Flow vector scale for quiver arrows
smooth_kernel_size = 5;    % Median filter size for smoothing flow
pyramid_levels = 3;        % Number of pyramid levels

% Video Setup
videoFile = 'video_test.mp4';  % Change this to your video path
vidReader = VideoReader(videoFile);

% Read first frame and preprocess
frame1 = readFrame(vidReader);
if size(frame1,3) == 3
    im1 = double(rgb2gray(frame1));
else
    im1 = double(frame1);
end

% Build pyramids for first frame
pyramid1 = cell(pyramid_levels,1);
pyramid1{1} = im1;
for l = 2:pyramid_levels
    pyramid1{l} = imresize(pyramid1{l-1}, 0.5);
end

% Create figure for visualization
hFig = figure('Name','Lucas-Kanade Optical Flow with Velocity','NumberTitle','off');

while hasFrame(vidReader) && ishandle(hFig)
    tic;
    
    % Read next frame and preprocess
    frame2 = readFrame(vidReader);
    if size(frame2,3) == 3
        im2 = double(rgb2gray(frame2));
    else
        im2 = double(frame2);
    end
    
    % Build pyramid for second frame
    pyramid2 = cell(pyramid_levels,1);
    pyramid2{1} = im2;
    for l = 2:pyramid_levels
        pyramid2{l} = imresize(pyramid2{l-1}, 0.5);
    end
    
    % Initialize flow at coarsest pyramid level
    u = zeros(size(pyramid1{pyramid_levels}));
    v = zeros(size(pyramid1{pyramid_levels}));
    
    % Process pyramid from coarse to fine
    for l = pyramid_levels:-1:1
        if l < pyramid_levels
            % Upsample flow to current level size [ Bilinear interpolation on flow values ]
            u = 2 * imresize(u, size(pyramid1{l})); 
            v = 2 * imresize(v, size(pyramid1{l}));
            
            % Warp second image at current level
            [x, y] = meshgrid(1:size(pyramid1{l},2), 1:size(pyramid1{l},1));
            %warped_im2(i,j) = interpolated_value_of_pyramid2{l}_at_(x(i,j)+u(i,j), y(i,j)+v(i,j))
            warped_im2 = interp2(pyramid2{l}, x + u, y + v, 'linear', 0); %warped_im2(x,y) [ Bilinear interpolation on image pixels ]
        else
            warped_im2 = pyramid2{l};  %warped_im2(x,y)
        end
        
        % Compute image gradients
        kernel_x = 0.25*[-1 1; -1 1];
        kernel_y = 0.25*[-1 -1; 1 1];
        kernel_t = 0.25*ones(2);
        
        fx = conv2(0.5*(pyramid1{l} + warped_im2), kernel_x, 'same');
        fy = conv2(0.5*(pyramid1{l} + warped_im2), kernel_y, 'same');
        ft = conv2(pyramid1{l} - warped_im2, kernel_t, 'same');
        
        % Pad gradients to handle window borders
        fx = padarray(fx, [half_window half_window], 'replicate');
        fy = padarray(fy, [half_window half_window], 'replicate');
        ft = padarray(ft, [half_window half_window], 'replicate');
        
        % Initialize incremental flow vectors
        du = zeros(size(pyramid1{l}));
        dv = zeros(size(pyramid1{l}));
        
        % Calculate incremental flow vectors for each pixel
        for i = 1:size(pyramid1{l},1)
            for j = 1:size(pyramid1{l},2)
                fx_local = fx(i:i+2*half_window, j:j+2*half_window);
                fy_local = fy(i:i+2*half_window, j:j+2*half_window);
                ft_local = ft(i:i+2*half_window, j:j+2*half_window);
                
                fx_vec = fx_local(:);
                fy_vec = fy_local(:);
                ft_vec = ft_local(:);
                
                A = [fx_vec fy_vec];
                b = -ft_vec;
                
                if rank(A'*A) == 2
                    nu = (A'*A) \ (A'*b);
                    du(i,j) = nu(1);
                    dv(i,j) = nu(2);
                end
            end
        end
        
        % Update flow vectors
        u = u + du;
        v = v + dv;
    end
    
    % Smooth flow fields using median filter
    u = medfilt2(u, [smooth_kernel_size smooth_kernel_size]);
    v = medfilt2(v, [smooth_kernel_size smooth_kernel_size]);
    
    % Compute velocity magnitude and direction
    velocity_magnitude = sqrt(u.^2 + v.^2);
    velocity_direction = atan2(v, u);  % radians
    
    % Compute average velocity magnitude & angle (pixels per frame)
    avg_velocity = mean(velocity_magnitude(:));
    avg_velocity_direction= mean(velocity_direction(:));
    
    %%%%%%%%%% --- MOTION-COMPENSATED TEMPORAL DENOISING (ADAPTIVE WEIGHTING) --- %%%%%%%%%%
    % Warp previous frame im1 to current frame im2 using flow (u,v)
    [X, Y] = meshgrid(1:size(im2,2), 1:size(im2,1));
    im1_warped = interp2(im1, X + u, Y + v, 'linear', 0);  % im1_warped aligned to im2

    % Compute normalized motion magnitude (0 to 1)
    motion_mag = velocity_magnitude;
    motion_mag_norm = motion_mag / (max(motion_mag(:)) + eps);  % avoid division by zero

    % Compute adaptive weight for previous frame: less weight where motion is high
    weight_prev = exp(-8 * motion_mag_norm);  % tuning factor 8 controls sensitivity, adjust as needed

    % Combine frames using spatially varying weights to reduce blur on moving objects
    denoised_frame = weight_prev .* im1_warped + (1 - weight_prev) .* im2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Display denoised frame
    imshow(uint8(denoised_frame));
    hold on;

    
    quiver(X(1:step:end, 1:step:end), Y(1:step:end, 1:step:end),u(1:step:end, 1:step:end)*scale, v(1:step:end, 1:step:end)*scale,'y', 'LineWidth', 1);

    hold off;
    title(sprintf('O.F.V ( Avg Velocity: %.4f < %.4rad  px/frame )', avg_velocity,avg_velocity_direction));
    
    drawnow;
    
    % Update previous frame and pyramids for next iteration
    im1 = im2;
    pyramid1{1} = im1;
    for l = 2:pyramid_levels
        pyramid1{l} = imresize(pyramid1{l-1}, 0.5);
    end
    
    fprintf('Frame processed in %.3f seconds, Avg Velocity: %.4f < %.4frad pixels/frame\n', toc, avg_velocity,avg_velocity_direction);
end
