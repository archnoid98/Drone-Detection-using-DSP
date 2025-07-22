clc; clear; close all;

% Parameters for Wiener filter
wiener_window_size = 7;  
half_win_wiener = floor(wiener_window_size / 2);

% Parameters for optical flow + temporal denoising
window_size = 15;          
half_window = floor(window_size/2);
step = 10;                 
scale = 7;                 
smooth_kernel_size = 5;    
pyramid_levels = 3;        

% Video Setup
videoFile = 'video_test.mp4';  % Change this to your video path
vidReader = VideoReader(videoFile);

% Read first frame and preprocess
frame1_raw = readFrame(vidReader);

% Convert to grayscale if needed
if size(frame1_raw,3) == 3
    frame1_gray = im2double(rgb2gray(frame1_raw));
else
    frame1_gray = im2double(frame1_raw);
end

% --- Wiener filter on first frame ---

% Estimate noise variance using MAD (median absolute deviation)
diff = abs(frame1_gray - medfilt2(frame1_gray, [3 3]));
noise_var = (median(diff(:)) / 0.6745)^2;

% Pad the image for Wiener filtering
padded_noisy = padarray(frame1_gray, [half_win_wiener half_win_wiener], 'symmetric');

% Local mean and variance
kernel = ones(wiener_window_size) / (wiener_window_size^2);
local_mean = conv2(padded_noisy, kernel, 'valid');
local_mean_sq = conv2(padded_noisy.^2, kernel, 'valid');
local_var = local_mean_sq - local_mean.^2;

% Wiener filter formula
w = max(0, ((local_var - noise_var) ./ (local_var + eps)));
frame1 = local_mean + w .* (frame1_gray - local_mean);
frame1 = max(0, min(1, frame1));  % clamp to [0,1]

% Build pyramid for first frame
pyramid1 = cell(pyramid_levels,1);
pyramid1{1} = frame1;
for l = 2:pyramid_levels
    pyramid1{l} = imresize(pyramid1{l-1}, 0.5);
end

% Create figure for visualization
hFig = figure('Name','Wiener + Motion-Compensated Temporal Denoising','NumberTitle','off');

% Process frames in video
while hasFrame(vidReader) && ishandle(hFig)
    tic;
    
    % Read next frame and preprocess
    frame2_raw = readFrame(vidReader);
    if size(frame2_raw,3) == 3
        frame2_gray = im2double(rgb2gray(frame2_raw));
    else
        frame2_gray = im2double(frame2_raw);
    end
    
    % --- Wiener filter on current frame ---
    padded_noisy = padarray(frame2_gray, [half_win_wiener half_win_wiener], 'symmetric');
    local_mean = conv2(padded_noisy, kernel, 'valid');
    local_mean_sq = conv2(padded_noisy.^2, kernel, 'valid');
    local_var = local_mean_sq - local_mean.^2;
    w = max(0, ((local_var - noise_var) ./ (local_var + eps)));
    frame2 = local_mean + w .* (frame2_gray - local_mean);
    frame2 = max(0, min(1, frame2));
    
    % Build pyramid for second frame
    pyramid2 = cell(pyramid_levels,1);
    pyramid2{1} = frame2;
    for l = 2:pyramid_levels
        pyramid2{l} = imresize(pyramid2{l-1}, 0.5);
    end
    
    % Initialize flow at coarsest pyramid level
    u = zeros(size(pyramid1{pyramid_levels}));
    v = zeros(size(pyramid1{pyramid_levels}));
    
    % Process pyramid from coarse to fine
    for l = pyramid_levels:-1:1
        if l < pyramid_levels
            u = 2 * imresize(u, size(pyramid1{l})); 
            v = 2 * imresize(v, size(pyramid1{l}));
            
            [x, y] = meshgrid(1:size(pyramid1{l},2), 1:size(pyramid1{l},1));
            warped_im2 = interp2(pyramid2{l}, x + u, y + v, 'linear', 0);
        else
            warped_im2 = pyramid2{l};
        end
        
        kernel_x = 0.25*[-1 1; -1 1];
        kernel_y = 0.25*[-1 -1; 1 1];
        kernel_t = 0.25*ones(2);
        
        fx = conv2(0.5*(pyramid1{l} + warped_im2), kernel_x, 'same');
        fy = conv2(0.5*(pyramid1{l} + warped_im2), kernel_y, 'same');
        ft = conv2(pyramid1{l} - warped_im2, kernel_t, 'same');
        
        fx = padarray(fx, [half_window half_window], 'replicate');
        fy = padarray(fy, [half_window half_window], 'replicate');
        ft = padarray(ft, [half_window half_window], 'replicate');
        
        du = zeros(size(pyramid1{l}));
        dv = zeros(size(pyramid1{l}));
        
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
        
        u = u + du;
        v = v + dv;
    end
    
    % Smooth flow fields
    u = medfilt2(u, [smooth_kernel_size smooth_kernel_size]);
    v = medfilt2(v, [smooth_kernel_size smooth_kernel_size]);
    
    velocity_magnitude = sqrt(u.^2 + v.^2);
    velocity_direction = atan2(v, u);
    
    avg_velocity = mean(velocity_magnitude(:));
    avg_velocity_direction= mean(velocity_direction(:));
    
    % Motion-compensated temporal denoising
    [X, Y] = meshgrid(1:size(frame2,2), 1:size(frame2,1));
    im1_warped = interp2(frame1, X + u, Y + v, 'linear', 0);
    
    motion_mag_norm = velocity_magnitude / (max(velocity_magnitude(:)) + eps);
    weight_prev = exp(-8 * motion_mag_norm);
    
    denoised_frame = weight_prev .* im1_warped + (1 - weight_prev) .* frame2;
    
    % Show result
    imshow(denoised_frame);
    hold on;
    quiver(X(1:step:end, 1:step:end), Y(1:step:end, 1:step:end), ...
        u(1:step:end, 1:step:end)*scale, v(1:step:end, 1:step:end)*scale, ...
        'y', 'LineWidth', 1);
    hold off;
    title(sprintf('Wiener + Motion-Compensated Temporal Denoising\n Avg Velocity: %.4f < %.4frad px/frame', avg_velocity, avg_velocity_direction));
    drawnow;
    
    % Update previous frame & pyramid for next iteration
    frame1 = frame2;
    pyramid1{1} = frame1;
    for l = 2:pyramid_levels
        pyramid1{l} = imresize(pyramid1{l-1}, 0.5);
    end
    
    fprintf('Frame processed in %.3f seconds, Avg Velocity: %.4f < %.4frad pixels/frame\n', toc, avg_velocity, avg_velocity_direction);
end
