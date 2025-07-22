clc; clear; close all;

% Read noisy image
noisy = im2double(imread('noisy_image.jpg'));

% Convert to grayscale if needed
if size(noisy, 3) == 3
    noisy = rgb2gray(noisy);
end

% Parameters
window_size = 7;  % local window size (odd)
half_win = floor(window_size / 2);

% Estimate noise variance using MAD (Median Absolute Deviation)
diff = abs(noisy - medfilt2(noisy, [3 3]));
noise_var = (median(diff(:)) / 0.6745)^2;  

fprintf('Estimated noise variance: %.6f\n', noise_var);

% Pad the noisy image (symmetric padding for better edge handling)
padded_noisy = padarray(noisy, [half_win half_win], 'symmetric');

% Precompute local mean using 'conv2' (much faster than nested loops)
kernel = ones(window_size) / (window_size^2);
local_mean = conv2(padded_noisy, kernel, 'valid');

% Precompute local variance using 'conv2' and mean subtraction
local_mean_sq = conv2(padded_noisy.^2, kernel, 'valid');
local_var = local_mean_sq - local_mean.^2;

% Wiener filter formula (vectorized)
w = max(0, ((local_var - noise_var) ./ (local_var + eps)));
filtered = local_mean + w .* (noisy - local_mean);

% Then clamp pixel intensities to valid range [0, 1]
filtered = max(0, min(1, filtered));

% Show results
figure;
subplot(2,1,1); imshow(noisy); title('Noisy Image');
subplot(2,1,2); imshow(filtered); title('Filtered Image (Optimized Wiener)');