clc; 
clear; 
close all;

% Parameters
sigma = 25; % noise standard deviation
p = 8; % patch size
Ww = 19; % search window size 
K1 = 8; % number of similar patches for Hard-thresholding
K2 = 16; % number of similar patches for Wiener
beta = 2; % Kaiser window parameter
st = 4; % step size 
lambda = 2.7; % threshold parameter

% Load image
try
    I = imread('drone_bm3d.jpg');
    if size(I,3) == 3
        I = rgb2gray(I);
    end
    I = im2double(I);
    
    % Resize if too large
    if size(I,1) > 256 || size(I,2) > 256
        I = imresize(I, [128 128]);
        fprintf('Image resized to 128x128\n');
    end
catch
    error('Failed to load image');
end

% Add noise
noisyImg = I + sigma/255 * randn(size(I));
noisyImg = max(0, min(1, noisyImg));

%% Hard-thresholding stage (First Pass)
[H,W] = size(noisyImg);
basicAcc = zeros(H,W);
wAcc = zeros(H,W);

%Kaiser window
kw = kaiser(p, beta);
win = kw * kw.';
win = win / sum(win(:));

% Process each reference patch
for r = 1:st:H-p+1
    for c = 1:st:W-p+1
        % Reference patch
        ref = noisyImg(r:r+p-1, c:c+p-1);
        
        % Block matching - find similar patches
        half = floor((Ww - p)/2);
        
        r0 = max(r - half, 1);%top boundary
        c0 = max(c - half, 1);%left boundary
        r1 = min(r + half, H - p + 1);%bottom boundary
        c1 = min(c + half, W - p + 1);%right boundary
        
        distances = [];
        positions = [];
        ref_mean = mean(ref(:));
        ref_std = std(ref(:));
        
        for x = r0:r1
            for y = c0:c1
                blk = noisyImg(x:x+p-1, y:y+p-1);
                blk_mean = mean(blk(:));
                blk_std = std(blk(:));
                
                if ref_std > 1e-6 && blk_std > 1e-6
                    ncc = sum((ref(:)-ref_mean).*(blk(:)-blk_mean))/(ref_std*blk_std*numel(ref));
                    dist = 2 - 2*ncc;
                else
                    dist = sum((ref(:)-blk(:)).^2)/numel(ref);
                end
                
                distances(end+1) = dist;
                positions(end+1,:) = [x y]; %top-left coordinate (x, y) of the candidate patch
            end
        end
        
        [~,o] = sort(distances); %o -> sorted distances' Indices
        sel = o(1:min(K1,length(distances)));
        idx = positions(sel,:);
        
        % Stack similar patches
        G = zeros(p,p,size(idx,1));
        for k = 1:size(idx,1)
            xy = idx(k,:);
            G(:,:,k) = noisyImg(xy(1):xy(1)+p-1, xy(2):xy(2)+p-1);
        end
        
        % 3D transform (2D DCT + 1D DCT)

        %2D DCT
        C = zeros(size(G));
        for k = 1:size(G,3)
            C(:,:,k) = dct2(G(:,:,k)); %converts spatial patch data into frequency components
        end

        %1D DCT along 3rd Dimension
        if size(G,3) > 1
            for i = 1:p
                for j = 1:p
                    signal = squeeze(C(i,j,:));
                    C(i,j,:) = dct(signal);
                end
            end
        end
        
        % Hard thresholding
        T = lambda * sigma / 255;
        C_thresh = C;
        C_thresh(abs(C) < T) = 0;
        C_thresh(1,1,:) = C(1,1,:); % Keep DC component
        
        % Inverse transform
        Gf = zeros(size(C_thresh));
        temp = C_thresh;

        %1D IDCT along 3rd Dimension
        if size(G,3) > 1
            for i = 1:p
                for j = 1:p
                    signal = squeeze(temp(i,j,:));
                    temp(i,j,:) = idct(signal);
                end
            end
        end

        %2D IDCT
        for k = 1:size(temp,3)
            Gf(:,:,k) = idct2(temp(:,:,k));
        end
        
        % Aggregate results
        for k = 1:size(idx,1)
            xy = idx(k,:);
            patch_region_r = xy(1):xy(1)+p-1;
            patch_region_c = xy(2):xy(2)+p-1;
            
            basicAcc(patch_region_r, patch_region_c) =basicAcc(patch_region_r, patch_region_c) + Gf(:,:,k) .* win;
            wAcc(patch_region_r, patch_region_c) =wAcc(patch_region_r, patch_region_c) + win;
        end
    end
end

% Normalize first stage result
wAcc(wAcc < 1e-6) = 1e-6; %avoid division by zero 
basicImg = basicAcc ./ wAcc;
basicImg = max(0, min(1, basicImg));

%% Wiener filtering stage (Second Pass)
finalAcc = zeros(H,W);
finalW = zeros(H,W);
sigma_n = sigma / 255;

for r = 1:st:H-p+1
    for c = 1:st:W-p+1
        % Reference patch from basic estimate
        ref = basicImg(r:r+p-1, c:c+p-1);
        
        % Block matching
        half = floor((Ww - p)/2);
        r0 = max(r - half, 1);
        c0 = max(c - half, 1);
        r1 = min(r + half, H - p + 1);
        c1 = min(c + half, W - p + 1);
        
        distances = [];
        positions = [];
        ref_mean = mean(ref(:));
        ref_std = std(ref(:));
        
        for x = r0:r1
            for y = c0:c1
                blk = basicImg(x:x+p-1, y:y+p-1);
                blk_mean = mean(blk(:));
                blk_std = std(blk(:));
                
                if ref_std > 1e-6 && blk_std > 1e-6
                    ncc = sum((ref(:)-ref_mean).*(blk(:)-blk_mean))/(ref_std*blk_std*numel(ref));
                    dist = 2 - 2*ncc;
                else
                    dist = sum((ref(:)-blk(:)).^2)/numel(ref);
                end
                
                distances(end+1) = dist;
                positions(end+1,:) = [x y];
            end
        end
        
        [~,o] = sort(distances);
        sel = o(1:min(K2,length(distances)));
        idx = positions(sel,:);
        
        % Stack patches from both estimates
        Gb = zeros(p,p,size(idx,1));
        Gn = zeros(p,p,size(idx,1));
        for k = 1:size(idx,1)
            xy = idx(k,:);
            Gb(:,:,k) = basicImg(xy(1):xy(1)+p-1, xy(2):xy(2)+p-1);
            Gn(:,:,k) = noisyImg(xy(1):xy(1)+p-1, xy(2):xy(2)+p-1);
        end
        
        % 3D transforms
        Cb = zeros(size(Gb));
        Cn = zeros(size(Gn));
        for k = 1:size(Gb,3)
            Cb(:,:,k) = dct2(Gb(:,:,k));
            Cn(:,:,k) = dct2(Gn(:,:,k));
        end
        if size(Gb,3) > 1
            for i = 1:p
                for j = 1:p
                    Cb(i,j,:) = dct(squeeze(Cb(i,j,:)));
                    Cn(i,j,:) = dct(squeeze(Cn(i,j,:)));
                end
            end
        end
        
        % Wiener filtering
        signal_var = Cb.^2;
        wiener_weights = signal_var ./ (signal_var + sigma_n^2);
        Cw = Cn .* wiener_weights; %Cw -> filtered 3D DCT coefficients
        
        % Inverse transform
        temp = Cw;
        if size(Gb,3) > 1
            for i = 1:p
                for j = 1:p
                    temp(i,j,:) = idct(squeeze(temp(i,j,:)));
                end
            end
        end
        Gf = zeros(size(temp));
        for k = 1:size(temp,3)
            Gf(:,:,k) = idct2(temp(:,:,k));
        end
        
        % Calculate aggregation weight
        weight = mean(wiener_weights(:));
        if weight < 1e-6
            weight = 1e-6;
        end
        
        % Aggregate results
        for k = 1:size(idx,1)
            xy = idx(k,:);
            patch_region_r = xy(1):xy(1)+p-1;
            patch_region_c = xy(2):xy(2)+p-1;
            
            finalAcc(patch_region_r, patch_region_c) =finalAcc(patch_region_r, patch_region_c) + Gf(:,:,k) .* (win * weight);
            finalW(patch_region_r, patch_region_c) =finalW(patch_region_r, patch_region_c) + win * weight;
        end
    end
end

% Normalize final result
finalW(finalW < 1e-6) = 1e-6;
finalImg = finalAcc ./ finalW;
finalImg = max(0, min(1, finalImg));

%% Evaluation and Display
mse_noisy = mean((I(:) - noisyImg(:)).^2);
mse_denoised = mean((I(:) - finalImg(:)).^2);
psnr_noisy = 10*log10(1/mse_noisy);
psnr_denoised = 10*log10(1/mse_denoised);

fprintf('PSNR - Noisy: %.2f dB, Denoised: %.2f dB\n', psnr_noisy, psnr_denoised);

figure('Position', [100 100 1200 400]);
subplot(1,3,1); imshow(I); title('Original');
subplot(1,3,2); imshow(noisyImg); 
title(sprintf('Noisy (\\sigma=%d, PSNR=%.1f dB)', sigma, psnr_noisy));
subplot(1,3,3); imshow(finalImg); 
title(sprintf('Denoised BM3D (PSNR=%.1f dB)', psnr_denoised));