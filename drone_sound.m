clc;
clear all;
close all;

% Sampling frequency and duration
fs = 44100;
duration = 10;  % Duration of audio (in seconds)
t = 0:1/fs:duration;  % Time vector

f0 = 150;  % Base frequency for propeller RPM

% Drone harmonics (base tone and its harmonics)
base_signal = sin(2*pi*f0*t) + 0.5*sin(2*pi*2*f0*t) + 0.25*sin(2*pi*3*f0*t);

% White noise (instead of pink noise)
whiteNoise = randn(size(t));  % Generate white noise

% Normalize the white noise
whiteNoise = whiteNoise / max(abs(whiteNoise));

% Normalize signal to avoid clipping
base_signal = base_signal + 0.3 * whiteNoise;  % Add white noise
base_signal = base_signal / max(abs(base_signal));  % Normalize

% Create video settings
frameSize = [480, 640, 3];  % Frame size (480x640)
fps = 30;  % Frames per second
numFrames = fps * duration;  % Total number of frames

% Initial drone state
x = 0; y = 0; z = 0;  % Position (X, Y, Z)
vx = 2; vy = 2; vz = 0;  % Initial velocity (X, Y, Z)
scale = 0.05;  % Initial size of the drone
active = true;  % Drone starts active
droneImgOrig = imread('drone.png');  % Drone image

% Generate each frame
for k = 1:numFrames
    bg = zeros(frameSize, 'uint8');  % Background (black)
    frame = bg;  % Initialize frame
    
    % Simulate drone motion (random movement)
    swayX = 2 * sin(2 * pi * k / 100); 
    swayY = 2 * cos(2 * pi * k / 120); 
    swayS = 0.005 * sin(2 * pi * k / 180);  % Size change
    
    % Smoothly update the position with continuous motion
    x = x + vx + swayX;  
    y = y + vy + swayY;  
    scale = scale + swayS; 
    scale = min(max(scale, 0.02), 0.1);  % Scale between 2% and 10%
    
    % Update Z-axis motion (move closer or further away)
    z = z + vz;  % Update Z position (representing depth)
    
    % Doppler effect: Adjust pitch based on Z-axis (distance)
    doppler_factor = 1 + (vz / 10);  % Adjust Doppler effect based on velocity (vz)
    pitch_signal = sin(2 * pi * f0 * doppler_factor * t) + ...
                   0.5 * sin(2 * pi * 2 * f0 * doppler_factor * t) + ...
                   0.25 * sin(2 * pi * 3 * f0 * doppler_factor * t);
               
    % Noise level modulation: Increase noise intensity based on speed (vx, vy, vz)
    noise_intensity = 0.3 + 0.7 * sqrt(vx^2 + vy^2 + vz^2);  % Higher speed -> more noise
    adjusted_signal = pitch_signal + noise_intensity * whiteNoise;  % Add white noise
    
    % Normalize audio again to avoid clipping
    adjusted_signal = adjusted_signal / max(abs(adjusted_signal)); 
    
    % Pan the audio based on x position (left-right)
    pan_factor = x / frameSize(2);  % Horizontal movement affects stereo pan
    if pan_factor > 0
        % More sound in the right channel
        left_signal = adjusted_signal * (1 - pan_factor);
        right_signal = adjusted_signal * pan_factor;
    else
        % More sound in the left channel
        left_signal = adjusted_signal * abs(pan_factor);
        right_signal = adjusted_signal * (1 - abs(pan_factor));
    end
    
    % Combine both channels (stereo)
    stereo_signal = [left_signal', right_signal'];
    
    % Apply volume decay based on distance (Z-axis)
    distance = sqrt(x^2 + y^2 + z^2);  % Calculate distance from origin
    volume_factor = max(0, 1 - distance / 500);  % Decay volume with distance
    stereo_signal = stereo_signal * volume_factor;  % Apply volume decay

    % Play the audio at each frame (simulating real-time playback)
    sound(stereo_signal, fs);  % Play audio
    
    % Handle drone image placement
    droneImg = imresize(droneImgOrig, scale);  % Resize drone image based on scale
    [h, w, ~] = size(droneImg);  % Get drone image dimensions
    
    % Check if drone is within the frame
    if x > frameSize(2) || x + w < 0 || y > frameSize(1) || y + h < 0
        % Reset drone if it moves out of frame
        x = -w; 
        y = randi([1, frameSize(1) - h]); 
        vx = 4 + rand() * 2; 
        vy = randn();  % New direction after moving out of frame
    else
        % Top-left corner's pixel
        xClamped = round(min(max(x, 1), frameSize(2) - w)); 
        yClamped = round(min(max(y, 1), frameSize(1) - h)); 

        % Check indices are within bounds before placing the drone
        if xClamped + w - 1 <= frameSize(2) && yClamped + h - 1 <= frameSize(1)
            frame(yClamped+(1:h), xClamped+(1:w), :) = droneImg; % Overlay the drone image
        end
    end

    % Display current frame
    imshow(frame);
    pause(1 / fps);  % Wait for the next frame
end


