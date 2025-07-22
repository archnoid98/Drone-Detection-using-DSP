clc;
clear all;
close all;

%drone image
droneImgOrig = imread('drone.png');

%video settings
frameSize = [480, 640, 3]; % (480x640) & 3 color channels (RGB)
fps = 30;                  
duration = 10;             
numFrames = fps * duration; 

%initial drone state
nextEntryFrame = 1;        
active = false;           
x = 0; y = 0;             
vx = 0; vy = 0;           
scale = 0.05;             

%generate each frame
for k = 1:numFrames
    bg = zeros(frameSize, 'uint8'); %bg as black img
    frame = bg; 
    
    if active
        %wind-like swaying
        swayX = 2 * sin(2 * pi * k / 100); 
        swayY = 2 * cos(2 * pi * k / 120); 
        swayS = 0.005 * sin(2 * pi * k / 180); %size change

        % Update motion
        x = x + vx + swayX;  
        y = y + vy + swayY;  
        scale = scale + swayS; 
        scale = min(max(scale, 0.02), 0.1); %scale between 2% and 10% of original size

        droneImg = imresize(droneImgOrig, scale); %resize
        [h, w, ~] = size(droneImg);  

        %drone is completely out of frame -> deactivate it
        if x > frameSize(2) || x + w < 0 || y > frameSize(1) || y + h < 0
            active = false;
            nextEntryFrame = k + randi([15, 60]); %waiting 0.5–2 seconds before next drone entry
        else
            %top-left corner's pixel
            xClamped = round(min(max(x, 1), frameSize(2) - w)); 
            yClamped = round(min(max(y, 1), frameSize(1) - h)); 

            %check indices are within bounds before placing the drone
            if xClamped + w - 1 <= frameSize(2) && yClamped + h - 1 <= frameSize(1)
                frame(yClamped+(1:h), xClamped+(1:w), :) = droneImg; %overlay the drone image
            end
        end
    end

    %drone is not active -> activate it after a random delay
    if ~active && k >= nextEntryFrame
        %randomly choose an entry side for the drone (1=left, 2=right, 3=top, 4=bottom)
        side = randi(4);

        %scaling for z-axis
        scale = 0.02 + 0.08 * rand();
        droneImg = imresize(droneImgOrig, scale); 
        [h, w, ~] = size(droneImg);

        %initial position and velocity based (entry side)
        switch side
            case 1 %left->right
                x = -w; y = randi([1, frameSize(1) - h]); vx = 4 + rand()*2; vy = randn();
            case 2 %right->left
                x = frameSize(2); y = randi([1, frameSize(1) - h]); vx = -4 - rand()*2; vy = randn();
            case 3 %top->bottom
                x = randi([1, frameSize(2) - w]); y = -h; vx = randn(); vy = 3 + rand()*2;
            case 4 %bottom->top
                x = randi([1, frameSize(2) - w]); y = frameSize(1); vx = randn(); vy = -3 - rand()*2;
        end
        active = true;  % Activate drone
    end

    % Display current frame
    imshow(frame);
    pause(1 / fps);
end


