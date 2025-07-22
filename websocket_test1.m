clc;
clear all;
close all;

% Import WebSocket client library in Python
py.importlib.import_module('websocket');

% Create WebSocket connection to Python server
ws = py.websocket.create_connection('ws://192.168.147.247:8765');

% Read the image file as binary data (preserves format)
fid = fopen('drone.png', 'rb');
img_data = fread(fid, '*uint8');
fclose(fid);

% Convert to base64
encoded_img = matlab.net.base64encode(img_data);

% Send the base64-encoded image to the server
ws.send(encoded_img);

% Close the WebSocket connection
ws.close();