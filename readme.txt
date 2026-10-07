# Drone Detection with DSP (MATLAB)

MATLAB experiments for detecting a drone by its **sound** and its **image**, built around a synthetic test scene. A drone image moves across a video frame with realistic drift, and a matching propeller sound changes pitch, volume and stereo position with its motion. The video side is then cleaned up and tracked with classic signal-processing methods: an adaptive Wiener filter, pyramidal Lucas-Kanade optical flow, motion-compensated temporal denoising and BM3D.

Everything is written in plain MATLAB, and the optical flow and the BM3D denoiser are implemented from scratch rather than called from a toolbox.

---

## 1. What is in this repository

| File | What it does |
|---|---|
| `drone_movement.m` | Synthetic video: a drone enters from a random side, drifts with wind-like sway, changes size (depth), leaves, and re-enters after a random 0.5–2 s pause |
| `drone_sound.m` | The same scene with sound: propeller harmonics, Doppler shift, speed-dependent noise, stereo panning and distance-based volume |
| `filter_wiener.m` | Adaptive (local-statistics) Wiener filter for a single image, with noise variance estimated by MAD |
| `optical_flow.m` | Pyramidal Lucas-Kanade optical flow on a video, with flow arrows and average velocity |
| `motion_compensated_temporal_denoising.m` | Optical flow used to warp the previous frame onto the current one before averaging, with less averaging where motion is high |
| `merged_wiener_optical_flow.m`, `merged.m` | Wiener filtering combined with optical flow and temporal denoising in one pipeline |
| `bm3d_scratch.m` | BM3D image denoising implemented from scratch (block matching, hard thresholding, Wiener stage), with PSNR before and after |
| `bm3d_builtin.m` | The same comparison using the official BM3D MATLAB package (`bm3d_matlab_package_4.0.3/`) |
| `shihab_imgs_bm3d.m` | Batch BM3D denoising of a folder of frames, with a log file; results in `Denoised_Results_NEW/` |
| `camera_test.m` | Temporal averaging of resized frames from a video file |
| `webcam_access.m` | Opens the default webcam and shows a live preview |
| `websocket_test1.m` | Sends an image to a Python WebSocket server as base64, for passing frames to another program |
| `drone.png`, `drone_audio.wav`, `*.mp4` | Test image, audio and videos used by the scripts |

---

## 2. Synthetic drone scene

### 2.1 Motion (`drone_movement.m`)

The frame is 480 × 640 RGB at 30 fps for 10 s (300 frames) on a black background.

- **Entry:** `randi(4)` picks the side the drone enters from (1 = left, 2 = right, 3 = top, 4 = bottom). Its size is set to 2–10 % of the original image with `0.02 + 0.08 * rand()`.
- **Velocity:** the main direction moves at 3–6 pixels per frame across the screen. The sideways component is `randn()`, so most drones drift slightly while a few drift noticeably.
- **Sway:** every frame adds a small wind-like wobble, `2·sin(2πk/100)` in x and `2·cos(2πk/120)` in y, plus a slow size change of `0.005·sin(2πk/180)` to suggest the drone moving closer and further away. The size is kept between 2 % and 10 %.
- **Exit and re-entry:** once the drone is completely outside the frame it is deactivated, and a new one appears after a random 15–60 frames (0.5–2 s).

### 2.2 Sound (`drone_sound.m`)

The audio is sampled at 44.1 kHz.

- **Propeller tone:** a 150 Hz fundamental plus its 2nd and 3rd harmonics at half and quarter amplitude.
- **Doppler:** the pitch is scaled by `1 + vz/10`, so it rises as the drone approaches and falls as it moves away.
- **Noise:** white noise is added, and it gets louder as the drone moves faster.
- **Stereo position:** the horizontal position sets the left/right balance.
- **Distance:** the volume decays linearly with distance from the origin.

---

## 3. Image coordinates and clamping

These conventions matter when placing the drone image inside the frame.

- **Origin:** MATLAB images are indexed from **(1, 1)** at the **top-left** corner.
- **Axes:** x (column) increases to the right, and y (row) increases **downward**.
- **Drone position:** `(x, y)` is the top-left corner of the drone image.
- **Drone extent:** a drone of width `w` and height `h` covers columns `x` to `x + w − 1` and rows `y` to `y + h − 1`. For example, with `y = 50` and `h = 40` the drone covers rows 50 to 89.

**Clamping.** While the drone is partly off screen, its position is clamped so the pasted image never goes past the frame edges:

```matlab
xClamped = round(min(max(x, 1), frameSize(2) - w));   % leftmost column
yClamped = round(min(max(y, 1), frameSize(1) - h));   % topmost row
```

`xClamped` is the leftmost column of the drone and `xClamped + w − 1` its rightmost column. A final bounds check runs before the image is written into the frame.

**Random number functions used:**

| Function | Returns |
|---|---|
| `randi(4)` | A random integer from 1 to 4 |
| `randi([15, 60])` | A random integer from 15 to 60 |
| `rand()` | A uniform random number between 0 and 1 |
| `randn()` | A normally distributed number with mean 0 and standard deviation 1 (about 68 % of values fall between −1 and 1) |

---

## 4. Video processing

### 4.1 Adaptive Wiener filter (`filter_wiener.m`)

Each pixel is filtered using the mean and variance of its 7 × 7 neighbourhood. Flat areas are smoothed strongly, while edges, where local variance is high, are kept. The noise variance is estimated automatically from the image with the median absolute deviation (MAD) of the difference between the image and a 3 × 3 median-filtered copy.

### 4.2 Pyramidal Lucas-Kanade optical flow (`optical_flow.m`)

The flow is estimated on a 3-level image pyramid, from coarse to fine, using a 15 × 15 window. At each level the second frame is warped by the current flow estimate with bilinear interpolation, and the remaining motion is solved from the spatial and temporal gradients. The flow field is median-filtered, drawn as arrows, and summarised as an average speed and direction in pixels per frame.

### 4.3 Motion-compensated temporal denoising

Averaging consecutive frames removes noise but blurs anything that moves. Here the previous frame is first warped along the optical flow so it lines up with the current frame. The weight given to the previous frame is `exp(−8 · motion)`, so still regions are averaged strongly while fast-moving regions, such as the drone, keep their detail.

### 4.4 BM3D

`bm3d_scratch.m` implements the two BM3D stages directly: similar 8 × 8 patches are grouped by block matching in a 19 × 19 search window, filtered together in the transform domain (hard thresholding, then a Wiener stage), and put back with weighted averaging. It reports PSNR before and after denoising. `bm3d_builtin.m` runs the official BM3D package on the same kind of input for comparison.

---

## 5. Running the scripts

1. Open the repository folder in MATLAB. The Image Processing Toolbox is needed for `imresize`, `medfilt2` and `imnoise`.
2. Run `drone_movement.m` to see the synthetic scene, or `drone_sound.m` to hear it as well.
3. The video scripts read `video_test.mp4` by default; change `videoFile` at the top to use another video.
4. `filter_wiener.m` expects an image called `noisy_image.jpg`, so put one in the folder first.
5. `bm3d_builtin.m` and `shihab_imgs_bm3d.m` contain a hard-coded package path (`D:\DSP Project\...`). Change `bm3d_path` to where `bm3d_matlab_package_4.0.3/bm3d` sits on your machine.
6. `webcam_access.m` needs the MATLAB Support Package for USB Webcams. `websocket_test1.m` needs Python with the `websocket-client` package, and the server address in the script changed to your own.

---

## 6. Limitations

- The drone scene is synthetic: a single drone on a plain black background. Real footage with clouds, birds and camera shake is much harder.
- Detection is not yet automatic; the scripts produce the cleaned video and motion field that a detector would use.
- The audio and video sides are simulated together but processed separately.

## Author

**Jarif Shahriar Ahmed** ([@archnoid98](https://github.com/archnoid98)), EEE, Bangladesh University of Engineering and Technology (BUET)
