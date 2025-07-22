Clamp Logic: This ensures that the drone's image stays within the bounds of the frame, preventing it from being drawn outside the visible area

xClamped -> leftmost pixel
xClamped + w - 1 -> rightmost pixel
randi(4) -> generates randomly between [1 2 3 4]
rand() -> (0,1)
randn() -> (-1,1)

x -> horizontal position top-left corner of the drone

y -> vertical position top-left corner of the drone

(x, y) -> top-left corner

=>In most computer graphics systems, including MATLAB:
The top-left corner of the screen or image has coordinates (x = 0, y = 0)
As you move down from the top-left corner, the y-values increase, i.e., moving downwards corresponds to increasing y values
If the top-left corner of an image is at y = 50 and the height of the image is h = 40, the bottommost point is at 50 + 40 = 90

x increases to the right 
y increases downward