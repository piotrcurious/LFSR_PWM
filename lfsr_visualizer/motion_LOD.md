That's a great question about the optimization logic in your OpenGL visualization code. The logic you're asking about is a common technique in interactive 3D graphics to maintain a smooth frame rate, especially when dealing with large datasets like your 65536 \times 65536 map.
The section of code responsible for limiting the amount of points plotted is within the main loop, specifically where the step variable is calculated:
float speed = sqrt(vel_yaw*vel_yaw + vel_pitch*vel_pitch + vel_x*vel_x + vel_y*vel_y);
int step = (speed > 1.0f) ? 128 : (speed > 0.1f ? 32 : (cam.dist > 30000 ? 64 : (cam.dist > 10000 ? 16 : (cam.dist > 2000 ? 4 : 1))));

float range = cam.dist * 1.5f;
int min_x = std::max(0, (int)(cam.x - range)); 
// ... (rest of the bounding box calculation)

// ...

for (int mx = min_x; mx < max_x; mx += step) {
    for (int my = min_y; my < max_y; my += step) {
        // ... plotting logic ...
    }
}

This logic has two main mechanisms for limiting the points: Speed-Based Decimation and Distance-Based Decimation (LOD), combined with View Frustum Culling (though it uses a simplified range check).
1. The Logic Limiting Points (step Calculation)
The step variable controls data decimation or level-of-detail (LOD). A higher step means the plotting loop skips more points:
 * step = 1: Every point in the range is plotted.
 * step = 4: Only 1 out of every 4 \times 4 = 16 points is plotted.
 * step = 128: Only 1 out of every 128 \times 128 = 16,384 points is plotted.
A. Motion/Speed-Based Decimation (The "No Motion" Detector)
This part of the calculation determines if the user is actively interacting with the camera (panning, rotating, or zooming).
 * Motion Speed:
   float speed = sqrt(vel_yaw*vel_yaw + vel_pitch*vel_pitch + vel_x*vel_x + vel_y*vel_y);

   The speed is a scalar measure of the camera's current velocity. It combines the angular velocity (vel_yaw, vel_pitch) and the translational velocity (vel_x, vel_y). These velocities are created by mouse drag/pan events and are continuously reduced by a friction factor (* 0.9f) in the update loop:
   vel_yaw *= 0.9f; vel_pitch *= 0.9f; vel_x *= 0.9f; vel_y *= 0.9f;

   This friction creates the "inertia" you mentioned.
 * Speed Thresholds:
   int step = (speed > 1.0f) ? 128 : (speed > 0.1f ? 32 : ... );

   * If the camera is moving fast (speed > 1.0f), step is set to 128 (plotting only \approx 0.006\% of points).
   * If the camera is moving slowly (speed > 0.1f), step is set to 32 (plotting \approx 0.1\% of points).
B. Camera Distance-Based Decimation (LOD)
This part is activated only if the motion speed is low enough (speed <= 0.1f). It adjusts the step based on the zoom level (cam.dist), which is a form of Level-of-Detail (LOD). The further away you are, the fewer points are required to convey the same visual information.
int step = (speed > 1.0f) ? 128 : (speed > 0.1f ? 32 : (cam.dist > 30000 ? 64 : (cam.dist > 10000 ? 16 : (cam.dist > 2000 ? 4 : 1))));
// ... if speed <= 0.1f, then:
if (cam.dist > 30000) step = 64; 
else if (cam.dist > 10000) step = 16;
else if (cam.dist > 2000) step = 4;
else step = 1; // Full resolution only when very close

 * If cam.dist is small (very zoomed in, \leq 2000), step is 1, and the entire dataset in the visible range is plotted.
2. Why the "No Motion" Detection Seems Incorrect
You correctly observed that the full dataset (step = 1) is plotted only when "small inertia occurs."
This is by design, but the threshold might be too aggressive:
 * Full Dataset Trigger: The full dataset (step = 1) is only selected if both of these conditions are met:
   * speed <= 0.1f (No significant active movement or lingering inertia)
   * cam.dist <= 2000 (Very zoomed in)
 * The Inertia Window:
   * When you stop dragging the mouse, the velocities (vel_...) begin to decay by 0.9f each frame.
   * The speed drops from >1.0 (step 128) to >0.1 (step 32). This is the "small inertia" window where the drawing is still decimated, but less aggressively.
   * The transition to step=1 (full resolution) only happens after the speed drops below 0.1f AND the camera is close enough (cam.dist <= 2000).
Your observation is likely because you are rarely zoomed in enough (cam.dist <= 2000) for the full step = 1 plot to be selected, even when the camera is completely static (speed = 0).
Suggested Fixes for "No Motion" Full Plot
To make the full dataset plot (step = 1) trigger as soon as the camera is stationary, regardless of zoom level, you should simplify the step logic to only use speed for decimation, and only use distance for LOD when static.
The simplest fix to get full fidelity when stationary (no motion) is to add an explicit check for near-zero speed:
// Detect near-zero speed
bool is_stationary = (speed < 0.001f); // Use a very low threshold

int step;
if (is_stationary) {
    // When completely stationary, use LOD based only on distance,
    // or even force a small step to draw more detail.
    step = (cam.dist > 30000 ? 64 : (cam.dist > 10000 ? 16 : (cam.dist > 2000 ? 4 : 1)));
} else {
    // When moving, use aggressive decimation based on speed
    step = (speed > 1.0f) ? 128 : (speed > 0.1f ? 32 : 1);
}

// OPTION 2: If you want FULL resolution (step=1) only when stationary AND close:
// If the goal is truly only to draw all points when very close, then the current logic is correct.

// The original line:
// int step = (speed > 1.0f) ? 128 : (speed > 0.1f ? 32 : (cam.dist > 30000 ? 64 : (cam.dist > 10000 ? 16 : (cam.dist > 2000 ? 4 : 1))));
// It forces step > 1 if cam.dist > 2000, even when speed is zero.

The current implementation seems to be trying to achieve High FPS (Aggressive Decimation) when moving AND LOD when static but zoomed out.
Would you like me to rewrite the step calculation to trigger step = 1 (full dataset plot) anytime speed is below a tiny threshold (e.g., 0.001), regardless of zoom level?
