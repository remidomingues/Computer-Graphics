Computer graphics
=================
Ray tracing
-----------
- Objects: triangles and spheres
- Materials:
    - Diffuse (absorption)
    - Specular (reflection)
    - Diffuse and specular (absorption and reflection)
    - Glass (refraction and reflection)
- Direct illumination + constant indirect illumination
- Soft shadows (NxN light sources) according to the following light sources distribution:
    - Uniform
    - Jittered by stochastic sampling
- Anti-aliasing:
    - Uniform 8x
    - Jittered by stochastic sampling 2x, 4x, 8x, 16x, 64x
- Multithreading (processing and anti-aliasing)

![Alt text](raytracing/cornell_box.png?raw=true "Cornell Box")

Rasterisation
-------------
- Triangles
- Depth buffer
- Direct illumination by interpolated pixel positions + constant indirect illumination
