# Ray Tracer 

## Introduction
This project is a basic ray tracer developed for the **Middle East Technical University, Department of Computer Engineering, CENG 477 - Introduction to Computer Graphics (Fall 2024-2025)** course. The goal of this project is to simulate light propagation and object interaction using ray tracing techniques, producing high-quality rendered images.

## Features
- **Ray-Object Intersections**: Supports sphere and triangle intersections.
- **Blinn-Phong Shading Model**: Computes ambient, diffuse, and specular reflections.
- **Point and Ambient Light Sources**: Simulates light attenuation and illumination.
- **Shadow Ray Casting**: Determines whether an object is in shadow.
- **Recursive Reflection Handling**: Handles mirror materials with a recursion depth limit.
- **Back-face Culling**: Optimizes rendering by ignoring non-visible faces.


### Compilation & Execution
```sh
# Extract the project archive
$ tar -xf raytracer.tar.gz

# Compile the project
$ make

# Run the ray tracer with an XML scene file
$ ./raytracer scene.xml
```


## Rendering Algorithm Overview
1. **Ray Generation**: Primary rays are cast from the camera through each pixel.
2. **Intersection Testing**: Checks if the ray intersects with any object in the scene.
3. **Shading Computation**: Applies Blinn-Phong shading based on light sources.
4. **Shadow Testing**: Determines if an intersection point is occluded from a light source.
5. **Reflection Handling**: Recursively traces reflected rays for mirror-like materials.
6. **Color Clamping**: Ensures final RGB values remain within [0, 255] range.
7. **Image Output**: Stores the rendered image in PPM format.

[](png_outputs/killeroo.png)

