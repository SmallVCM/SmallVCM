/*
 * This is published under Apache 2.0
 */

#include <vector>
#include <cmath>
#include "math.hxx"
#include "ray.hxx"
#include "geometry.hxx"
#include "camera.hxx"
#include "framebuffer.hxx"
#include "scene.hxx"
#include "eyelight.hxx"

int main(int argc, const char *argv[])
{
    int iterations = 1;
    if(argc > 1)
        iterations = atoi(argv[1]);

    Scene scene;
    scene.LoadCornellBox();

    Framebuffer fbuffer;
    fbuffer.Setup(scene.mCamera.mResolution);
    fbuffer.Clear();

    EyeLight renderer;

    for(int iter=0; iter < iterations; iter++)
    {
        renderer.RunIteration(scene, fbuffer);
    }

    fbuffer.SavePPM("cb.ppm");
    fbuffer.SavePFM("cb.pfm");
}