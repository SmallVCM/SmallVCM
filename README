================================================================================
 SmallVCM
================================================================================

SmallVCM is a small physically based renderer that implements the vertex
connection and merging algorithm described in the paper

"Light Transport Simulation with Vertex Connection and Merging"
 Iliyan Georgiev, Jaroslav Krivanek, Tomas Davidovic, Philipp Slusallek
 ACM Transactions on Graphics 31(6) (SIGGRAPH Asia 2012)

as well as a number of other algorithms, notably including progressive photon
mapping, (progressive) bidirectional photon mapping, and bidirectional path
tracing. The code compiles to a command line program that can render images of a
number of predefined scenes using the provided algorithms.

Disclaimer:
  * Unless you really care, you can safely ignore all licenses in the source
    code.
  * This code is meant for educational purposes, and is not the code that was
    used to render the images in the aforementioned paper. The provided scenes
	are too simple to provide a complete understanding of the performance of
	every implemented rendering algorithm and the differences between them.
  * We are aware that the description below is not as detailed as it can be, and
    apologize for any errors and confusion.
  * If you have any questions and/or input w.r.t. improving and adding
    explanations, feel free to contact Tomas Davidovic <tomas@davidovic.cz>,
    the primary maintainer of this project, or Iliyan Georgiev <me@iliyan.com>,
	the primary author of the above paper, and we will figure out whatever you
	might need.
	
With that out of the way, let's go over the usual stuff.

================================================================================
1) INSTALLATION and COMPILATION
================================================================================

Synopsis: Compile smallvcm.cxx with OpenMP. If you don't have C++11 support,
define LEGACY_RNG (automatic for VS2008, found in rng.hxx).

The whole program consists of one C++ source file and a multiple header files.
It was developed in VS2010, however we did some limited testing on Linux, and
the provided Makefile works for g++ 4.4 and 4.6.3 at least.

The major hurdle can come from the fact that the C++11 <random> library is used,
but an alternative random number generator is also provided (`make old_rng` on
Linux). The code expects OpenMP is available, but getting rid of that is very
straightforward (simply comment out the few #pragma omp directives in the code).

Other than that, there are no dependencies, so simply compile smallvcm.cxx.

================================================================================
2) OPERATION
================================================================================

Quick start: Run `smallvcm --report -t 10`. In about 5-6 minutes it will
generate an index.html file that compares 7 different algorithms on 4 different
Cornell box variants (listed below).

The features and settings of the program can be explored by running
`smallvcm --help`, which outputs the following information:


Usage: smallvcm [ -s <scene_id> | -a <algorithm> |
           -t <time> | -i <iteration> | -o <output_name> | --report ]

    -s  Selects the scene (default 0):
          0    glossy small spheres + sun (directional)
          1    glossy large mirror sphere + ceiling (area)
          2    glossy small spheres + point
          3    glossy small spheres + background (env. lighting)
    -a  Selects the rendering algorithm (default vcm):
          el   eye light
          pt   path tracing
          lt   light tracing
          ppm  progressive photon mapping
          bpm  bidirectional photon mapping
          bpt  bidirectional path tracing
          vcm  vertex connection and merging
    -t  Number of seconds to run the algorithm
    -i  Number of iterations to run the algorithm (default 1)
    -o  User specified output name, with extension .bmp or .hdr (default .bmp)
    --report
        Renders all scenes using all algorithms and generates an index.html file
        that displays all images. Obeys the -t and -i options, ignores the rest.
        Recommended usage: --report -i 1   (fastest preview)
        Recommended usage: --report -t 10  (takes 5.5 min)
        Recommended usage: --report -t 60  (takes 30 min)

    Note: Time (-t) takes precedence over iterations (-i) if both are defined


'glossy' applies to the floor of the Cornell box.
'small spheres' variants have one mirror and one glass spheres in the box.	
	
The program can run in two modes:
1) If --report is not set, a single image of the specified scene will be
   rendered using the specified algorithm. If no option is specified, the output
   is a 512x512 image of scene 0 is rendered using vertex connection and merging
   with 1 iteration.
2) Setting the --report option renders all scenes using all algorithms, obeying
   the (optional) number of iterations and/or maximum runtime for each
   scene-algorithm configuration, ignoring the other options.

All default settings are set in the ParseCommandline function in config.hxx.
Some settings have no command line switch, but can be changed in the code:

  mNumThreads     Number of rendering threads (default 0, means 1 thread/core)
  mBaseSeed       Seed for random number generators (default 1234)
  mMinPathLength  Minimal path length (i.e. number of segments) (default 0)
  mMaxPathLength  Maximal path length (i.e. number of segments) (default 10)
  mResolution     Image resolution (default 512x512)
  mRadiusFactor   Scene diameter fraction for the merging radius (default 0.003)
  mRadiusAlpha    Merging radius reduction parameter (default 0.75)
  
================================================================================
3) VCM RENDERER
================================================================================

The VertexCM renderer implements a number of algorithms that share almost
identical code paths. The main differences between the algorithms lie in the
multiple importance sampling (MIS) weight computation, as well as shortcuts
through unused code. In order to make the understanding of the code easier,
below we describe how the VertexCM renderer operates. On a high level, it runs
in three stages:
  1) Light sub-path tracing (ppm, bpm, bpt, vcm)
  2) Range search hash grid construction over light vertices (ppm, bpm, vcm)
  3) Camera sub-path tracing (all but lt)

PathVertex (also PathElement variant and typedefs CameraVertex and LightVertex)
is the basic structure describing the state of a random walk. The only unusual
members are dVCM, dVC, dVM, which are used for iterative MIS weight computation:
  dVCM  used for both connections (bpt, vcm) and merging (bpm, vcm)
  dVC   used for connections (bpt, vcm)
  dVM   used for merging (bpm, vcm)
  
Note: All bidirectional algorithms sample the same number of light and camera
sub-paths per iteration, which is the number of pixels in the image. 

* Light tracing (lt)
  Utilizes only light sub-path tracing. Each path vertex is directly connected
  to camera and then discarded (i.e. not stored). No MIS, hash grid, or camera
  tracing are used.

* Progressive photon mapping (ppm)
  Light sub-paths are traced, storing their vertices (as LightVertex objects) on
  surfaces with non-specular (i.e. non-delta) materials, and building a hash
  grid over them. The camera sub-paths are traced until hitting a non-specular
  surface, where merging with light vertices (i.e. photon lookup) is performed,
  terminating the camera sub-path thereafter. No MIS is used, and the radiance
  from directly hit lights is accounted for only when all surface interactions
  on the path are specular.

* Bidirectional photon mapping (bpm)
  An extension to ppm, terminates camera sub-paths stochastically (unlike ppm)
  and performs merging at all non-specular vertices. MIS is used (dVCM and dVM)
  to weight the different possible ways of constructing the same path by merging
  at any (non-specular) interior path vertex.

* Bidirectional path tracing (bpt)
  Light sub-paths are traced, their non-specular vertices are first connected to
  the camera (as in light tracing) and then stored (without a hash grid). Next,
  the camera sub-paths are traced, connecting each non-specular vertex to a
  light source and to all non-specular vertices of the light sub-path
  corresponding to the current pixel. MIS is used (dVCM, dVC).

* Vertex connection and merging (vcm)
  Effectively a combination of bidirectional photon mapping and bidirectional
  path tracing. Light sub-path tracing projects and stores the non-specular
  vertices, and also builds a hash grid over them. In the camera sub-path
  tracing, each vertex is connected to a light source, to the vertices of the
  corresponding light sub-path, and also merged with the nearby vertices of all
  light sub-paths. MIS is used (dVCM, dVM, dVC).

================================================================================
4) FEATURES and LIMITATIONS
================================================================================

The renderer was originally intended to be a compact reference implementation, 
akin to smallpt, however it grew over time. Here is a list of the features and
the limitations of the framework:

Infrastructural features:
  * All basic light source types -- area, point, directional, and env. map --
    are supported, although the current env. map implementation uses a constant
	radiance distribution.
  * Basic surface scattering models are implemented, including diffuse, glossy
    (Phong), as well as specular reflection and refraction. It should be fairly
    straightforward to implement new materials. Also, the material interfaces
    should suffice for most purposes, as the bidirectional algorithms provided
    are already quite demanding on them.
  * The material/shading instance for a given ray hit point is represented by
    a BSDF object. In addition to storing the the surface scattering properties,
    this object also holds the local shading frame, as well we the incoming
	(fixed) direction; all its methods (sample, evaluate, pdf) compute their
	results always w.r.t. this direction.
	
Rendering features:
  * A simple renderer with eye light (dot normal) shading for fast previews.
    Red color denotes backface orientation. Implementation is in eyelight.hxx.
  * Traditional path tracing with next even estimation (area and env. map).
    Kept as a separate implementation is in pathtracer.hxx.
  * Light tracing (lt), progressive photon mapping (ppm), bidirectional photon
    mapping (bpm), bidirectional path tracing (bpt), and our vertex connection
    and merging (vcm). All these are  implemented in the VertexCM renderer, with
	code path switches for the different algorithms.

Limitations:
  * No acceleration structure for ray intersection (can be added to scene.hxx).
  * Scenes are hard-coded (see LoadCornellBox in scene.hxx).
  * The ppm algorithm does not handle diffuse+specular materials correctly.
    This limitation can be lifted by adding a parameter to the BSDF::Sample
	method that would specify the types of scattering events to be sampled.
	The ppm implementation could then be modified to continue camera sub-paths
	only for the specular parts of diffuse+specular materials.
  * Each area light is an individual triangle, and requires its very own
    material for identification. This can and should be changed if code is
	used for scenes with complex area light sources. Also, the way of finding
	whether and which area light is hit by a random ray can be improved.
  * No shading normals. Extending the framework with shading normals should
    happen within the BSDF object (which already supports adjoint BSDFs required
	by refraction).
  * No infrastructural support for participating media or subsurface scattering.
