![](https://www.antillevisuals.com/technical-research/cycles_mod_splines_as_hair.jpg)

# Render splines using Cycles hair primitives
This is a Cycles modification that allows you to skip meshing splines entirely and render them as hair. Needless to say, with this method you can render *lots* of them. Hello large scale graphs! This feature is primarily intented for CPU rendering because GPUs don't have access to enough memory for large-scale rendering and Cycles isn't an out-of-core renderer.

## Features
1. Splines to hair: render all splines within a curve object as hair in one click in the UI, also easily scriptable.
2. Radius variation: splines are rendered with their radii so this is more than simple tapering hair.
3. Spline interpolation: all curve types are supported. Two interpolations may occur and affect hair, see Usage below.
4. Per-spline material: supported. You need to add the materials to the curve object and then set the material_index of each spline.
5. Two custom values on spline points: `key` and `value`, two floats. This takes 8 bytes extra per spline point in memory but gives you the ability to modulate your shaders based on these values as they're available in `HairInfo` node.
6. Extra HairInfo slots: `Spline Index`, `Splines Count`, `Spline Length`, `Custom Key Data`, `Custom Value Data`

*[Only for custom key-value data]*
*Warning* When you save .blend files with this version, *do not save them with another version of Blender* or custom values will be lost! You've been warned.

## How to use it?
You need to compile Blender with these sources. You need to have git installed on your system or skip step 1 and download sources from github.
Make sure you have python3.7+ installed as this is required by Blender 2.8.

In your console run the following commands (valid for Linux/MacOS/Windows):
1. `git clone git://github.com/nantille/blender_28_mod`
2. `cd blender_28_mod`
3. `make update`
4. `make`

If everything compiles fine (fingers crossed!), you will have a new Blender executable in relative path `../build_[platform]/bin/`
The path to the blender executable is given at the end of the compilation.

When the above steps produce an error relative to locale and addons, follow these steps:
1. `git remote set-url origin git://git.blender.org/blender.git`
2. `git submodule sync`
3. `git submodule update --init --recursive`
4. `git submodule foreach git checkout master`
5. `git submodule foreach git pull --rebase origin master`
6. `git remote set-url origin https://github.com/nantille/blender_28_mod.git`
7. `make update`
8. `make`

If you would like to compile Blender with the latest sources published by Blender Institute,
you can try rebasing with the following command:
`git pull --rebase http://git.blender.org/blender.git master`

If after rebasing you get a compilation error such as missing libraries, run again `make update`

If you have a Cmake Error mentioning `Python executable missing` and you have Python installed, you probably have the wrong version. Blender 2.8 requires Python 3.7+.

## Version
This is a modification for Blender 2.8+.

## Usage
1. Open this custom version of Blender 2.8.
2. Select Cycles as render engine.
3. Select or add a curve object on the scene.
4. Under `Object Data` panel at the bottom, enable `Render as Hair`.
5. Make sure that in panel `Scene`, under `Geometry`, the option `Use Hair` is enabled.
6. Tweak options under `Use Hair`, set `Shape` to `Thick` and `Primitive` to `Curve Segments`.

## Hair != beveled curves
Please note that when a spline is rendered as a hair primitive, it has *no mesh* and so all meshing related features like `fill`, `twisting`, `taper`, `bevel`, `bevel_depth` will be ignored. Cyclic curves are not supported though this could be added easily.

## Testing
1. Copy-paste or import the code of `/blender/intern/cycles/blender/addon/splines_as_hair_sample.py`
2. At the bottom of the script, tweak how many splines you want to have generated.
3. Run the script. If you generate large number of splines, like 1 million, it will easily take 15 minutes.
4. Add a material and play with HairInfo.Intercept, HairInfo.Key and HairInfo.Value

## Issues
None at the moment.
Don't hesitate to report any issue you see.