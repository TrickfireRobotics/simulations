# Starfield skybox

`starfield.png` is built from NASA/Goddard Space Flight Center's
[Deep Star Maps 2020](https://svs.gsfc.nasa.gov/4851/), an all-sky map
derived from ESA Gaia DR2 data.

Credit: NASA/Goddard Space Flight Center Scientific Visualization Studio.
Gaia DR2 data: ESA/Gaia/DPAC.

The source `starmap_2020_8k.exr` was tone-mapped (linear to sRGB) and
downsized to a 4096x2048 equirectangular PNG, applied as an unlit,
double-sided material on a large sphere (see `model.sdf`). This isn't a
real cubemap skybox: gz-sim (Harmonic) parses `<scene><sky><cubemap_uri>`
but never wires it to the renderer, so the sphere is the standard
workaround.
