# Region

## Table of Contents

* [Overview](#overview)
* [The Standard Regions](#the-standard-regions)
* [Accessing Pixels](#accessing-pixels)
   * [Iterators](#iterators)
* [Moving and Resizing](#moving-and-resizing)
* [Zoom](#zoom)
* [Reclassifying](#reclassifying)
* [Advanced Topics](#advanced-topics)
   * [Complex Constructor](#complex-constructor)
   * [regenerate_fovea_colour](#regenerate_fovea_colour)
   * [Why new?](#why-new)
* [Additional Technical Information](#additional-technical-information)
   * [Purpose](#purpose)
   * [Cache](#cache)
   * [Row Major and Column Major](#row-major-and-column-major)
   * [YUV422](#yuv422)
   * [Fovea](#fovea)

## Overview

The `RegionI` (Region Iterator) class is the core system through which vision algorithms function. A Region of
Interest (RoI) is a rectangular section of a single frame from top or bottom camera. `RegionI` is a class containing
tools to make  creating and using RoI as easy as possible, while allowing for the level of optimisation required in the
SPL. A `RegionI` stores the position, width and height of an RoI. It also stores a zoom level and, optionally, a link
to a colour classified view of the RoI.

WARNING: Examples have been compiled and run, but have not been thoroughly debugged. Tests used early 2021 rUNSWift.

## The Standard Regions

The starting point for all regions can be found in `VisionInfoMiddle.full_regions`. If you're dealing with a standard
detector, inheriting `DetectorInterface`, `VisionInfoMiddle` is provided as `info_middle` in the `detect`
function:

```c++
virtual void detect(const VisionInfoIn& info_in,
                    VisionInfoMiddle& info_middle, VisionInfoOut& info_out) = 0;
```

`full_regions` is a `std::vector<RegionI>` containing two regions, the first is the full top camera image, the
second is the full bottom camera image. Also important is `info_middle.roi`, another `std::vector<RegionI>`. `roi`
contains all the regions found during the Region of Interest finding portion of the vision pipeline. At time of writing
it is primarily populated by the `ColourROI` class, which attempts to find connected blobs of pixels classified as
white in the colour classification stage.

If you are writing a new RoI finder, or the existing RoI are not appropriate to your application, use `full_regions`.
In every other case `roi` is recommended, as it lets you quickly discard many uninteresting parts of the image, such
as empty field green.

## Accessing Pixels

The primary purpose of `RegionI` is to efficiently access both raw and colour classified pixels in the camera image.
The colour classified pixels within a region can be accessed using functions and iterators referencing the words
"colour" or "fovea". The raw yuv values given to rUNSWift by the camera can be accessed using functions and iterators
referencing the world "raw".

In live code you should nearly always use iterators to access pixel values as they are much faster than the functions
that take x, y value parameters. The x, y value functions should only be used if you are accessing a relatively small
number of pixels within a region. You might only need a single line, a few widely separated pixels in irregular
positions, or otherwise <10% of the region. The other case you might use these functions is during early prototyping, as
it can sometimes be easier to think about these while attempting to code an algorithm for the first time. You will need
to convert this kind code to the use of iterators before use on a robot or it is likely to be far too slow.

### Iterators

An iterator is essentially a pointer to a single pixel within a region. Iterators move from the top left to the bottom
right in row major order. `RegionI` iterators are very efficient at scanning though a RoI this way, so for most
applications this is your most computationally efficient option.

There are two kinds of iterator used by `RegionI`, both of which inherit from the core `iterator` class interface,
`iterator_fovea` and `iterator_raw`. At time of writing `iterator_fovea` is only used to access colour classified
pixel values. `iterator_raw` is used to access the underlying yuv values given by the camera. The two iterators
have identical functionality, except that `iterator_colour` is accessed with `colour()` (or a few other functions
that grab adjacent pixels) and `iterator` raw is accessed with `getY()`, `getU()` and `getV()`. Examples will
that use one type of iterator should be easy to translate to the other.

The core way of using a `RegionI` iterator is:

```c++
RegionI::iterator_fovea end = region.end_fovea();
for(RegionI::iterator_fovea it = region.begin_fovea(); it<end; ++it)
{
    Colour pixel = it.colour();
    // Do stuff with pixel.
}
```

An obvious extension is the case where you need to compare horizontally adjacent pixels:

```c++
int width = region.getCols();
int x=1;
RegionI::iterator_raw it = region.begin_raw();
RegionI::iterator_raw end = region.end_raw();
uint8_t last_pixel = it.getY();

++it;
for(; it<end; ++it)
{
    if(x==width)
    {
        x=1;
        last_pixel = it.getY();
        ++it;
    }
    uint8_t pixel = it.getY();
    // Do stuff with pixel and last_pixel.
    ++x;
}
```

Sometimes you want to scan in column major rather than row major order. Due to the way memory and the cache works (see
Additional Technical Information) this should be done by scanning the region in row major order and buffering the
results in a vector:

```c++
int width = region.getCols();
RegionI::iterator_fovea it = region.begin_fovea();
RegionI::iterator_fovea end = region.end_fovea();
std::vector<Colour> last_pixels(width);

// Grab the pixel values of the first row.
for(std::vector<Colour>::iterator pixel = last_pixels.begin(); pixel<last_pixels.end(); ++pixel)
{
    *pixel = it.colour();
    ++it;
}

// Run through the remaining rows.
while(it<end)
{
    for(std::vector<Colour>::iterator pixel = last_pixels.begin(); pixel<last_pixels.end(); ++pixel)
    {
        Colour last_pixel = *pixel;
        *pixel = it.colour();
        // Do stuff with pixel and last_pixel.
        ++it;
    }
}
```

## Moving and Resizing

A new `RegionI` can created from an existing `RegionI` with a different location and/or size. There are two
functions provided for this purpose:

```c++
RegionI subRegion(const Point& offset, const Point& size) const;
RegionI subRegion(const BBox& new_box) const;
```

Both functions allow for both new location and size, they simply allow you to work with `Point` or `BBox` as fits
your situation. Both also work relative to the existing region, so offset of (0, 0) will result in the same location as
the existing `RegionI`. They also function at the same zoom/density as the existing `RegionI`. Zoom and density are
explained in the Zoom section, so look there you want an explanation of those concepts.

`Point`, found in `robot/types/Point.hpp` is actually just a typedef of Eigen::Vector2i, which is itself simply an
array of two `int`. Eigen allows you to access the data pretty much however you like, so `point.x() = 1;`,
`point[0] = 1;` and `point(0) = 1;` all set the point's x value to 1. New points are created with
`Point point = Point(x, y);`.

`BBox` is a `struct` found in `robot/types/BBox.hpp`. It defines a rectangle and provides functions to manipulate
that rectangle. The rectangle consists of an upper left `Point`, a, and a bottom right `Point`, b, both of which are
public and can be altered as normal for points. `BBox` provides a number of functions. Take a look at those in
`BBox.hpp`to see if one does what you want before just directly using a and b.

Here are a few examples of moving and/or resizing a `RegionI` to get you started:

Create a new `RegionI` in a new location relative to an existing `RegionI`:

```c++
Point offset = Point(x, y);
RegionI *new_region =
    new RegionI(old_region.subRegion(offset, old_region.getBoundingBoxRel().b));
```

Rescale a `RegionI` while keeping the centre of the RoI in place:

```c++
// Works whether x > 1 or x < 1.
float scaleFactor = x;
RegionI *new_region = new RegionI(
    old_region.subRegion(old_region.getBoundingBoxRel().expand(scaleFactor)));
```

Double the height of a `RegionI` while maintaining the position of the upper left point:

```c++
BBox new_box = old_region.getBoundingBoxRel();
new_box.b.y() *= 2;
RegionI *new_region = new RegionI(old_region.subRegion(new_box));
```

This is a deliberately obscure case to demonstrate advanced use, normally you would just start with `full_regions` to
do this, which makes the code as simple as that above. If that somehow wasn't an option, here is how to create a new
`RegionI` in a new location given in full frame coordinates from any `RegionI`:

```c++
Point offset = Point(x, y);
BBox new_bbox_rel = old_region.getBoundingBoxRel();
new_bbox_rel.a -= old_region.getBoundingBoxRaw().a / old_region.getDensity();
new_bbox_rel.a += offset;
RegionI *new_region = new RegionI(old_region.subRegion(new_bbox_rel));
```

A final note: Any time moving or resizing a `RegionI` would take you outside the frame the region will automatically
be constrained to a size and position inside the frame. The result is a `RegionI` covering whatever part of your
desired rectangle remains inside the frame.

## Zoom

Looking at every pixel in an image is computationally expensive. In order to avoid doing this rUNSWift makes extensive
use of subsampling style zoom. This just means we skip pixels, looking at pixels in every nth row and column. Density is
the term we use to describe the number of pixels skipped. At density 1, we're looking at every pixel, the resolution of
the raw image. At density 2 only pixels in even rows and columns are looked at. So (0, 0), (0, 2) and (2, 0) are sampled
at density 2, but (1, 2) and (0, 1) are not. A region's density is accessed with the `getDensity` function.

To allow for simpler implementation `RegionI` only supports density values that are a power of 2 (1 is 2^0). Two
functions allow zooming in and out:

```c++
RegionI zoomIn(const int factor=2,
                            const bool regenerate_fovea_colour=true) const;
RegionI zoomOut(const int factor=2,
                            const bool regenerate_fovea_colour=true) const;
```

The meaning of `regenerate_fovea_colour` is covered in the Advanced Use section. Generally, just ignore it so it is
automatically set to `true`. `zoomIn` decreases density, so the new density is density/factor. `zoomOut` increases
density, so the new density is density*factor. Neither function will change the size or location of the RoI, so after
calling `zoomIn` iterating through the region will cover more pixels.

Here are some examples of zooming to get you started:

Zoom in by a factor of 2 (so density is halved):

```c++
RegionI *new_region = new RegionI(old_region.zoomIn());
```

Zoom out by a factor of 4 (so density is quadrupled):

```c++
RegionI *new_region = new RegionI(old_region.zoomOut(4));
```

Calling `zoomIn` when density is already 1 has undefined behaviour.

## Reclassifying

The colour classification by default is optimised to give good results in the RoI finding stage of vision. This may not
be appropriate for other applications. To handle this case `RegionI` provides functions to reclassify the covered
region with a different set of colour classification parameters. A recent example of practical use is reclassifying
parts of the ball that are in shadow to better detect black spots in these areas.

A single function provides this:

```c++
RegionI reclassify(const int window_size, const int thresholding_value) const;
```

Here is a simple example of how to use this function:

```c++
RegionI *new_region =
    new RegionI(old_region.reclassify(new_window_size, new_thresholding_value));
```

For information on what `window_size` and `thresholding_value` do see the Adaptive Thresholding section of the
Vision documentation.

## Advanced Topics

This section covers a few advanced topics slightly more complex than what was discussed in other sections.

### Complex Constructor

`RegionI` provides an advanced function for occasions where you want to change several things about a region at once:

```c++
RegionI *new_region(const Point& offset, const Point& size,
    const bool zoom_in=true, const int factor=1,
    const int window_size=UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
    const int thresholding_value=UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
                            const bool regenerate_fovea_colour=true) const;
```

The parameters do exactly the same thing as done by the other region creation functions, but combined into one larger
function. Here are some examples:

In this example the new `RegionI` is zoomed out and uses new adaptive thresholding settings, but doesn't change
position:

```c++
RegionI *new_region = new RegionI(old_region.new_region(
    old_region.getBoundingBoxRel().a, old_region.getBoundingBoxRel().b,
                    false, 2, new_window_size, new_thresholding_value));
```

### regenerate_fovea_colour

Another advanced topic is `regenerate_fovea_colour`, which appears both in this and the zoom functions. To use this
parameter correctly you will need to understand a little of the underlying complexity handled by `RegionI`.

Every `RegionI` points to a `Fovea`, which contains both a pointer to the raw image and a colour classified image.
Several`RegionI` can share a single `Fovea`, and will try to do so as much as possible for efficiency reasons. In
fact at  time of writing all the `RegionI` in `roi` point to one of the two fovea generated when the
`full_regions` `RegionI` were created.

When a new `RegionI` is created using any of the functions discussed here (only the original `full_regions` should
use the actual constructor) there is a parent `RegionI` from which the new child `RegionI` is being generated. The
child `RegionI` automatically determines if the parent's `Fovea` provides the colour information it needs. If the
parent's `Fovea` does not provide the information needed a new `Fovea` is generated that does. A new `Fovea` is
needed under one of three cases:

* The parent's `Fovea` was generated with a higher density value than the child requires (i.e. the parent's `Fovea` is too zoomed out).
* The parent's `Fovea` doesn't cover the area the child's window covers.
* The parent's `Fovea` uses different colour classification parameters to the child.

`regenerate_fovea_colour` allows you to override this behaviour. If you know you will never need colour from a newly
created `RegionI` you can set `regenerate_fovea_colour` to `false` to prevent computation time being spent
creating an unnecessary colour classification. This basically means you intend to use the raw pixel colours only.

In this example a `RegionI` is zoomed in and expanded with very little overhead by preventing `RegionI` from
performing a new colour classification:

```c++
BBox expanded_region = old_region.expand(2.0f);
RegionI *new_region = new RegionI(old_region.new_region(expanded_region.a,
    expanded_region.b, true, 2, UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
                            UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE, false);
```

### Why `new`?

A natural question to ask about the examples here is:

```c++
    // Why use this:
    RegionI *new_region = new RegionI(old_region.func(params));
    // Rather than just:
    RegionI new_region = old_region.func(params);
```

The answer goes back to a bug rUNSWift had on the V5s. The program would randomly crash. Eventually the problem was
traced to a memory issue, despite overall RAM usage being low. It seemed like creating the colour classification arrays
used by `Fovea` without using new was, for some reason, putting the arrays on the stack, and naturally causing a stack
overflow. The problem was fixed by changing `Fovea` creation to  use `new` whenever creating its large arrays AND to
make sure Fovea themselves were created with `new`. Yes, it shouldn't have worked that way. But it did.

When `RegionI` was introduced later on the problem reappeared. To fix it we had to make sure any time a new
`RegionI` that would generate a new `Fovea` was created it was created with `new`. Rather than worrying about
which `RegionI` actually end up generating new `Fovea` we simply adopted the strategy of creating all `RegionI`
with `new`.

At time of writing no one has tested whether the V6 Nao have the peculiar V5 behaviours that forced us to do all this.

## Additional Technical Information

None of the information in this section should be needed to make use of `RegionI`. This section exists only to provide
a plain text explanation of how `RegionI` works and why.

### Purpose

Frequently vision algorithms need to examine some portion of the full image rather than the full frame. Obvious examples
are rectangles around possible balls, field line intersections and robots. Programmers with less experience in
optimisation and/or who aren't aware of some of the peculiarities of the Nao processor and rUNSWift codebase tended to
do this with code that is 10 to 100 times slower than what `RegionI` can achieve. Those with more expertise still took
some time to write and debug this code each time it came up, and often errors would sneak in to master this way.

The goal of `RegionI` is to provide functions that do these tasks as quickly as possible. Additionally, as `RegionI`
is used throughout vision, its code has been thoroughly examined and debugged to ensure it works correctly. Finally, if
someone needs a new tool for looking through pixels (at time of writing scan lines have come up) their work can be added
to `RegionI` so that future work can easily make use of that method.

### Cache

One of the reasons why `RegionI` can perform so much better than naive methods is cache optimisation. Modern
processors attempt to predict which data is likely to be accessed next and load it from RAM into a smaller chunk of
memory that can be much more rapidly accessed by the CPU (the cache). Processors usually assume nearby memory blocks are
more likely to be accessed. Ultimately this means the fastest way to read memory is in the same order as it is stored in
RAM. `RegionI` iterators are designed to do this as much as possible with as little overhead as possible.

### Row Major and Column Major

Internally 2D images are stored as 1D arrays. In rUNSWift this means the image is unravelled row by row into this 1D
arrangement. The last pixel of the 1st row is followed by the 1st pixel of the 2nd row, then the rest of the 2nd row,
then the 1st pixel of the 3rd row, and so on. This is called row major order, with the most common alternative being
column major order (i.e. as above but replacing "row" with "column"). All images in rUNSWift are in row major order. The
matrix used by Eigen are actually in column major order, though this doesn't come up often in vision.

### YUV422

The raw image received by rUNSWift from the camera is in YUV422 format. YUV images were designed for the period in which
colour TV was just being introduced. The Y channel contained all the information needed by older black and white TVs
(i.e. the greyscale image), while U and V contain the colour components of the image. U and V don't have a simple
meaning, and are best understood by looking at YUV colour charts online.

YUV422 uses a format which saves space by focussing on the Y channel to the detriment of the U and V channels. Every
pixel gets its own Y value, while a single U and V value are shared by two horizontally adjacent pixels. The values in
the raw image are therefor an array with the form YUYVYUYV...YUYV. The first pixel is `y=im[0]; u=im[1]; v=im[3]`
while the second is `y=im[2]; u=im[1]; v=im[3]`. This means odd and even pixels have different relative locations of
their colour values, adding complexity to the implementation of `RegionI`.

### Fovea

Throughout `RegionI` are references to `Fovea`. The `Fovea` class is an older system in rUNSWift that was once
directly accessed by vision algorithms. With the introduction of `RegionI` `Fovea` should only every be accessed
through a `RegionI`.

`Fovea` contain both a reference to the original raw image and any processed images used by rUNSWift. While this
currently is only the colour classified image, `Fovea` used to handle edge and blurred images as well. If extra images
like this are reintroduced they should be placed in `Fovea` access through `RegionI::iterator_fovea` with a new
function similar to the existing `.colour()`.

While initially only a relatively low resolution full frame `Fovea` is created `Fovea` was designed to generate new
higher resolution images of subsections of the image. This is the source of the name, drawing analogy with the human
eye's fovea focussing on a particular location.

It is useful to keep `RegionI` and `Fovea` separate as they perform quite different tasks when looked at closely.
`Fovea` is designed to make use of actual arrays that have been generated in full. Back when `Fovea` was used alone
this meant zooming out for a lower resolution image or looking at only a segment of the current `Fovea` image would
incur significant cost as the classified image was regenerated at that location and resolution.

`RegionI` takes a step back from the actual internal array and focussed on only what it needs to meet the requirements
of the `RegionI`'s parameters. `RegionI` will only create new classified images (by creating new `Fovea`) when it
needs to zoom in to locations and resolutions beyond what the current `Fovea` can provide, or needs a different set of
colour classification parameters. `RegionI` can work on a subsection of an actual array, or subsample the pixels of
that array to provide the same effect as zooming out. An arbitrary number of `RegionI` can work off a single `Fovea`
as long as that `Fovea` provides the classified pixels they need.
