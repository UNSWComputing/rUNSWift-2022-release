#ifndef REGION_TEST
#include "perception/vision/Region/Region.hpp"
#else
#include "Region.hpp"
#endif

#include "soccer.hpp"

/**
 * Standard Constructor
 *  Produces a region-space
 * @bounding_box Bounding box reflecting the top-left and bottom-right points in raw-space
 * @is_top_camera True if this region will reflect the top camera, False if bottom camera
 * @foveae The relevant top or bottom  fovea containing meta on the raw image
 * @density the ZOOM_IN density of this region-space with respect to raw-space
 */
RegionI::RegionI(BBox bounding_box, bool is_top_camera, Fovea& fovea,
                                                                    int density)
                     : is_top_camera_(is_top_camera),
                      this_fovea_(&fovea),
                      n_raw_cols_in_region_(bounding_box.width() * density),
                      n_raw_rows_in_region_(bounding_box.height() * density),
                      density_to_raw_(density),
                      y_offset_raw_(bounding_box.a.y() * density),
                      x_offset_raw_(bounding_box.a.x() * density),
                      bounding_box_rel_(bounding_box),
                      bounding_box_fovea_(bounding_box),
                      bounding_box_raw_(bounding_box)
{
    bounding_box_raw_.a *= density;
    bounding_box_raw_.b *= density;
    raw_to_fovea_density_ = fovea.getDensity();
    bounding_box_fovea_.a = bounding_box_raw_.a/fovea.getDensity();
    bounding_box_fovea_.b = bounding_box_raw_.b/fovea.getDensity();
    if (is_top_camera) {
        raw_total_width_ = TOP_IMAGE_COLS;
    } else {
        raw_total_width_ = BOT_IMAGE_COLS;
    }
    fovea_width_ = this_fovea_->getBBox().width();

    // As this constructor is only called when the full frame is being generated
    // the saliance/colour array is always created
    colour_array_set_ = true;
}

/**
 * Copy-Constructor
 *  Produces a region2-space from a region1-space
 * @region region1-space object
 * @box a Relative Bounding box from the source region
 * @rel_density the magnitude of the density change between region1-space and region2-space
 *   such that region2-density = region1-density * rel_density if ZOOMING IN or ;
 *   such that region2-density = region1-density / rel_density if ZOOMING OUT or ;
 * @rel_density_multiplier a #define on whether we are ZOOMING IN or ZOOMING OUT
 * @regenerate_fovea_colour whether the fovea colour classification data should be regenerated at the new density.
 */
void RegionI::init_(const RegionI& region, BBox box, int rel_density = 1,
                     int rel_density_multiplier = DENSITY_MAINTAIN,
                     int window_size = UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
                     int percentage = UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
                     const bool regenerate_fovea_colour = false)
{
    // Make sure box is within bounds of the image
    // Leftmost x value is solution to
    // x * region.density_to_raw_ + region.x_offset_raw_ = 0
    // Rightmost x value is solution to
    // x * region.density_to_raw_ + region.x_offset_raw_ = raw_total_width_
    // Topmost y value is solution to
    // y * region.density_to_raw_ + region.y_offset_raw_ = 0
    // Botmost y value is solution to
    // y * region.density_to_raw_ + region.y_offset_raw_ = raw_total_rows_
    int raw_total_height;

    if (is_top_camera_) {
        raw_total_height = TOP_IMAGE_ROWS;
    }
    else {
        raw_total_height = BOT_IMAGE_ROWS;
    }

    int min_x = -region.x_offset_raw_ / region.density_to_raw_;
    int max_x = (raw_total_width_ - region.x_offset_raw_) / region.density_to_raw_;
    int min_y = -region.y_offset_raw_ / region.density_to_raw_;
    int max_y = (raw_total_height - region.y_offset_raw_) / region.density_to_raw_;

    box.a.x() = std::min(std::max(box.a.x(), min_x), max_x);
    box.b.x() = std::min(std::max(box.b.x(), min_x), max_x);
    box.a.y() = std::min(std::max(box.a.y(), min_y), max_y);
    box.b.y() = std::min(std::max(box.b.y(), min_y), max_y);

    x_offset_raw_ = box.a.x() * region.density_to_raw_ + region.x_offset_raw_;
    y_offset_raw_ = box.a.y() * region.density_to_raw_ + region.y_offset_raw_;

    if (rel_density_multiplier == DENSITY_INCREASE) {
        density_to_raw_ = region.density_to_raw_ * rel_density;
    }
    else if (rel_density_multiplier == DENSITY_DECREASE) {
        density_to_raw_ = region.density_to_raw_ / rel_density;
    }
    else {
        density_to_raw_ = region.density_to_raw_;
    }

    n_raw_cols_in_region_ = box.width() * region.density_to_raw_;
    n_raw_rows_in_region_ = box.height() * region.density_to_raw_;

    // Create the bounding boxes.
    bounding_box_rel_ = BBox(Point(0, 0), Point(n_raw_cols_in_region_ /
                     density_to_raw_, n_raw_rows_in_region_ / density_to_raw_));

    // Fovea relative bounding box.
    bounding_box_fovea_ = BBox(
        Point((x_offset_raw_ / raw_to_fovea_density_) -
                this_fovea_->getBBox().a.x(),
                (y_offset_raw_ / raw_to_fovea_density_) -
                this_fovea_->getBBox().a.y()),
        Point((x_offset_raw_ + n_raw_cols_in_region_) /
                raw_to_fovea_density_ - this_fovea_->getBBox().a.x(),
                (y_offset_raw_ + n_raw_rows_in_region_) /
                        raw_to_fovea_density_ - this_fovea_->getBBox().a.y()));

    // Raw image bounding box.
    bounding_box_raw_ = BBox(Point(x_offset_raw_, y_offset_raw_),
                             Point(x_offset_raw_ + n_raw_cols_in_region_,
                                   y_offset_raw_ + n_raw_rows_in_region_));

    int currentWindowSize, currentPercentage;

    currentWindowSize = this_fovea_->getWindowSize();
    currentPercentage = this_fovea_->getPercentage();
    if(window_size == UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE)
    {
        window_size = this_fovea_->getWindowSize();
        percentage = this_fovea_->getPercentage();
    }

    // If the position of the new bounding box has moved to be outside the old, the fovea will need to be regenerated
    bool bb_change = false;
    BBox foveaBB = this_fovea_->getBBox();
    if (bounding_box_fovea_.a.x() < 0 ||
        bounding_box_fovea_.a.y() < 0 ||
        bounding_box_fovea_.b.x() > foveaBB.width() ||
        bounding_box_fovea_.b.y() > foveaBB.height())
    {
        bb_change = true;
    }

    // If anything has changed then the colour array needs to be regenerated
    if((density_to_raw_ < raw_to_fovea_density_ || bb_change ||
        window_size != currentWindowSize || percentage != currentPercentage))
        colour_array_set_ = false;
    else
        colour_array_set_ = true;
    // If needed generate a new fovea at the required density/size/thresholding values.
    if(!colour_array_set_ && regenerate_fovea_colour)
    {
        colour_array_set_ = true;
        raw_to_fovea_density_ = density_to_raw_;
        this_fovea_ = this_fovea_->getChildFovea(BBox(
            Point(x_offset_raw_ / raw_to_fovea_density_, y_offset_raw_ /
                                                         raw_to_fovea_density_),
            Point((x_offset_raw_ + n_raw_cols_in_region_) /
                  raw_to_fovea_density_, (y_offset_raw_ + n_raw_rows_in_region_)
                                                      / raw_to_fovea_density_)),
                       density_to_raw_, is_top_camera_, regenerate_fovea_colour,
                                    window_size, percentage);
        bounding_box_fovea_ = BBox(Point(0, 0), Point(bounding_box_rel_.b.x(),
                                                      bounding_box_rel_.b.y()));
    }

    fovea_width_ = this_fovea_->getBBox().width();
}


/**
 * Overload: Copy-assignment
 */
RegionI& RegionI::operator=(const RegionI& region) {
    bounding_box_rel_ = region.bounding_box_rel_;
    bounding_box_fovea_ = region.bounding_box_fovea_;
    bounding_box_raw_ = region.bounding_box_raw_;
    is_top_camera_ = region.is_top_camera_;
    this_fovea_ = region.this_fovea_;
    n_raw_cols_in_region_ = region.n_raw_cols_in_region_;
    n_raw_rows_in_region_ = region.n_raw_rows_in_region_;
    density_to_raw_ = region.density_to_raw_;
    y_offset_raw_ = region.y_offset_raw_;
    x_offset_raw_ = region.x_offset_raw_;
    raw_total_width_ = region.raw_total_width_;
    raw_to_fovea_density_ = region.raw_to_fovea_density_;
    fovea_width_ = region.fovea_width_;
    colour_array_set_ = region.colour_array_set_;
    return *this;
}

/**
 * Returns iterator_raw to the beginning of the region-space
 * @return iterator_raw to the beginning of the region-space
 */
RegionI::iterator_raw RegionI::begin_raw() const {
    return iterator_raw(
        this,
        getLinearPosFromXYRaw_(0,0),
        x_offset_raw_,
        y_offset_raw_,
        density_to_raw_,
        raw_total_width_,
        n_raw_cols_in_region_
    );
}
RegionI::iterator_fovea RegionI::begin_fovea() const {
    int s = raw_to_fovea_density_;
    if(!colour_array_set_) {
        // Warning for incorrectly generated colour array
        // The region that this iterator is linked to does
        // not have a valid colour array and should be regenerated
        std::cout << "---POTENTIAL SEGMENTATION FAULT---\nColour array not correctally generated\n";
    }
    return iterator_fovea(
        this,
        getLinearPosFromXYFovea_(0,0),
        bounding_box_fovea_.a.x(),
        bounding_box_fovea_.a.y(),
        (density_to_raw_/s),
        fovea_width_,
        (n_raw_cols_in_region_/s)
    );
}

/**
 * Creates a new region from an existing region. The new region may have
 * a different position and size compared to the old region.
 * @offset the position of the upper left corner of the new region
 *   relative to the upper left corner of the existing region.
 * @size the size of the new region at the same density as the existing
 *   region.
 */
RegionI RegionI::subRegion(const Point& offset, const Point& size) const
{
    RegionI new_region(*this);
    new_region.init_(*this, BBox(offset, size), 1, DENSITY_MAINTAIN,
        UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
        UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE, true);
    return(new_region);
}

/**
 * Creates a new region from an existing region. The new region may have
 * a different position and size compared to the old region.
 * @new_box a new bounding box positioned such that the upper left corner
 *   is relative to the old bounding box and botha are at the same density.
 */
RegionI RegionI::subRegion(const BBox& new_box) const
{
    RegionI new_region(*this);
    new_region.init_(*this, new_box, 1, DENSITY_MAINTAIN,
        UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
        UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE, true);
    return(new_region);
}

/**
 * Creates a new, zoomed in, region from an existing region. The entire
 * existing region is covered, resulting in more pixels at region density.
 * @factor The factor by which to zoom in. Should be a power of 2.
 * @regenerate_fovea_colour Whether to regenerate the colour classification.
 *   Only set this to false if you are sure you don't need the classified
 *   image.
 */
RegionI RegionI::zoomIn(const int factor,
                                const bool regenerate_fovea_colour) const
{
    RegionI new_region(*this);
    new_region.init_(*this, BBox(Point(0,0), Point(getCols(), getRows())),
        factor, DENSITY_DECREASE, UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
        UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE, regenerate_fovea_colour);
    return(new_region);
}

/**
 * Creates a new, zoomed out, region from an existing region. The region is
 * not expanded, resulting in fewer pixels at region density.
 * @factor The factor by which to zoom out. Should be a power of 2.
 * @regenerate_fovea_colour Whether to regenerate the colour classification.
 *   Only set this to false if you are sure you don't need the classified
 *   image.
 */
RegionI RegionI::zoomOut(const int factor,
                                const bool regenerate_fovea_colour) const
{
    RegionI new_region(*this);
    new_region.init_(*this, BBox(Point(0,0), Point(getCols(), getRows())),
        factor, DENSITY_INCREASE, UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
        UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE, regenerate_fovea_colour);
    return(new_region);
}

/**
 * Regenerates the colour classification within the region using the window
 * size and thresholding value provided.
 * @window_size The size of the window used in adaptive thresholding.
 * @thresholding_value The relative threshold used to determine whether a
 *   pixel is considered white or not white during adaptive thresholding.
 */
RegionI RegionI::reclassify(const int window_size,
                                        const int thresholding_value) const
{
    RegionI new_region(*this);
    new_region.init_(*this, BBox(Point(0,0), Point(getCols(), getRows())),
        1, DENSITY_MAINTAIN, window_size, thresholding_value, true);
    return(new_region);
}

/**
 * Allows for a complex regeneration of a region when you need to change
 * many things at once.
 * @offset the position of the upper left corner of the new region
 *   relative to the upper left corner of the existing region. Point(0,0)
 *   if no change is needed.
 * @size the size of the new region at the same density as the existing
 *   region. Point(region.getCols(), region.getRows()) if unchanged.
 * @factor The factor by which to zoom out. Should be a power of 2.
 * @window_size The size of the window used in adaptive thresholding.
 * @thresholding_value The relative threshold used to determine whether a
 *   pixel is considered white or not white during adaptive thresholding.
 * @regenerate_fovea_colour Whether to regenerate the colour classification.
 *   Only set this to false if you are sure you don't need the classified
 *   image.
 */
RegionI RegionI::new_region(const Point& offset, const Point& size,
    const bool zoom_in, const int factor, const int window_size,
    const int thresholding_value, const bool regenerate_fovea_colour) const
{
    // The new region.
    RegionI new_region(*this);

    // What kind of zoom to perform.
    int zoom_type = DENSITY_MAINTAIN;

    // Determine the zoom type required.
    if(factor != 1)
    {
        if(zoom_in)
            zoom_type = DENSITY_DECREASE;
        else
            zoom_type = DENSITY_INCREASE;
    }

    // Create the new region.
    new_region.init_(*this, BBox(offset, size), factor, zoom_type,
                                window_size, thresholding_value, true);

    // Return the new region.
    return(new_region);
}
