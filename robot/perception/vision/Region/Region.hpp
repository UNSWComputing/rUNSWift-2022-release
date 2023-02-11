#ifndef PERCEPTION_VISION_REGION_H_
#define PERCEPTION_VISION_REGION_H_

#include <list>
#include <map>

#ifndef REGION_TEST
#include "types/BBox.hpp"
#include "types/CombinedFovea.hpp"
#else
#define TOP_IMAGE_COLS 1024
#define BOT_IMAGE_COLS 1024
#define TOP_SALIENCY_DENSITY 8
#define BOT_SALIENCY_DENSITY 8
#include "Fovea.hpp"
#include "BBox.hpp"
#endif

#define DENSITY_MAINTAIN 0
#define DENSITY_INCREASE 1
#define DENSITY_DECREASE 2

/**
 * Enumerated type for certainty map
 * add more as required
 */
enum RegionIType {
    BALL,
    GOAL,
    ROBOT,
    N_TYPES     // keep this last
};

namespace offnao {
   class Vision_RegionI;
}

/**
 * Region is in essence a bounding box describing a region within the full fovea
 * There are a few key terms to get your head around:
 *  - region-space: a space where (x,y) refer to the  a bounding box and density
 *  - raw-space: a space where (x,y) refer to the absolute pixel positions on the raw
 *      image and the density is 1 because the image is at full sampling
 * These relationships can be summarised by:
 *  region-space.x = raw-space.x + region-space.x_offset + region-space.x * region-space.density
 *  region-space.y = raw-space.y + region-space.y_offset + region-space.y * region-space.density
 *
 * Learn to use Regions! See README.md (open this folder on Github) for guidance.
 */
class RegionI {

public:

    //////////////////////////////////
    // Iterator sub class
    //////////////////////////////////
    #include "perception/vision/Region/RegionIterator.tcc"
    #include "perception/vision/Region/RegionIteratorRaw.tcc"
    #include "perception/vision/Region/RegionIteratorFovea.tcc"

    /**
     * Empty constructor
     *  Used in serialisation.
     */
    RegionI() {}

    /**
     * Standard Constructor
     *  Produces a region-space
     * @bounding_box Bounding box reflecting the top-left and bottom-right points in raw-space
     * @is_top_camera True if this region will reflect the top camera, False if bottom camera
     * @foveae The relevant top or bottom  fovea containing meta on the raw image
     * @density the ZOOM_IN density of this region-space with respect to raw-space
     */
    RegionI(BBox bounding_box, bool is_top_camera, Fovea& fovea, int density);

    /**
     * Overload: Copy-assignment
     */
    RegionI& operator=(const RegionI&);

    /**
     * Standard copy constructor
     */
    RegionI(const RegionI& region)
    {
        *this = region;
    }

    /**
     * Creates a new region from an existing region. The new region may have
     * a different position and size compared to the old region.
     * @offset the position of the upper left corner of the new region
     *   relative to the upper left corner of the existing region.
     * @size the size of the new region at the same density as the existing
     *   region.
     */
    RegionI subRegion(const Point& offset, const Point& size) const;

    /**
     * Creates a new region from an existing region. The new region may have
     * a different position and size compared to the old region.
     * @new_box a new bounding box positioned such that the upper left corner
     *   is relative to the old bounding box and botha are at the same density.
     */
    RegionI subRegion(const BBox& new_box) const;

    /**
     * Creates a new, zoomed in, region from an existing region. The entire
     * existing region is covered, resulting in more pixels at region density.
     * Factor of 2 results in 4x the number of pixels (2x for each dimension).
     * @factor The factor by which to zoom in. Should be a power of 2.
     * @regenerate_fovea_colour Whether to regenerate the colour classification.
     *   Only set this to false if you are sure you don't need the classified
     *   image.
     */
    RegionI zoomIn(const int factor=2,
                                const bool regenerate_fovea_colour=true) const;

    /**
     * Creates a new, zoomed out, region from an existing region. The region is
     * not expanded, resulting in fewer pixels at region density.
     * Factor of 2 results in 1/4 the number of pixels (1/2 for each dimension).
     * @factor The factor by which to zoom out. Should be a power of 2.
     * @regenerate_fovea_colour Whether to regenerate the colour classification.
     *   Only set this to false if you are sure you don't need the classified
     *   image.
     */
    RegionI zoomOut(const int factor=2,
                                const bool regenerate_fovea_colour=true) const;

    /**
     * Regenerates the colour classification within the region using the window
     * size and thresholding value provided.
     * @window_size The size of the window used in adaptive thresholding.
     * @thresholding_value The relative threshold used to determine whether a
     *   pixel is considered white or not white during adaptive thresholding.
     */
    RegionI reclassify(const int window_size,
                                            const int thresholding_value) const;

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
    RegionI new_region(const Point& offset, const Point& size,
        const bool zoom_in=true, const int factor=1,
        const int window_size=UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
        const int thresholding_value=UNDEFINED_ADAPTIVE_THRESHOLDING_VALUE,
                                const bool regenerate_fovea_colour=true) const;

    /**
     * For a region-relative (x,y) pixel, return the
     *  pointer to the raw YUV value.
     * Slower than iterators. Use for prototyping or cases where
     *  only a small set of pixels are needed.
     * @x region-relative x coordinate
     * @y region-relative x coordinate
     * @return pointer to the raw YUV value of the pixel
     */
    inline const
    #ifdef REGION_TEST
    int
    #else
    uint8_t*
    #endif
    getPixelRaw(int x, int y) const {
        return getPixelRaw_(getLinearPosFromXYRaw_(x, y));
    }

    /**
     * For a region-relative (x,y) pixel, return the
     *  reference to the colour enum type
     * Slower than iterators. Use for prototyping or cases where
     *  only a small set of pixels are needed.
     * @x region-relative x coordinate
     * @y region-relative x coordinate
     * @return reference to the colour enum type of the pixel
     */
    inline
    #ifdef REGION_TEST
    int
    #else
    Colour
    #endif
    getPixelColour(int x, int y) const {
        return getPixelColour_(getLinearPosFromXYFovea_(x, y));
    }

    /**
     * Returns true if the underlying image that this
     *  region is referencing is the top camera. False if not
     * @return True if top camera, false if bottom camera
     */
    inline bool isTopCamera() const { return is_top_camera_; }

    /**
     * Get the number of columns this region represents in region-space
     * @return the number of columns this region represents in region-space
     */
    inline int getCols() const { return (n_raw_cols_in_region_ / density_to_raw_); }

    /**
     * Get the number of rows this region represents in region-space
     * @return the number of rows this region represents in region-space
     */
    inline int getRows() const { return (n_raw_rows_in_region_ / density_to_raw_); }

    /**
     * Get the density of this region-space with respect to the raw-space
     * @return density of this region-space with respect to the raw-space
     */
    inline int getDensity() const { return density_to_raw_; }

    /**
     * Get the density of this region-space with respect to the raw-space
     * @return density of this region-space with respect to the raw-space
     */
    inline int getFoveaDensity() const { return raw_to_fovea_density_; }

    /**
     * Get the bounding box that represents the top-left and bottom-right endpoints
     *  of this region-space
     * @return bounding box for the top-left and bottom-right endpoints  of this region-space
     */
    inline BBox getBoundingBoxRel() const { return bounding_box_rel_; }

    /**
     * Get the bounding box that represents the top-left and bottom-right endpoints
     *  of this region-space
     * @return bounding box for the top-left and bottom-right endpoints  of this region-space
     */
    inline BBox getBoundingBoxFovea() const { return bounding_box_fovea_; }

    /**
     * Get the bounding box that represents the top-left and bottom-right endpoints
     *  of this region-space
     * @return bounding box for the top-left and bottom-right endpoints  of this region-space
     */
    inline BBox getBoundingBoxRaw() const { return bounding_box_raw_; }

    /**
     * Returns iterator to the beginning of the region-space for raw values
     * @return iterator to the beginning of the region-space for raw values
     */
    iterator_raw begin_raw() const;

    /**
     * Returns iterator to the beginning of the region-space for fovea values
     * @return iterator to the beginning of the region-space for fovea values
     */
    iterator_fovea begin_fovea() const;

    /**
     * Returns iterator to one past the end of the region-space for raw values
     * @return iterator to one past the end of the region-space for raw values
     */
    inline iterator_raw end_raw() const {
        return iterator_raw(
            this,
            getLinearPosFromXYRaw_(0, n_raw_rows_in_region_),
            -1,-1,-1,-1,-1
        );
    }

    /**
     * Returns iterator to one past the end of the region-space for fovea values
     * @return iterator to one past the end of the region-space for fovea values
     */
    inline iterator_fovea end_fovea() const {
        return iterator_fovea(
            this,
            getLinearPosFromXYFovea_(0,
            n_raw_rows_in_region_/raw_to_fovea_density_),
            -1,-1,-1,-1,-1
        );
    }

    /**
     * Returns iterator to a point within a region for raw pixel data access.
     * @return iterator to a point within a region.
     */
    iterator_raw get_iterator_raw(Point point) const {
        return iterator_raw(
            this,
            getLinearPosFromXYRaw_(point.x(), point.y()),
            x_offset_raw_,
            y_offset_raw_,
            density_to_raw_,
            raw_total_width_,
            n_raw_cols_in_region_
        );
    }

    /**
     * Returns iterator to a point within a region for fovea data access.
     * @return iterator to a point within a region.
     */
    iterator_fovea get_iterator_fovea(Point point) const
    {
        return iterator_fovea(
            this,
            getLinearPosFromXYFovea_(point.x(), point.y()),
            bounding_box_fovea_.a.x(),
            bounding_box_fovea_.a.y(),
            (density_to_raw_/raw_to_fovea_density_),
            fovea_width_,
            (n_raw_cols_in_region_/raw_to_fovea_density_)
        );
    }

    /**
     * Returns a pointer to a underlying fovea. This is used for blackboard to
     * access the colour saliency arrays.
     */
    inline const Fovea* getInternalFovea() const {
        return this_fovea_;
    }

    /**
     * For vatnao
     */

    void setFovea(Fovea &fovea) {
        this_fovea_ = &fovea;
        raw_to_fovea_density_ = fovea.getDensity();
        bounding_box_fovea_.a = bounding_box_raw_.a/fovea.getDensity();
        bounding_box_fovea_.b = bounding_box_raw_.b/fovea.getDensity();
        fovea_width_ = this_fovea_->getBBox().width();
    };

    /*
     * Serialisation of regions so that they can be sent to offnao and other
     * remote programs.
     */
    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version) {
        ar & is_top_camera_;
        ar & bounding_box_rel_;
        ar & bounding_box_fovea_;
        ar & bounding_box_raw_;
        ar & n_raw_cols_in_region_;
        ar & n_raw_rows_in_region_;
        ar & density_to_raw_;
        ar & y_offset_raw_;
        ar & x_offset_raw_;
        ar & raw_total_width_;
        ar & raw_to_fovea_density_;
        ar & fovea_width_;
    }

   friend void serialise(const RegionI&, offnao::Vision_RegionI &);
   friend void deserialise(RegionI&, const offnao::Vision_RegionI &);
private:

    /**
     * Initialises a region with the specified information.
     */
    void init_(const RegionI& region, BBox box, int rel_density,
        int rel_density_multiplier, const int window_size, const int percentage,
        const bool regenerate_fovea_colour);

    /**
     * For a region-relative linear position (2d scan position
     *  from top left to bottom right, return the
     *  pointer to the raw YUV value
     * @linear_pos 2d scan position from top left to bottom right
     * @return pointer to the raw YUV value of the pixel
     */
    inline const
    #ifdef REGION_TEST
    int
    #else
    uint8_t*
    #endif
    getPixelRaw_(int linear_pos) const {
        return this_fovea_->getRawYUV(linear_pos);
    }

    /**
     * For a region-relative linear position (2d scan position
     *  from top left to bottom right, return the
     *  reference to the colour enum type
     * @linear_pos 2d scan position from top left to bottom right
     * @return reference to the colour enum type of the pixel
     */
    inline
    #ifdef REGION_TEST
    int
    #else
    Colour
    #endif
    getPixelColour_(int linear_pos) const {
        return this_fovea_->getFoveaColour(linear_pos);
    }

    /**
     * For a region-relative x, y coordinate, return the linear
     *  scan position (top left to bottom right) for that
     *  coordinate in raw-relative units.
     * @x horizontal coordinate in the region-relative space
     * @y vertical coordinate in the region-relative space
     * @return linear position of the pixel in the raw-space
     */
    inline int getLinearPosFromXYRaw_(int x, int y) const {
        return (x_offset_raw_ + x*density_to_raw_)*2 + raw_total_width_ * (y_offset_raw_ + y*density_to_raw_)*2;
    }

    /**
     * For a region-relative x, y coordinate, return the linear
     *  scan position (top left to bottom right) for that
     *  coordinate in fovea-relative units.
     * @x horizontal coordinate in the region-relative space
     * @y vertical coordinate in the region-relative space
     * @return linear position of the pixel in the fovea-space
     */
    inline int getLinearPosFromXYFovea_(int x, int y) const {
        return(((x*density_to_raw_)/raw_to_fovea_density_ +
                bounding_box_fovea_.a.x()) +
               ((y*density_to_raw_)/raw_to_fovea_density_ +
                bounding_box_fovea_.a.y()) * this_fovea_->getBBox().width());
    }

    bool is_top_camera_;

    Fovea* this_fovea_;

    // Number of columns this region covers in terms of raw-space
    int n_raw_cols_in_region_;
    int n_raw_rows_in_region_;

    // Density of region-space with respect to raw-space
    int density_to_raw_;

    // (x,y) offsets of the top-left point of this region-space with respect to raw-space
    int y_offset_raw_;
    int x_offset_raw_;

    // Other information needed for class to operate
    int raw_total_width_;
    int raw_to_fovea_density_;

    // The width of the fovea. Needed for serialisation to not require the fovea.
    // cleaner solution would be to serialise the whole fovea, compressed
    int fovea_width_;

    // For checking if the colour array has been correctly generated for the region
    bool colour_array_set_;

    BBox bounding_box_rel_;
    BBox bounding_box_fovea_;
    BBox bounding_box_raw_;

};

#endif
