/**
 * A fovea iterator. This iterator is used to get colour classified pixels.
 */
class iterator_fovea : public iterator {
public:

    iterator_fovea(const RegionI* const region, int pos, int x_offset, int y_offset, int density,
             int x_total_width, int x_region_width) :
        iterator(region, pos, x_offset, y_offset, density, density,
             x_total_width, x_region_width) {}

    /**
     * Get the colour enum at this pixel iteration point
     * @return colour enum
     */
    #ifdef REGION_TEST
    inline int
    #else
    inline Colour
    #endif
    colour() const { return getRegion_()->getPixelColour_(getLinearPos_()); }

    #ifdef REGION_TEST
    inline int
    #else
    inline Colour
    #endif
    colourAbove() const { return getRegion_()->getPixelColour_(getAbovePos_()); }

    #ifdef REGION_TEST
    inline int
    #else
    inline Colour
    #endif
    colourBelow() const { return getRegion_()->getPixelColour_(getBelowPos_()); }

    #ifdef REGION_TEST
    inline int
    #else
    inline Colour
    #endif
    colourLeft() const { return getRegion_()->getPixelColour_(getLeftPos_()); }

    #ifdef REGION_TEST
    inline int
    #else
    inline Colour
    #endif
    colourRight() const { return getRegion_()->getPixelColour_(getRightPos_()); }
};
