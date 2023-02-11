/**
 * A raw iterator. This iterator is used to get the raw yuv422 pixel values from the image
 * read from the camera.
 */
class iterator_raw : public iterator {
public:

    iterator_raw(const RegionI* const region, int pos, int x_offset, int y_offset, int density,
             int x_total_width, int x_region_width) :
        iterator(region, pos, x_offset*2, y_offset, density*2, density,
             x_total_width*2, x_region_width*2) {}

    /**
     * Get the raw pixel values at this pixel iteration point
     * @return a pointer to the raw YUV value
     */
    inline const
    #ifdef REGION_TEST
    int
    #else
    uint8_t*
    #endif
    raw() { return getRegion_()->getPixelRaw_(getLinearPos_()); }

    /**
     * Get the raw pixel Y,U and V values at this pixel iteration point
     * @return the Y, U or V value
     */
    #ifdef REGION_TEST
    inline int
    #else
    inline uint8_t
    #endif
    getY() {return(*raw());}

    #ifdef REGION_TEST
    inline int
    #else
    inline uint8_t
    #endif
    getU() {return(*(raw()+(1+2*(getLinearPos_() & 1)))); }

    #ifdef REGION_TEST
    inline int
    #else
    inline uint8_t
    #endif
    getV() {return(*(raw()+(1+2*(getLinearPos_() & 0)))); }

};
