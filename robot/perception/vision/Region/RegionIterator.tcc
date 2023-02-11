/**
 * The base from which other region iterators inherit. Should not be used directly.
 */
class iterator {
public:
    // Boilerplate typedefs for iterator
    typedef iterator self_type;
    typedef std::forward_iterator_tag iterator_category;
    typedef int different_type;

    /**
     * Constructor
     * @x raw-relative x coordinate
     * @y raw-relative y coordinate
     * @region region that we are iterating over
     */
    iterator(const RegionI* const region, int pos, int x_offset,
             int y_offset, int x_density, int y_density, int x_total_width,
                               int x_region_width)
       : region_(region),
       pos_(pos),
       x_offset_(x_offset),
       y_offset_(y_offset),
       x_density_(x_density),
       y_density_(y_density),
       x_total_width_(x_total_width),
       x_region_width_(x_region_width),
       x_region_width_rel_(x_region_width / x_density),
       row_begin_(x_offset_ + y_offset*x_total_width_ - x_density),
       row_end_(x_offset_ + y_offset*x_total_width_ + x_region_width_),
       n_raw_steps_to_jump_(x_total_width * y_density) {}

    /**
     * Prefix overload for ++
     * @return a reference to *this
     */
    inline self_type operator++() {
        self_type i = *this;
        iterate_();
        return i;
    }

    /**
     * Postfix overload for ++
     * @return a copy of *this prior to increment
     */
    inline self_type& operator++(int) {
        iterate_();
        return *this;
    }

    /**
     * Prefix overload for --
     * @return a reference to *this
     */
    inline self_type operator--() {
        self_type i = *this;
        iterate_back_();
        return i;
    }

    /**
     * Postfix overload for --
     * @return a copy of *this prior to increment
     */
    inline self_type& operator--(int) {
        iterate_back_();
        return *this;
    }

    /**
     * Jumps this iterator down by exactly one row.
     */
    inline self_type& next_row()
    {
        row_begin_ += n_raw_steps_to_jump_;
        row_end_ += n_raw_steps_to_jump_;
        pos_ += n_raw_steps_to_jump_;
        return *this;
    }

    /**
     * Jumps this iterator down by exactly one row.
     */
    inline self_type& last_row()
    {
        row_begin_ -= n_raw_steps_to_jump_;
        row_end_ -= n_raw_steps_to_jump_;
        pos_ -= n_raw_steps_to_jump_;
        return *this;
    }

    /**
     * Get the region-relative x value that this iterator is up to
     * @return region-relative x value that this iterator is up to
     */
    inline int x() const { return (xAbs() - x_offset_) / x_density_; }

    /**
     * Get the region-relative y value that this iterator is up to
     * @return region-relative y value that this iterator is up to
     */
    inline int y() const { return (yAbs() - y_offset_) / y_density_; }

    /**
     * Get the image-relative x value that this iterator is up to
     *  this applies to the full resolution - raw image.
     * @return image-relative x value that this iteraotr is up to
     */
    inline int xAbs() const { return pos_ % x_total_width_; }

    /**
     * Get the image-relative y value that this iterator is up to
     *  this applies to the full resolution - raw image.
     * @return image-relative y value that this iteraotr is up to
     */
    inline int yAbs() const { return (int)(pos_ / x_total_width_); }

    /**
     * Operator==
     */
    inline bool operator==(const self_type& rhs) const { return pos_ == rhs.pos_; }

    /**
     * Operator!=
     */
    inline bool operator!=(const self_type& rhs) const { return !operator==(rhs); }

    /**
     * Operator<
     */
    inline bool operator<(const self_type& rhs) const {
        return pos_ < rhs.pos_;
    }

protected:

    inline int getLinearPos_() const {
        return pos_;
    }

    inline int getLeftPos_() const {
        return pos_-x_density_;
    }

    inline int getAbovePos_() const {
        return pos_-x_total_width_*x_density_*y_density_;
    }

    inline int getBelowPos_() const {
        return pos_+x_total_width_*x_density_*y_density_;
    }

    inline int getRightPos_() const {
        return pos_+x_density_;
    }


    inline const RegionI* getRegion_() const {
        return region_;
    }

private:

    inline void iterate_() {
        pos_ += x_density_;
        if (pos_ == row_end_) {
            row_begin_ += n_raw_steps_to_jump_;
            row_end_ += n_raw_steps_to_jump_;
            pos_ = row_end_ - x_region_width_;
        }
    }

    inline void iterate_back_() {
        pos_ -= x_density_;
        if (pos_ == row_begin_) {
            row_begin_ -= n_raw_steps_to_jump_;
            row_end_ -= n_raw_steps_to_jump_;
            pos_ = row_end_ - x_density_;
        }
    }

    // The linear position that our iterator is up to, the x/y densities,
    //  as well as the total width of columns, the number of columns within
    //  the region box, and the x, y offset from the top-left corner.
    const RegionI* const region_;
    int pos_;
    int x_offset_;
    int y_offset_;
    int x_density_;
    int y_density_;
    int x_total_width_;
    int x_region_width_;
    int x_region_width_rel_;

    // For region iteration
    int row_begin_;
    int row_end_;
    int n_raw_steps_to_jump_;
};
