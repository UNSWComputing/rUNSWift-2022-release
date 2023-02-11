#ifndef RR_COORD_HPP
#define RR_COORD_HPP

#include <math.h>
#include <string.h>
#include <Eigen/Eigen>
#include <boost/serialization/version.hpp>
#include "types/Point.hpp"

struct RRCoord {
    /**
     * RRCoord
     * (Robot Relative Coord)
     *
     * @param heading       heading from robot to object
     * @param distance      distance from robot to object
     * @param orientation   angle between robot front and object front
     */
    RRCoord(float distance, float heading = 0, float orientation = 0)
        : vec(distance, heading, orientation)
    {
        var.setZero();
    }

    Point toCartesian() const
    {
        return Point(distance()*cosf(heading()), distance()*sinf(heading()));
    }

    RRCoord()
    {
        vec.setZero();
        var.setZero();
    }

    RRCoord(const RRCoord &other)
    {
        this->vec = other.vec;
        this->var = other.var;
    }

    RRCoord& operator=(const RRCoord &other)
    {
        this->vec = other.vec;
        this->var = other.var;

        return *this;
    }

    Eigen::Vector3f vec;
    Eigen::Matrix<float, 3, 3> var;

    /* Distance to object */
    float distance() const
    {
        return vec[0];
    }

    float setDistance(float distance)
    {
        return vec[0] = distance;
    }

    /* Distance between coords, squared */
    float distanceSquared(const RRCoord& other) const
    {
        // The magnitudes of the vectors.
        const float mag0 = vec[0];
        const float mag1 = other.vec[0];

        // The angles of the vectors.
        const float angle0 = vec[1];
        const float angle1 = other.vec[1];

        // The squares of the magnitudes of the vectors.
        const float square0 = mag0 * mag0;
        const float square1 = mag1 * mag1;

        // Return the squared distance between the vectors.
        return(square0 + square1 - 2.0 * mag0 * mag1 * cos(angle0 - angle1));
    }

    /* Distance between coords */
    float distance(const RRCoord& other) const
    {
        return(sqrt(distanceSquared(other)));
    }

    /* Heading to object */
    float heading() const
    {
        return vec[1];
    }

    float setHeading(float heading)
    {
        return vec[1] = heading;
    }

    /* Angle between robot's front and object's front */
    float orientation() const
    {
        return vec[2];
    }

    float setOrientation(float orientation)
    {
        return vec[2] = orientation;
    }

    bool operator== (const RRCoord &other) const
    {
        return vec == other.vec;
    }

    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version);
};

inline std::ostream& operator<<(std::ostream& os, const RRCoord& coord)
{
    os << "(" << coord.distance() << ", " << coord.heading() << ", " << coord.orientation() << ")";
    return os;
}

typedef enum
{
    DIST,
    HEADING,
    ORIENTATION,
} RRCoordEnum;

#include "types/RRCoord.tcc"

#endif // RR_COORD_HPP
