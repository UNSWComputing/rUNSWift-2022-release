#ifndef PERCEPTION_VISION_RANSAC_H_
#define PERCEPTION_VISION_RANSAC_H_

#include <vector>

#include "types/RansacTypes.hpp"
#include "types/Point.hpp"

namespace RANSAC
{
   /**
    * Ransac generators
    */

   /**
    * Acceptors determine whether or not a point should be considered
    * part of the consensus set. They are bound to a single instance
    * of a RANSAC<Type> and update as each point is added
    *
    * Note: These types are defined as defaults. There is nothing stopping you
    *       from writing your own Generators and Acceptors
    */

   template <typename T> class Acceptor
   {
      public:
         Acceptor(T &item, float error);
         bool accept(T &item, Point p) const;
         void finalise(T &item) const;
   };

   template <> class Acceptor<RANSACLine>
   {
      public:
         Acceptor(RANSACLine &line, float error);

         bool accept(Point p);
         void finalise();
      private:
         RANSACLine &line;

         const float denom;
         const float e2;
   };

   /*
   template <> class Acceptor<RANSACCorner>
   {
      public:
         Acceptor(RANSACCorner &corner);
         bool accept  (Point p, float error);
         void finalise();
      private:
         Acceptor<RANSALLine> line[2];
   };
   */

   template <class T> class Generator
   {
      bool operator() (
            T &item,
            const std::vector<Point> &points,
            unsigned int *seed) const;
   };

   template <> class Generator<RANSACLine>
   {
   public:
      bool operator() (
            RANSACLine &item,
            const std::vector<Point> &points,
            unsigned int *seed) const;
   };

   /**
    * Implementation of the RANSAC algorithm for finding a straight line
    * amongst a noisy set of points.
    *
    * @return           Whether or not a line has been found
    *
    * @param points     The dataset to look in, vector of Points
    * @param result     The chosen line
    * @param cons       Pointer to the concensus set array
    * @param k          Maximum number of iterations of RANSAC
    * @param e          Maximum distance a point can be from a line to be in its
    *                   concensus set
    * @param n          The minimum number of points needed to form a concensus set
    * @param            seed A seed value
    **/
   bool findLine(const std::vector<Point>  &points,
                 std::vector<bool>        **cons,
                 RANSACLine                &result,
                 unsigned int               k,
                 float                      e,
                 unsigned int               n,
                 std::vector<bool>          cons_buf[2],
                 unsigned int              *seed
                );

   /**
    * As above, but includes a slope constraint to keep the slope close to 1,
    * ie. slope <= slopeConstraint and slope >= 1/slopeConstraint, a negative value
    * means there is no constraint
    **/
   bool findLineConstrained(const std::vector<Point>  &points,
                 std::vector<bool>        **cons,
                 RANSACLine                &result,
                 unsigned int               k,
                 float                      e,
                 unsigned int               n,
                 std::vector<bool>          cons_buf[2],
                 unsigned int              *seed,
                 float                      slopeConstraint
                );

   /**
    * Implementation of the RANSAC algorithm for finding a circle with a
    * known radius amongst a noisy set of points.
    *
    * @return           Whether or not a line has been found
    *
    * @param radius     Radius of circle to search for
    * @param points     The dataset to look in, vector of Points
    * @param result     The chosen circle
    * @param cons       Pointer to the concensus set array
    * @param k          Maximum number of iterations of RANSAC
    * @param e          Maximum distance a point can be from a line to be in its
    *                   concensus set
    * @param n          The minimum number of points needed to form a concensus set
    * @param            seed A seed value
    **/
   bool findCircleOfRadius(const std::vector<Point>  &points,
                           float                      radius,
                           std::vector<bool>        **cons,
                           RANSACCircle              &result,
                           unsigned int               k,
                           float                      e,
                           unsigned int               n,
                           std::vector<bool>          cons_buf[2],
                           unsigned int              *seed
                          );

      /**
       * Implementation of the RANSAC algorithm for finding a circle with a
       * known radius amongst a noisy set of points.
       *
       * @return           Whether or not a line has been found
       *
       * @param radius     Radius of circle to search for
       * @param points     The dataset to look in, vector of Points
       * @param result     The chosen circle
       * @param cons       Pointer to the concensus set array
       * @param k          Maximum number of iterations of RANSAC
       * @param e          Maximum distance a point can be from a line to be in its
       *                   concensus set
       * @param n          The minimum number of points needed to form a concensus set
       * @param            seed A seed value
       **/
      bool findLinesAndCircles(const std::vector<Point>  &points,
                                     float                      radius,
                                     std::vector<bool>        **cons,
                                     RANSACLine                 &resultLine,
                                     RANSACCircle               &resultCircle,
                                     unsigned int               k,
                                     float                      e,
                                     unsigned int               n,
                                     std::vector<bool>          cons_buf[2],
                                     unsigned int              *seed
                                    );
   /**
    * Implementation of the RANSAC algorithm for finding a circle with a
    * known radius amongst a noisy set of points.
    *
    * @return           Whether or not a line has been found
    *
    * @param radius     Radius of circle to search for
    * @param radius_e   Allowable error in radius
    * @param points     The dataset to look in, vector of Points
    * @param result     The chosen circle
    * @param cons       Pointer to the concensus set array
    * @param k          Maximum number of iterations of RANSAC
    * @param e          Maximum distance a point can be from a line to be in its
    *                   concensus set
    * @param n          The minimum number of points needed to form a concensus set
    * @param            seed A seed value
    **/
   bool findCircleOfRadius3P(const std::vector<Point>  &points,
                             float                      radius,
                             float                      radius_e,
                             std::vector<bool>        **cons,
                             RANSACCircle              &result,
                             unsigned int               k,
                             float                      e,
                             unsigned int               n,
                             std::vector<bool>          cons_buf[2],
                             unsigned int              *seed
                            );

   // Same as above but reject circles with centre outside the given bounds
   bool findCircleOfRadius3PInsideBounds(const std::vector<Point>  &points,
                             float                      radius,
                             float                      radius_e,
                             std::vector<bool>        **cons,
                             RANSACCircle              &result,
                             unsigned int               k,
                             float                      e,
                             unsigned int               n,
                             std::vector<bool>          cons_buf[2],
                             unsigned int              *seed,
                             int min_bound_x,
                             int max_bound_x,
                             int min_bound_y,
                             int max_bound_y
                            );

    // Commented out as these seems to be a ball detection algorithm from Dave's Ball Detector (2016)
   // /**
   //  * Implementation of the RANSAC algorithm for finding a circle
   //  * given a set of points. Radius is unknown, radius error 5%
   //  *
   //  * @return           Whether or not a line has been found
   //  *
   //  * @param points     The dataset to look in, vector of Points
   //  * @param result     The chosen circle
   //  * @param cons       Pointer to the concensus set array
   //  * @param k          Maximum number of iterations of RANSAC
   //  * @param e          Maximum distance a point can be from a line to be in its
   //  *                   concensus set
   //  * @param n          The minimum number of points needed to form a concensus set
   //  * @param            seed A seed value
   //  **/
   // bool RansacSoccerBall3P(VisionFrame               &frame,
   //                         const std::vector<Point>  &points,
   //                         std::vector<bool>        **cons,
   //                         RANSACCircle              &result,
   //                         unsigned int               k,
   //                         float                      e,
   //                         unsigned int               n,
   //                         std::vector<bool>          cons_buf[2],
   //                         unsigned int              *seed
   //                         );

   template <class T, class G = Generator<T>, class A = Acceptor<T> >
   class Ransac
   {
   public:
      bool operator() (
            const G                   &generator,
            const std::vector<Point>  &points,
            std::vector<bool>        **cons,
            T                         &result,
            unsigned int               k,
            float                      e,
            unsigned int               n,
            std::vector<bool>          cons_buf[2],
            unsigned int              *seed
           );
   };

};

#include "Ransac.tcc"

#endif
