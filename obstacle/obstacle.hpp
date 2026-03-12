/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : optimizer.hpp                                         */
/*                                                                            */
/* Interface to obstacle class for micromouse simulations                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
namespace obstacle
{

constexpr double OFFICIAL_POST_SIZE {12.07}; /* 12.07mm */

} /* obstacle namespace */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace obstacle
{

class Post
{
    public:
        geometry::RectangularHitbox hitbox;
        Post(const geometry::Point& center, 
            double horizontal_size_adjustment, double vertical_size_adjustment);

        void translate(double dx, double dy) noexcept;
        void rotate(const geometry::Point& center, double angle_rad) noexcept;

        bool operator==(const Post& other) const noexcept;
};

} /* obstacle namespace */

#endif /* OBSTACLE_HPP_ */
