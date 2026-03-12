/*-------------------------------- FILE INFO ---------------------------------*/
/* Filename           : interactions.hpp                                      */
/*                                                                            */
/* Interface to interactions between geometrical classes                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#ifndef INTERACTIONS_HPP_
#define INTERACTIONS_HPP_

/*----------------------------------------------------------------------------*/
/*                             Public Definitions                             */
/*----------------------------------------------------------------------------*/
/* none */

/*----------------------------------------------------------------------------*/
/*                             Public Declarations                            */
/*----------------------------------------------------------------------------*/
namespace geometry
{

std::optional<double> ray_hitbox_distance(const Ray& ray,
    const RectangularHitbox& hitbox);

} /* geometry namespace */


#endif /* INTERACTIONS_HPP_ */
