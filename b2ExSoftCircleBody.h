/*
 *	Soft circle body for Box2D by Maciej Sawitus, 2011
 *	Feel free to use for any projects (commercial or non-commercial) without letting me know.
 *	I do not guarantee that this code works.
 */
#ifndef B2_EX_SOFT_CIRCLE_BODY_H
#define B2_EX_SOFT_CIRCLE_BODY_H

#include "Box2D.h"

//! Soft circle description
struct b2ExSoftCircleBodyDef
{
  b2ExSoftCircleBodyDef() :
    center(0.0f, 0.0f),
    radius(5.0f),
    numParts(6),
    softness(0.2f),
    density(1.0f),
    friction(0.3f),
    jointFrequencyHz(4.0f),
    jointDampingRatio(0.5f)
  {}

  b2Vec2 center;  //!< Soft circle center
  float radius;   //!< Soft circle radius
  int numParts;   //!< Number of parts to be created around the soft circle center; each part is actually another circle and linked using distance joint with its neighbours and one central (also circle) body
  float softness; //!< Softness value within 0..1

  // Sub-circles' properties (see b2CircleDef)

  float density;
  float friction;

  // Distance joints' properties (see b2DistanceJointDef)

  float jointFrequencyHz;
  float jointDampingRatio;
};

/**
 *  @struct b2ExSoftCircleBody
 *  @brief Soft body consisting of number of parts (each being circle placed around soft body origin) and one center circle body; all linked using distance joints;
 *    soft circle body with X number of parts internally creates X+1 bodies and X*2 joints and does one additional memory allocation
 */
struct b2ExSoftCircleBody;

//! Creates complex soft circle body in the world
b2ExSoftCircleBody* b2ExSoftCircleBody_Create(b2World* world, const b2ExSoftCircleBodyDef* def);
//! Destroy soft circle body
void b2ExSoftCircleBody_Destroy(b2ExSoftCircleBody* circle);

//! Gets number of parts of the soft circle body
int b2ExSoftCircleBody_GetNumParts(b2ExSoftCircleBody* circle);
//! Gets part of the soft circle body at given index
b2Body* b2ExSoftCircleBody_GetPart(b2ExSoftCircleBody* circle, int index);
//! Gets center body of the soft circle body
b2Body* b2ExSoftCircleBody_GetCenter(b2ExSoftCircleBody* circle);

#endif // B2_EX_SOFT_CIRCLE_BODY_H