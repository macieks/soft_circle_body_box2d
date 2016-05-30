/*
 *	Soft circle body for Box2D by Maciej Sawitus, 2011
 *	Feel free to use for any projects (commercial or non-commercial) without letting me know.
 *	I do not guarantee that this code works.
 */
#include "b2ExSoftCircleBody.h"

struct b2ExSoftCircleBody
{
  int m_numParts;
  b2Body** m_parts;
  b2Body* m_center;
  b2Joint** m_partJoints;
  b2Joint** m_centerJoints;
  b2World* m_world;
};

b2ExSoftCircleBody* b2ExSoftCircleBody_Create(b2World* world, const b2ExSoftCircleBodyDef* def)
{
  b2Assert(0.0f <= def->softness && def->softness <= 1.0f);

  // Allocate memory

  char* memory = (char*) b2Alloc(sizeof(b2ExSoftCircleBody) + (sizeof(b2Body*) + sizeof(b2Joint*) * 2) * def->numParts);
  if (!memory)
    return NULL;

  b2ExSoftCircleBody* circle = (b2ExSoftCircleBody*) memory;
  memory += sizeof(b2ExSoftCircleBody);
  circle->m_world = world;
  circle->m_numParts = def->numParts;
  circle->m_parts = (b2Body**) memory;
  memory += sizeof(b2Body*) * def->numParts;
  circle->m_partJoints = (b2Joint**) memory;
  memory += sizeof(b2Joint*) * def->numParts;
  circle->m_centerJoints = (b2Joint**) memory;

  // Determine part radius

  const float angleStep = (3.14159265358979323846f * 2.0f) / def->numParts;
  const float sinHalfAngle = sinf(angleStep * 0.5f);
  const float subCircleRadius = sinHalfAngle * def->radius / (1.0f + sinHalfAngle);

  // Create parts

  b2CircleDef sd;
  sd.radius = subCircleRadius;
  sd.density = def->density;
  sd.friction = def->friction;

  float angle = 0;
  for (int i = 0; i < def->numParts; i++)
  {
    b2Vec2 offset(sinf(angle), cosf(angle));
    offset *= def->radius - subCircleRadius;

    b2BodyDef bd;
    bd.position = def->center + offset;
    b2Body* body = world->CreateBody(&bd);

    body->CreateShape(&sd);
    body->SetMassFromShapes();

    circle->m_parts[i] = body;

    angle += angleStep;
  }

  // Create center circle

  b2BodyDef bd;
  bd.position = def->center;
  circle->m_center = world->CreateBody(&bd);

  sd.radius = (def->radius - subCircleRadius * 2.0f) * (1.0f - def->softness);
  circle->m_center->CreateShape(&sd);
  circle->m_center->SetMassFromShapes();

  // Create links between circles

  b2DistanceJointDef jointDef;
  for (int i = 0; i < def->numParts; i++)
  {
    const int neighborIndex = (i + 1) % def->numParts;

    // 1st link between neighbors

    jointDef.Initialize(
      circle->m_parts[i], circle->m_parts[neighborIndex],
      circle->m_parts[i]->GetWorldCenter(), circle->m_parts[neighborIndex]->GetWorldCenter());
    jointDef.collideConnected = true;
    jointDef.frequencyHz = def->jointFrequencyHz;
    jointDef.dampingRatio = def->jointDampingRatio;

    circle->m_partJoints[i] = world->CreateJoint(&jointDef);

    // 2nd link with center body

    jointDef.Initialize(circle->m_parts[i], circle->m_center, circle->m_parts[i]->GetWorldCenter(), def->center);
    jointDef.collideConnected = true;
    jointDef.frequencyHz = def->jointFrequencyHz;
    jointDef.dampingRatio = def->jointDampingRatio;

    circle->m_centerJoints[i] = world->CreateJoint(&jointDef);
  }

  return circle;
}

void b2ExSoftCircleBody_Destroy(b2ExSoftCircleBody* circle)
{
  for (int i = 0; i < circle->m_numParts; i++)
  {
    circle->m_world->DestroyJoint(circle->m_partJoints[i]);
    circle->m_world->DestroyJoint(circle->m_centerJoints[i]);
  }
  for (int i = 0; i < circle->m_numParts; i++)
    circle->m_world->DestroyBody(circle->m_parts[i]);
  circle->m_world->DestroyBody(circle->m_center);
  b2Free(circle);
}

int b2ExSoftCircleBody_GetNumParts(b2ExSoftCircleBody* circle)
{
  return circle->m_numParts;
}

b2Body* b2ExSoftCircleBody_GetPart(b2ExSoftCircleBody* circle, int index)
{
  b2Assert(0 <= index && index < circle->m_numParts);
  return circle->m_parts[index];
}

b2Body* b2ExSoftCircleBody_GetCenter(b2ExSoftCircleBody* circle)
{
  return circle->m_center;
}