//
//  Intersections.cpp
//
#include "Intersections.h"
#include "GJK.h"

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, contact_t & contact ) 
{
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	const Vec3 ab = bodyB->m_position - bodyA->m_position;
	contact.normal = ab;
	contact.normal.Normalize();

	const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
	const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

	// intersection of the AB vector with the sphere surfaces...
	contact.ptOnA_WorldSpace = bodyA->m_position + contact.normal * sphereA->m_radius;
	contact.ptOnB_WorldSpace = bodyB->m_position - contact.normal * sphereB->m_radius;

	const float radiusAB = sphereA->m_radius + sphereB->m_radius;
	const float lengthSquare = ab.GetLengthSqr();
	if (lengthSquare <= (radiusAB * radiusAB))
	{
		return true;
	}

	return false;
}

/*
====================================================
Intersect
====================================================
*/
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact ) {
	// TODO: Add Code

	return false;
}






















