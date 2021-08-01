//
//  Contact.cpp
//
#include "Contact.h"

/*
====================================================
ResolveContact
====================================================
*/
void ResolveContact( contact_t & contact ) 
{
	Body* bodyA = contact.bodyA;
	Body* bodyB = contact.bodyB;

	const float invMassA = bodyA->m_invMass;
	const float invMassB = bodyB->m_invMass;

	const float elasticityA = bodyA->m_elasticity;
	const float elasticityB = bodyB->m_elasticity;
	const float elasticity = elasticityA * elasticityB;

	// collision impulse
	const Vec3& n = contact.normal;
	const Vec3 vab = bodyA->m_linearVelocity - bodyB->m_linearVelocity;
	const float ImpulseJ = -(1.0f + elasticity) * vab.Dot(n) / (invMassA + invMassB);
	const Vec3 vectorImpulseJ = n * ImpulseJ;

	bodyA->ApplyImpulseLinear(vectorImpulseJ * 1.0f);
	bodyB->ApplyImpulseLinear(vectorImpulseJ * -1.0f);

	// move the objects apart
	const float tA = bodyA->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);
	const float tB = bodyB->m_invMass / (bodyA->m_invMass + bodyB->m_invMass);

	const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
	bodyA->m_position += ds * tA;
	bodyB->m_position -= ds * tB;
}