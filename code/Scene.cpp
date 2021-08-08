//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) 
	{
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) 
	{
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() 
{
	Body body;

	// dynamic bodies
	for (int x = 0; x < 6; x++)
	{
		for (int y = 0; y < 6; y++)
		{
			float radius = 0.5f;
			float xx = float(x - 1) * radius * 2.5f;
			float yy = float(y - 1) * radius * 2.5f;
			body.m_position = Vec3(xx, yy, 10.0f);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_invMass = 1.0f;
			body.m_elasticity = 0.5f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}

	// floor
	for (int x = 0; x < 3; x++)
	{
		for (int y = 0; y < 3; y++)
		{
			float radius = 80.0f;
			float xx = float(x - 1) * radius * 0.25f;
			float yy = float(y - 1) * radius * 0.25f;
			body.m_position = Vec3(xx, yy, -radius);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_invMass = 0.0f;
			body.m_elasticity = 0.99f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			m_bodies.push_back(body);
		}
	}
}

// TODO: Need to put this somewhere sensible...

int CompareContacts(const void* p1, const void* p2)
{
	// return value... (used in qsort)
	// -1 = p1 before p2
	// 0 = p1 and p2 same time
	// 1 = p1 after p2

	contact_t a = *(contact_t*)p1;
	contact_t b = *(contact_t*)p2;

	if (a.timeOfImpact < b.timeOfImpact)
	{
		return -1;
	}

	if (a.timeOfImpact == b.timeOfImpact)
	{
		return 0;
	}

	return 1;
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) 
{
	// Apply gravity
	for (int i = 0; i < m_bodies.size(); i++)
	{
		Body* body = &m_bodies[i];

		float mass = 1.0f / body->m_invMass;
		Vec3 impulseGravity = Vec3(0.0f, 0.0f, -10.0f) * mass * dt_sec;
		body->ApplyImpulseLinear(impulseGravity);
	}

	// Broadphase
	std::vector<collisionPair_t> collisionPairs;
	BroadPhase(m_bodies.data(), (int)m_bodies.size(), collisionPairs, dt_sec);

	// NarrowPhase
	int numContacts = 0;
	const int maxContacts = (int)(m_bodies.size() * m_bodies.size());

	contact_t* contacts = (contact_t*)alloca(sizeof(contact_t) * maxContacts);	// TODO: This needs freeing
	for (int i = 0; i < collisionPairs.size(); i++)
	{
		const collisionPair_t& pair = collisionPairs[i];
		Body* bodyA = &m_bodies[pair.a];
		Body* bodyB = &m_bodies[pair.b];

		if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
		{
			continue;
		}

		contact_t contact;
		if (Intersect(bodyA, bodyB, dt_sec, contact))
		{
			contacts[numContacts] = contact;
			numContacts++;
		}
	}

	if (numContacts > 1)
	{
		qsort(contacts, numContacts, sizeof(contact_t), CompareContacts);
	}

	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; i++)
	{
		contact_t& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;

		for (int j = 0; j < m_bodies.size(); j++)
		{
			m_bodies[j].Update(dt);
		}

		ResolveContact(contact);
		accumulatedTime += dt;
	}

	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < m_bodies.size(); i++)
		{
			m_bodies[i].Update(timeRemaining);
		}
	}

#if 0
	int numContacts = 0;
	const int maxContacts = (int)(m_bodies.size() * m_bodies.size());

	contact_t* contacts = (contact_t*)alloca(sizeof(contact_t) * maxContacts);	// TODO: This needs freeing
	for (int i = 0; i < m_bodies.size(); i++)
	{
		for (int j = i+1; j < m_bodies.size(); j++)
		{
			Body* bodyA = &m_bodies[i];
			Body* bodyB = &m_bodies[j];

			if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
			{
				continue;
			}

			contact_t contact;
			if (Intersect(bodyA, bodyB, dt_sec, contact))
			{
				contacts[numContacts] = contact;
				numContacts++;
			}
		}
	}

	if (numContacts > 1)
	{
		qsort(contacts, numContacts, sizeof(contact_t), CompareContacts);
	}

	float accumulatedTime = 0.0f;
	for (int i = 0; i < numContacts; i++)
	{
		contact_t& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;

		Body* bodyA = contact.bodyA;
		Body* bodyB = contact.bodyB;

		if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
		{
			continue;
		}

		for (int j = 0; j < m_bodies.size(); j++)
		{
			m_bodies[j].Update(dt);
		}

		ResolveContact(contact);
		accumulatedTime += dt;
	}

	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < m_bodies.size(); i++)
		{
			m_bodies[i].Update(timeRemaining);
		}
	}
#endif
}