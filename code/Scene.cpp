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
	body.m_position = Vec3( 1, 0, 10 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.5f;
	body.m_shape = new ShapeSphere( 1.0f );
	body.m_friction = 0.5f;
	m_bodies.push_back( body );

	body.m_position = Vec3(0, 1, 15);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.5f;
	body.m_shape = new ShapeSphere(1.0f);
	body.m_friction = 0.5f;
	m_bodies.push_back(body);

	body.m_position = Vec3( 0, 0, -1000 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_invMass = 0.0f;
	body.m_elasticity = 1.0f;
	body.m_shape = new ShapeSphere( 1000.0f );
	body.m_friction = 0.5f;
	m_bodies.push_back( body );
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

	// check for collisions
	for (int i = 0; i < m_bodies.size(); i++)
	{
		for (int j = i + 1; j < m_bodies.size(); j++)
		{
			Body* bodyA = &m_bodies[i];
			Body* bodyB = &m_bodies[j];

			if (bodyA->m_invMass == 0.0f && bodyB->m_invMass == 0.0f)
			{
				continue;
			}

			contact_t contact;
			if (Intersect(bodyA, bodyB, dt_sec, contact))
			{
				ResolveContact(contact);
			}
		}
	}

	// position update
	for (int i = 0; i < m_bodies.size(); i++)
	{
		m_bodies[i].Update(dt_sec);
	}
}