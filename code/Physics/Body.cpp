//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
m_position( 0.0f ),
m_orientation( 0.0f, 0.0f, 0.0f, 1.0f ),
m_shape( NULL ) 
{}

Vec3 Body::GetCenterOfMassWorldSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	const Vec3 pos = m_position + m_orientation.RotatePoint(centerOfMass);
	return pos;
}

Vec3 Body::GetCenterOfMassModelSpace() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	return centerOfMass;
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPt) const
{
	Vec3 tmp = worldPt - GetCenterOfMassWorldSpace();
	Quat inverseOrient = m_orientation.Inverse();
	Vec3 bodySpace = inverseOrient.RotatePoint(tmp);
	return bodySpace;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& worldPt) const
{
	Vec3 worldSpace = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(worldPt);
	return worldSpace;
}

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	if (m_invMass == 0.0f)
		return;

	m_linearVelocity += impulse * m_invMass;
}

void Body::ApplyImpulseAngular(const Vec3& impulse)
{
	if (m_invMass == 0.0f)
		return;

	m_angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

	const float maxAngularSpeed = 30.0f;

	if (m_angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed)
	{
		m_angularVelocity.Normalize();
		m_angularVelocity *= maxAngularSpeed;
	}
}

Mat3 Body::GetInverseInertiaTensorBodySpace() const
{
	Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	return invInertiaTensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const
{
	Mat3 inertiaTensor = m_shape->InertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	Mat3 orient = m_orientation.ToMat3();
	invInertiaTensor = orient * invInertiaTensor * orient.Transpose();
	return invInertiaTensor;
}
