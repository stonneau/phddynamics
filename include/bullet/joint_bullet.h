/**
* \file joint_bullet.h
* \brief functions to create Bullet entities from a kinematic chain made of joints.
* \author Steve T.
* \version 0.1
* \date 11/05/2013
*
*/


#ifndef _JOINT_BULLET
#define _JOINT_BULLET

#define _USE_MATH_DEFINES

#include "btBulletDynamicsCommon.h"
#include "kinematics/joint.h"
#include "mathDefs.h"

namespace kinematics
{
namespace bullet
{
	
	typedef kinematics::joint<btScalar, btScalar, 3, 5, false> joint_t;	// Joint model used for bullet simulation

	inline btRigidBody* localCreateRigidBody (btDynamicsWorld* m_ownerWorld, btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		btVector3 localInertia(0,0,0);
		if (mass != 0.f) // is dynamic
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, shape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);
		return body;
	}
		
	inline void BuildOneJoint(const btVector3& origin, const joint_t& joint, btDynamicsWorld* m_ownerWorld, btCollisionShape** m_shapes, btRigidBody** m_bodies, btTypedConstraint** m_joints, unsigned int& id, unsigned int parentId)
	{
		const btVector3 vUp(0,1,0);
		// TODO parametrize capsule radius
		const btScalar radius = 0.15;
		btVector3 offSetVector(btScalar(joint.offset[0]), btScalar(joint.offset[1]),  btScalar(joint.offset[2]));
		// Setup the geometry
		//The total height of a btCapsuleShape is height+2*radius
		m_shapes[id] = new btCapsuleShape(radius, offSetVector.length() - 2 * radius);
		
		// Setup the rigid body
		btTransform offset;
		offset.setIdentity();
  		offset.setOrigin(origin + offSetVector / 2);
		
		btVector3 vToBone(offSetVector);
		vToBone.normalize();
		btVector3 vAxis = vUp.cross(vToBone);
		btScalar angle = acos(vUp.dot(vToBone));
		if(angle != 0)
		offset.setRotation(btQuaternion(vAxis, angle));

		m_bodies[id] = localCreateRigidBody(m_ownerWorld, btScalar(1.), offset, m_shapes[id]);
		
		if(id > 0) 
		{
			// Now setup the constraints
			btHingeConstraint* hingeC;
			btVector3 parentOffset(btScalar(joint.parent->offset[0]), btScalar(joint.parent->offset[1]),  btScalar(joint.parent->offset[2]));

			/*btTransform localA, localB;
			
			localA.setIdentity(); localB.setIdentity();
			localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
			localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));*/
			btTransform localA; localA.setIdentity();
			localA.setOrigin(btVector3(btScalar(0.), parentOffset.length() / 2, btScalar(0.)));
			btTransform localB; localB.setIdentity();
			localB.setOrigin(btVector3(btScalar(0.), - offSetVector.length() / 2, btScalar(0.)));
			hingeC =  new btHingeConstraint(*m_bodies[id - 1], *m_bodies[id], localA, localB, true);
			hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
			m_joints[id] = hingeC;
			hingeC->setDbgDrawSize(0.5f);

			m_ownerWorld->addConstraint(m_joints[id], true);
		}

		btVector3 newOrigin(origin + offSetVector);

		for(int i=0; i< joint.nbChildren_; ++i)
		{
  			BuildOneJoint(newOrigin, *(joint.children[i]), m_ownerWorld, m_shapes, m_bodies, m_joints, ++id, id);
		}
	}
	
	///  \brief Creates a bullet entity given a joint description
	///	 If Safe is set to true, an exception will be thrown if it is not possible to create the file.
	///  \param joint the root joint of the kinematic tree to convert
	///  \return a bullet entity
	inline bool MakeBulletEntity(const joint_t& joint, btDynamicsWorld* m_ownerWorld)
	{
		btVector3 origin(0,0,0);
		btCollisionShape* m_shapes[100];
		btRigidBody* m_bodies[100];
		btTypedConstraint* m_joints[100];
		unsigned int id = 0;
		BuildOneJoint(origin, joint, m_ownerWorld, m_shapes, m_bodies, m_joints, id, -1);
		return false;
	}
} // namespace bullet
} // namespace kinematics
#endif //_JOINT_BULLET