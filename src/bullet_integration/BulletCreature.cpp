#include "bullet/BulletCreature.h"

using namespace kinematics;
using namespace kinematics::bullet;


namespace
{
	btRigidBody* localCreateRigidBody (btDynamicsWorld* m_ownerWorld, btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
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

	void BuildOneJoint(const btTransform& currentTransform, const btVector3& origin, const joint_def_t& joint, btDynamicsWorld* m_ownerWorld, btCollisionShape** m_shapes, btRigidBody** m_bodies, btTypedConstraint** m_joints, unsigned int& id, int parentId)
	{
		const btVector3 vUp(0,1,0);
		const btVector3 vForward(1,0,0);
		const btVector3 vBack(0,0,1);

		// TODO parametrize capsule radius
		btVector3 offSetVector(btScalar(joint.offset[0]), btScalar(joint.offset[1]),  btScalar(joint.offset[2]));
		btVector3 parentOffset;
		btScalar radius = offSetVector.length() * btScalar(0.1);
		btScalar length = offSetVector.length() - 2 * radius;
		if(parentId == 0)
		{
			btVector3 rootOffSet(offSetVector); rootOffSet.normalize();
			parentOffset = rootOffSet * btScalar(0.05);
			offSetVector += parentOffset;
		}
		else
		{
			parentOffset = btVector3(btScalar(joint.parent->offset[0]), btScalar(joint.parent->offset[1]),  btScalar(joint.parent->offset[2]));
		}
		// Setup the geometry
		//The total height of a btCapsuleShape is height+2*radius
		m_shapes[id] = new btCapsuleShapeX(radius, length - btScalar(0.1));
		
		// Setup the rigid body
		btTransform offset;
		offset.setIdentity();
  		offset.setOrigin(origin + offSetVector / 2);
		
		btVector3 vToBone(offSetVector);
		vToBone.normalize();
		btVector3 vAxis = vToBone.cross(vForward);
		btScalar dot = vToBone.dot(vForward);
		btScalar angle = acos(vToBone.dot(vForward));
		if(vAxis.z() < 0)
		{
			//angle = - angle;
		}
		if(angle != 0)
		{
			/*if(angle == acos(btScalar(-1)))
			{
				offset.setRotation(btQuaternion(vUp, angle));
			}
			else
			{*/
				offset.setRotation(btQuaternion(vBack, angle));
			//}
		}

		m_bodies[id] = localCreateRigidBody(m_ownerWorld, btScalar(1.), offset, m_shapes[id]);
		
		// Now setup the constraints
		switch(joint.constraintType)
		{
			case kinematics::cone:
			{
				btConeTwistConstraint* coneC;
				btTransform localA; localA.setIdentity();
				localA.setOrigin(btVector3(btScalar(0.), parentOffset.length() / 2, btScalar(0.)));
				btTransform localB; localB.setIdentity();
				localB.setOrigin(btVector3(btScalar(0.), - offSetVector.length() / 2, btScalar(0.)));
				coneC =  new btConeTwistConstraint(*m_bodies[parentId], *m_bodies[id], localA, localB);
				m_joints[id] = coneC;
				coneC->setDbgDrawSize(0.5f);
				coneC->setLimit(btScalar(joint.maxAngleValues[0] * DegreesToRadians), btScalar(joint.maxAngleValues[1])  * DegreesToRadians, joint.maxAngleValues[2] * DegreesToRadians);
				m_ownerWorld->addConstraint(m_joints[id], true);
				break;
			}
			case kinematics::constrainedPoint:
			{
				btGeneric6DofConstraint* constraint;

				btTransform localA; localA.setIdentity();
				localA.setOrigin(btVector3(parentOffset.length() / 2, btScalar(0.), btScalar(0.)));

				btTransform localB; localB.setIdentity();
				//localB.setRotation(( offset.getRotation() * currentTransform.getRotation()));
				localB.setOrigin(btVector3(-length / 2, btScalar(0.), btScalar(0.)));

				constraint =  new btGeneric6DofConstraint(*m_bodies[parentId], *m_bodies[id], localA, localB, false);
				m_joints[id] = constraint;
				constraint->setDbgDrawSize(0.5f);
				btVector3 limitsDown(btScalar(joint.minAngleValues[0] * DegreesToRadians), btScalar(joint.minAngleValues[1])  * DegreesToRadians, joint.minAngleValues[2] * DegreesToRadians);
				btVector3 limitsUp(btScalar(joint.maxAngleValues[0] * DegreesToRadians), btScalar(joint.maxAngleValues[1])  * DegreesToRadians, joint.maxAngleValues[2] * DegreesToRadians);
				constraint->setAngularLowerLimit(limitsDown);
				constraint->setAngularUpperLimit(limitsUp);

				m_ownerWorld->addConstraint(m_joints[id], true);
				break;
			}
			//localA.setRotation(offset.getRotation() * currentTransform.getRotation());
				//localA.setRotation((offset.getRotation()).inverse());
			/*
				coneC->setLinearLowerLimit(btVector3(0,0,0));
				coneC->setLinearUpperLimit(btVector3(0,0,0));*/
			default:
			{
				btHingeConstraint* hingeC;
				btTransform localA; localA.setIdentity();
				localA.setOrigin(btVector3(btScalar(0.), parentOffset.length() / 2, btScalar(0.)));
				btTransform localB; localB.setIdentity();
				localB.setOrigin(btVector3(btScalar(0.), - offSetVector.length() / 2, btScalar(0.)));
				hingeC =  new btHingeConstraint(*m_bodies[parentId], *m_bodies[id], localA, localB, true);
				btVector3 limitsDown(btScalar(joint.minAngleValues[0] * DegreesToRadians), btScalar(joint.minAngleValues[1])  * DegreesToRadians, joint.minAngleValues[2] * DegreesToRadians + Pi);
				btVector3 limitsUp(btScalar(joint.maxAngleValues[0] * DegreesToRadians), btScalar(joint.maxAngleValues[1])  * DegreesToRadians, joint.maxAngleValues[2] * DegreesToRadians + Pi);
					
				hingeC->setLimit(limitsDown[0], limitsUp[0]);
				m_joints[id] = hingeC;
				hingeC->setDbgDrawSize(0.5f);

				m_ownerWorld->addConstraint(m_joints[id], true);
			}
		}

		btVector3 newOrigin(origin + offSetVector);

		btTransform newTransform; newTransform.setIdentity();
		newTransform.setRotation(currentTransform.getRotation() * offset.getRotation());
		newTransform.setOrigin(newOrigin);
		int newParentId = id;
		for(unsigned int i=0; i< joint.nbChildren_; ++i)
		{
  			BuildOneJoint(newTransform, newOrigin, *(joint.children[i]), m_ownerWorld, m_shapes, m_bodies, m_joints, ++id, newParentId);
		}
	}
	
	void BuildOneJoint2(const btTransform& currentTransform, const btVector3& origin, const joint_def_t& joint, btDynamicsWorld* m_ownerWorld, btCollisionShape** m_shapes, btRigidBody** m_bodies, btTypedConstraint** m_joints, unsigned int& id, int parentId)
	{
		const btVector3 vUp(0,1,0);
		const btVector3 vForward(1,0,0);
		const btVector3 vBack(0,0,1);

		// retrieving joint offset from parent
		btVector3 offSetVector(btScalar(joint.offset[0]), btScalar(joint.offset[1]),  btScalar(joint.offset[2]));
		btVector3 parentOffset;

		//computing radius and length of capsule; radius is more or less equal to offset
		btScalar radius = offSetVector.length() * btScalar(0.1); 
		btScalar length = offSetVector.length() - 2 * radius;
		if(parentId == 0)
		{
			// if current joint is root, for now juste create a small capsule representing root
			btVector3 rootOffSet(offSetVector); rootOffSet.normalize();
			parentOffset = rootOffSet * btScalar(0.05);
			offSetVector += parentOffset;
		}
		else
		{
			parentOffset = btVector3(btScalar(joint.parent->offset[0]), btScalar(joint.parent->offset[1]),  btScalar(joint.parent->offset[2]));
		}
		// Setup the geometry
		//The total height of a btCapsuleShape is height+2*radius
		m_shapes[id] = new btCapsuleShapeX(radius, length - btScalar(0.1));
		
		// Setup the rigid body
		btTransform offset;
		offset.setIdentity();
  		offset.setOrigin(origin + offSetVector / 2);
		
		// computing rotation necessary for capsule
		btVector3 vToBone(offSetVector);
		vToBone.normalize();
		btVector3 vAxis = vToBone.cross(vForward);
		btScalar dot = vToBone.dot(vForward);
		btScalar angle = acos(vToBone.dot(vForward));
		if(angle != 0)
		{
			offset.setRotation(btQuaternion(vBack, angle));
		}

		m_bodies[id] = localCreateRigidBody(m_ownerWorld, btScalar(1.), offset, m_shapes[id]);
		
		btVector3 newOrigin(origin + offSetVector);

		btTransform newTransform; newTransform.setIdentity();
		newTransform.setRotation(currentTransform.getRotation() * offset.getRotation());
		newTransform.setOrigin(newOrigin);
		int newParentId = id;
		// method is recursive, call it with children
		for(unsigned int i=0; i< joint.nbChildren_; ++i)
		{
  			BuildOneJoint2(newTransform, newOrigin, *(joint.children[i]), m_ownerWorld, m_shapes, m_bodies, m_joints, ++id, newParentId);
		}
	}

	void BuildOneJoint3(const btTransform& currentTransform, const btVector3& origin, const joint_def_t& joint, btDynamicsWorld* m_ownerWorld, btCollisionShape** m_shapes, btRigidBody** m_bodies, btTypedConstraint** m_joints, unsigned int& id, int parentId)
	{
		const btVector3 vUp(0,1,0);
		const btVector3 vForward(1,0,0);
		const btVector3 vBack(0,0,1);

		//
		btVector3 offSetVector(btScalar(joint.offset[0]), btScalar(joint.offset[1]),  btScalar(joint.offset[2]));
		btVector3 parentOffset;
		btScalar radius = offSetVector.length() * btScalar(0.1);
		btScalar length = offSetVector.length() - 2 * radius;
		if(parentId == 0)
		{
			btVector3 rootOffSet(offSetVector); rootOffSet.normalize();
			parentOffset = rootOffSet * btScalar(0.05);
			offSetVector += parentOffset;
		}
		else
		{
			parentOffset = btVector3(btScalar(joint.parent->offset[0]), btScalar(joint.parent->offset[1]),  btScalar(joint.parent->offset[2]));
		}
		// Setup the geometry
		//The total height of a btCapsuleShape is height+2*radius
		m_shapes[id] = new btCapsuleShapeX(radius, length - btScalar(0.1));
		
		// Setup the rigid body
		btTransform offset;
		offset.setIdentity();
  		offset.setOrigin(origin + offSetVector / 2);
		
		btVector3 vToBone(offSetVector);
		vToBone.normalize();
		btVector3 vAxis = vToBone.cross(vForward);
		btScalar dot = vToBone.dot(vForward);
		btScalar angle = acos(vToBone.dot(vForward));
		if(angle != 0)
		{
			offset.setRotation(btQuaternion(vBack, angle));
		}

		m_bodies[id] = localCreateRigidBody(m_ownerWorld, btScalar(1.), offset, m_shapes[id]);
		
		// Now setup the constraints
		btGeneric6DofConstraint* constraint;

		btTransform localA; localA.setIdentity();
		localA.setOrigin(btVector3(parentOffset.length() / 2, btScalar(0.), btScalar(0.)));

		btTransform localB; localB.setIdentity();
		localB.setRotation(( offset.getRotation() * currentTransform.getRotation()));
		localB.setOrigin(btVector3(-length / 2, btScalar(0.), btScalar(0.)));

		constraint =  new btGeneric6DofConstraint(*m_bodies[parentId], *m_bodies[id], localA, localB, false);
		m_joints[id] = constraint;
		constraint->setDbgDrawSize(0.5f);
		btVector3 limitsDown(btScalar(joint.minAngleValues[0] * DegreesToRadians), btScalar(joint.minAngleValues[1])  * DegreesToRadians, joint.minAngleValues[2] * DegreesToRadians);
		btVector3 limitsUp(btScalar(joint.maxAngleValues[0] * DegreesToRadians), btScalar(joint.maxAngleValues[1])  * DegreesToRadians, joint.maxAngleValues[2] * DegreesToRadians);
		constraint->setAngularLowerLimit(limitsDown);
		constraint->setAngularUpperLimit(limitsUp);
		m_ownerWorld->addConstraint(m_joints[id], false); 


		btVector3 newOrigin(origin + offSetVector);

		btTransform newTransform; newTransform.setIdentity();
		newTransform.setRotation(currentTransform.getRotation() * offset.getRotation());
		newTransform.setOrigin(newOrigin);
		int newParentId = id;
		for(unsigned int i=0; i< joint.nbChildren_; ++i)
		{
  			BuildOneJoint3(newTransform, newOrigin, *(joint.children[i]), m_ownerWorld, m_shapes, m_bodies, m_joints, ++id, newParentId);
		}
	}

	
	void BuildRootJoint(const btVector3& origin, const joint_def_t& joint, btDynamicsWorld* m_ownerWorld, btCollisionShape** m_shapes, btRigidBody** m_bodies, btTypedConstraint** m_joints)
	{
		btVector3 newOrigin(origin[0] + joint.offset[0], origin[1] + joint.offset[1], origin[2] + joint.offset[2]);
		// creating a small shape on which sons will be attached.
		unsigned int id = 0;
		btTransform offset;
		offset.setIdentity();
		offset.setOrigin(newOrigin);
		m_shapes[0] = new btSphereShape(btScalar(0.1));
		m_bodies[0] = localCreateRigidBody(m_ownerWorld, btScalar(0), offset, m_shapes[0]);
		for(unsigned int i=0; i< joint.nbChildren_; ++i)
		{
  			BuildOneJoint(offset, newOrigin, *(joint.children[i]), m_ownerWorld, m_shapes, m_bodies, m_joints, ++id, 0);
		}
	}
}

BulletCreature::BulletCreature(const joint_def_t& jointDef, btDynamicsWorld* m_ownerWorld)
	: nbBodies_(jointDef.count())
	, m_ownerWorld_(m_ownerWorld)
{
		btVector3 origin(0,1,-2);
		BuildRootJoint(origin, jointDef, m_ownerWorld_, m_shapes_, m_bodies_, m_joints_);
}

BulletCreature::~BulletCreature()
{
	int i;
	// Remove all constraints
	for ( i = 0; i < nbBodies_; ++i)
	{
		m_ownerWorld_->removeConstraint(m_joints_[i]);
		delete m_joints_[i]; m_joints_[i] = 0;
	}

	// Remove all bodies and shapes
	for ( i = 0; i < nbBodies_; ++i)
	{
		m_ownerWorld_->removeRigidBody(m_bodies_[i]);
			
		delete m_bodies_[i]->getMotionState();
		delete m_bodies_[i]; m_bodies_[i] = 0;
		delete m_shapes_[i]; m_shapes_[i] = 0;
	}
}


