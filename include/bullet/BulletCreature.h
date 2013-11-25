/**
* \file BulletCreature.h
* \brief representation of a kinematic chain in the bullet library.
* \author Steve T.
* \version 0.1
* \date 11/05/2013
*
*/


#ifndef _BULLET_CREATURE
#define _BULLET_CREATURE

#define _USE_MATH_DEFINES
#define MAX_CHILDREN 20

#include "btBulletDynamicsCommon.h"
#include "kinematics/joint.h"
#include "mathDefs.h"


namespace kinematics
{
namespace bullet
{
typedef kinematics::joint<btScalar, btScalar, 3, 5, false> joint_def_t;	// Joint model used for bullet simulation

class BulletCreature {

public:
	 BulletCreature(const joint_def_t& /*jointDef*/, btDynamicsWorld* m_ownerWorld);
	~BulletCreature();

private:
	BulletCreature& BulletCreature::operator =(const BulletCreature&);
	BulletCreature(const BulletCreature&);
	
public:
	const int nbBodies_;

private:
	btCollisionShape* m_shapes_[MAX_CHILDREN];
	btRigidBody* m_bodies_[MAX_CHILDREN];
	btTypedConstraint* m_joints_[MAX_CHILDREN];
	btDynamicsWorld* m_ownerWorld_;
};


} // namespace bullet
} // namespace kinematics
#endif //_BULLET_CREATURE