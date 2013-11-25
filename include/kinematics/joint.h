/**
* \file joint.h
* \brief Description of a 2 or 3 dimensional joint
* \author Steve T.
* \version 0.1
* \date 10/12/2013
*
* The base struct used for all kinematics operations.
* 
*/
#ifndef _STRUCT_JOINT
#define _STRUCT_JOINT

namespace kinematics
{

enum constraint_type
{
	point,
	cone,
	effector,
	constrainedPoint,
	unknown
};

/// \struct joint
/// \brief Description of a kinematic joint of dimension Dim (2 or 3). Contains information
/// about angle limits, as well as eventual parents and children.
/// if Safe is false, no verification is made on the evaluation of the curve.
template<typename Numeric=float, typename Angle=Numeric, int Dim=3, int MaxChildren=5, bool Safe=false>
struct joint
{
	/* Constructors - destructors */
	///\brief Constructor
	explicit joint()
	: nbChildren_(0)
	, parent(0)
	, constraintType(unknown)
	{
		for(unsigned int i = 0; i < Dim; ++i)
		{
			minAngleValues[i] = -360;
			maxAngleValues[i] = 360;
			defaultAngleValues[i] = 0;
			offset[i] = 0;
		}
		for(unsigned int i = 0; i < MaxChildren; ++i)
		{
			children[i] = 0;
		}
	}
	
	///\brief Destructor
	~joint(){};

	///\brief deletes joint children, then the joint itself
	void free()
	{
		for(unsigned int i = 0; i < nbChildren_; ++i)
		{
			children[i]->free();
		}
		delete this;
	}
	/* Constructors - destructors */
	
	/*Operations*/
	///  \brief Assigns a child to the current joint, increases nbChildren_ count.
	///  Also sets child parent to current joint.
	///  \param child the new child
	///  \param return : the value x(t)
	void add_child(joint* child)
	{
		if(Safe && (nbChildren_ > MaxChildren || child->parent ))
		{
			// TODO
		}
		else
		{
			children[nbChildren_] = child;
			++nbChildren_;
			child->parent = this;
		}
	}
	/*Operations*/

	/*Helpers*/
	///  \brief Counts the number of nodes composing the tree consisting in the
	/// current joint and its sons.
	///  \param return : the number of nodes
	unsigned int count() const
	{
		int res = 1;
		for(unsigned int i = 0; i < nbChildren_; ++i)
		{
			res += children[i]->count();
		}
		return res;
	}
	/*Helpers*/
	
	/*Attributes*/
	Angle minAngleValues[Dim]; /*!< minimum angle boundaries, in degrees [-360, 360], for joint along x, y, and z (if Dim = 3) axes */
	Angle defaultAngleValues[Dim]; /*!< default angle values, in degrees [-360, 360], for joint along x, y, and z (if Dim = 3) axes */
	Angle maxAngleValues[Dim]; /*!< maximum angle boundaries, in degrees [-360, 360], for joint along x, y, and z (if Dim = 3) axes */
	Numeric offset[Dim]; /*!< vector indicating the direction and distance of the joint relative to its parent */
	joint* children[MaxChildren]; /*!< array indicating the current joint children */
	unsigned int nbChildren_; /*!< number of children connected to the current joint */
	joint* parent; /*!< pointer to eventual joint parent. Empty if joint is Root. */
	constraint_type constraintType;
	/*Attributes*/
};

}// end namespace kinematics
#endif //_STRUCT_JOINT
