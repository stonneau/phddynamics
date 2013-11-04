/**
* \file pid.h
* \brief struct implementing a pid controller for an arbitrary number of independant variables.
* \author Steve T.
* \version 0.1
* \date 10/23/2013
*/


#ifndef _STRUCT_PID
#define _STRUCT_PID

#include <Eigen/Dense>

namespace
{
	template<typename Numeric, int Dim, typename Matrix>
	Matrix MakeMatrix(const Numeric& value)
	{
		Matrix result;
		for(int i = 0; i<Dim; ++i)
		{
			result.diagonal()[i] = value;
		}
		return result;
	}
}

namespace pid
{

/// \struct pid
/// \brief A pid controller using diagonal matrices for gain storage.
template<typename Time=float, typename Numeric=Time, int Dim=5, typename Variable= Eigen::Matrix<Numeric, Dim, 1> >
struct pid
{
	typedef Variable var_t;
	typedef Time 	 time_t;
	typedef Numeric	 num_t;
	typedef Eigen::DiagonalMatrix<Numeric, Dim, Dim> gain_t;

/* Constructors - destructors */
	public:
	///\brief Constructor
	///\param kp vector describing the proportional gain for each variable
	///\param ki vector describing the integral gain for each variable
	///\param kd vector describing the derivative gain for each variable; do not fill to use a PI controller
	pid(const var_t& kp, const var_t& ki, const var_t& kd=var_t::Zero)
	: kp_(kp.asDiagonal())
	, ki_(ki.asDiagonal())
	, kd_(kd.asDiagonal())
	, integral_(var_t::Zero())
	, error_(var_t::Zero())
	{
		// NOTHING
	}
	
	public:
	///\brief Constructor. Use this constructor if all gains are the same for every variable
	///\param kp proportional gain for all variables
	///\param ki integral gain for all variables
	///\param kd derivative gain for all variables; do not fill to use a PI controller
	pid(const num_t& kp, const num_t& ki, const num_t& kd=0)
	: integral_(var_t::Zero())
	, error_(var_t::Zero())
	, kp_(MakeMatrix<num_t, Dim, gain_t>(kp))
	, ki_(MakeMatrix<num_t, Dim, gain_t>(ki))
	, kd_(MakeMatrix<num_t, Dim, gain_t>(kd))
	{
		// NOTHING
	}

	///\brief Destructor
	~pid()
	{
		// NOTHING
	}
/* Constructors - destructors */

/*Helpers*/
	///  \brief Resets the pid by setting error and integral terms to 0
	void reset()
	{
		integral_ = var_t::Zero();
		error_ = var_t::Zero();
	}
/*Helpers*/

/*Operations*/
	///  \brief Evaluation of the cubic spline at time t.
	///  \param t : time elapsed since last update
	///  \param t : processValue current values for the variables
	///  \param t : setPoint values we want the variables to reach
	///  \param return : the new input for the Manipulated variable (MV)
	virtual var_t operator()(time_t dt, const var_t& processValue, const var_t& setPoint)
	{
		var_t currentError, derivate;
		currentError = setPoint - processValue;
		integral_ = integral_ + currentError * dt;
		derivate = (currentError - error_) / dt;
		error_ = currentError;
		return (kp_ * currentError + ki_ * integral_ + kd_ * derivate).transpose();
	}
/*Operations*/

	const gain_t kp_, ki_, kd_;
	var_t integral_, error_;
};
}
#endif //_STRUCT_PID

