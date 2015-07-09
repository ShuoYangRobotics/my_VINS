#ifndef _QUATERNION_ALIAS_H_
#define _QUATERNION_ALIAS_H_
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Eigen {

template<typename Scalar, int Options = AutoAlign> class QuaternionAlias;

namespace internal {
template<typename _Scalar,int _Options>
struct traits<QuaternionAlias<_Scalar,_Options> >
{
	typedef QuaternionAlias<_Scalar,_Options> PlainObject;
	typedef _Scalar Scalar;
	typedef Matrix<_Scalar,4,1,_Options> Coefficients;
	enum{
		IsAligned = internal::traits<Coefficients>::Flags & AlignedBit,
		Flags = IsAligned ? (AlignedBit | LvalueBit) : LvalueBit
	};
};
}

template<typename _Scalar, int _Options>
class QuaternionAlias : public Quaternion<_Scalar,_Options>
{

	typedef Quaternion<_Scalar,_Options> Base;
	//typedef QuaternionAlias<_Scalar,_Options> This;
	enum { IsAligned = internal::traits<QuaternionAlias>::IsAligned };

public:
	typedef _Scalar Scalar;
	typedef Matrix<Scalar,3,1> Vector3;
	typedef Matrix<Scalar,3,3> Matrix3;

	typedef typename internal::traits<QuaternionAlias>::Coefficients Coefficients;
	typedef typename Base::AngleAxisType AngleAxisType;

	QuaternionAlias() : Base() {}

	QuaternionAlias(const Scalar& w, const Scalar& x, const Scalar& y, const Scalar& z) : Base(w, x, y, z) {}

	QuaternionAlias(const Scalar* data) : Base(data) {}

	template<typename Derived>
	QuaternionAlias(const QuaternionBase<Derived>& other) : Base(other) {}

	QuaternionAlias(const AngleAxisType& aa) : Base(aa) {}

	template<typename Derived>
	QuaternionAlias(const MatrixBase<Derived>& other) : Base(other) {}

/*
	template<typename OtherScalar, int OtherOptions>
	explicit inline QuaternionAlias(const QuaternionAlias<OtherScalar, OtherOptions>& other)
	{ m_coeffs = other.coeffs().template cast<Scalar>(); }
*/

	template<class OtherDerived> EIGEN_STRONG_INLINE QuaternionAlias<_Scalar, _Options> operator* (const QuaternionBase<OtherDerived>& other) const
	{
//		return (*this).Base::operator*( other );
		return other.operator*( *this );
	}
	QuaternionAlias<Scalar> conjugate() const
	{
		return QuaternionAlias<Scalar>(this->w(),-this->x(),-this->y(),-this->z());
	}	
	//template<class OtherDerived> EIGEN_STRONG_INLINE Derived& operator*= (const QuaternionBase<OtherDerived>& q) 
	Matrix3 toRotationMatrix() const 
	{
		return this->conjugate().Base::toRotationMatrix();
	}
	EIGEN_STRONG_INLINE Vector3 _transformVector(const Vector3& v) const
	{
		return this->conjugate().Base::_transformVector( v );
	}

};

}

#endif//_QUATERNION_ALIAS_H_
