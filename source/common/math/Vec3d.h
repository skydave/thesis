/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once







namespace math
{
	//
	//
	//
	class Vec3d
	{
	public:
        Vec3d( );
        Vec3d( const double &x, const double &y, const double &z );

		~Vec3d( );


        void  set( const double &x, const double &y, const double &z );

		double                                getLength( void )const; // returns the cartesian length of the vector
		double                         getSquaredLength( void )const; // returns the un-square-rooted length of the vector 
		void                      setLength( const double &fLength ); // scales the vector to the specified length
		void                                       normalize( void ); // normalizes the vector
		void                                          negate( void ); // negates the vector

		void                          reflect( const Vec3d &normal ); // reflects the vector at the given normal



		bool                          operator==( const Vec3d &rhs );
		bool                          operator!=( const Vec3d &rhs );
		
		bool                          operator+=( const Vec3d &rhs );
		bool                          operator-=( const Vec3d &rhs );

		bool                         operator+=( const double &rhs );
		bool                         operator-=( const double &rhs );
		bool                         operator*=( const double &rhs );
		bool                         operator/=( const double &rhs );

		const double&                      operator[]( int i ) const;
		double&                                  operator[]( int i );

		union
		{
			struct
			{
				double x, y, z;
			};
			double v[3];
		};
	};
}