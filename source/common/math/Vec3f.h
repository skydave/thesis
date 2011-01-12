/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once







namespace math
{
	///
	/// \brief simple vector class
	///
	class Vec3f
	{
	public:
        Vec3f( );
        Vec3f( const float &x, const float &y, const float &z );

		~Vec3f( );


        void  set( const float &x, const float &y, const float &z );

		float                                getLength( void )const; ///< returns the cartesian length of the vector
		float                         getSquaredLength( void )const; ///< returns the un-square-rooted length of the vector 
		void                      setLength( const float &fLength ); ///< scales the vector to the specified length
		void                                      normalize( void ); ///< normalizes the vector
		void                                         negate( void ); ///< negates the vector

		void                         reflect( const Vec3f &normal ); ///< reflects the vector at the given normal



		bool                         operator==( const Vec3f &rhs );
		bool                         operator!=( const Vec3f &rhs );
		
		bool                         operator+=( const Vec3f &rhs );
		bool                         operator-=( const Vec3f &rhs );

		bool                         operator+=( const float &rhs );
		bool                         operator-=( const float &rhs );
		bool                         operator*=( const float &rhs );
		bool                         operator/=( const float &rhs );

		const float&                      operator[]( int i ) const;
		float&                                  operator[]( int i );

		union
		{
			struct
			{
				float x, y, z;
			};
			float v[3];
		};
	};
}