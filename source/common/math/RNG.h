/*---------------------------------------------------------------------

Random Number Generator class
Realistic Ray Tracing 2nd Ed.
(Uses a linear congruence algorithm)

----------------------------------------------------------------------*/
#pragma once


namespace math
{
	typedef unsigned long long ull_t;

	///
	/// \brief the Random Number Generator may be used to generate a random number within [0,1]
	///
	class RNG
	{
    public:
        RNG(unsigned long long _seed = 7564231ULL)
        {
            seed = _seed;
            mult = 62089911ULL;
            llong_max = 4294967295ULL;
            float_max = 4294967295.0f;
        }

        float operator()();
        void setSeed(unsigned long long _seed = 7564231ULL);
        
        unsigned long long seed;
        unsigned long long mult;
        unsigned long long llong_max;
        float float_max;
	};

	/// use the parethesis operator to get a new random value
	inline float RNG::operator()()
	{
		seed = mult * seed;
		return float(seed % llong_max) / float_max;
	}

	inline void RNG::setSeed(unsigned long long _seed)
	{
		seed = _seed;
	}

	///  global random number generator for fast and easy random-number-access
	static RNG g_randomNumber;
}
