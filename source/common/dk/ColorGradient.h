/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once
#include <vector>
#include <algorithm>
#include "math/Math.h"


namespace dk
{


	///
	/// \brief maps skalars into linear interpolated colors
    ///
	class ColorGradient
	{
	public:

		///
		/// \brief a sample is a known point of our mapping function where a output is defined for a given input.
		///
		struct Sample
		{
			Sample( float input, math::Color output ) : m_input(input), m_output(output)
			{
			};

            bool operator<( const Sample& a )
			{
                return m_input < a.m_input;
			}

			float               m_input;
			math::Color        m_output;
		};

		///
		/// adds a sample to the sample list
		///
		void addSample( float input, math::Color output )
		{
			// add the sample to the list of samples
			m_samples.push_back( Sample(input, output) );

			// sort them
			std::sort( m_samples.begin(), m_samples.end() );
		}

		///
		/// returns the index-identified sample by value
		///
		Sample getSample( size_t index )
		{
			return m_samples[ index ];
		}

		///
		/// returns the number of samples
		///
		size_t getSampleCount()
		{
			return m_samples.size();
		}

		///
		/// removes all samples from the samplerlist
		///
		void clearSamples()
		{
			m_samples.clear();
		}


		///
		/// samples the mapping function at the specified input
		///
		math::Color getOutput( const float &input )
		{
			// if there is no sample
			if( !(m_samples.size() > 0) )
				// the output is 0.0f by definition
				return math::Color( 0.0f, 0.0f, 0.0f );

			// check wether our input is within the boundary of our samples
			if( input <= m_samples[0].m_input )
				// input is smaller then the input value of the first sample
				// return the output of the first sample (TODO: handle outofbound cases)
				return m_samples[0].m_output;

			// check wether our input is within the boundary of our samples
			if( input >= m_samples.back().m_input )
				// input is greater then the input value of the last sample
				// return the output of the last sample (TODO: handle outofbound cases)
				return m_samples.back().m_output;

			// now we can be sure that there are at least 2 samples and input value is greater or equal to
			// the input of the first sample and it is less then the input of the last sample

			// look for the first sample which has an input which is greater then the input we are looking for
			std::vector<Sample>::iterator it;
			for( it = m_samples.begin(); it != m_samples.end(); ++it )
				// if the current sample has an input which is greater then the input we are looking for
				// then cancel the loop
				if( (*it).m_input > input )
					break;

			// ok we have the first sample whose input is greater than the input we are looking for 

			// perform linear interpolation between this and the previous sample
			float u = (input - (*(it-1)).m_input) / ((*it).m_input - (*(it-1)).m_input);
			return (1.0f - u)*(*(it-1)).m_output + u*(*it).m_output;
		}

		///
		/// set output value of index specified sample
		///
		void setSampleOutput( const size_t &index, const math::Color &output )
		{
			m_samples[ index ].m_output = output;
		}

		///
		/// set input value of index specified sample
		///
		void setSampleInput( const size_t &index, const float &input )
		{
			m_samples[ index ].m_input = input;
			// sort them
			std::sort( m_samples.begin(), m_samples.end() );
		}

		///
		/// this operator is overloaded for convenience so that
		/// the Sampler looks and behaves as a normal c function
		///
		math::Color operator()( const float &input )
		{
            return getOutput( input );
        }


		// predefined stress gradients ----------------------------------------------

		/// engineering colorgradient usefull for showing stresses or temperature
		static ColorGradient Stress()
		{
			ColorGradient result;
			result.addSample( 0.0f, math::Color::From255( 0, 0, 143) );
			result.addSample( 0.11f, math::Color::From255( 0, 0, 255) );
			result.addSample( 0.36f, math::Color::From255( 0, 255, 255) );
			result.addSample( 0.62f, math::Color::From255( 255, 255, 0) );
			result.addSample( 0.86f, math::Color::From255( 255, 0, 0) );
			result.addSample( 1.0f, math::Color::From255( 119, 0, 0) );
			return result;
		}

		/// simple black to white gradient
		static ColorGradient BlackWhite()
		{
			ColorGradient result;
			result.addSample( 0.0f, math::Color( 0.0f, 0.0f, 0.0f) );
			result.addSample( 1.0f, math::Color( 1.0f, 1.0f, 1.0f) );
			return result;
		}

	private:
		std::vector<Sample> m_samples;  // the list of samples which define the mapping function
	};



}