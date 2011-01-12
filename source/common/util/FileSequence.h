/*---------------------------------------------------------------------

FileSequence is a small utilitiy class which is helpfull when working
with sequences of files, like image sequences or meshobject sequences.
The FileSequence class tries to detect the naming convention of the
sequence and offers an stl-like iterator to loop through all filenames
of the sequence. Only straight number sequences (2,3,4,5 etc.)
are detected.

The following cases are valid:

simulation23.0001.obj
simulation23.0002.obj etc.

----------------------------------------------------------------------*/
#pragma once

#include <string>
#include <vector>
#include <fstream>






namespace dk
{
	namespace util
	{
		///
		/// \brief very usefull helper for working with sequences of files
		///
		class FileSequence
		{
		public:

			///
			/// \brief stl-style iterator which can iterate over entries within a file sequence
			///
			class iterator
			{
				friend FileSequence;

				//
				// constructor
				// FileSequence is the class which this iterator belongs to
				// index is the entry of the element this iterator is currently pointing at
				//
				iterator( const FileSequence *_holder, int _index ) : holder(_holder), index(_index)
				{
				}

			public:
				//
				// standard constructor
				//
				iterator(  ) : holder(0), index(-1)
				{
				}

				//
				// dereference operator
				//
				std::string operator*() const
				{
					return( holder->getFileNameFromIndex( index ) );
				} 

				//
				// increment operator
				//
				iterator&  operator++()
				{
					++index;
					return( *this );
				}

				//
				// decrement operator
				//
				iterator&  operator--()
				{
					--index;
					return( *this );
				}

				//
				// this comparison operator returns true if the index of this iterator
				// is not equal to the other one
				//
				bool operator!=( const iterator& other )
				{
					return( index != other.index );
				}

				//
				// this comparison operator returns true if the index of this iterator
				// is equal to the other one false otherwise
				//
				bool operator==( const iterator& other )
				{
					return( index == other.index );
				}

			private:
				const FileSequence *holder; // the class which created this iterator
				int                  index; // the current entry within the sequence
			};



			///
			/// standard constructor
			///
			FileSequence()
			{
				prefix = "";
				suffix = "";
				padding = 0;
				firstIndex = 0;
				lastIndex = -1;
				namingRule = PREFIXPADDEDIDXSUFFIX;
			}

			///
			/// constructs the sequence using the given first filename
			///
			FileSequence( const std::string &nameOfFirstFile )
			{
				// the difficult task is: detect the naming convention from given filename
				prefix = "";
				suffix = "";
				padding = 0;
				firstIndex = 0;
				lastIndex = -1;
				namingRule = PREFIXPADDEDIDXSUFFIX;

				bool sequenceExists = false;


				// this vector holds all groups of digits within the filename
				// each of these groups may be the index within the sequence
				std::vector< std::pair<int, int> > digitGroups;

				// scan for digit groups ---------------------------------------
				bool openGroup = false;
				int characterPosition = (int)nameOfFirstFile.size()-1;

				// go through all chars of the string from right to left
				for( std::string::const_reverse_iterator it=nameOfFirstFile.rbegin(); it != nameOfFirstFile.rend(); ++it )
				{
					// is the current char a number?
					if( isDigit( *it ) )
					{
						// yes - was the last value also a number?
						if( openGroup )
						{
							// then we enlarge the current group of numbers within the filename
							digitGroups.back().first = characterPosition;
						}else
						{
							// open up a new group of digits within the filename
							digitGroups.push_back( std::make_pair( characterPosition, characterPosition ) );
							openGroup = true;
						}
					}else
						// we make sure that the current digit group is closed
						openGroup = false;

					// we move ahead
					--characterPosition;
				}

				// if no digitGroups had been found, then there will never be a sequence
				if( !digitGroups.size() )
				{
					// check if the current file exists
					std::ifstream file( nameOfFirstFile.c_str(), std::ios::in | std::ios::binary );

					// could the file be opened -> does it exist?
					if( file )
					{
						// yes! - then close the file
						file.close();

						// the file exsts, so we have a sequence of only one file
						// this policy is used to make the usage of the filesequence more convinient,
						// since in many situations a sequence of only one file without a numbering is used

						// we change the naming rule, so that the filename of the sequence is
						// identical to the prefix string within the filesequence class
						namingRule = PREFIXONLY;
						prefix = nameOfFirstFile;
						// we change the indices, so that we have exactly one file within the sequence
						firstIndex = 0;
						lastIndex = 0;
					}
					
					return;
				}

				// find the group which identifies the sequence ---------------------------------------
				// these temporary variables will hold the values we get from analysing the first digitGroup
				std::string tprefix = "";
				std::string tsuffix = "";
				int tpadding = 0;
				int tfirstIndex = 0;
				int tlastIndex = -1;

				// now try to build a sequence for each digitgroup starting from the most right one
				for( std::vector< std::pair<int, int> >::iterator it = digitGroups.begin(); it != digitGroups.end(); ++it )
				{
					// get the prefix and suffix strings for the current group --------------
					prefix = std::string( nameOfFirstFile.begin(), nameOfFirstFile.begin() + (*it).first );
					if( nameOfFirstFile.begin() + (*it).second + 1 != nameOfFirstFile.end() )
						suffix = std::string( nameOfFirstFile.begin() + (*it).second + 1, nameOfFirstFile.end() );
					else
						suffix = "";

					// get the string which is supposed to encode the position within the sequence
					std::string number( nameOfFirstFile.begin() + (*it).first, nameOfFirstFile.begin() + (*it).second + 1 );

					// get first index from the number string ------------------------------
					char format[128];

					// if the number contains more than one digit and the first number is a zero...
					if( (number.size() > 1) && (number[0] == '0') )
					{
						// ...we know that the number is zero padded
						padding = (int)number.size();

						// this doesnt work
						//sprintf_s( format, 128, "%%0%ii", padding );
						//sscanf_s( number.c_str(), format,  &firstIndex );
						size_t nonZeroPos = 0;

						while( (nonZeroPos<number.size())&&(number[nonZeroPos] == '0') )
							++nonZeroPos;

						std::string num = number.substr( nonZeroPos, number.size() );

						sscanf_s( num.c_str(), "%i",  &firstIndex );
					}else
					{
						// we dont know wether the number is zero-padded, but we dont need to care because
						// any succeding number will be greater than the current one and so wont be padded anymore
						padding = 0;

						sscanf_s( number.c_str(), "%i",  &firstIndex );
					}

					// try to detect a sequence by constructing the name of the next file and look wether it exists
					lastIndex = firstIndex;

					do
					{
						// construct the name of the next file
						std::string nextFileName = assemble( lastIndex + 1 );

						// try to open it
						std::ifstream file( nextFileName.c_str(), std::ios::in | std::ios::binary );

						// could the file be opened -> does it exist?
						if( file )
						{
							// yes! - then close the file
							file.close();

							// indicate that a sequence has been detected
							sequenceExists = true;

							// adjust the lastIndex
							++lastIndex;
						}else
							// if we have found a sequence but could not access the next file, then
							// we are one behind the last entry of the sequence and can stop looking
							// if we have not found a sequence we have to skip also
							break;
					}while( 1 );


					

					// if we have found a sequence, then we can leave the loop, else try the next one
					if( sequenceExists )
						break;
					else
					{
						// if we currently have worked with the first digitGroup, then we will store
						// the values so that if we dont find any sequence we can construct a sequence
						// from the values of the first digitGroup
						tprefix = prefix;
						tsuffix = suffix;
						tpadding = padding;
						tfirstIndex = firstIndex;
						tlastIndex = lastIndex;


						// reset values
						prefix = "";
						suffix = "";
						padding = 0;
						firstIndex = 0;
						lastIndex = -1;
					}
				}



				// we havent found a sequence, but the fileName itsself had a digitGroup in it, so
				// we will build a fileSequence with only one entry by using the values of the first digitGroup
				if( !sequenceExists )
				{
					prefix = tprefix;
					suffix = tsuffix;
					padding = tpadding;
					firstIndex = tfirstIndex;
					lastIndex = tlastIndex;
				}
			}

			///
			/// Returns an iterator which points to the first filename in the sequence
			///
			const iterator begin( void ) const
			{
				return iterator( this, firstIndex );
			}

			///
			/// Returns an iterator which points to the one-past-last element in the sequence
			///
			const iterator end( void ) const
			{
				return iterator( this, lastIndex+1 );
			}

			///
			/// Returns the number of entries in the filesequence
			///
			const size_t size() const
			{
				if( (lastIndex<0)||(firstIndex<0) )
					return 0;
				else
					// we add one since the lastIndex is a valid element
					return (size_t)(lastIndex - firstIndex + 1);
			}

			///
			/// Returns a filename for the index-specified entry within the sequence
			///
			std::string getFileNameFromIndex( int index ) const
			{
				// check if the index is valid
				if( (index >= firstIndex)&&(index <= lastIndex) )
					// assembles the filename from given index
					return assemble( index );

				// invalid index
				return "";
			}

		private:
			int             firstIndex;  // index of the first valid file in the sequence
			int              lastIndex;  // index of the last valid file in the sequence
			std::string         prefix;  // all chars before the index within the filename template
			int                padding;  // how much leading zeros? 0/1 means no leading zeros
			std::string         suffix;  // all chars after the index

			enum
			{
				PREFIXONLY,              // this rule states that the filename is identical to the prefix string (used for single file sequences)
				PREFIXPADDEDIDXSUFFIX    // this is the standard rule: prefix+padded sequence index+suffix
			}               namingRule;  // the naming rule indicates, how the name of the current file within the sequence is assembled


			bool isDigit( const char c )
			{
				if( (c >= 48)&&(c <= 57) )
					return true;
				return false;		
			}

			//
			// assembles the filename from given index without checking wether the index
			// is within valid range
			// the way the filename of the sequence is assembled from given index is defined through the
			// member namingRule
			//
			std::string assemble( int index ) const
			{
				switch(namingRule)
				{
				case PREFIXONLY:
					return prefix;
					break;
				case PREFIXPADDEDIDXSUFFIX:
					{
						// build the filename from the detected naming convention
						char result[ 2048 ];
						char format[ 2048 ];

						if( padding < 2 )
							// no padding
							sprintf_s( format, 2048, "%%s%%i%%s" );
						else
							// padding
							sprintf_s( format, 2048, "%%s%%0%ii%%s", padding );
						
						// assemble result
						sprintf_s( result, 2048, format, prefix.c_str(), index, suffix.c_str() );

						return result;
					}break;
				default:
					// never should reach this
					break;
				}
				return "";
			}

		};



	} // namespace util
} // namespace dk