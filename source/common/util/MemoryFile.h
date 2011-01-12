/*---------------------------------------------------------------------

The MemoryFile is a simple utility class for writing stuff into a
memory block. The MemoryFile class is designed to be used equally to
a common file.

Later I should use the stl iostream stuff for this.

----------------------------------------------------------------------*/
#pragma once
#include <memory.h>
#include <string>
#include <vector>
#include <fstream>




///
/// \brief is a very simple interface for working on abstract files (similar to std::streams but more simple)
///
class MemoryFile
{
public:
	///
	/// constructor
	///
	MemoryFile()
	{
		m_mem = 0;
		m_reserved = 0;
		m_fileSize = 0;
		m_filePointer = 0;
	}

	///
	/// constructor which will initialize the memoryfile from a given file on harddisk
	///
	MemoryFile( std::string filename )
	{
		m_mem = 0;
		m_reserved = 0;
		m_fileSize = 0;
		m_filePointer = 0;

		// open the file
		std::ifstream file( filename.c_str(), std::ios::in | std::ios::binary );


		// check if the file exists
		if( file )
		{
			// yes! - then read the data

			// get the filesize
			file.seekg(0, std::ios::end);
			std::ifstream::pos_type size = file.tellg();
			file.seekg(0);

			// alloc memory
			resize(size);

			// read filedata into memory
			file.read( m_mem, size );

			// done
			file.close();
		}
	}

	///
	/// destructor
	///
	~MemoryFile()
	{
		if( m_mem )
			free( m_mem );
		m_mem = 0;
		m_reserved = 0;
		m_fileSize = 0;
		m_filePointer = 0;
	}


	/// reserves some memory (size is in bytes)
	void reserve( long reserveSize )
	{
		m_reserved = reserveSize;
		m_mem = (char *)realloc( m_mem, reserveSize*sizeof(char) );
	}

	/// resizes the file
	void resize( long newSize )
	{
		// if the content area of the file is bigger then the current
		// allocated memory:
		if( newSize > m_reserved )
			// then get some new memory
			reserve( newSize + 1000000 );

		// and set the filesize
		m_fileSize = newSize;
	}

	/// writes data from given memory into the file
	long write( const char *srcMem, long size )
	{
		// check whether enough memory is reserved
		if( m_filePointer + size >= m_reserved )
			// get some more memory
			reserve( m_filePointer + size + 1000000 );

		// copy the memory
		memcpy( (void *)&m_mem[m_filePointer], srcMem, size );

		// move the filePointer ahead
		m_filePointer += size;

		// adjust the size of the content if the filepointer had been moved ahead
		if( m_filePointer > m_fileSize )
			m_fileSize = m_filePointer;

		return size;
	}

	/// reads data from file into given memory
	long read( char *dstMem, long size )
	{
		long readAmount = size;
		// check whether the size of the file is big enough for the read operation
		if( m_filePointer + size > m_fileSize )
			// the filepointe would read over the end of the file
			// we read only until the end
			readAmount = m_fileSize - m_filePointer;

		// copy the memory
		memcpy( (void *)dstMem, &m_mem[m_filePointer], readAmount );

		// move the filePointer ahead
		m_filePointer += readAmount;

		return readAmount;
	}


	/// similar to seekf
	void setFilePointer( long pos )
	{
		m_filePointer = pos;
	}


	///
	/// returns the size of the file in memory
	///
	long size()
	{
		return m_fileSize;
	}

	///
	/// returns the memory which contains the files content
	///
	void *getMemory()
	{
		return m_mem;
	}





private:
	char                            *m_mem; // pointer to the memory block
	long                        m_reserved; // current size of the allocated memoryblock
	long                        m_fileSize; // current size of the data within the memory
	long                     m_filePointer; // position of the file pointer within the file
};