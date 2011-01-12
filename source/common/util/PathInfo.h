/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once



#include <vector>
#include <string>

namespace dk
{
	///
	/// \brief holds a bunch of additional class which are usefull in a variety of situations but not very important
	///
	namespace util
	{

		bool fileExists( const std::string &path ); // returns true if a file with the given path exists and the program has read permission

		///
		/// \brief provides multiple static methods to work with strings containing a (file)path
		///
		class PathInfo
		{

		public:

			// standard-constructor
			PathInfo();

			// virtual destructor
			virtual ~PathInfo();


			// this character separates two folders (may be '/' or '\')
			static const char separator;


			// returns the drive's letter from the passed path
			static const std::string getDrive(const std::string Path);

			// returns the directory from the passed path
			static const std::string getDirectory(const std::string Path);

			// returns the file's folder from the passed path
			static const std::string getFolder(const std::string Path);

			// returns the file's name from the passed path
			static const std::string getName(const std::string Path);

			// returns the file's title from the passed path
			static const std::string getTitle(const std::string Path);

			// returns the file's extension from the passed path
			static const std::string getExtension(const std::string Path);

			// returns the passed path with this new extension
			static const std::string getPathAs(const std::string Path, const std::string Extension);

			// returns the absolute path starting at Origin
			static const std::string getAbsPath(const std::string RelPath, const std::string Origin);

			// returns the relative path starting at Origin
			static const std::string getRelPath(const std::string AbsPath, const std::string Origin);


		protected:

			// divides a path up into pieces
			static int doSplitPath(const std::string& Path, std::vector<std::string>& Pieces);

		private:

			// this buffer is used by any static method
			static std::string m_buffer;
		};


	}
}