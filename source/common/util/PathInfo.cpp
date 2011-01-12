/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "PathInfo.h"

namespace dk
{
	namespace util
	{
		// this buffer is used by any static method
		std::string PathInfo::m_buffer;

		// this character separates two folders
		const char PathInfo::separator('\\');


		//
		// returns true if a file with the given path exists and the program has read permission
		// it will return false if the file doesnt exist or no read permision is given
		//
		bool fileExists( const std::string &path )
		{
			FILE* fp = NULL;

			//will not work if you do not have read permissions

			//to the file, but if you don't have read, it

			//may as well not exist to begin with.

			fopen_s( &fp, path.c_str(), "rb" );
			if( fp != NULL )
			{
				fclose( fp );
				return true;
			}

			return false;
		}


		PathInfo::PathInfo()
		{
		}

		PathInfo::~PathInfo()
		{
		}


		const std::string PathInfo::getDrive(const std::string Path)
		{
			m_buffer = "";

			std::string path(Path);

			int colon = (int)path.find(':');

			if (colon > 0)
			{
				m_buffer = path[colon - 1];
			}

			return m_buffer;
		}



		const std::string PathInfo::getDirectory(const std::string Path)
		{
			m_buffer = "";

			std::string path(Path);

			// the highest valid index
			int last = (int)path.size() - 1;

			// no valid index
			if (last < 0) return "";

			// last separator
			int sep = (int)path.rfind(separator);

			// separator is last character
			if (sep == last)
			{
				m_buffer = path;
			}

			// separator is anywhere else
			else if (sep != std::string::npos)
			{
				m_buffer = path.substr(0, (sep + 1));
			}

			return m_buffer;
		}



		const std::string PathInfo::getFolder(const std::string Path)
		{
			m_buffer = "";

			// get directory
			std::string dir = getDirectory(Path);

			// next to last separator
			int sep;
			sep = (int)dir.rfind(separator);
			sep = (int)dir.rfind(separator, (sep - 1));

			// no next to last separator
			if (sep == std::string::npos) return "";

			// get folder
			m_buffer = dir.substr((sep + 1), (dir.size() - sep - 2));

			return m_buffer;
		}



		const std::string PathInfo::getName(const std::string Path)
		{
			m_buffer = "";

			std::string path(Path);

			// number of characters in path
			int psize = (int)path.size(); 

			// number of characters in dir
			int dsize = (int)getDirectory(Path).size();

			if (dsize < psize)
			{
				m_buffer = path.substr(dsize, (psize - dsize));
			}

			// there is no name
			else return "";

			return m_buffer;
		}


		const std::string PathInfo::getTitle(const std::string Path)
		{
			m_buffer = "";

			// get name
			std::string name = getName(Path);

			// last dot
			int dot = (int)name.rfind(".");

			// no dot
			if (dot == std::string::npos)
			{
				m_buffer = name;
			}

			// dot is anywhere else
			else
			{
				m_buffer = name.substr(0, dot);
			}

			return m_buffer;
		}


		const std::string PathInfo::getExtension(const std::string Path)
		{
			m_buffer = "";

			// get name
			std::string name = getName(Path);

			// last dot
			int dot = (int)name.rfind(".");

			// no dot
			if (dot == std::string::npos) return "";

			// get extension
			m_buffer = name.substr((dot + 1), (name.size() - dot - 1));

			return m_buffer;
		}



		const std::string PathInfo::getPathAs(const std::string Path, const std::string Extension)
		{
			m_buffer = "";

			std::string ex = Extension;

			while(ex.size() && (ex[0] == '.'))
			{
				ex.erase(0, 1);
			}

			std::string dir = getDirectory(Path);
			std::string title = getTitle(Path);

			// no title -> no extension
			if (title.empty()) return "";

			m_buffer  = dir;
			m_buffer += title;
			m_buffer += ".";
			m_buffer += ex;

			return m_buffer;
		}


		const std::string PathInfo::getAbsPath(const std::string RelPath, const std::string Origin)
		{
			m_buffer = "";

			std::string relpath(RelPath);
			std::string name = getName(RelPath);
			std::string dir  = getDirectory(Origin);

			m_buffer = "";

			// RelPath is already absolute
			if (relpath.find(':') != std::string::npos)
			{
				m_buffer = relpath;
				return m_buffer.c_str();
			}

			// separated folders
			std::vector<std::string> fPath;
			std::vector<std::string> fOrigin;

			// split into folders
			doSplitPath(relpath, fPath);
			doSplitPath(dir, fOrigin);

			// nothing splitted
			if (fPath.empty()) return "";
			if (fOrigin.empty()) return "";

			// remove 'current dir'
			if (fPath[0] == ".") fPath.erase(fPath.begin());
			if (fOrigin[0] == ".") fOrigin.erase(fOrigin.begin());

			// nemove filename
			if (name.size()) fPath.pop_back();

			// number of dotted folders
			int dotted;

			// count dotted folders
			for(dotted = 0; dotted < (int)fPath.size(); dotted++)
			{
				if (fPath[dotted] != "..") break;
			}

			// add absolute pieces
			for(int abs = 0; abs < (int)(fOrigin.size() - dotted); abs++)
			{
				m_buffer += fOrigin[abs];
				m_buffer += separator;
			}

			// add relative pieces
			for(int rel = dotted; rel < (int)fPath.size(); rel++)
			{
				m_buffer += fPath[rel];
				m_buffer += separator;
			}

			// add filename
			m_buffer += name;

			return m_buffer;
		}



		const std::string PathInfo::getRelPath(const std::string AbsPath, const std::string Origin)
		{
			m_buffer = "";

			std::string abspath(AbsPath);
			std::string name = getName(AbsPath);
			std::string dir  = getDirectory(Origin);

			m_buffer = "";

			// AbsPath is already relative
			if (abspath.find(':') == std::string::npos)
			{
				m_buffer = abspath;
				return m_buffer.c_str();
			}

			// separated folders
			std::vector<std::string> fPath;
			std::vector<std::string> fOrigin;

			// split into folders
			doSplitPath(abspath, fPath);
			doSplitPath(dir, fOrigin);

			// nothing splitted
			if (fPath.empty()) return "";
			if (fOrigin.empty()) return "";

			// remove 'current dir'
			if (fPath[0] == ".") fPath.erase(fPath.begin());
			if (fOrigin[0] == ".") fOrigin.erase(fOrigin.begin());

			// remove filename
			if (name.size()) fPath.pop_back();

			// number of equal folders
			int equal;

			// count equal folders (from the beginning)
			for(equal = 0; ((equal < (int)fOrigin.size()) && (equal < (int)fPath.size())); equal++)
			{
				// get length
				int size1 = (int)fPath[equal].size();
				int size2 = (int)fOrigin[equal].size();

				// different folders found (length)
				if (size1 != size2)
				{
					break;
				}

				// different folders found (lexicographically)
				else if (_strnicmp(fPath[equal].c_str(), fOrigin[equal].c_str(), size1))
				{
					break;
				}
			}

			// different drives
			if (equal == 0)
			{
				m_buffer = abspath;
				return m_buffer.c_str();
			}

			// move up
			for(int up = equal; up < (int)fOrigin.size(); up++)
			{
				m_buffer += "..";
				m_buffer += separator;
			}

			// move down
			for(int dn = equal; dn < (int)fPath.size(); dn++)
			{
				m_buffer += fPath[dn];
				m_buffer += separator;
			}

			// add filename
			m_buffer += name;

			// current directory
			if (m_buffer.empty())
			{
				m_buffer = ".\\";
			}

			return m_buffer;
		}


		int PathInfo::doSplitPath(const std::string& Path, std::vector<std::string>& Pieces)
		{
			std::string piece;

			// find separators
			for(int i = 0; i < (int)Path.size(); i++)
			{
				char c = Path[i];

				if (c == separator)
				{
					if (piece.empty()) continue;

					Pieces.push_back(piece);

					piece = "";
				}

				else piece += c;
			}

			// add last piece
			if (piece.size())
			{
				Pieces.push_back(piece);
			}

			// return number of pieces
			return (int)Pieces.size();
		}

	}
}