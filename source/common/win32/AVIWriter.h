#pragma once
#include <windows.h>
#include <comdef.h>
#include <memory.h>
#include <tchar.h>
#include <string.h>
#include <vfw.h>
#include <assert.h>

///
/// \brief can be used to write a sequence of images into a video (depends in win32api)
///
class AVIWriter
{
public:
	AVIWriter();

	// Inplace constructor with BITMAPINFOHEADER
	AVIWriter(LPCTSTR sFileName, LPBITMAPINFOHEADER lpbih, DWORD dwRate);

	~AVIWriter();

	HRESULT InitEngine(); // Initialize engine and choose codex

	HRESULT AddFrame(BYTE* bmBits);  // brief Adds a frame to the movie

	
	void ReleaseEngine(); // Release ressources allocated for movie and close file.

	
	void SetBitmapHeader(LPBITMAPINFOHEADER lpbih); // Sets bitmap info as in lpbih


	LPBITMAPINFOHEADER GetBitmapHeader()  // returns a pointer to bitmap info
	{
		return &m_bih;
	};

	void SetFileName(LPCTSTR _sFileName)  // sets the name of the ouput file (should be .avi)
	{
		m_sFile=_sFileName; MakeExtAvi();
	};

	void SetRate(DWORD dwRate)       // Sets FrameRate (should equal or greater than one)
	{
		m_dwRate=dwRate;
	};

	LPCTSTR GetLastErrorMessage() const     // returns last  error message
	{
		return m_sError;
	};


protected:	
	_bstr_t m_sFile;                 // name of output file
	DWORD m_dwRate;                	// Frame rate 

	BITMAPINFOHEADER m_bih; // structure contains information for a single stream
	_bstr_t m_sError;        // last error string
private:
	void MakeExtAvi();
	long m_lFrame;           // frame counter

	PAVIFILE m_pAVIFile; // file interface pointer
	PAVISTREAM m_pStream; // Address of the stream interface
	PAVISTREAM m_pStreamCompressed; // Address of the compressed video stream
};

