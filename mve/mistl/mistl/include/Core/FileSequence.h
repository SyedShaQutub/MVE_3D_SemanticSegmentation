/****************************************************************************
**
** This file is part of FVVR, Free-Viewpoint Video Renderer
** covered by the terms of the GNU Lesser General Public License (LGPL)
** Copyright(c) 2008 by Jonathan Starck
**
** FileSequence.h
**
** FileSequenceC handles numbered file sequences
**   e.g name%05d.ext for name00000.ext name00001.ext ...
** Construct from a template e.g name%05d.ext or from an example e.g name00000.ext
** Access the range of the sequence and the filename for a particular number
**
****************************************************************************/

#ifndef FILESEQUENCE_H
#define FILESEQUENCE_H

#include <string>
#include <sstream>
#include <algorithm>
#include <stdio.h>

class FileSequenceC
{
public:

	FileSequenceC()
	: m_start(0), m_end(-1), m_valid(false)
	{ }
	
public:
// Creation
	
	bool SetTemplate(const std::string& filename, const std::string& cwdir = "./")
	{
    	SplitPath(filename, cwdir);
		m_valid = (m_name.find('%') != std::string::npos);
		UpdateRange();
		return m_valid;
	}
	// Create a file sequence given the template specification e.g /path/file.%05d.ext
	// Optionally provide the current working directory to define relative file paths
	
	bool CreateTemplate(const std::string& filename, const std::string& cwdir = "./")
    {
    	SplitPath(filename, cwdir);
  		SetNameTemplate();    		
		UpdateRange();
		return m_valid;
    }
	// Create a file sequence given an example filename e.g /path/file.00000.ext
	// Optionally provide the current working directory to define relative file paths

	void ClipRange(int start, int end)
	{
		if (start > m_start) m_start = start;
		if (end < m_end) m_end = end;
	}
	// Set a specific range to use in the file sequence
	
public:
// Access
	
	bool IsSequence() const
	{
		return m_valid;
	}
	// Return whether this is a valid file sequence
	
	int StartFrame() const
	{
		return m_start;
	}
	// Return the first frame number
	
	int EndFrame() const
	{
		return m_end;
	}
	// Return the last frame number
		
	const std::string& Path() const
	{
		return m_path;
	}
	// Return the path component of the sequence
	
	const std::string& Name() const
	{
		return m_name;
	}
	// Return the template name for the sequence
	
	const std::string& Extension() const
	{
		return m_ext;
	}
	// Return the extension for the sequence

	std::string FileTemplate() const
	{
    	std::stringstream filename;
    	filename << m_path << '/' << m_name << '.' << m_ext;
    	return filename.str();
	}
	// Return the full template
	
	std::string File(int idx, bool bCheckRange = true) const
	{
		char pFile[255];
		if (bCheckRange && m_valid)
		{
			if (idx < m_start) idx = m_start;
			if (idx > m_end) idx = m_end;
		}
		sprintf(pFile, m_name.c_str(), idx);
    	std::stringstream filename;
    	filename << m_path << '/' << pFile << '.' << m_ext;
    	return filename.str();
	}
	// Return the idx number file in the sequence
	
public:
	
	void UpdateRange()
	{
		m_start = 0;
		m_end = -1;
		if (m_valid)
		{
			// Set the first and last index
			FILE *fp;
			while ( !(fp = fopen(File(m_start, false).c_str(), "r")) && m_start<99999) 
			{
				m_start++;
			}
			fclose(fp);
			m_end = m_start;
			while ( (fp = fopen(File(m_end, false).c_str(), "r")) && m_end<99999) 
			{
				fclose(fp);	
				m_end++;
			}
			m_end--;
			if (fp) fclose(fp);
		}
	}
	// Determine the range of the sequence from the file system
	
private:
	
	void SplitPath(const std::string& fullpath, const std::string& cwdir)
	{
		m_ext = "";
		m_path = "";
		m_name = fullpath;
		// Fix relative file paths
		if (m_name.size()>2 && m_name[0] != '/' && m_name[0] != '\\' && m_name[1] != ':')
		{
			m_name.insert(0, "/");
			m_name.insert(0, cwdir.c_str());
		}
		int size = m_name.size();
		// Ensure a Unix style path
		std::replace(m_name.begin(),m_name.end(),'\\','/');
		// Extract the file path
		std::string::size_type last = m_name.find_last_of('/');
		if (last != std::string::npos)
		{
			m_path = m_name.substr(0,last);
			m_name = m_name.substr(last+1,size-last-1);
		}
		// Extract the file extension
		last = m_name.find_last_of('.');
		if (last != std::string::npos)
		{
			m_ext = m_name.substr(last+1,size-last-1);
			m_name = m_name.substr(0,last);
		}
	}
	// Split the given file into the path, filename and extension
	
	void SetNameTemplate()
	{
		m_valid = true;
		if (m_name.find('%') == std::string::npos)
		{
			// Find the last continuous numbered section in the file name
			std::string::size_type last = m_name.find_last_of("0123456789");
			if (last != std::string::npos)
			{
				std::string::size_type first = last, next = last;
				while ((next = m_name.find_last_of("0123456789", first-1)) != std::string::npos) first = next;
		    	std::stringstream format;
		    	format << m_name.substr(0,first) <<  "%0" << (last-first+1) << "d" <<  m_name.substr(last+1,m_name.size()-last-1); // << '\0'; 
		    	m_name = format.str();
			}
			else
			{
				m_valid = false;
			}
		}
	}
	// Set the name template from an example file name
		
private:
	std::string m_path;
	std::string m_name;
	std::string m_ext;
	int m_start, m_end;
	bool m_valid;
};

#endif /*FILESEQUENCE_H*/
