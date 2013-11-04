/**
* \file tools.h
* \brief useful functions to perform tests
* \author Steve T.
* \version 0.1
* \date 10/12/2013
*/

#ifndef _TOOLS_TEST
#define _TOOLS_TEST

#include <string>
#include <iostream>
#include <stdio.h>

namespace tools
{
///  \brief Compares two text files and return whether they are identical
///  \param filename1 first file to compare
///  \param filename2 second file to compare
///  \return true if files have the same content, false otherwise
bool CompareFiles(const std::string& filename1, const std::string& filename2)
{
	FILE *fp1, *fp2;
    int ch1, ch2;
    fp1 = fopen(filename1.c_str(), "r");
    fp2 = fopen(filename2.c_str(), "r") ;
	if(fp1 == 0)
	{
		std::cout << "Cannot open " << filename1 << "for reading "<< std::endl;
		return false;
	}
    else if(fp2 == 0)    
	{
		std::cout << "Cannot open " << filename2 << "for reading "<< std::endl;
		return false;
	}
    else
	{
		ch1 = getc(fp1);
		ch2 = getc(fp2);
		while((ch1!=EOF) && (ch2!=EOF) && (ch1 == ch2))
		{
			ch1 = getc(fp1);
			ch2 = getc(fp2) ;
		}
		fclose (fp1);
		fclose (fp2);
		if(ch1 == ch2)
		{
			return true;
		}
		else
		{
			return false;
		}
   }
}
}// end namespace toolds
#endif //_TOOLS_TEST
