/**
* \file Exports.h
* \brief Exports definition for the bullet integration dll (windows)
* \author Steve T.
* \version 0.1
* \date 11/05/2013
*/

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        #ifdef BULLET_DLLEXPORT
                #define BULLET_API __declspec(dllexport)
        #else
                #define BULLET_API __declspec(dllimport)
        #endif
#else
        #define BULLET_API 
#endif