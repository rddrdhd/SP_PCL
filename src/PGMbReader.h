//
// Created by rddrdhd on 26.03.22.
//

#ifndef MAIN_PGMBREADER_H
#define MAIN_PGMBREADER_H

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Structure for storing the
// image data
typedef struct PGMImage {
    char pgmType[3];
    unsigned char** data;
    unsigned int width;
    unsigned int height;
    unsigned int maxValue;
} PGMImage;

class PGMbReader {
public:
    static void ignoreComments(FILE* fp);
    static bool openPGM(PGMImage* pgm,
                 const char* filename);
    static void printImageDetails(PGMImage* pgm,
                           const char* filename);
};


#endif //MAIN_PGMBREADER_H
