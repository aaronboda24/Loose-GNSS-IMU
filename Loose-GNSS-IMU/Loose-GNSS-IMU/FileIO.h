#pragma once

/*
* FileIO.h
* A class for handling file input and output 
*  Created on: Jun 11, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#ifndef FILEIO_H_
#define FILEIO_H_

class FileIO
{
public:
	// Function Declarations
	void fileSafeIn(std::string filename, std::ifstream& fin);
	void fileSafeOut(std::string filename, std::ofstream& fout);
	int streamSize(std::ifstream &inputfile);
	void logger(std::string output_filename, std::string input_filename, std::ofstream& fout);
};

#endif /* FILEIO_H_ */
