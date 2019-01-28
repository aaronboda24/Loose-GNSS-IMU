/*
* FileIO.cpp
* A class for handling file input and output 
*  Created on: Jun 11, 2018
*      Author: Aaron Boda
*/

#include "pch.h"
#include "FileIO.h"
using namespace std;

// File Opener --> Initiates File pointer safely
void FileIO::fileSafeIn(string filename, ifstream& fin) {
	// Use input file stream, detect file
	ifstream inputfile(filename);
	// Check if file can be opened
	if (!inputfile.is_open())
	{
		perror("Error while opening file");
	}
	// Check is file can be read
	else if (inputfile.bad())
	{
		perror("Error while reading file");
	}
	else
	{
		// If no errors, we safely open the file...
		fin.open(filename);
	}
}

// A function to generate a LOG File for errors in Observation File
void FileIO::logger(string output_filename, string input_filename, ofstream& fout) {
	// Creating NEW LOG file
	fout.open(output_filename); fout.close();
	fout.open(output_filename, ios::app);
	// Preparing a format for LOG file
	fout << "-----------------------------------------------------------------------------------------\n";
	fout << "-----------------------------------------------------------------------------------------\n";
	fout << "THIS FILE CONTAINS RECORDS OF ANY UNEXPECTED BREAKS OR MISSING INFORMATION IN DATA FILES.\n";
	fout << "-----------------------------------------------------------------------------------------\n";
	fout << "-----------------------------------------------------------------------------------------\n";
	fout << "OBSERVATION FILE NAME: " << input_filename << "\n" << "\n";
	fout << std::left
		<< std::setw(30) << "EPOCH INFORMATION"
		<< std::setw(20) << "PRN"
		<< std::setw(20) << "MISSING OBSERVATION" << "\n";
	fout << "-----------------------------------------------------------------------------------------\n";
}

// A function to initialize the updated output datafile
void FileIO::fileSafeOut(string output_filename, ofstream& fout) {
	// Creating NEW LOG file
	fout.open(output_filename); fout.close();
	fout.open(output_filename, ios::app);
}

// A function to find size of input stream
int FileIO::streamSize(ifstream &inputfile) {
	// Initializing variables
	int count = 0; string line;
	// Finding Size of Stream for File...
	inputfile.seekg(0, ifstream::end);
	int size = (int)inputfile.tellg();
	inputfile.seekg(0, ifstream::beg);
	return size;
}
