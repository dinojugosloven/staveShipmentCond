//#include "C:\Users/dino/library/AtlasStyle/AtlasStyle.C"
/*
#include "TH1F.h"
#include "TStyle.h"
#include "TGraph.h"
#include "TFile.h"
#include "TCanvas.h"
#include "TColor.h"
#include "TLegend.h"
#include "TAxis.h"
#include "TTree.h"*/

#include <cstdio>
#include <iostream>


/*! \brief Read binary data output from SparkFun Razor
*
*  Should be called before the analysis file.
*
*  Fills root TTree and saves it to a file
*  Replacing the .dat extension to .txt
*/
int readData(const string dataFile) {

	//const char* dataFile_c = "2_Raw_Data/LOG2.TXT";
	FILE* p_file = fopen(dataFile.c_str(), "br");
	if (p_file == NULL) return(-1);
	printf("File %s opened.\n", dataFile.c_str());

	//create a new ROOT filelength
	string newFileName = dataFile;
	size_t posInString = dataFile.find(".DAT");
	newFileName.replace(posInString, 5, ".TXT");

	// Open output file
	FILE* p_outFile = fopen(newFileName.c_str(), "w");

	//! \TODO read the header and print it 
	char headerString[128];
	fgets(headerString, 128, p_file);
	printf("Read buffer %s\n", headerString);
	fputs(headerString, p_outFile);

	printf("Please wait...\n");
	char binaryString[10];
	while (!feof(p_file))
	{
		int nOfReads = fscanf(p_file, "%10c", binaryString);
		printf("Number of reads: %d string: %s\n", nOfReads, binaryString);
		if (nOfReads != 1) break;
		unsigned long time = binaryString[0] << 24;
		time |= binaryString[1] << 16;
		time |= binaryString[2] << 8;
		time |= binaryString[3];
		short ax = (binaryString[4] << 8) | binaryString[5];
		short ay = (binaryString[6] << 8) | binaryString[7];
		short az = (binaryString[8] << 8) | binaryString[9];
		fprintf(p_outFile, "%lu,%hd,%hd,%hd\n", time, ax, ay, az);
	};


	fclose(p_file);
	fclose(p_outFile);

	return 0;
}