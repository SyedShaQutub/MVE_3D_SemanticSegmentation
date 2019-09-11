//
//                 INTEL CORPORATION PROPRIETARY INFORMATION
//
//    This software is supplied under the terms of a license agreement or
//    nondisclosure agreement with Intel Corporation and may not be copied
//    or disclosed except in accordance with the terms of that agreement.
//    Copyright (C) 2014 Intel Corporation. All Rights Reserved.
//
//    ## specifics
//
//    Mistl - Multi-camera Image STructure Library
//    Author: Jonas Scheer
//


#include	"mistl/Camera.h"
#include	"mistl/Error.h"
#include	<stdlib.h>
#include 	<stdio.h>
#include 	<string.h>



void
PrMsg() {
	std::cout<<"Converts camera files to a ycam camera files: \ne.g camera.yml --> camera.ycam\n";
	std::cout<<"Usage: camToYcam camera-file_1 [camera-file_2 camera-file_3 ...] \n\n";
	    exit(-1);
}


int main(int argc, char** argv)
{

	if(argc<=1) {
		PrMsg();
		return -1;
	}

	bool convertBack = true;
	for(int i=1; i<argc; i++) {

		if(argv[i] == "-cb") {
			convertBack = true;
			continue;
		}

		try {

			// create new filename
			char* fn = argv[i];
			const char *cp = strrchr(fn,'.');
			if(!strcmp(cp, ".ycam"))
				continue;

			int length = 0;

			while(fn != cp) {
				fn++;
				length++;
			}
			char file[length + 5];
			char fileName[strlen(argv[i])+3];
			strcpy (fileName,"cb_");
			strcat (fileName, argv[i]);

			int c=0;
			fn = argv[i];
			while(fn != cp) {
				file[c] = *fn;
				c++;
				fn++;
			}
			strcat(file, ".ycam");


			// read and convert camera file
			mistl::Camera cam;
			ReadCamera(cam, argv[i]);
			WriteCamera(cam , file);
			if(convertBack) {
				ReadCamera(cam, file);
//				std::cout<<"asdadasdsad: "<< fileName <<"\n";
				WriteCamera(cam , fileName);
			}

		}
		catch ( mistl::Error e) {
			std::cout<<"Error: "<<e.GetMsg()<<"\n";
			PrMsg();
			return -1;
		}
	}

	return 0;
}
