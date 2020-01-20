#!/bin/sh

Debug/LinuxController \
	./SupportingMaterial/MLFiles/PersonDogCatBirdMobilenetSSD/voc.names \
	./SupportingMaterial/MLFiles/PersonDogCatBirdMobilenetSSD/deploy.prototxt \
	./SupportingMaterial/MLFiles/PersonDogCatBirdMobilenetSSD/deploy.caffemodel \
	.5 \
	./SupportingMaterial/MLFiles/PersonDogCatBirdYOLO/voc.names \
	./SupportingMaterial/MLFiles/PersonDogCatBirdYOLO/deploy.cfg \
	./SupportingMaterial/MLFiles/PersonDogCatBirdYOLO/deploy.weights \
	.2 \
	.3;