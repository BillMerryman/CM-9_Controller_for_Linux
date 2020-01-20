/** @file visionManager.h
 *  @brief Function prototypes for managing images/vision.

 *  @author Bill Merryman
 *  @bug No known bugs.
 *
 *
 *  Created on: Dec 9, 2019
 *
 */

#ifndef VISIONMANAGER_HPP_
#define VISIONMANAGER_HPP_

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initializes the image/vision subsystem
 *
 * 	Currently gets pointers to the portion of PRU driver allocated memory that will
 * 	be used for holding the image data acquired by the PRU from the OV7675 camera module,
 * 	and a flag to let the application processor know that a complete frame is ready. This
 * 	function also sets up two image instances in memory, one just using the pointer to the
 * 	PRU driver allocated memory, the other having its own image memory to hold the masked
 * 	image processed by cvMoments. It also creates two windows, one for each of the images.
 * 	Also sets up the font to write text to the image windows.
 *
 *	@param	namesFile the file with the list of class names
 *	@param	modelFile the file that defines the DNN network
 *	@param	weightsFile the file containing the weights for the network
 * 	@return void.
 *
 */
void visionManagerInitialize(const char *caffeNamesFile,
								const char *prototxtFile,
								const char *caffemodelFile,
								float caffeConf,
								const char *darknetNamesFile,
								const char *cfgFile,
								const char *weightsFile,
								float darknetConf,
								float darknetNMSThreshold);

/** @brief Uninitializes the image/vision subsystem
 *
 *	This just garbage collects the image header for the main image, the entire image
 *	for the mask image, and the two windows used to display them.
 *
 * 	@return void.
 *
 */
void visionManagerUninitialize();

/** @brief The 'main loop' for acquiring and processing images.
 *
 * 	This function checks to make sure an image frame is ready for processing. If so,
 * 	it uses cvInRangeS find every pixel with a color in the specified range (pink/red in
 * 	this case), and builds a black and white mask image for pixels that match or don't match.
 * 	This mask is then run through cvMoments to find matching areas larger than an
 * 	arbitrary size, calculate the center position of the area, and draw an indicator
 * 	on the image at that point, with a text representation of the coordinates. We then
 * 	update the windows to display these images, and set the flag to let the PRU know
 * 	we are ready to consume another frame.
 *
 * 	@return void.
 *
 */
void visionManagerProcess(char key);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

void visionManagerInitializeCaffe();

void visionManagerInitializeDarknet();

void visionManagerProcessNone();

void visionManagerProcessThreshold();

void visionManagerCaptureThreshold();

void visionManagerProcessCaffe();

void visionManagerProcessDarknet();

#endif

#endif /* VISIONMANAGER_HPP_ */
