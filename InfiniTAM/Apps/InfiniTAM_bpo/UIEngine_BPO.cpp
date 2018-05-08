// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "UIEngine_BPO.h"

#include <string.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else

#include <GL/glut.h>

#endif

#ifdef FREEGLUT

#include <GL/freeglut.h>

#else
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "glut64")
#endif
#endif

#include "../../ITMLib/ITMLibDefines.h"
#include "../../ITMLib/Core/ITMBasicEngine.h"
#include "../../ITMLib/Core/ITMBasicSurfelEngine.h"
#include "../../ITMLib/Core/ITMMultiEngine.h"

#include "../../ORUtils/FileUtils.h"
#include "../../InputSource/FFMPEGWriter.h"
#include "../../ITMLib/Utils/ITMBenchmarkUtils.h"
#include "../../ITMLib/Core/ITMDynamicEngine.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

namespace bench = ITMLib::Bench;

UIEngine_BPO* UIEngine_BPO::instance;

static void Safe_GlutBitmapString(void* font, const char* str) {
	size_t len = strlen(str);
	for (size_t x = 0; x < len; ++x) {
		glutBitmapCharacter(font, str[x]);
	}
}

void UIEngine_BPO::GlutDisplayFunction() {
	UIEngine_BPO* uiEngine = UIEngine_BPO::Instance();

	// get updated images from processing thread
	uiEngine->mainEngine->GetImage(uiEngine->outImage[0], uiEngine->outImageType[0], &uiEngine->freeviewPose,
	                               &uiEngine->freeviewIntrinsics);

	if (!uiEngine->inStepByStepMode) {
		for (int w = 1; w < NUM_WIN; w++)
			uiEngine->mainEngine->GetImage(uiEngine->outImage[w], uiEngine->outImageType[w]);
	}

	// do the actual drawing
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3f(1.0f, 1.0f, 1.0f);
	glEnable(GL_TEXTURE_2D);

	ITMUChar4Image** showImgs = uiEngine->outImage;
	Vector4f* winReg = uiEngine->winReg;
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	{
		glLoadIdentity();
		glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		{
			glEnable(GL_TEXTURE_2D);
			for (int w = 0; w < NUM_WIN; w++) {// Draw each sub window
				if (uiEngine->outImageType[w] == ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN) continue;
				glBindTexture(GL_TEXTURE_2D, uiEngine->textureId[w]);
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, showImgs[w]->noDims.x, showImgs[w]->noDims.y, 0, GL_RGBA,
				             GL_UNSIGNED_BYTE, showImgs[w]->GetData(MEMORYDEVICE_CPU));
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				glBegin(GL_QUADS);
				{
					glTexCoord2f(0, 1);
					glVertex2f(winReg[w][0], winReg[w][1]); // glVertex2f(0, 0);
					glTexCoord2f(1, 1);
					glVertex2f(winReg[w][2], winReg[w][1]); // glVertex2f(1, 0);
					glTexCoord2f(1, 0);
					glVertex2f(winReg[w][2], winReg[w][3]); // glVertex2f(1, 1);
					glTexCoord2f(0, 0);
					glVertex2f(winReg[w][0], winReg[w][3]); // glVertex2f(0, 1);
				}
				glEnd();
			}
			glDisable(GL_TEXTURE_2D);
		}
		glPopMatrix();
	}
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	switch (uiEngine->trackingResult) {
		case 0:
			glColor3f(1.0f, 0.0f, 0.0f);
			break; // failure
		case 1:
			glColor3f(1.0f, 1.0f, 0.0f);
			break; // poor
		case 2:
			glColor3f(0.0f, 1.0f, 0.0f);
			break; // good
		default:
			glColor3f(1.0f, 1.0f, 1.0f);
			break; // relocalising
	}

	glRasterPos2f(0.85f, -0.962f);

	char str[200];
	sprintf(str, "%04.2lf", uiEngine->processedTime);
	Safe_GlutBitmapString(GLUT_BITMAP_HELVETICA_18, (const char*) str);
	glColor3f(1.0f, 0.0f, 0.0f);
	if (uiEngine->inStepByStepMode) {
		glRasterPos2f(-0.98f, -0.95f);
		sprintf(str, "e/esc: exit \t d: one step");
		Safe_GlutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*) str);
	} else {
		glRasterPos2f(-0.98f, -0.90f);
		const char* modeName;
		const char* followOrFreeview;
		if (uiEngine->freeviewActive) {
			modeName = uiEngine->colourModes_freeview[uiEngine->currentColourMode].name;
			followOrFreeview = "follow camera";
		} else {
			modeName = uiEngine->colourModes_main[uiEngine->currentColourMode].name;
			followOrFreeview = "free viewpoint";
		}

		sprintf(str, "n: one frame \t b: continous \t e/esc: exit \t r: reset \t k: save \t l: load \t"
		             " f: %s \t c: colours (currently %s) \t t: turn fusion %s", followOrFreeview, modeName,
		        uiEngine->integrationActive ? "off" : "on");
		Safe_GlutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*) str);
		glRasterPos2f(-0.98f, -0.95f);
		sprintf(str, "Alt+n: one frame (record) \t i: %d frames \t d: one step", uiEngine->autoIntervalFrameCount);
		Safe_GlutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*) str);
	}
	glutSwapBuffers();
	uiEngine->needsRefresh = false;
}

void UIEngine_BPO::GlutIdleFunction() {
	UIEngine_BPO* uiEngine = UIEngine_BPO::Instance();

	switch (uiEngine->mainLoopAction) {
		case PROCESS_FRAME:
			uiEngine->recordWarpUpdatesForNextFrame = false;
			uiEngine->ProcessFrame();
			uiEngine->processedFrameNo++;
			uiEngine->mainLoopAction = PROCESS_PAUSED;
			uiEngine->needsRefresh = true;
			break;
		case PROCESS_SINGLE_STEP:
			if (!uiEngine->UpdateProcessFrame()) {
				uiEngine->inStepByStepMode = false;
			}
			uiEngine->mainLoopAction = PROCESS_PAUSED;
			uiEngine->needsRefresh = true;
			break;
		case PROCESS_FRAME_RECORD:
			uiEngine->recordWarpUpdatesForNextFrame = true;
			uiEngine->ProcessFrame();
			uiEngine->processedFrameNo++;
			uiEngine->mainLoopAction = PROCESS_PAUSED;
			uiEngine->needsRefresh = true;
			break;
		case PROCESS_VIDEO:
			uiEngine->ProcessFrame();
			uiEngine->processedFrameNo++;
			uiEngine->needsRefresh = true;
			break;
			//TODO: either remove dead code or {uncomment, test, fix} -Greg (GitHub: Algomorph)
			//case SAVE_TO_DISK:
			//	if (!uiEngine->actionDone)
			//	{
			//		char outFile[255];

			//		ITMUChar4Image *saveImage = uiEngine->saveImage;

			//		glReadBuffer(GL_BACK);
			//		glReadPixels(0, 0, saveImage->noDims.x, saveImage->noDims.x, GL_RGBA, GL_UNSIGNED_BYTE, (unsigned char*)saveImage->GetData(false));
			//		sprintf(outFile, "%s/out_%05d.ppm", uiEngine->outFolder, uiEngine->processedFrameNo);

			//		SaveImageToFile(saveImage, outFile, true);

			//		uiEngine->actionDone = true;
			//	}
			//	break;
		case PROCESS_N_FRAMES:
			uiEngine->ProcessFrame();
			uiEngine->processedFrameNo++;
			uiEngine->needsRefresh = true;
			if ((uiEngine->processedFrameNo - uiEngine->autoIntervalFrameStart) >= uiEngine->autoIntervalFrameCount) {
				uiEngine->mainLoopAction = PROCESS_PAUSED;
				bench::PrintAllCumulativeTimes();
			}
			break;
		case EXIT:
#ifdef FREEGLUT
			glutLeaveMainLoop();
#else
			exit(0);
#endif
			break;
		case PROCESS_PAUSED:
		default:
			break;
	}

	if (uiEngine->needsRefresh) {
		glutPostRedisplay();
	}
}

void UIEngine_BPO::GlutKeyUpFunction(unsigned char key, int x, int y) {
	UIEngine_BPO* uiEngine = UIEngine_BPO::Instance();
	int modifiers = glutGetModifiers();
	if (uiEngine->inStepByStepMode) {
		switch (key) {
			case 'q':
			case 'e':
			case 27: // esc key
				printf("exiting ...\n");
				uiEngine->mainLoopAction = UIEngine_BPO::EXIT;
				break;
			case 'd':
				uiEngine->mainLoopAction = UIEngine_BPO::PROCESS_SINGLE_STEP;
				break;
			default:
				break;
		}
	} else {
		switch (key) {
			case 'n':
				if (modifiers & GLUT_ACTIVE_ALT) {
					printf("saving scenes before tracking and warp updates, processing one frame, and recording the warp updates ...\n");
					uiEngine->mainLoopAction = UIEngine_BPO::PROCESS_FRAME_RECORD;
				} else {
					printf("processing one frame ...\n");
					uiEngine->mainLoopAction = UIEngine_BPO::PROCESS_FRAME;
				}
				break;
			case 'i':
				printf("processing %d frames ...\n", uiEngine->autoIntervalFrameCount);
				uiEngine->autoIntervalFrameStart = uiEngine->processedFrameNo;
				uiEngine->mainLoopAction = UIEngine_BPO::PROCESS_N_FRAMES;
				break;
			case 'b':
				printf("processing input source ...\n");
				uiEngine->mainLoopAction = UIEngine_BPO::PROCESS_VIDEO;
				break;
			case 's':
				if (uiEngine->isRecording) {
					printf("stopped recoding disk ...\n");
					uiEngine->isRecording = false;
				} else {
					printf("started recoding disk ...\n");
					uiEngine->currentFrameNo = 0;
					uiEngine->isRecording = true;
				}
				break;
			case 'v':
				if ((uiEngine->rgbVideoWriter != NULL) || (uiEngine->depthVideoWriter != NULL)) {
					printf("stop recoding video\n");
					delete uiEngine->rgbVideoWriter;
					delete uiEngine->depthVideoWriter;
					uiEngine->rgbVideoWriter = NULL;
					uiEngine->depthVideoWriter = NULL;
				} else {
					printf("start recoding video\n");
					uiEngine->rgbVideoWriter = new FFMPEGWriter();
					uiEngine->depthVideoWriter = new FFMPEGWriter();
				}
				break;
			case 'q':
			case 'e':
			case 27: // esc key
				printf("exiting ...\n");
				uiEngine->mainLoopAction = UIEngine_BPO::EXIT;
				break;
			case 'f':
				uiEngine->currentColourMode = 0;
				//TODO: replace this whole if/else block with a separate function, use this function during initialization as well -Greg (Github: Algomorph)
				if (uiEngine->freeviewActive) {
					uiEngine->outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
					uiEngine->outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;

					uiEngine->freeviewActive = false;
				} else {
					uiEngine->outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED;
					uiEngine->outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;

					uiEngine->freeviewPose.SetFrom(uiEngine->mainEngine->GetTrackingState()->pose_d);
					if (uiEngine->mainEngine->GetView() != NULL) {
						uiEngine->freeviewIntrinsics = uiEngine->mainEngine->GetView()->calib.intrinsics_d;
						uiEngine->outImage[0]->ChangeDims(uiEngine->mainEngine->GetView()->depth->noDims);
					}

					ITMMultiEngine<ITMVoxel, ITMVoxelIndex>* multiEngine = dynamic_cast<ITMMultiEngine<ITMVoxel, ITMVoxelIndex>*>(uiEngine->mainEngine);
					if (multiEngine != NULL) {
						int idx = multiEngine->findPrimaryLocalMapIdx();
						if (idx < 0) idx = 0;
						multiEngine->setFreeviewLocalMapIdx(idx);
					}

					uiEngine->freeviewActive = true;
				}
				uiEngine->needsRefresh = true;
				break;
			case 'c':
				uiEngine->currentColourMode++;
				if (((uiEngine->freeviewActive) &&
				     ((unsigned) uiEngine->currentColourMode >= uiEngine->colourModes_freeview.size())) ||
				    ((!uiEngine->freeviewActive) &&
				     ((unsigned) uiEngine->currentColourMode >= uiEngine->colourModes_main.size())))
					uiEngine->currentColourMode = 0;
				uiEngine->needsRefresh = true;
				break;
			case 't': {
				uiEngine->integrationActive = !uiEngine->integrationActive;

				ITMBasicEngine<ITMVoxel, ITMVoxelIndex>* basicEngine = dynamic_cast<ITMBasicEngine<ITMVoxel, ITMVoxelIndex>*>(uiEngine->mainEngine);
				if (basicEngine != NULL) {
					if (uiEngine->integrationActive) basicEngine->turnOnIntegration();
					else basicEngine->turnOffIntegration();
				}

				ITMBasicSurfelEngine<ITMSurfelT>* basicSurfelEngine = dynamic_cast<ITMBasicSurfelEngine<ITMSurfelT>*>(uiEngine->mainEngine);
				if (basicSurfelEngine != NULL) {
					if (uiEngine->integrationActive) basicSurfelEngine->turnOnIntegration();
					else basicSurfelEngine->turnOffIntegration();
				}
			}
				break;
			case 'w': {
				printf("saving scene to model ... ");
				uiEngine->mainEngine->SaveSceneToMesh("mesh.stl");
				printf("done\n");
			}
				break;
			case 'r': {
				ITMBasicEngine<ITMVoxel, ITMVoxelIndex>* basicEngine = dynamic_cast<ITMBasicEngine<ITMVoxel, ITMVoxelIndex>*>(uiEngine->mainEngine);
				if (basicEngine != NULL) basicEngine->resetAll();

				ITMBasicSurfelEngine<ITMSurfelT>* basicSurfelEngine = dynamic_cast<ITMBasicSurfelEngine<ITMSurfelT>*>(uiEngine->mainEngine);
				if (basicSurfelEngine != NULL) basicSurfelEngine->resetAll();
			}
				break;
			case 'k': {
				printf("saving scene to disk ... ");

				try {
					uiEngine->mainEngine->SaveToFile();
					printf("done\n");
				}
				catch (const std::runtime_error& e) {
					printf("failed: %s\n", e.what());
				}
			}
				break;
			case 'l': {
				printf("loading scene from disk ... ");

				try {
					uiEngine->mainEngine->LoadFromFile();
					printf("done\n");
				}
				catch (const std::runtime_error& e) {
					printf("failed: %s\n", e.what());
				}
			}
				break;
			case '[':
			case ']': {
				ITMMultiEngine<ITMVoxel, ITMVoxelIndex>* multiEngine = dynamic_cast<ITMMultiEngine<ITMVoxel, ITMVoxelIndex>*>(uiEngine->mainEngine);
				if (multiEngine != NULL) {
					int idx = multiEngine->getFreeviewLocalMapIdx();
					if (key == '[') idx--;
					else idx++;
					multiEngine->changeFreeviewLocalMapIdx(&(uiEngine->freeviewPose), idx);
					uiEngine->needsRefresh = true;
				}
			}
				break;
			case 'd':
				if (uiEngine->BeginProcessingFrame()) {
					uiEngine->freeviewActive = true;
					uiEngine->inStepByStepMode = true;
					uiEngine->mainLoopAction = PROCESS_SINGLE_STEP;
					uiEngine->needsRefresh = true;
				}
				break;

			default:
				break;
		}
	}

	if(uiEngine->inStepByStepMode) {
		uiEngine->outImageType[0] = uiEngine->colourMode_stepByStep.type;
	}else{
		if (uiEngine->freeviewActive) uiEngine->outImageType[0] = uiEngine->colourModes_freeview[uiEngine->currentColourMode].type;
		else uiEngine->outImageType[0] = uiEngine->colourModes_main[uiEngine->currentColourMode].type;
	}
}

void UIEngine_BPO::GlutMouseButtonFunction(int button, int state, int x, int y) {
	UIEngine_BPO* uiEngine = UIEngine_BPO::Instance();

	if (state == GLUT_DOWN) {
		switch (button) {
			case GLUT_LEFT_BUTTON:
				uiEngine->mouseState = 1;
				break;
			case GLUT_MIDDLE_BUTTON:
				uiEngine->mouseState = 3;
				break;
			case GLUT_RIGHT_BUTTON:
				uiEngine->mouseState = 2;
				break;
			default:
				break;
		}
		uiEngine->mouseLastClick.x = x;
		uiEngine->mouseLastClick.y = y;

		glutSetCursor(GLUT_CURSOR_NONE);
	} else if (state == GLUT_UP && !uiEngine->mouseWarped) {
		uiEngine->mouseState = 0;
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}
}

static inline Matrix3f createRotation(const Vector3f& _axis, float angle) {
	Vector3f axis = normalize(_axis);
	float si = sinf(angle);
	float co = cosf(angle);

	Matrix3f ret;
	ret.setIdentity();

	ret *= co;
	for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) ret.at(c, r) += (1.0f - co) * axis[c] * axis[r];

	Matrix3f skewmat;
	skewmat.setZeros();
	skewmat.at(1, 0) = -axis.z;
	skewmat.at(0, 1) = axis.z;
	skewmat.at(2, 0) = axis.y;
	skewmat.at(0, 2) = -axis.y;
	skewmat.at(2, 1) = axis.x;
	skewmat.at(1, 2) = -axis.x;
	skewmat *= si;
	ret += skewmat;

	return ret;
}

void UIEngine_BPO::GlutMouseMoveFunction(int x, int y) {
	UIEngine_BPO* uiEngine = UIEngine_BPO::Instance();

	if (uiEngine->mouseWarped) {
		uiEngine->mouseWarped = false;
		return;
	}

	if (!uiEngine->freeviewActive || uiEngine->mouseState == 0) return;

	Vector2i movement;
	movement.x = x - uiEngine->mouseLastClick.x;
	movement.y = y - uiEngine->mouseLastClick.y;

	Vector2i realWinSize(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	// Does not work if the window is smaller than 40x40
	Vector2i activeWinTopLeft(20, 20);
	Vector2i activeWinBottomRight(realWinSize.width - 20, realWinSize.height - 20);
	Vector2i activeWinSize(realWinSize.width - 40, realWinSize.height - 40);

	bool warpNeeded = false;

	if (x < activeWinTopLeft.x) {
		x += activeWinSize.x;
		warpNeeded = true;
	} else if (x >= activeWinBottomRight.x) {
		x -= activeWinSize.x;
		warpNeeded = true;
	}

	if (y < activeWinTopLeft.y) {
		y += activeWinSize.y;
		warpNeeded = true;
	} else if (y >= activeWinBottomRight.y) {
		y -= activeWinSize.y;
		warpNeeded = true;
	}

	if (warpNeeded) {
		glutWarpPointer(x, y);
		uiEngine->mouseWarped = true;
	}

	uiEngine->mouseLastClick.x = x;
	uiEngine->mouseLastClick.y = y;

	if ((movement.x == 0) && (movement.y == 0)) return;

	static const float scale_rotation = 0.005f;
	static const float scale_translation = 0.0025f;

	switch (uiEngine->mouseState) {
		case 1: {
			// left button: rotation
			Vector3f axis((float) -movement.y, (float) -movement.x, 0.0f);
			float angle = scale_rotation * sqrt((float) (movement.x * movement.x + movement.y * movement.y));
			Matrix3f rot = createRotation(axis, angle);
			uiEngine->freeviewPose.SetRT(rot * uiEngine->freeviewPose.GetR(), rot * uiEngine->freeviewPose.GetT());
			uiEngine->freeviewPose.Coerce();
			uiEngine->needsRefresh = true;
			break;
		}
		case 2: {
			// right button: translation in x and y direction
			uiEngine->freeviewPose.SetT(uiEngine->freeviewPose.GetT() +
			                            scale_translation * Vector3f((float) movement.x, (float) movement.y, 0.0f));
			uiEngine->needsRefresh = true;
			break;
		}
		case 3: {
			// middle button: translation along z axis
			uiEngine->freeviewPose.SetT(
					uiEngine->freeviewPose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, (float) movement.y));
			uiEngine->needsRefresh = true;
			break;
		}
		default:
			break;
	}
}

void UIEngine_BPO::GlutMouseWheelFunction(int button, int dir, int x, int y) {
	UIEngine_BPO* uiEngine = UIEngine_BPO::Instance();

	static const float scale_translation = 0.05f;

	uiEngine->freeviewPose.SetT(
			uiEngine->freeviewPose.GetT() + scale_translation * Vector3f(0.0f, 0.0f, (dir > 0) ? -1.0f : 1.0f));
	uiEngine->needsRefresh = true;
}

/**
 * \brief Initialize the UIEngine using the specified settings
 * \param argc arguments to the main function
 * \param argv number of arguments to the main function
 * \param imageSource source for images
 * \param imuSource source for IMU data
 * \param mainEngine main engine to process the frames
 * \param outFolder output folder for saving results
 * \param deviceType type of device to use for some tasks
 * \param frameIntervalLength automatically process this number of frames after launching the UIEngine,
 * set interval to this number of frames
 */
void UIEngine_BPO::Initialise(int& argc, char** argv, ImageSourceEngine* imageSource, IMUSourceEngine* imuSource,
                              ITMMainEngine* mainEngine,
                              const char* outFolder, ITMLibSettings::DeviceType deviceType, int frameIntervalLength) {
	this->inStepByStepMode = false;
	this->freeviewActive = true;
	this->integrationActive = true;
	this->currentColourMode = 0;
	this->autoIntervalFrameCount = frameIntervalLength;

	this->colourModes_main.emplace_back("shaded greyscale", ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST);
	this->colourModes_main.emplace_back("integrated colours", ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME);
	this->colourModes_main.emplace_back("surface normals", ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_NORMAL);
	this->colourModes_main.emplace_back("confidence", ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_CONFIDENCE);

	this->colourModes_freeview.emplace_back("canonical", ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_CANONICAL);
	this->colourModes_freeview.emplace_back("shaded greyscale", ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_SHADED);

	//_DEBUG (re-enable 3 modes below) -Greg (GitHub:Algomorph)
//	this->colourModes_freeview.emplace_back("integrated colours",
//	                                        ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME);
//	this->colourModes_freeview.emplace_back("surface normals",
//	                                        ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_NORMAL);
//	this->colourModes_freeview.emplace_back("confidence",
//	                                        ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_CONFIDENCE);

	this->imageSource = imageSource;
	this->imuSource = imuSource;
	this->mainEngine = mainEngine;
	{
		size_t len = strlen(outFolder);
		this->outFolder = new char[len + 1];
		strcpy(this->outFolder, outFolder);
	}

	int textHeight = 60; // Height of text area, 2 lines

	winSize.x = (int) (1.5f * (float) (imageSource->getDepthImageSize().x));
	winSize.y = imageSource->getDepthImageSize().y + textHeight;
	float h1 = textHeight / (float) winSize.y, h2 = (1.f + h1) / 2;
	winReg[0] = Vector4f(0.0f, h1, 0.665f, 1.0f);   // Main render
	winReg[1] = Vector4f(0.665f, h2, 1.0f, 1.0f);   // Side sub window 0
	winReg[2] = Vector4f(0.665f, h1, 1.0f, h2);     // Side sub window 2

	this->isRecording = false;
	this->currentFrameNo = 0;
	this->rgbVideoWriter = nullptr;
	this->depthVideoWriter = nullptr;

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(winSize.x, winSize.y);
	glutCreateWindow("InfiniTAM");
	glGenTextures(NUM_WIN, textureId);

	glutDisplayFunc(UIEngine_BPO::GlutDisplayFunction);
	glutKeyboardUpFunc(UIEngine_BPO::GlutKeyUpFunction);
	glutMouseFunc(UIEngine_BPO::GlutMouseButtonFunction);
	glutMotionFunc(UIEngine_BPO::GlutMouseMoveFunction);
	glutIdleFunc(UIEngine_BPO::GlutIdleFunction);

#ifdef FREEGLUT
	glutMouseWheelFunc(UIEngine_BPO::GlutMouseWheelFunction);
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, 1);
#endif

	bool allocateGPU = false;
	if (deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

	for (int w = 0; w < NUM_WIN; w++)
		outImage[w] = new ITMUChar4Image(imageSource->getDepthImageSize(), true, allocateGPU);

	inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, allocateGPU);
	inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, allocateGPU);
	inputIMUMeasurement = new ITMIMUMeasurement();

	saveImage = new ITMUChar4Image(imageSource->getDepthImageSize(), true, false);

	outImageType[0] = this->freeviewActive ? this->colourModes_freeview[this->currentColourMode].type
	                                       : this->colourModes_main[this->currentColourMode].type;
	outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
	outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
	if (inputRGBImage->noDims == Vector2i(0, 0)) outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_UNKNOWN;

	mainLoopAction = autoIntervalFrameCount ? PROCESS_N_FRAMES : PROCESS_PAUSED;
	autoIntervalFrameStart = 0;
	mouseState = 0;
	mouseWarped = false;
	needsRefresh = false;
	processedFrameNo = 0;
	processedTime = 0.0f;

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaThreadSynchronize());
#endif

	sdkCreateTimer(&timer_instant);
	sdkCreateTimer(&timer_average);

	sdkResetTimer(&timer_average);

	printf("initialised.\n");
}

void UIEngine_BPO::SaveScreenshot(const char* filename) const {
	ITMUChar4Image screenshot(GetWindowSize(), true, false);
	GetScreenshot(&screenshot);
	SaveImageToFile(&screenshot, filename, true);
}

void UIEngine_BPO::GetScreenshot(ITMUChar4Image* dest) const {
	glReadPixels(0, 0, dest->noDims.x, dest->noDims.y, GL_RGBA, GL_UNSIGNED_BYTE, dest->GetData(MEMORYDEVICE_CPU));
}

void UIEngine_BPO::ProcessFrame() {
	if (!imageSource->hasMoreImages()) return;
	imageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (imuSource != NULL) {
		if (!imuSource->hasMoreMeasurements()) return;
		else imuSource->getMeasurement(inputIMUMeasurement);
	}

	if (isRecording) {
		char str[250];

		sprintf(str, "%s/%04d.pgm", outFolder, currentFrameNo);
		SaveImageToFile(inputRawDepthImage, str);

		if (inputRGBImage->noDims != Vector2i(0, 0)) {
			sprintf(str, "%s/%04d.ppm", outFolder, currentFrameNo);
			SaveImageToFile(inputRGBImage, str);
		}
	}
	if ((rgbVideoWriter != NULL) && (inputRGBImage->noDims.x != 0)) {
		if (!rgbVideoWriter->isOpen())
			rgbVideoWriter->open("out_rgb.avi", inputRGBImage->noDims.x, inputRGBImage->noDims.y, false, 30);
		rgbVideoWriter->writeFrame(inputRGBImage);
	}
	if ((depthVideoWriter != NULL) && (inputRawDepthImage->noDims.x != 0)) {
		if (!depthVideoWriter->isOpen())
			depthVideoWriter->open("out_d.avi", inputRawDepthImage->noDims.x, inputRawDepthImage->noDims.y, true, 30);
		depthVideoWriter->writeFrame(inputRawDepthImage);
	}

	sdkResetTimer(&timer_instant);
	sdkStartTimer(&timer_instant);
	sdkStartTimer(&timer_average);


	ITMTrackingState::TrackingResult trackerResult;
	mainEngine->recordNextFrameWarps = this->recordWarpUpdatesForNextFrame;
	//actual processing on the mailEngine
	if (imuSource != NULL)
		trackerResult = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	else trackerResult = mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

	trackingResult = (int) trackerResult;

#ifndef COMPILE_WITHOUT_CUDA
	ORcudaSafeCall(cudaThreadSynchronize());
#endif
	sdkStopTimer(&timer_instant);
	sdkStopTimer(&timer_average);

	//processedTime = sdkGetTimerValue(&timer_instant);
	processedTime = sdkGetAverageTimerValue(&timer_average);

	currentFrameNo++;
}

void UIEngine_BPO::Run() { glutMainLoop(); }

void UIEngine_BPO::Shutdown() {
	sdkDeleteTimer(&timer_instant);
	sdkDeleteTimer(&timer_average);

	if (rgbVideoWriter != NULL) delete rgbVideoWriter;
	if (depthVideoWriter != NULL) delete depthVideoWriter;

	for (int w = 0; w < NUM_WIN; w++)
		delete outImage[w];

	delete inputRGBImage;
	delete inputRawDepthImage;
	delete inputIMUMeasurement;

	delete[] outFolder;
	delete saveImage;
	delete instance;
	instance = NULL;
}

bool UIEngine_BPO::BeginProcessingFrame() {
	if (!imageSource->hasMoreImages()) return false;

	auto* dynamicEngine = dynamic_cast<ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>*>(mainEngine);

	if (dynamicEngine == nullptr) return false;


	imageSource->getImages(inputRGBImage, inputRawDepthImage);

	if (imuSource != NULL) {
		if (!imuSource->hasMoreMeasurements()) return false;
		else imuSource->getMeasurement(inputIMUMeasurement);
	}

	if (isRecording) {
		char str[250];

		sprintf(str, "%s/%04d.pgm", outFolder, currentFrameNo);
		SaveImageToFile(inputRawDepthImage, str);

		if (inputRGBImage->noDims != Vector2i(0, 0)) {
			sprintf(str, "%s/%04d.ppm", outFolder, currentFrameNo);
			SaveImageToFile(inputRGBImage, str);
		}
	}
	if ((rgbVideoWriter != NULL) && (inputRGBImage->noDims.x != 0)) {
		if (!rgbVideoWriter->isOpen())
			rgbVideoWriter->open("out_rgb.avi", inputRGBImage->noDims.x, inputRGBImage->noDims.y, false, 30);
		rgbVideoWriter->writeFrame(inputRGBImage);
	}
	if ((depthVideoWriter != NULL) && (inputRawDepthImage->noDims.x != 0)) {
		if (!depthVideoWriter->isOpen())
			depthVideoWriter->open("out_d.avi", inputRawDepthImage->noDims.x, inputRawDepthImage->noDims.y, true, 30);
		depthVideoWriter->writeFrame(inputRawDepthImage);
	}


	//ITMTrackingState::TrackingResult trackerResult;
	mainEngine->recordNextFrameWarps = this->recordWarpUpdatesForNextFrame;
	//actual processing on the mailEngine
	if (imuSource != NULL)
		dynamicEngine->BeginProcessingFrameInStepByStepMode(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);
	else dynamicEngine->BeginProcessingFrameInStepByStepMode(inputRGBImage, inputRawDepthImage);
	return true;
}

bool UIEngine_BPO::UpdateProcessFrame() {
	auto* dynamicEngine = dynamic_cast<ITMDynamicEngine<ITMVoxelCanonical, ITMVoxelLive, ITMVoxelIndex>*>(mainEngine);
	if (dynamicEngine == nullptr) return false;
	bool keepProcessingFrame = dynamicEngine->UpdateCurrentFrameSingleStep();
	if (!keepProcessingFrame) {
		trackingResult = dynamicEngine->GetStepByStepTrackingResult();
#ifndef COMPILE_WITHOUT_CUDA
		ORcudaSafeCall(cudaThreadSynchronize());
#endif
		currentFrameNo++;
	}
	return keepProcessingFrame;
}
