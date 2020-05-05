/*
 * FLIR_viewer.h
 *
 *  Created on: Feb 28, 2020
 *      Author: kict
 */

#ifndef SRC_FLIR_VIEWER_INCLUDE_FLIR_VIEWER_FLIR_VIEWER_H_
#define SRC_FLIR_VIEWER_INCLUDE_FLIR_VIEWER_FLIR_VIEWER_H_

#include <vector>
#include <stdio.h>
#include <semaphore.h>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include "cam_viewer/cam_viewer.h"

/* from cam_viewer.h
 * typedef struct camera_info {
	struct shm_data *p;
	int id;
	sem_t *mutex;
	sem_t *debugLock;
	int *quit;
} cam_info_t;
 *
 */
struct shm_data;

class FLIR_Viewer {

	enum chunkDataType
	{
		IMAGE,
		NODEMAP
	};
	static const chunkDataType chosenChunkData = IMAGE;

	enum {WHITE=0, RED=1};

public:
	struct FLIR_info: camera_info {

		Spinnaker::CameraPtr pCam;

	};

private:

	std::vector<int> ids;
	std::vector<sem_t *> mutex;
	std::vector<shm_data *> shmData;
	std::vector<FLIR_info> info;
	std::vector<pthread_t> grabThreads;

	Spinnaker::SystemPtr system;
	Spinnaker::CameraList camList;

	int quit;



public:
	FLIR_Viewer(std::vector<int> _ids, const std::string & mutexPrefix);
	void Initialize();
	void Finalize();
	~FLIR_Viewer();
private:
	std::string GetDeviceID(Spinnaker::CameraPtr pCam);
	static void* AcquireImages(void* arg);

	static int SaveFrame(int seqNo, int width, int height, int strideSize, char * data);
	static int WriteInShm(Spinnaker::ImagePtr image, FLIR_info *pInfo);

#ifdef _DEBUG
// Disables heartbeat on GEV cameras so debugging does not incur timeout errors
	static int DisableHeartbeat(Spinnaker::CameraPtr pCam, Spinnaker::GenApi::INodeMap& nodeMap, Spinnaker::GenApi::INodeMap& nodeMapTLDevice);

#endif


};


#endif /* SRC_FLIR_VIEWER_INCLUDE_FLIR_VIEWER_FLIR_VIEWER_H_ */
