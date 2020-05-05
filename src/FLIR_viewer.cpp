/*
 * FLIR_viewer.cpp
 *
 *  Created on: Feb 28, 2020
 *      Author: kict
 */


#include <stdio.h>
#include <string>
#include <sstream>
#include <string.h>

#include <ace/Shared_Memory_SV.h>

#include "ros/ros.h"

#include "FLIR_viewer/FLIR_viewer.h"
#include "client_interface/shm_manager.h"
#include "client_interface/client_interface.h"


#include <iostream>
#include <sstream>



using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
// Use the following enum and global constant to select whether chunk data is
// displayed from the image or the nodemap.











using namespace std;

pthread_t tID;



FLIR_Viewer::FLIR_Viewer(vector<int> _ids, const string & mutexPrefix) : ids(_ids), quit(0) {

	string mutexName;




	for (int k=0; k<ids.size(); ++k){

		stringstream ss;
		ss << ids[k];
		mutexName = mutexPrefix+ss.str();

		sem_t * m = sem_open(mutexName.c_str(), O_RDWR);
		if (m == SEM_FAILED) {
			stringstream msg;
			msg << "Error in initializing mutex " << ids[k];
			THROW(RobotException, msg.str().c_str());
		}


	    ACE_Shared_Memory_SV shm_client(client_interface::ShmManager::SHM_KEY_START+ids[k], sizeof (shm_data));

	    char *shm = (char *) shm_client.malloc ();

	    if (shm == NULL) {
	    	sem_close(m);

			stringstream msg;
			msg << "Error in initializing shm " << ids[k];
			THROW(RobotException, msg.str().c_str());
	    }

	    shm_data * pData = new (shm) shm_data;


		if(sem_wait(m) < 0) {
			THROW(RobotException, "Error in acquiring mutex");
		}


		if(!(pData->len > MAX_DATA_SIZE)) {
			sem_post(m);
			sem_close(m);

			stringstream msg;
			msg << "shared data occupied for id " << ids[k];
			THROW(RobotException, msg.str().c_str());
		}

		pData->len = 0;


		if(sem_post(m) < 0) {
			THROW(RobotException, "Error in releasing mutex");
		}



		mutex.push_back(m);
		shmData.push_back(pData);


	}

	ROS_DEBUG("I am in constructor");


	try {


		Initialize();
	}
	catch (const exception & e) {

		Finalize();
		THROW(RobotException, e.what());
	}

}


FLIR_Viewer::~FLIR_Viewer() {
	ROS_DEBUG("I am in destructor");
	Finalize();
}

void FLIR_Viewer::Finalize() {
	quit = 1;


	assert(info.size() == grabThreads.size());
	//unsigned int camListSize = camList.GetSize();
    for (unsigned int i = 0; i < grabThreads.size(); i++)
    {
        // Wait for all threads to finish
        void* exitcode;
        int rc = pthread_join(grabThreads[i], &exitcode);
        if (rc != 0)
        {
            cout << "Handle error from pthread_join returned for camera at index " << i << endl;
        }
        else if ((int)(intptr_t)exitcode == 0) // check thread return code for each camera
        {
            cout << "Grab thread for camera at index " << i
                 << " exited with errors."
                    "Please check onscreen print outs for error details"
                 << endl;
        }
    }


    // Clear CameraPtr array and close all handles
    /*
    for (unsigned int i = 0; i < camListSize; i++)
    {
        pCamList[i] = 0;
    }
    */


    // Delete array pointer
    //if (pCamList)
    	//delete[] pCamList;

    // Delete array pointer
    grabThreads.resize(0);
    info.resize(0);

	for (int k=0; k<ids.size(); ++k) {
		sem_wait(mutex[k]);

		shmData[k]->len = MAX_DATA_SIZE+1;			// image processor left

		sem_post(mutex[k]);

		sem_close(mutex[k]);

	}

}

string FLIR_Viewer::GetDeviceID(CameraPtr pCam) {
	int result = 0;
	int err = 0;

	try
	{
		// Retrieve TL device nodemap and print device information
		INodeMap& nodeMap = pCam->GetTLDeviceNodeMap();

		FeatureList_t features;
		CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
		if (IsAvailable(category) && IsReadable(category))
		{
			category->GetFeatures(features);
			FeatureList_t::const_iterator it;
			for (it = features.begin(); it != features.end(); ++it)
			{
				CNodePtr pfeatureNode = *it;

				if (pfeatureNode->GetName() == "DeviceID") {
					CValuePtr pValue = (CValuePtr)pfeatureNode;
					if (IsReadable(pValue) == false)
						THROW(RobotException, "Error in reading device id");
					return string(pValue->ToString().c_str());

				}

			}
		}

		return string();

	}
	catch (Spinnaker::Exception& e)
	{
		THROW(RobotException, e.what());

	}

}


void FLIR_Viewer::Initialize() {

	// Print out current library version


	try
	{


		system = System::GetInstance();
		const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();


		int tryCount = 10;

		unsigned int camListSize = 0;
		while(tryCount > 0) {

			camList.Clear();
			camList = system->GetCameras();


			camListSize = camList.GetSize();
			ROS_DEBUG_STREAM("Number of cameras detected: " << camListSize);
			// Finish if there are no cameras
			if (camListSize != ids.size())
			{
				// Clear camera list before releasing system

				ROS_ERROR("Try %d more times", tryCount);
				ros::Duration(1.0).sleep();

			}
			else
				break;

		}

		if (tryCount == 0) {
			camList.Clear();
			system->ReleaseInstance();
			THROW(RobotException, "Camera(s) not found");

		}




		for (int k=0; k < ids.size(); ++k) {
			int i=0;
			for (i=0; i<camListSize; ++i) {
				string deviceID = GetDeviceID(camList.GetByIndex(i));
				ROS_INFO("deviceID is %s", deviceID.c_str());
				string url = "";

				sem_wait(mutex[k]);
				url = shmData[k]->url;
				sem_post(mutex[k]);

				ROS_DEBUG("URL IS %s", url.c_str());
				if (deviceID == url && deviceID.length() > 0) {
					FLIR_info cur;
					cur.id = ids[k];
					cur.debugLock = NULL;
					cur.mutex = mutex[k];
					cur.p = shmData[k];
					cur.quit = &quit;
					cur.pCam = camList.GetByIndex(i);
					info.push_back(cur);
				}

			}

			// camera for id = ids[k] is not found
			if (i == camListSize) {
				ROS_ERROR("Camera for id=%d not found", ids[k]);
			}


		}


		// Create an array of CameraPtrs. This array maintenances smart pointer's reference
		// count when CameraPtr is passed into grab thread as void pointer

		// Create an array of handles
		//pCamList = new CameraPtr[camListSize];
		//grabThreads = new pthread_t[info.size()];

		grabThreads.resize(info.size());

		for (unsigned int i = 0; i < grabThreads.size(); i++)
		{
			// Select camera
			//pCamList[i] = camList.GetByIndex(i);
			// Start grab thread
			int err = pthread_create(&(grabThreads[i]), nullptr, &AcquireImages, &info[i]);
			assert(err == 0);


		}
	}


	catch (Spinnaker::Exception& e)
	{
		THROW(RobotException, e.what());
	}

}

	   //THROW(RobotException, "test");


void* FLIR_Viewer::AcquireImages(void* arg)
{
	FLIR_Viewer::FLIR_info * pInfo = (FLIR_Viewer::FLIR_info *) arg;
    CameraPtr pCam = pInfo->pCam;////*((CameraPtr*)arg);

    try
    {
        // Retrieve TL device nodemap
        INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        // Retrieve device serial number for filename
        CStringPtr ptrStringSerial = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");

        std::string serialNumber = "";

        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            serialNumber = ptrStringSerial->GetValue();
        }

        cout << endl
             << "[" << serialNumber << "] "
             << "*** IMAGE ACQUISITION THREAD STARTING"
             << " ***" << endl
             << endl;

        // Print device information
        //PrintDeviceInfo(nodeMapTLDevice, serialNumber);

        // Initialize camera
        pCam->Init();

#ifdef _DEBUG
        cout << endl << endl << "*** DEBUG ***" << endl << endl;

        // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
        if (DisableHeartbeat(pCam, pCam->GetNodeMap(), pCam->GetTLDeviceNodeMap()) != 0)
        {
            return (void*)0;

        }

        cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
#endif

        // Set acquisition mode to continuous
        CEnumerationPtr ptrAcquisitionMode = pCam->GetNodeMap().GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            cout << "Unable to set acquisition mode to continuous (node retrieval; camera " << serialNumber
                 << "). Aborting..." << endl
                 << endl;
            return (void*)0;

        }

        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval " << serialNumber
                 << "). Aborting..." << endl
                 << endl;
            return (void*)0;

        }

        int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        cout << "[" << serialNumber << "] "
             << "Acquisition mode set to continuous..." << endl;

        // Begin acquiring images
        pCam->BeginAcquisition();

        cout << "[" << serialNumber << "] "
             << "Started acquiring images..." << endl;

        //
        // Retrieve, convert, and save images for each camera
        //
        const unsigned int k_numImages = 10;

        cout << endl;

        int result = 0;

        for (unsigned int imageCnt = 0; *(pInfo->quit) == 0 && result == 0; imageCnt++)
        {
            try
            {
                // Retrieve next received image and ensure image completion
                ImagePtr pResultImage = pCam->GetNextImage();

                if (pResultImage->IsIncomplete())
                {
                    cout << "[" << serialNumber << "] "
                         << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl
                         << endl;
                }
                else
                {
                    // Convert image to mono 8
                    ImagePtr convertedImage = pResultImage->Convert(PixelFormat_RGB8);//PixelFormat_Mono8, HQ_LINEAR);

                    //SaveFrame(imageCnt, convertedImage->GetWidth(), convertedImage->GetHeight(), convertedImage->GetStride(), (char *) convertedImage->GetData());

                    result = WriteInShm(convertedImage, pInfo);



                    ROS_DEBUG("Width:%d Height:%d StrideSize:%d PixelFormatName:%s", convertedImage->GetWidth(), convertedImage->GetHeight(), convertedImage->GetStride(), convertedImage->GetPixelFormatName().c_str());
                    // Create a unique filename
                    ostringstream filename;

                    filename << "/tmp/AcquisitionMultipleThread-";
                    if (serialNumber != "")
                    {
                        filename << serialNumber.c_str();
                    }

                    filename << "-" << imageCnt << ".jpg";

                    // Save image
                    //convertedImage->Save(filename.str().c_str());



                    // Print image information
                    cout << "[" << serialNumber << "] "
                         << "Grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth()
                         << ", height = " << pResultImage->GetHeight() << ". Image saved at " << filename.str() << endl;
                }

                // Release image
                pResultImage->Release();



                cout << endl;
            }
            catch (Spinnaker::Exception& e)
            {
                cout << "[" << serialNumber << "] "
                     << "Error: " << e.what() << endl;
            }

            //usleep(500000);

        }

        // End acquisition
        pCam->EndAcquisition();
        // Deinitialize camera
        pCam->DeInit();

        return (void*)1;

    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return (void*)0;
    }
}

#ifdef _DEBUG
// Disables heartbeat on GEV cameras so debugging does not incur timeout errors
int FLIR_Viewer::DisableHeartbeat(CameraPtr pCam, INodeMap& nodeMap, INodeMap& nodeMapTLDevice)
{
    cout << "Checking device type to see if we need to disable the camera's heartbeat..." << endl << endl;
    //
    // Write to boolean node controlling the camera's heartbeat
    //
    // *** NOTES ***
    // This applies only to GEV cameras and only applies when in DEBUG mode.
    // GEV cameras have a heartbeat built in, but when debugging applications the
    // camera may time out due to its heartbeat. Disabling the heartbeat prevents
    // this timeout from occurring, enabling us to continue with any necessary debugging.
    // This procedure does not affect other types of cameras and will prematurely exit
    // if it determines the device in question is not a GEV camera.
    //
    // *** LATER ***
    // Since we only disable the heartbeat on GEV cameras during debug mode, it is better
    // to power cycle the camera after debugging. A power cycle will reset the camera
    // to its default settings.
    //

    CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
    if (!IsAvailable(ptrDeviceType) && !IsReadable(ptrDeviceType))
    {
        cout << "Error with reading the device's type. Aborting..." << endl << endl;
        return -1;
    }
    else
    {
        if (ptrDeviceType->GetIntValue() == DeviceType_GEV)
        {
            cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..." << endl << endl;
            CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
            if (!IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat))
            {
                cout << "Unable to disable heartbeat on camera. Continuing with execution as this may be non-fatal..."
                     << endl
                     << endl;
            }
            else
            {
                ptrDeviceHeartbeat->SetValue(true);
                cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode." << endl;
                cout << "         Power cycle camera when done debugging to re-enable the heartbeat..." << endl << endl;
            }
        }
        else
        {
            cout << "Camera does not use GigE interface. Resuming normal execution..." << endl << endl;
        }
    }
    return 0;
}
#endif


int FLIR_Viewer::SaveFrame(int seqNo, int width, int height, int strideSize, char * data) {
	FILE *pFile;
	char szFilename[32];
	//static uint8_t * data[MAX_DATA_SIZE];
	//static uint8_t * data1[MAX_DATA_SIZE];
	//int linesize = width;

	int  y;

	sprintf(szFilename, "/tmp/Frame%d.ppm", seqNo);
	pFile=fopen(szFilename, "wb");

	if(pFile==NULL)
		return -1;

	// Write header
	fprintf(pFile, "P6\n%d %d\n255\n", width, height);

	// Write pixel data
	int ret;


	//__ARGBToRGBA(data, linesize, data1, linesize1, width, height);
	fwrite(data, 1, strideSize*height, pFile);
	fclose(pFile);

	return 0;
}

//#define TEST_IT

int FLIR_Viewer::WriteInShm(ImagePtr image, FLIR_info *pInfo) {


	int ret = 0;
#ifndef TEST_IT

		if(sem_wait(pInfo->mutex) < 0) {
			ROS_ERROR("Error in accessing mutex at %d", pInfo->id);
			return -1;
		}
#endif


		if((pInfo->p->len < MAX_DATA_SIZE)) { // don't touch if its ready to terminate

			char *data = (char *)pInfo->p->data;

			int width = image->GetWidth();
			int height = image->GetHeight();
			int stride = image->GetStride();


			sprintf(data, "P6\n%d %d\n255\n", width, height);
			int len = strlen(data);
			data += len;

			//char debug_info[MAX_LEN];



#ifdef _DEBUG
			assert(stride*height < MAX_DATA_SIZE-len);
#endif
			memcpy(data, image->GetData(), stride*height);


			len += stride*height;
			pInfo->p->len = len;
			pInfo->p->width = width;
			pInfo->p->height = height;

			char cur_time[MAX_LEN];
			print_time(cur_time);
			uint64_t timestamp = image->GetTimeStamp();

			snprintf(pInfo->p->timestamp, MAX_LEN, "%s(%" PRId64 ")", cur_time, timestamp);
			//pInfo->p->timestamp



		}

		if(pInfo->p->url[0] == NULL) {
			ROS_INFO("terminate req received from remote system \n");
			ret = 1;


		}


#ifndef TEST_IT
		if(sem_post(pInfo->mutex) < 0) {
			ROS_ERROR("Error in releasing mutex at %d", pInfo->id);
			return -1;
		}
#endif



	return ret;
}
















