/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

//#include "/usr/local/include/libfreenect/libfreenect.h"
#include <ros/ros.h>
#include "/home/pc_willem/ros/libfreenect/include/libfreenect-audio.h"
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <threemxl/micview.h>
#include <threemxl/MultiArrayFloat32.h>
pthread_t freenect_thread;
volatile int die = 0;

static freenect_context* f_ctx;
static freenect_device* f_dev;

typedef struct {
	int32_t* buffers[4];
	int max_samples;
	int current_idx;  // index to the oldest data in the buffer (equivalently, where the next new data will be placed)
	int new_data;
} capture;

capture state;

int paused = 0;

pthread_mutex_t audiobuf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t audiobuf_cond = PTHREAD_COND_INITIALIZER;

int win_h, win_w;

void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown) {
	pthread_mutex_lock(&audiobuf_mutex);
	capture* c = (capture*)freenect_get_user(dev);
	if(num_samples < c->max_samples - c->current_idx) {
		memcpy(&(c->buffers[0][c->current_idx]), mic1, num_samples*sizeof(int32_t));
		memcpy(&(c->buffers[1][c->current_idx]), mic2, num_samples*sizeof(int32_t));
		memcpy(&(c->buffers[2][c->current_idx]), mic3, num_samples*sizeof(int32_t));
		memcpy(&(c->buffers[3][c->current_idx]), mic4, num_samples*sizeof(int32_t));
	} else {
		int first = c->max_samples - c->current_idx;
		int left = num_samples - first;
		memcpy(&(c->buffers[0][c->current_idx]), mic1, first*sizeof(int32_t));
		memcpy(&(c->buffers[1][c->current_idx]), mic2, first*sizeof(int32_t));
		memcpy(&(c->buffers[2][c->current_idx]), mic3, first*sizeof(int32_t));
		memcpy(&(c->buffers[3][c->current_idx]), mic4, first*sizeof(int32_t));
		memcpy(c->buffers[0], &mic1[first], left*sizeof(int32_t));
		memcpy(c->buffers[1], &mic2[first], left*sizeof(int32_t));
		memcpy(c->buffers[2], &mic3[first], left*sizeof(int32_t));
		memcpy(c->buffers[3], &mic4[first], left*sizeof(int32_t));
	}
	c->current_idx = (c->current_idx + num_samples) % c->max_samples;
	c->new_data = 1;
	pthread_cond_signal(&audiobuf_cond);
	pthread_mutex_unlock(&audiobuf_mutex);
}

void* freenect_threadfunc(void* arg) {
	while(!die && freenect_process_events(f_ctx) >= 0) {
		// If we did anything else in the freenect thread, it might go here.
	}
	freenect_stop_audio(f_dev);
	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);
	return NULL;
}

void MicView::sendMicData() {
	pthread_mutex_lock(&audiobuf_mutex);
	while(!state.new_data)
		pthread_cond_wait(&audiobuf_cond, &audiobuf_mutex);
	state.new_data = 0;
	int i;
	int base_idx = state.current_idx;

	// Technically, we should hold the lock until we're done actually drawing
	// the lines, but this is sufficient to ensure that the drawings align
	// provided we don't reallocate buffers.
	pthread_mutex_unlock(&audiobuf_mutex);
	
	// This is kinda slow.  It should be possible to compile each sample
	// window into a glCallList, but that's overly complex.
		
	audio_pub = nh_.advertise<threemxl::MultiArrayFloat32>("/audiokinect", 1);
	threemxl::MultiArrayFloat32 array;

	for( i = 0; i < state.max_samples; i++) {
	  array.mic1.push_back((float)(state.buffers[0][(base_idx + i) % state.max_samples]));        
	};
	for( i = 0; i < state.max_samples; i++) {
	  array.mic2.push_back((float)(state.buffers[1][(base_idx + i) % state.max_samples]));        
	};
	for( i = 0; i < state.max_samples; i++) {
	  array.mic3.push_back((float)(state.buffers[2][(base_idx + i) % state.max_samples]));        
	};
	for( i = 0; i < state.max_samples; i++) {
	  array.mic4.push_back((float)(state.buffers[3][(base_idx + i) % state.max_samples]));        
	};
	array.data = ros::Time::now();
	audio_pub.publish(array);
}

void MicView::spin(){
  ros::Rate looprate(50);
  while(ros::ok()){
    ros::spinOnce();
    sendMicData();
    looprate.sleep();
  }
  
  
}

int main(int argc, char** argv) {
	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}
	freenect_set_log_level(f_ctx, FREENECT_LOG_INFO);
	freenect_select_subdevices(f_ctx, FREENECT_DEVICE_AUDIO);

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);
	if (nr_devices < 1) {
		freenect_shutdown(f_ctx);
		return 1;
	}

	int user_device_number = 0;
	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		return 1;
	}

	state.max_samples = 256 * 60;
	state.current_idx = 0;
	state.buffers[0] = (int32_t*)malloc(state.max_samples * sizeof(int32_t));
	state.buffers[1] = (int32_t*)malloc(state.max_samples * sizeof(int32_t));
	state.buffers[2] = (int32_t*)malloc(state.max_samples * sizeof(int32_t));
	state.buffers[3] = (int32_t*)malloc(state.max_samples * sizeof(int32_t));
	memset(state.buffers[0], 0, state.max_samples * sizeof(int32_t));
	memset(state.buffers[1], 0, state.max_samples * sizeof(int32_t));
	memset(state.buffers[2], 0, state.max_samples * sizeof(int32_t));
	memset(state.buffers[3], 0, state.max_samples * sizeof(int32_t));
	freenect_set_user(f_dev, &state);

	freenect_set_audio_in_callback(f_dev, in_callback);
	freenect_start_audio(f_dev);

	int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		freenect_shutdown(f_ctx);
		return 1;
	}
	ros::init(argc, argv, "kinect_audio");
	MicView mv;
	mv.spin();

	return 0;
}
