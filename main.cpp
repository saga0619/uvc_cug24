/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (C) 2010-2012 Ken Tossell
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor other contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <thread>
#include "libuvc/libuvc.h"

int seq = 0;

void *image_data = NULL;

volatile bool first_received = false;

void cb(uvc_frame_t *frame, void *ptr)
{
    uvc_frame_t *bgr;
    uvc_error_t ret;
    cv::Mat im;
    bgr = uvc_allocate_frame(frame->width * frame->height * 3);
    if (!bgr)
    {
        printf("unable to allocate bgr frame!");
        return;
    }

    // ret = uvc_uyvy2rgb(frame, bgr);
    // ret = uvc_mjpeg2rgb(frame, bgr);
    // if (ret)
    // {
    //     uvc_perror(ret, "uvc_any2bgr");
    //     uvc_free_frame(bgr);
    //     return;
    // }

    // printf("%d,%d \n", frame->data_bytes, bgr->data_bytes);
    // memcpy(image_data, bgr->data, bgr->width * bgr->height * 3);

    // cv::Mat im = cv::Mat(720, 1280, CV_8UC3);
    if (frame->frame_format == UVC_FRAME_FORMAT_UYVY)
    {
        // printf("uyvy\n");
        im = cv::Mat(720, 1280, CV_8UC2, frame->data);
        cv::cvtColor(im, im, cv::COLOR_YUV2BGR_UYVY);
    }
    else if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG)
    {
        // printf("mjpeg\n");
        ret = uvc_mjpeg2rgb(frame, bgr);
        if (ret)
        {
            uvc_perror(ret, "uvc_any2bgr");
            uvc_free_frame(bgr);
            return;
        }
        im = cv::Mat(720, 1280, CV_8UC3, bgr->data);
        cv::cvtColor(im, im, cv::COLOR_RGB2BGR);
        uvc_free_frame(bgr);
    }

    // im.data = (uchar *)frame->data;

    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);

    cv::imshow("test", im);
    cv::waitKey(1);

    // uvc_free_frame(bgr);

    if (frame->sequence % 60 == 0)
    {
        printf(" * got image %u\n", frame->sequence);
    }

    seq = frame->sequence;

    first_received = true;
}

int main(int argc, char **argv)
{
    uvc_context_t *ctx;
    uvc_error_t res;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;

    cv::Mat im_rgb, im_bgr;

    uvc_frame_format format_type;

    if (argc == 2)
    {
        if (argv[1] == "mjpeg")
        {
            printf("mjpeg selected \n");
            format_type = UVC_FRAME_FORMAT_MJPEG;
        }
        else if (argv[1] == "yuyv")
        {
            printf("yuyv selected\n");
            format_type = UVC_FRAME_FORMAT_YUYV;
        }
        else
        {
            printf("select yuyv or mjpeg\n");
            return true;
        }
    }

    im_rgb.create(720, 1280, CV_8UC3);

    res = uvc_init(&ctx, NULL);

    int total_time = 60;

    if (res < 0)
    {
        uvc_perror(res, "uvc_init");
        return res;
    }

    puts("UVC initialized");

    res = uvc_find_device(
        ctx, &dev,
        0, 0, NULL);

    if (res < 0)
    {
        uvc_perror(res, "uvc_find_device");
    }
    else
    {
        puts("Device found");

        res = uvc_open(dev, &devh);

        if (res < 0)
        {
            uvc_perror(res, "uvc_open");
        }
        else
        {
            puts("Device opened");

            uvc_print_diag(devh, stderr);

            res = uvc_get_stream_ctrl_format_size(
                devh, &ctrl, format_type, 1280, 720, 120);
            //   res = uvc_get_stream_ctrl_format_size(
            //       devh, &ctrl, UVC_FRAME_FORMAT_YUYV, 640, 480, 30
            //   );
            uvc_print_stream_ctrl(&ctrl, stderr);

            if (res < 0)
            {
                uvc_perror(res, "get_mode");
            }
            else
            {
                res = uvc_start_streaming(devh, &ctrl, cb, (void *)12345, 0);

                if (res < 0)
                {
                    uvc_perror(res, "start_streaming");
                }
                else
                {
                    puts("Streaming for 10 seconds...");
                    uvc_error_t resAEMODE = uvc_set_ae_mode(devh, 4);

                    uvc_perror(resAEMODE, "set_ae_mode");

                    uvc_error_t resEXP = uvc_set_exposure_abs(devh, 80);
                    uvc_perror(resEXP, "set_exp_abs");

                    //   int i;
                    //   for (i = 1; i <= 10; i++) {
                    //     /* uvc_error_t resPT = uvc_set_pantilt_abs(devh, i * 20 * 3600, 0); */
                    //     /* uvc_perror(resPT, "set_pt_abs"); */
                    //     uvc_error_t resEXP = uvc_set_exposure_abs(devh, 20 + i * 5);
                    //     uvc_perror(resEXP, "set_exp_abs");

                    //     sleep(1);
                    //   }

                    // printf("test2\n");
                    // while (!first_received)
                    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));

                    // printf("test3\n");
                    // auto t_start = std::chrono::high_resolution_clock::now();

                    // while (true)
                    // {

                    // printf("test4\n");
                    //     im_rgb.data = (uchar *)image_data;

                    //     // cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
                    //     // cv::imshow("test", im_rgb);
                    //     // cv::waitKey(1);

                    //     if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count() > 10000)
                    //     {
                    //         break;
                    //     }
                    // }

                    sleep(total_time);
                    uvc_stop_streaming(devh);
                    puts("Done streaming.");
                }
            }

            uvc_close(devh);
            puts("Device closed");
        }

        uvc_unref_device(dev);
    }

    uvc_exit(ctx);

    float fps_avg = seq / total_time;
    printf("UVC exited with frame : %d, avg : %f fps\n ", seq, fps_avg);

    return 0;
}
