/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
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

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <diagnostic_updater/publisher.h>
#include <stdio.h>
#include <limits.h>

#define BASESIZE 1000000
int oldwidth = 0;
int oldheight = 0;
uint8_t base[BASESIZE];

#define NUMSKIP 400

FILE *gnuplotfile;

inline void insert(int *max, int val)
{
  if (val < max[0])
    return;

  int i = 1;
  while (val > max[i] && i < NUMSKIP)
  {
    max[i - 1] = max[i];
    i++;
  }

  max[i - 1] = val;
}

void callback(const sensor_msgs::ImageConstPtr& msg)
{
  int width = msg->width;
  int height = msg->height;

  int max[2][NUMSKIP];
  int maxv[NUMSKIP];
  bzero(max, sizeof(max));
  bzero(maxv, sizeof(maxv));

  if (oldwidth != width || oldheight != height)
  {
    oldwidth = width;
    oldheight = height;
    for (int i = 0; i < width * height; i++)
      base[i] = 0;
    fprintf(stderr, "Resetting base. Show a black screen!\n");
  }

  /*assert(width * height < BASESIZE);

  for (int i = 0; i < width * height; i++)
    if (base[i] > msg->uint8_data.data[i])
      base[i] = msg->uint8_data.data[i];
  */
  for (int x = 1; x < width - 1; x++)
  {
    for (int y = 0; y < height - 1; y++)
    {                
      int a = (x + y) & 1;

      //if (a == 0)
      //  continue;
        
      int i = y * width + x;
      int i1 = i + width - 1;
      int i2 = i1 + 2;
      int p = msg->data[i] - base[i];
      int p1 = msg->data[i1] - base[i1];
      int p2 = msg->data[i2] - base[i2];

      int d1 = abs(p - p1);
      int d2 = abs(p - p2);

      insert(max[a], d1);
      insert(max[a], d2);
//        insert(maxv, p);
    }
  }

//  for (int i = 0; i < NUMSKIP; i++)
//    printf("%i ", max[i]);
//  printf(": %i %i %i %i\n", width, height, *max, *maxv);
  printf("%i %i\n", max[0][0], max[1][0]);
  fflush(stdout);
}

int startx, starty, endx, endy;

int bound(int x, int min, int max)
{
  if (x < min)
    return min;
  if (x > max)
    return max;
  return x;
}

void callback2(const sensor_msgs::ImageConstPtr& msg)
{
  int width = msg->width;
  int height = msg->height;
  int sx = bound(startx, 0, width - 3);
  int sy = bound(starty, 0, height - 3);
  int ex = bound(endx, sx + 3, width);
  int ey = bound(endy, sy + 3, height);

//  printf("%i %i %i %i %i %i %i %i\n", startx, sx, starty, sy, endx, ex, endy, ey);

  if (oldwidth != width || oldheight != height)
  {
    oldwidth = width;
    oldheight = height;
    for (int i = 0; i < width * height; i++)
      base[i] = 0;
    fprintf(stderr, "Resetting base. Show a black screen!\n");
  }

  /*assert(width * height < BASESIZE);

  for (int i = 0; i < width * height; i++)
    if (base[i] > msg->uint8_data.data[i])
      base[i] = msg->uint8_data.data[i];
  */
  fprintf(gnuplotfile, "set yrange [-1:255]\n");
  fprintf(gnuplotfile, "set terminal x11\n");
  fprintf(gnuplotfile, "plot \"-\" using 0:1 with lines\n");
  for (int x = sx + 1; x < ex - 1; x++)
  {
    int max = 0;

    for (int y = sy + 1; y < ey - 1; y++)
    {                
      int a = (x + y) & 1;

      if (a == 0)
        continue;
        
      int i = y * width + x;
      int i1 = i + width - 1;
      int i2 = i1 + 2;
      int p = msg->data[i] - base[i];
      int p1 = msg->data[i1] - base[i1];
      int p2 = msg->data[i2] - base[i2];

      int d1 = abs(p - p1);
      int d2 = abs(p - p2);

      if (d1 > max) max = d1;
      if (d2 > max) max = d2;
    }
    fprintf(gnuplotfile, "%i\n", max);
  }
  fprintf(gnuplotfile, "e\n\n");
  fflush(gnuplotfile);
}

int main(int argc, char **argv)
{
  gnuplotfile = popen("gnuplot", "w");

  if (!gnuplotfile)
  {
    perror("popen call failed");
    return -1;
  }
  
  ros::init(argc, argv, "forearm_focus");
  ros::NodeHandle nh;

  nh.param("~startx", startx, -1);
  nh.param("~starty", starty, -1);
  nh.param("~endx", endx, INT_MAX);
  nh.param("~endy", endy, INT_MAX);

  ros::Subscriber sub = nh.subscribe("image", 1, callback2);
  ros::spin();
  return 0;
}
