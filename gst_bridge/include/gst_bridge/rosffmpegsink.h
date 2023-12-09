/* gst_bridge
 * Copyright (C) 2020-2021 Brett Downing <brettrd@brettrd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _GST_ROSFFMPEGSINK_H_
#define _GST_ROSFFMPEGSINK_H_

#include <gst/video/video-format.h>
#include <gst/base/gstbasesink.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesink.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>


G_BEGIN_DECLS

#define GST_TYPE_ROSFFMPEGSINK   (rosffmpegsink_get_type())
#define GST_ROSFFMPEGSINK(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ROSFFMPEGSINK,Rosffmpegsink))
#define GST_ROSFFMPEGSINK_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ROSFFMPEGSINK,RosffmpegsinkClass))
#define GST_IS_ROSFFMPEGSINK(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ROSFFMPEGSINK))
#define GST_IS_ROSFFMPEGSINK_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ROSFFMPEGSINK))

typedef struct _Rosffmpegsink Rosffmpegsink;
typedef struct _RosffmpegsinkClass RosffmpegsinkClass;

struct _Rosffmpegsink
{
  RosBaseSink parent;

  gchar* pub_topic;
  gchar* frame_id;
  // gchar* encoding; //ffmpeg topic encoding string
  gchar* init_caps; //optional caps override (used for limited apis)

  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr pub;

  int height;
  int width;

  // size_t step;   //bytes per pixel
  // gint endianness;
};

struct _RosffmpegsinkClass
{
  RosBaseSinkClass parent_class;

  // stick member function pointers here
  // along with member function pointers for signal handlers
};

GType rosffmpegsink_get_type (void);

G_END_DECLS

#endif
