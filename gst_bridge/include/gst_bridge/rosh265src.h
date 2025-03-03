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

#ifndef _GST_ROSH265SRC_H_
#define _GST_ROSH265SRC_H_

#include <gst/video/video-format.h>
#include <gst/base/gstbasesrc.h>
#include <gst_bridge/gst_bridge.h>
#include <gst_bridge/rosbasesrc.h>

//include ROS and ROS message formats
#include <rclcpp/rclcpp.hpp>
#include <h265_image_transport/msg/h265_packet.hpp>
#include <queue>  // std::queue
#include <mutex>  // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

G_BEGIN_DECLS

#define GST_TYPE_ROSH265SRC   (rosh265src_get_type())
#define GST_ROSH265SRC(obj)   (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ROSH265SRC,Rosh265src))
#define GST_ROSH265SRC_CAST(obj)        ((Rosh265src*)obj)
#define GST_ROSH265SRC_CLASS(klass)   (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ROSH265SRC,Rosh265srcClass))
#define GST_ROSH265SRC_GET_CLASS(obj)    (G_TYPE_INSTANCE_GET_CLASS ((obj), GST_TYPE_ROSH265SRC, Rosh265srcClass))
#define GST_IS_ROSH265SRC(obj)   (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ROSH265SRC))
#define GST_IS_ROSH265SRC_CLASS(obj)   (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ROSH265SRC))

typedef struct _Rosh265src Rosh265src;
typedef struct _Rosh265srcClass Rosh265srcClass;

struct _Rosh265src
{
  RosBaseSrc parent;
  gchar* sub_topic;
  gchar* frame_id;
  gchar* encoding;
  gchar* init_caps;

  bool msg_init;

  // XXX this is too much boilerplate.
  size_t msg_queue_max;
  std::queue<h265_image_transport::msg::H265Packet::ConstSharedPtr> msg_queue;
  std::mutex msg_queue_mtx;
  std::condition_variable msg_queue_cv;

  rclcpp::Subscription<h265_image_transport::msg::H265Packet>::SharedPtr sub;

  // int height;
  // int width;
  GstVideoFormat format;
};

struct _Rosh265srcClass
{
  RosBaseSrcClass parent_class;

  // stick member function pointers here
  // along with member function pointers for signal handlers
};

GType rosh265src_get_type (void);

G_END_DECLS

#endif
