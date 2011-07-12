
/***************************************************************************
 *  logview.h - Fawkes log view widget
 *
 *  Created: Mon Nov 02 13:08:29 2008
 *  Copyright  2008-2011  Tim Niemueller [www.niemueller.de]
 *             2011       SRI International
 *             2011       Intel Labs Pittsburgh
 *             2011       Carnegie Mellon University
 *             2011       Columbia University
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __ROS_SKILLGUI_LOGVIEW_H_
#define __ROS_SKILLGUI_LOGVIEW_H_

#include <gtkmm.h>


#include <ros/common.h>
#if ROS_VERSION_MAJOR > 1 || ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR >= 4
#  include <rosgraph_msgs/Log.h>
#  define LOG_MSGTYPE  rosgraph_msgs::Log
#else
#  include <roslib/Log.h>
#  define LOG_MSGTYPE  roslib::Log
#endif

#include <ros/ros.h>

#include <queue>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RosLogView
  : public Gtk::TreeView
{
 public:
  RosLogView(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder> &builder);
  ~RosLogView();

  void append_message(unsigned int log_level, struct timeval t,
		      const char *component, bool is_exception,
		      const char *message);

  void clear();

 private:
  virtual void on_row_inserted(const Gtk::TreeModel::Path& path,
			       const Gtk::TreeModel::iterator& iter);
  virtual void on_expose_notify(GdkEventExpose *event);

  void on_logmsg_received();
  void ros_logmsg_cb(const LOG_MSGTYPE::ConstPtr &msg);

 private:
  class LogRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    LogRecord();
    Gtk::TreeModelColumn<Glib::ustring> loglevel;
    Gtk::TreeModelColumn<Glib::ustring> time;
    Gtk::TreeModelColumn<Glib::ustring> component;
    Gtk::TreeModelColumn<Glib::ustring> message;
    Gtk::TreeModelColumn<Gdk::Color>    foreground;
    Gtk::TreeModelColumn<Gdk::Color>    background;
  };

  LogRecord __record;

  Glib::RefPtr<Gtk::ListStore> __list;

  bool                  __have_recently_added_path;
  Gtk::TreeModel::Path  __recently_added_path;

  ros::NodeHandle __rosnh;
  ros::Subscriber __sub_rosout;

  Glib::Mutex __received_mutex;
  std::queue<LOG_MSGTYPE::ConstPtr> __received_msgs;
  Glib::Dispatcher __received_dispatcher;
};

} // end namespace fawkes


#endif
