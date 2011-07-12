
/***************************************************************************
 *  nodemon_treeview.h - Nodemon tree view widget
 *
 *  Created: Fri Jul 01 21:55:00 2011
 *  Copyright  2008-2011  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __ROS_SKILLGUI_NODEMON_TREEVIEW_H_
#define __ROS_SKILLGUI_NODEMON_TREEVIEW_H_

#include <gtkmm.h>
#include <nodemon_msgs/NodeState.h>
#include <ros/ros.h>
#include <queue>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class RosErrorsTreeView
  : public Gtk::TreeView
{
 public:
  RosErrorsTreeView(BaseObjectType* cobject,
		     const Glib::RefPtr<Gtk::Builder> &builder);
  ~RosErrorsTreeView();

  void append_message(unsigned int state, struct timeval t,
		      const std::string &package,
		      const std::string &nodetype,
		      const std::string &machine_message,
		      const std::string &human_message);

  void clear();

 private:
  virtual void on_row_inserted(const Gtk::TreeModel::Path& path,
			       const Gtk::TreeModel::iterator& iter);
  virtual void on_expose_notify(GdkEventExpose *event);

  void on_nodestate_received();
  void ros_nodestate_cb(const nodemon_msgs::NodeState::ConstPtr &msg);

 private:
  class LogRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    LogRecord();
    Gtk::TreeModelColumn<Glib::ustring> state;
    Gtk::TreeModelColumn<Glib::ustring> time;
    Gtk::TreeModelColumn<Glib::ustring> package;
    Gtk::TreeModelColumn<Glib::ustring> nodetype;
    Gtk::TreeModelColumn<Glib::ustring> machine_message;
    Gtk::TreeModelColumn<Glib::ustring> human_message;
    Gtk::TreeModelColumn<Gdk::Color>    foreground;
    Gtk::TreeModelColumn<Gdk::Color>    background;
  };

  LogRecord __record;

  Glib::RefPtr<Gtk::ListStore> __list;

  sigc::signal<void,
    const nodemon_msgs::NodeState::ConstPtr> __signal_message_received;

  bool                  __have_recently_added_path;
  Gtk::TreeModel::Path  __recently_added_path;

  ros::NodeHandle __rosnh;
  ros::Subscriber __sub_nodemon;

  Glib::Mutex __received_mutex;
  std::queue<nodemon_msgs::NodeState::ConstPtr> __received_msgs;
  Glib::Dispatcher __received_dispatcher;
};

} // end namespace fawkes


#endif
