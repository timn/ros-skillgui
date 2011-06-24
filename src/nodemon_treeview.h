
/***************************************************************************
 *  nodemon_widget.h - SkillGUI node monitoring widget
 *
 *  Created: Fri Jun 03 10:57:14 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
 *             2011  SRI International
 *             2011  Carnegie Mellon University
 *             2011  Intel Labs Pittsburgh
 *             2011  Columbia University in the City of New York
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

#ifndef __SKILLGUI_NODEMON_WIDGET_H_
#define __SKILLGUI_NODEMON_WIDGET_H_

#include <gtkmm.h>

#include <nodemon_msgs/NodeState.h>
#include <ros/node_handle.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class NodemonTreeView : public Gtk::TreeView
{
 public:
  NodemonTreeView();
  NodemonTreeView(BaseObjectType* cobject,
		  const Glib::RefPtr<Gtk::Builder> &builder);
  ~NodemonTreeView();

  void set_nodehandle(ros::NodeHandle &nh);
  void set_enabled(bool enabled);
  void clear();

 private:
  void ctor();
  bool update();
  void add_node_to_cache(std::string nodename);

  void node_state_cb(const nodemon_msgs::NodeState::ConstPtr &msg);

  virtual void on_row_inserted(const Gtk::TreeModel::Path& path,
			       const Gtk::TreeModel::iterator& iter);
  virtual void on_expose_notify(GdkEventExpose *event);

  void on_selection_changed();
  void on_clear_clicked();
  void on_info_clicked();

 private:
  class NodemonRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    NodemonRecord();
    Gtk::TreeModelColumn<Glib::ustring> state;
    Gtk::TreeModelColumn<Glib::ustring> nodename;
    Gtk::TreeModelColumn<Glib::ustring> tooltip;
    Gtk::TreeModelColumn<Gdk::Color>    foreground;
    Gtk::TreeModelColumn<Gdk::Color>    background;
    Gtk::TreeModelColumn<ros::WallTime> last_update;
    Gtk::TreeModelColumn<nodemon_msgs::NodeState::ConstPtr> last_msg;
  };

  NodemonRecord __record;

  Glib::RefPtr<Gtk::ListStore> __list;

  std::string           __dot_ros_dir;
  std::string           __cache_path;
  bool                  __have_recently_added_path;
  Gtk::TreeModel::Path  __recently_added_path;

  ros::NodeHandle  __node_handle;
  ros::Subscriber  __state_sub;
  sigc::connection __timeout_connection;
  unsigned int __timeout;


  Gtk::ToolButton *tb_nodemon_info;
  Gtk::ToolButton *tb_nodemon_clear;
};

} // end namespace fawkes


#endif
