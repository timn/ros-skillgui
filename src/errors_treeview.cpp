
/***************************************************************************
 *  nodemon_treeview.cpp - ROS node monitoring tree view
 *
 *  Created: Mon Jul 11 16:34:00 2011
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

#include "errors_treeview.h"
namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class RosErrorsTreeView "nodemon_treeview.h"
 * ROS-specific node monitoring widget for SkillGUI.
 * This widget derives a Gtk::TreeView and provides an easy way to show
 * node monitoring events in a GUI application.
 * @author Tim Niemueller
 */


/** Constructor.
 * Special ctor to be used with Gtk::Builder's get_widget_derived().
 * @param cobject Gtk C object
 * @param builder Gtk::Builder
 */
RosErrorsTreeView::RosErrorsTreeView(BaseObjectType* cobject,
		 const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::TreeView(cobject)
{
  __list = Gtk::ListStore::create(__record);
  __have_recently_added_path = false;

  //__list->signal_row_inserted().connect(sigc::mem_fun(*this, &RosErrorsTreeView::on_row_inserted));
  set_model(__list);
  get_selection()->set_mode(Gtk::SELECTION_NONE);

  append_column("S", __record.state);
  append_column("Time", __record.time);
  int pkgcol = append_column("Package", __record.package);
  int nodetypecol = append_column("Node Type", __record.nodetype);
  int machmsgcol = append_column("Machine Message", __record.machine_message);
  int hummsgcol = append_column("Human Message", __record.human_message);

  // We stored the number of columns, for an index (which starts at 0) we need
  // to subtract 1
  pkgcol -= 1;
  nodetypecol  -= 1;
  machmsgcol -= 1;
  hummsgcol -= 1;

  Glib::ListHandle<Gtk::TreeViewColumn *> columns = get_columns();
  int colnum = -1;
  for (Glib::ListHandle<Gtk::TreeViewColumn *>::iterator c = columns.begin(); c != columns.end(); ++c) {
    ++colnum;
    Gtk::CellRenderer *cell_renderer = (*c)->get_first_cell_renderer();
    Gtk::CellRendererText *text_renderer = dynamic_cast<Gtk::CellRendererText *>(cell_renderer);
    if ( text_renderer ) {
#ifdef GLIBMM_PROPERTIES_ENABLED
      if ( (colnum == pkgcol) || (colnum == nodetypecol) ||
	   (colnum == machmsgcol) || (colnum == hummsgcol) )
      {
	(*c)->set_resizable();
      }
      if (colnum == pkgcol) {
	text_renderer->property_ellipsize().set_value(Pango::ELLIPSIZE_END);
      }

      (*c)->add_attribute(text_renderer->property_background_gdk(), __record.background);
      (*c)->add_attribute(text_renderer->property_foreground_gdk(), __record.foreground);
#else
      (*c)->add_attribute(*text_renderer, "background-gdk", __record.background);
      (*c)->add_attribute(*text_renderer, "foreground-gdk", __record.background);
#endif
    }
  }

  signal_expose_event().connect_notify(sigc::mem_fun(*this, &RosErrorsTreeView::on_expose_notify));
  __received_dispatcher.connect(sigc::mem_fun(*this, &RosErrorsTreeView::on_nodestate_received));
  
  __sub_nodemon = __rosnh.subscribe("/nodemon/state", 10,
				    &RosErrorsTreeView::ros_nodestate_cb, this);
}


/** Destructor. */
RosErrorsTreeView::~RosErrorsTreeView()
{
}


/** Clear all records. */
void
RosErrorsTreeView::clear()
{
  __list->clear();
}


/** Event handler when row inserted.
 * @param path path to element
 * @param iter iterator to inserted element
 */
void
RosErrorsTreeView::on_row_inserted(const Gtk::TreeModel::Path& path,
				   const Gtk::TreeModel::iterator& iter)
{
  Gtk::TreeModel::Path vstart, vend;
  Gtk::TreeModel::Path prev = path;

  bool got_range = get_visible_range(vstart, vend);

  bool has_prev = prev.prev();

  Gtk::TreeModel::Path prevprev = prev;
  bool has_prevprev = prevprev.prev();
  
  if (! got_range ||
      ( has_prev &&
	((vend == prev) ||
	 (__have_recently_added_path && (prev == __recently_added_path)))) ||
      ( has_prevprev && (vend == prevprev)) )
  {
    scroll_to_row(path);

    // the recently added stuff is required if multiple rows are inserted at
    // a time. In this case the widget wasn't redrawn and get_visible_range() does
    // not give the desired result and we have to "advance" it manually
    __have_recently_added_path = true;
    __recently_added_path = path;
  }
}


void
RosErrorsTreeView::on_expose_notify(GdkEventExpose *event)
{
  __have_recently_added_path = false;
}


void
RosErrorsTreeView::ros_nodestate_cb(const nodemon_msgs::NodeState::ConstPtr &msg)
{
  Glib::Mutex::Lock lock(__received_mutex);
  if (__last_msg.find(msg->nodename) != __last_msg.end()) {
    const nodemon_msgs::NodeState::ConstPtr &last = __last_msg[msg->nodename];
    if ( (last->time == msg->time) &&
         (last->machine_message == msg->machine_message) )
    {
      // ignore duplicate messages, e.g. received due to heartbeat
      return;
    }
  }

  __last_msg[msg->nodename] = msg;
  __received_msgs.push(msg);
  lock.release();
  __received_dispatcher();
}


void
RosErrorsTreeView::on_nodestate_received()
{
  Glib::Mutex::Lock lock(__received_mutex);
  const nodemon_msgs::NodeState::ConstPtr msg = __received_msgs.front();
  __received_msgs.pop();
  lock.release();

  struct timeval t;
  t.tv_sec  = msg->time.sec;
  t.tv_usec = msg->time.nsec / 1000;
  append_message(msg->state, t, msg->package, msg->nodetype,
		 msg->machine_message, msg->human_message);
}

/** Append a single message.
 * @param log_level log level
 * @param t time of the message
 * @param component component string for the message
 * @param is_exception true if essage was produced via an exception
 * @param message log message
 */
void
RosErrorsTreeView::append_message(unsigned int state, struct timeval t,
				  const std::string &package,
				  const std::string &nodetype,
				  const std::string &machine_message,
				  const std::string &human_message)
{
  Glib::ustring statestr;
  const char *timestr;
  char* time = NULL;
  Gdk::Color color;
  bool set_foreground = false;
  bool set_background = false;

  switch (state) {

  case nodemon_msgs::NodeState::STARTING:
    statestr = "\u2197";
    color.set("#444444");
    set_foreground = true;
    break;

  case nodemon_msgs::NodeState::RECOVERING:
    statestr = "\u267B";
    color.set("#FF8000");
    set_foreground = true;
    break;

  case nodemon_msgs::NodeState::WARNING:
    statestr = "\u26a0";
    color.set("#FF0000");
    set_foreground = true;
    break;

  case nodemon_msgs::NodeState::ERROR:
    statestr = "\u26A1";
    color.set("#B20000");
    set_foreground = true;
    break;

  case nodemon_msgs::NodeState::FATAL:
    statestr = "\u2620";
    color.set("#BE0000");
    set_foreground = true;
    break;

  case nodemon_msgs::NodeState::STOPPING:
    statestr = "\u2198";
    color.set("#444444");
    set_foreground = true;
    break;

  default: // ignore RUNNING messages
    return;
  }

  struct tm time_tm;
  localtime_r(&(t.tv_sec), &time_tm);
  if (asprintf(&time, "%02d:%02d:%02d.%06ld", time_tm.tm_hour,
	       time_tm.tm_min, time_tm.tm_sec, t.tv_usec) == -1) {
    timestr = "OutOfMemory";
  } else {
    timestr = time;
  }

  Gtk::TreeModel::Row row  = *__list->append();
  row[__record.state]           = statestr;
  row[__record.time]            = timestr;
  row[__record.package]         = package;
  row[__record.nodetype]        = nodetype;
  row[__record.machine_message] = machine_message;
  row[__record.human_message]   = human_message;
  if ( set_foreground )  row[__record.foreground] = color;
  if ( set_background )  row[__record.background] = color;

  Gtk::TreePath path(row);
  on_row_inserted(path, row);

  if (time) free(time);
}

/** @class RosErrorsTreeView::LogRecord <gui_utils/logview.h>
 * TreeView record for RosErrorsTreeView.
 */

/** Constructor. */
RosErrorsTreeView::LogRecord::LogRecord()
{
  add(state);
  add(time);
  add(package);
  add(nodetype);
  add(machine_message);
  add(human_message);
  add(foreground);
  add(background);
}


} // end namespace fawkes
