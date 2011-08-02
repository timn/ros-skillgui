
/***************************************************************************
 *  nodemon_widget.cpp - SkillGUI node monitoring widget
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

#include "nodemon_treeview.h"

#include <gtkmm.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

#define UPDATE_INTERVAL_MSEC 200
#define UPDATE_INTERVAL_SEC (UPDATE_INTERVAL_MSEC / 1000.)
#define TIMEOUT_SEC 5.0
#define DOT_ROS_DIR ".ros"
#define NODEMON_CACHE_FILE "nodemon_cache"


/** @class NodemonTreeView "nodemon_widget.h"
 * Node monitoring display widget.
 * @author Tim Niemueller
 */


/** Constructor. */
NodemonTreeView::NodemonTreeView()
{
  ctor();
}


/** Constructor.
 * Special ctor to be used with Gtk Builder's get_widget_derived().
 * @param cobject Gtk C object
 * @param builder Gtk Builder
 */
NodemonTreeView::NodemonTreeView(BaseObjectType* cobject,
				 const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::TreeView(cobject)
{
  ctor();

  builder->get_widget("tb_nodemon_info", tb_nodemon_info);
  builder->get_widget("tb_nodemon_clear", tb_nodemon_clear);

  tb_nodemon_info->signal_clicked().connect(sigc::mem_fun(*this, &NodemonTreeView::on_info_clicked));
  tb_nodemon_clear->signal_clicked().connect(sigc::mem_fun(*this, &NodemonTreeView::on_clear_clicked));

  __received_dispatcher.connect(sigc::mem_fun(*this, &NodemonTreeView::on_nodestate_received));
}


/** Destructor. */
NodemonTreeView::~NodemonTreeView()
{
}


/** Internal constructor method. */
void
NodemonTreeView::ctor()
{
  __list = Gtk::ListStore::create(__record);
  __have_recently_added_path = false;

  __list->signal_row_inserted().connect(sigc::mem_fun(*this, &NodemonTreeView::on_row_inserted));

  __list->set_sort_column(__record.nodename, Gtk::SORT_ASCENDING);
  set_model(__list);
  get_selection()->set_mode(Gtk::SELECTION_SINGLE);

  set_tooltip_column(__record.tooltip.index());

  //append_column("Time",      __record.time);
  //int compcol = append_column("Component", __record.component);
  //int msgcol  = append_column("Message",   __record.message);
  int statecol = append_column("S", __record.state);
  int compcol  = append_column("Nodename", __record.nodename);

  // We stored the number of columns, for an index (which starts at 0) we need
  // to subtract 1
  statecol -= 1;
  compcol -= 1;

  Glib::ListHandle<Gtk::TreeViewColumn *> columns = get_columns();
  int colnum = -1;
  for (Glib::ListHandle<Gtk::TreeViewColumn *>::iterator c = columns.begin(); c != columns.end(); ++c) {
    ++colnum;
    Gtk::CellRenderer *cell_renderer = (*c)->get_first_cell_renderer();
    Gtk::CellRendererText *text_renderer = dynamic_cast<Gtk::CellRendererText *>(cell_renderer);
    (*c)->set_sizing(Gtk::TREE_VIEW_COLUMN_FIXED);
    if ( text_renderer ) {
#ifdef GLIBMM_PROPERTIES_ENABLED
      if (colnum == compcol) {
	//(*c)->set_resizable();
	text_renderer->property_ellipsize().set_value(Pango::ELLIPSIZE_END);
      } else if (colnum == statecol) {
	(*c)->set_fixed_width(24);
	(*c)->set_alignment(0.45);
	text_renderer->set_fixed_size(-1, 24);
      }

      (*c)->add_attribute(text_renderer->property_background_gdk(), __record.background);
      (*c)->add_attribute(text_renderer->property_foreground_gdk(), __record.foreground);
#else
      (*c)->add_attribute(*text_renderer, "background-gdk", __record.background);
      (*c)->add_attribute(*text_renderer, "foreground-gdk", __record.foreground);
#endif
    }
  }

  set_fixed_height_mode(true);

  signal_expose_event().connect_notify(sigc::mem_fun(*this, &NodemonTreeView::on_expose_notify));
  get_selection()->signal_changed().connect(sigc::mem_fun(*this, &NodemonTreeView::on_selection_changed));

  // Setup and initialize from node cache
  __cache_path = "";
  // try to read cache
  const char *home = getenv("HOME");
  if (home != NULL) {
    __dot_ros_dir = std::string(home) + "/" DOT_ROS_DIR;
    __cache_path  = __dot_ros_dir + "/" NODEMON_CACHE_FILE;

    // just in case...
    mkdir(__dot_ros_dir.c_str(), 0700);

    FILE *f = fopen(__cache_path.c_str(), "r");
    if (f != NULL) {
      char tmp[1024];

      Glib::Mutex::Lock lock(__list_mutex);
      while (fgets(tmp, 1024, f) != NULL) {
	tmp[strlen(tmp) - 1] = '\0'; // remove newline

	Gtk::TreeModel::Row row = *__list->append();
	row[__record.nodename] = tmp;
	row[__record.last_update] = ros::WallTime::now();
      }
      fclose(f);
    }
  }

  set_enabled(true);

  __state_sub = __node_handle.subscribe("/nodemon/state", 10,
					&NodemonTreeView::node_state_cb, this);

}


/** Clear all records. */
void
NodemonTreeView::clear()
{
  Glib::Mutex::Lock lock(__list_mutex);
  __list->clear();
}


void
NodemonTreeView::set_enabled(bool enabled)
{
  if (enabled && ! __timeout_connection.connected()) {
    __timeout_connection = 
      Glib::signal_timeout().connect(sigc::mem_fun(*this,
						   &NodemonTreeView::update),
				     UPDATE_INTERVAL_MSEC);
  } else if (! enabled && __timeout_connection.connected()) {
    __timeout_connection.disconnect();
  }
}

/** Event handler when row inserted.
 * @param path path to element
 * @param iter iterator to inserted element
 */
void
NodemonTreeView::on_row_inserted(const Gtk::TreeModel::Path& path,
				 const Gtk::TreeModel::iterator& iter)
{
  Gtk::TreeModel::Path vstart, vend;
  Gtk::TreeModel::Path prev = path;
  get_visible_range(vstart, vend);
  prev = path;
  if (! get_visible_range(vstart, vend) ||
      ( prev.prev() &&
	((vend == prev) ||
	 (__have_recently_added_path && (__recently_added_path == prev)))) ) {
    scroll_to_row(path);

    // the recently added stuff is required if multiple rows are inserted at
    // a time. In this case the widget wasn't redrawn and get_visible_range() does
    // not give the desired result and we have to "advance" it manually
    __have_recently_added_path = true;
    __recently_added_path = path;
  }
}


void
NodemonTreeView::on_expose_notify(GdkEventExpose *event)
{
  __have_recently_added_path = false;
}


void
NodemonTreeView::on_selection_changed()
{
  Gtk::TreeModel::iterator c = get_selection()->get_selected();
  Gtk::TreeModel::Row row = *c;

  nodemon_msgs::NodeState::ConstPtr last_msg = row[__record.last_msg];
  if (last_msg &&
      ((last_msg->state == nodemon_msgs::NodeState::ERROR) ||
       (last_msg->state == nodemon_msgs::NodeState::FATAL) ||
       (last_msg->state == nodemon_msgs::NodeState::WARNING) ||
       (last_msg->state == nodemon_msgs::NodeState::RECOVERING)))
  {
    tb_nodemon_info->set_sensitive(true);
  } else {
    tb_nodemon_info->set_sensitive(false);
  }
}


void
NodemonTreeView::on_info_clicked()
{
  Gtk::TreeModel::iterator c = get_selection()->get_selected();
  Gtk::TreeModel::Row row = *c;

  nodemon_msgs::NodeState::ConstPtr last_msg = row[__record.last_msg];

  if (last_msg &&
      (last_msg->package != "") &&
      (last_msg->machine_message != ""))
  {
    std::string cmd = "gnome-open http://localhost:10117/errorkb/" +
      last_msg->package + "/" + last_msg->machine_message;
    system(cmd.c_str());
  }
}


void
NodemonTreeView::on_clear_clicked()
{
  Glib::Mutex::Lock lock(__list_mutex);
  __list->clear();
  if (__cache_path != "") {
    FILE *f = fopen(__cache_path.c_str(), "w");
    if (f != NULL) {
      fclose(f);
    }
  }
}


bool
NodemonTreeView::update()
{
  Glib::Mutex::Lock lock(__list_mutex);
  Gtk::TreeModel::Children children = __list->children();
  if (children.empty()) return true;
  Gtk::TreeModel::iterator c;
  for (c = children.begin(); c != children.end(); ++c) {
    Gtk::TreeModel::Row row = *c;
    nodemon_msgs::NodeState::ConstPtr last_msg = row[__record.last_msg];

    ros::WallTime now = ros::WallTime::now();
    const ros::WallTime &last_update = row[__record.last_update];

    bool timed_out =
      ((now - last_update).toSec() > TIMEOUT_SEC);

    bool state_hang = last_msg &&
      ((ros::Time::now() - last_msg->time).toSec() > TIMEOUT_SEC);

    if ( ! last_msg) {
      // no message received, yet
      row[__record.state] = "\u2300";
      row[__record.foreground] = Gdk::Color("#888888");

    } else if (timed_out &&
               (last_msg->state == nodemon_msgs::NodeState::STOPPING))
    {
      // 231b  hourglass, unsupported in common fonts
      row[__record.state] = "\u2198";
      row[__record.foreground] = Gdk::Color("#888888");

    } else if (timed_out) {
      // 231b  hourglass, unsupported in common fonts
      row[__record.state] = "\u231a";
      row[__record.foreground] = Gdk::Color("#B20000");

    } else {
      switch (last_msg->state) {
      case nodemon_msgs::NodeState::STARTING:
	row[__record.state] = "\u2197";
	row[__record.foreground] = Gdk::Color("#444444");
	break;

      case nodemon_msgs::NodeState::RECOVERING:
	row[__record.state] = "\u267B";
	row[__record.foreground] = Gdk::Color("#FF8000");
	break;

      case nodemon_msgs::NodeState::ERROR:
	row[__record.state] = "\u26A1";
	row[__record.foreground] = Gdk::Color("#FF0000");
	break;

      case nodemon_msgs::NodeState::WARNING:
	row[__record.state] = "\u26A0";
	row[__record.foreground] = Gdk::Color("#DD6D00");
	break;

      case nodemon_msgs::NodeState::FATAL:
	row[__record.state] = "\u2620";
	row[__record.foreground] = Gdk::Color("#B20000");
	break;

      case nodemon_msgs::NodeState::STOPPING:
	row[__record.state] = "\u2198";
	row[__record.foreground] = Gdk::Color("#888888");
	break;

      case nodemon_msgs::NodeState::RUNNING:
        if ((now - last_update).toSec() <= UPDATE_INTERVAL_SEC) {
          row[__record.state] = "\u2665";
          row[__record.foreground] = Gdk::Color("#000000");
        } else {
          row[__record.state] = " ";
          row[__record.foreground] = Gdk::Color("#000000");
        }
        break;

      default:
	row[__record.state] = " ";
	row[__record.foreground] = Gdk::Color("#000000");
	break;
      }

      if (state_hang) {
	// 231b  hourglass, unsupported in common fonts
	row[__record.state] = "\u231a";
	row[__record.foreground] = Gdk::Color("#B20000");
      }
    }
    /*
    if (last_msg && ((now - last_update).toSec() <= UPDATE_INTERVAL_SEC))
    {
      if ((last_msg->state == nodemon_msgs::NodeState::FATAL) || state_hang)
      {
	row[__record.state] = "\u2661";
	row[__record.foreground] = Gdk::Color("#B20000");
      } else {
	row[__record.state] = "\u2665";
	row[__record.foreground] = Gdk::Color("#000000");
      }
    }
    */

  }

  return true;
}


void
NodemonTreeView::add_node_to_cache(std::string nodename)
{
  if (__cache_path != "") {
    FILE *f = fopen(__cache_path.c_str(), "a");
    if (f != NULL) {
      fprintf(f, "%s\n", nodename.c_str());
      fclose(f);
    }
  }
}


/** Callback when receiving a node state message.
 * @param msg received message.
 */
void
NodemonTreeView::node_state_cb(const nodemon_msgs::NodeState::ConstPtr &msg)
{
  Glib::Mutex::Lock lock(__received_mutex);
  __received_msgs.push(msg);
  lock.release();
  __received_dispatcher();
}


void
NodemonTreeView::on_nodestate_received()
{
  Glib::Mutex::Lock received_lock(__received_mutex);
  const nodemon_msgs::NodeState::ConstPtr msg = __received_msgs.front();
  __received_msgs.pop();
  received_lock.release();

  Glib::Mutex::Lock lock(__list_mutex);
  Gtk::TreeModel::Children children = __list->children();
  Gtk::TreeModel::iterator c;
  Gtk::TreeModel::Row row;
  for (c = children.begin(); c != children.end(); ++c) {
    Gtk::TreeModel::Row tmprow = *c;
    Glib::ustring nodename = tmprow[__record.nodename];

    if (nodename == msg->nodename) {
      row = tmprow;
      break;
    }
  }

  if ( ! row) {
    row = *__list->append();
    row[__record.nodename] = msg->nodename;
    row[__record.last_update] = ros::WallTime::now();

    add_node_to_cache(msg->nodename);
  }

  nodemon_msgs::NodeState::ConstPtr last_msg = row[__record.last_msg];

  ros::WallTime msg_walltime(msg->time.sec, msg->time.nsec);

  if (((msg->state == nodemon_msgs::NodeState::ERROR) ||
       (msg->state == nodemon_msgs::NodeState::FATAL) ||
       (msg->state == nodemon_msgs::NodeState::WARNING) ||
       (msg->state == nodemon_msgs::NodeState::RECOVERING)) &&
      (msg->machine_message != "") &&
      (! last_msg || ((last_msg->time != msg->time) &&
		      (last_msg->machine_message != msg->machine_message))))
  {
    tm time_tm;
    time_t timet;
    timet = msg_walltime.sec;
    localtime_r(&timet, &time_tm);
           // stat  date  nodename              machine message
    size_t ml = 2 + 21 + msg->nodename.size() + msg->machine_message.size()
      // "[...] "    human message           NULL
      + 3 + msg->human_message.size()  + 1;
    char mstr[ml];

    sprintf(mstr, "%02d:%02d:%02d.%09u [%s] %s", time_tm.tm_hour,
	    time_tm.tm_min, time_tm.tm_sec, msg_walltime.nsec,
	    msg->machine_message.c_str(), msg->human_message.c_str());

    row[__record.tooltip] = mstr;
  }

  row[__record.last_update] = ros::WallTime::now();
  row[__record.last_msg]    = msg;
}


/** @class NodemonTreeView::NodemonRecord "nodemon_widget.h"
 * TreeView record for NodemonTreeView.
 */

/** Constructor. */
NodemonTreeView::NodemonRecord::NodemonRecord()
{
  add(state);
  add(nodename);
  add(tooltip);
  add(foreground);
  add(background);
  add(last_update);
  add(last_msg);
}



} // end namespace fawkes
