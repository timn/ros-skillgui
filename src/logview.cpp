
/***************************************************************************
 *  logview.cpp - Fawkes log view widget
 *
 *  Created: Mon Nov 02 13:19:03 2008
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

#include "logview.h"
namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class RosLogView "logview.h"
 * ROS-specific Log View widget for SkillGUI.
 * This widget derives a Gtk::TreeView and provides an easy way to show
 * log messages in a GUI application.
 * @author Tim Niemueller
 */


/** Constructor.
 * Special ctor to be used with Glade's get_widget_derived().
 * @param cobject Gtk C object
 * @param builder Gtk::Builder
 */
RosLogView::RosLogView(BaseObjectType* cobject,
		 const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::TreeView(cobject)
{
  __list = Gtk::ListStore::create(__record);
  __have_recently_added_path = false;

  //__list->signal_row_inserted().connect(sigc::mem_fun(*this, &RosLogView::on_row_inserted));
  set_model(__list);
  get_selection()->set_mode(Gtk::SELECTION_NONE);

  append_column("Level",     __record.loglevel);
  append_column("Time",      __record.time);
  int compcol = append_column("Component", __record.component);
  int msgcol  = append_column("Message",   __record.message);

  // We stored the number of columns, for an index (which starts at 0) we need
  // to subtract 1
  compcol -= 1;
  msgcol  -= 1;

  Glib::ListHandle<Gtk::TreeViewColumn *> columns = get_columns();
  int colnum = -1;
  for (Glib::ListHandle<Gtk::TreeViewColumn *>::iterator c = columns.begin(); c != columns.end(); ++c) {
    ++colnum;
    Gtk::CellRenderer *cell_renderer = (*c)->get_first_cell_renderer();
    Gtk::CellRendererText *text_renderer = dynamic_cast<Gtk::CellRendererText *>(cell_renderer);
    if ( text_renderer ) {
#ifdef GLIBMM_PROPERTIES_ENABLED
      if ( (colnum == compcol) || (colnum == msgcol) ) {
	(*c)->set_resizable();
      }
      if ( colnum == compcol ) {
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

  signal_expose_event().connect_notify(sigc::mem_fun(*this, &RosLogView::on_expose_notify));
  __signal_message_received.connect(sigc::mem_fun(*this, &RosLogView::on_logmsg_received));

  __sub_rosout = __rosnh.subscribe("/rosout_agg", 10,
				   &RosLogView::ros_logmsg_cb, this);
}


/** Destructor. */
RosLogView::~RosLogView()
{
}


/** Clear all records. */
void
RosLogView::clear()
{
  __list->clear();
}


/** Event handler when row inserted.
 * @param path path to element
 * @param iter iterator to inserted element
 */
void
RosLogView::on_row_inserted(const Gtk::TreeModel::Path& path,
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
RosLogView::on_expose_notify(GdkEventExpose *event)
{
  __have_recently_added_path = false;
}


void
RosLogView::ros_logmsg_cb(const LOG_MSGTYPE::ConstPtr &msg)
{
  __signal_message_received.emit(msg);
}


void
RosLogView::on_logmsg_received(const LOG_MSGTYPE::ConstPtr msg)
{
  struct timeval t;
  t.tv_sec  = msg->header.stamp.sec;
  t.tv_usec = msg->header.stamp.nsec / 1000;
  append_message(msg->level, t, msg->name.c_str(), false, msg->msg.c_str());
}

/** Append a single message.
 * @param log_level log level
 * @param t time of the message
 * @param component component string for the message
 * @param is_exception true if essage was produced via an exception
 * @param message log message
 */
void
RosLogView::append_message(unsigned int log_level, struct timeval t,
			   const char *component, bool is_exception,
			   const char *message)
{
  const char *loglevel;
  const char *timestr;
  char* time = NULL;
  Gdk::Color color;
  bool set_foreground = false;
  bool set_background = false;

  switch ( log_level ) {
  case LOG_MSGTYPE::DEBUG:
    loglevel = "DEBUG";
    color.set_rgb_p(0.4, 0.4, 0.4);
    set_foreground = true;
    break;
  case LOG_MSGTYPE::INFO:
    loglevel = "INFO";
    break;
  case LOG_MSGTYPE::WARN:
    loglevel = "WARN";
    color.set_rgb_p(1.0, 1.0, 0.7);
    set_background = true;
    break;
  case LOG_MSGTYPE::ERROR:
    loglevel = "ERROR";
    // no break here, we take the same color as FATAL!
  case LOG_MSGTYPE::FATAL:
    loglevel = "FATAL";
    color.set_rgb_p(1.0, 0.8, 0.8);
    set_background = true;
    break;
  default:
    loglevel = "NONE?";
    color.set_rgb_p(1.0, 0.0, 0.0);
    set_background = true;
    break;
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
  row[__record.loglevel]   = loglevel;
  row[__record.time]       = timestr;
  row[__record.component]  = component;
  if ( is_exception ) {
    row[__record.message]    = std::string("[EXCEPTION] ") + message;
  } else {
    row[__record.message]    = message;
  }
  if ( set_foreground )  row[__record.foreground] = color;
  if ( set_background )  row[__record.background] = color;

  Gtk::TreePath path(row);
  on_row_inserted(path, row);

  if (time) free(time);
}

/** @class RosLogView::LogRecord <gui_utils/logview.h>
 * TreeView record for RosLogView.
 */

/** Constructor. */
RosLogView::LogRecord::LogRecord()
{
  add(loglevel);
  add(time);
  add(component);
  add(message);
  add(foreground);
  add(background);
}



} // end namespace fawkes
