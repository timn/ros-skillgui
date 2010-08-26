
/***************************************************************************
 *  skillgui.h - Skill GUI
 *
 *  Created: Mon Nov 03 13:35:34 2008
 *  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __TOOLS_SKILLGUI_SKILLGUI_H_
#define __TOOLS_SKILLGUI_SKILLGUI_H_


#include <gtkmm.h>
#include <libglademm/xml.h>
#ifdef HAVE_GCONFMM
#  include <gconfmm.h>
#  define GCONF_PREFIX "/apps/fawkes/skillgui"
#endif

#ifndef USE_ROS
#  include <gui_utils/connection_dispatcher.h>
#  include <interfaces/SkillerInterface.h>
#  include <interfaces/SkillerDebugInterface.h>

namespace fawkes {
  class BlackBoard;
  class InterfaceDispatcher;
  class LogView;
  class Throbber;
  class PluginTreeView;
}
#else
#  include <ros/ros.h>
#  include <skiller/Graph.h>
#endif

#ifdef USE_PAPYRUS
class SkillGuiGraphViewport;
#else
class SkillGuiGraphDrawingArea;
#endif

class SkillGuiGtkWindow : public Gtk::Window
{
 public:  
  SkillGuiGtkWindow(BaseObjectType* cobject, const Glib::RefPtr<Gnome::Glade::Xml> &refxml);
  ~SkillGuiGtkWindow();

 private:
  void update_graph(std::string &graph_name, std::string &dotgraph);

  void on_config_changed();
  void on_contexec_toggled();
  void on_skill_changed();
  void on_graphupd_clicked();
  void on_update_disabled();
  void on_recording_toggled();
  void on_exit_clicked();
#ifndef USE_ROS
  void on_controller_clicked();
  void on_connection_clicked();
  void close_bb();
  void on_exec_clicked();
  void on_stop_clicked();
  void on_skiller_data_changed();
  void on_skdbg_data_changed();
  void on_agdbg_data_changed();
  void on_connect();
  void on_disconnect();
  void on_graphcolor_toggled();
  void on_graphdir_clicked();
  void send_graphdir_message(fawkes::SkillerDebugInterface *iface,
			     fawkes::SkillerDebugInterface::GraphDirectionEnum gd);
  void on_graphdir_changed(fawkes::SkillerDebugInterface::GraphDirectionEnum gd);
#else
  void ros_graphmsg_cb(const skiller::Graph::ConstPtr &msg);
  void on_graph_changed();
#endif

 private:
  class SkillStringRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    SkillStringRecord();
    Gtk::TreeModelColumn<Glib::ustring> skillstring;
  };
  SkillStringRecord __sks_record;


#ifndef USE_ROS
  fawkes::BlackBoard *bb;

  fawkes::ConnectionDispatcher connection_dispatcher;
  fawkes::InterfaceDispatcher  *__skiller_ifd;
  fawkes::InterfaceDispatcher  *__skdbg_ifd;
  fawkes::InterfaceDispatcher  *__agdbg_ifd;
#endif

  Gtk::ToolButton        *tb_connection;
  Gtk::ToolButton        *tb_exit;
  Gtk::Button            *but_exec;
  Gtk::Button            *but_stop;
  Gtk::ToggleButton      *but_continuous;
  Gtk::Button            *but_clearlog;
  Gtk::ComboBoxEntry     *cbe_skillstring;
  Gtk::Label             *lab_status;
  Gtk::Label             *lab_alive;
  Gtk::Label             *lab_continuous;
  Gtk::Label             *lab_skillstring;
  Gtk::Label             *lab_error;
  Gtk::ScrolledWindow    *scw_graph;
  Gtk::Notebook          *ntb_tabs;
  Gtk::DrawingArea       *drw_graph;
  Gtk::ToggleToolButton  *tb_skiller;
  Gtk::ToggleToolButton  *tb_agent;
  Gtk::ComboBoxText      *cb_graphlist;
  Gtk::ToolItem          *tb_graphlist;
  Gtk::ToolButton        *tb_graphsave;
  Gtk::ToolButton        *tb_graphopen;
  Gtk::ToolButton        *tb_graphupd;
  Gtk::ToggleToolButton  *tb_graphrecord;
  Gtk::ToolButton        *tb_controller;
  Gtk::ToolButton        *tb_zoomin;
  Gtk::ToolButton        *tb_zoomout;
  Gtk::ToolButton        *tb_zoomfit;
  Gtk::ToolButton        *tb_zoomreset;

  Gtk::MenuToolButton    *tb_graphdir;
  Gtk::ToggleToolButton  *tb_graphcolored;
  Gtk::MenuItem          *mi_graphdir;
  Gtk::MenuItem          *mi_bottom_top;
  Gtk::MenuItem          *mi_top_bottom;
  Gtk::MenuItem          *mi_left_right;
  Gtk::MenuItem          *mi_right_left;

  Glib::RefPtr<Gtk::ListStore> __sks_list;

#ifdef HAVE_GCONFMM
  Glib::RefPtr<Gnome::Conf::Client> __gconf;
#endif

#ifdef USE_PAPYRUS
  SkillGuiGraphViewport  *pvp_graph;
#else
  SkillGuiGraphDrawingArea *gda;
#endif

#ifdef USE_ROS
  ros::NodeHandle __rosnh;
  ros::Subscriber __sub_graph;
  Glib::Dispatcher __graph_changed;
  std::string __graph_name;
  std::string __graph;
#else
  fawkes::SkillerInterface *__skiller_if;
  fawkes::SkillerDebugInterface *__skdbg_if;
  fawkes::SkillerDebugInterface *__agdbg_if;

  fawkes::LogView         *__logview;
  fawkes::Throbber        *__throbber;
  fawkes::PluginTreeView  *__trv_plugins;
#endif
};

#endif
