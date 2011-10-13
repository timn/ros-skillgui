
/***************************************************************************
 *  skillgui.h - Skill GUI
 *
 *  Created: Mon Nov 03 13:35:34 2008
 *  Copyright  2008-2011  Tim Niemueller [www.niemueller.de]
 *             2010-2011  Carnegie Mellon University
 *             2010-2011  Intel Labs Pittsburgh
 *             2011       SRI International
 *             2011       Columbia University
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
#ifdef HAVE_GCONFMM
#  include <gconfmm.h>
#  define GCONF_PREFIX "/apps/fawkes/skillgui"
#endif

#ifndef USE_ROS
#  include <gui_utils/connection_dispatcher.h>
#  include <interfaces/SkillerInterface.h>
#  include <interfaces/SkillerDebugInterface.h>
#else
#  include <ros/ros.h>
#  include <actionlib/client/action_client.h>
#  include <skiller/Graph.h>
#  include <skiller/SetGraphColored.h>
#  include <skiller/SetGraphDirection.h>
#  include <skiller/ExecSkillAction.h>
#  include <cedar/SystemState.h>
#endif

#include <string>
#include <map>

namespace fawkes {
  class BlackBoard;
  class InterfaceDispatcher;
  class LogView;
  class Throbber;
  class PluginTreeView;
  class NodemonTreeView;
  class RosLogView;
  class RosErrorsTreeView;
}

#ifdef USE_PAPYRUS
class SkillGuiGraphViewport;
#else
class SkillGuiGraphDrawingArea;
#endif

class SkillGuiGtkWindow : public Gtk::Window
{
 public:  
  SkillGuiGtkWindow(BaseObjectType* cobject,
		    const Glib::RefPtr<Gtk::Builder> &builder);
  ~SkillGuiGtkWindow();

 private:
  void read_quickies();
  void write_quickies();
  void add_quickie_button(std::string label);
  void remove_quickie_button(std::string label);
  void run_quickie_dialog(bool edit, std::string label);

  void update_graph(std::string &graph_name, std::string &dotgraph);

  void on_config_changed();
  void on_contexec_toggled();
  void on_skill_changed();
  void on_graphupd_clicked();
  void on_update_disabled();
  void on_recording_toggled();
  void on_exit_clicked();
  void on_graphcolor_toggled();
  void on_followactivestate_toggled();
  void on_graphdir_clicked();
  void on_exec_clicked();
  void on_stop_clicked();
  void on_addquick_clicked();
  void on_rmquick_toggled();
  void on_editquick_toggled();
  void on_quick_clicked(std::string label);
#ifndef USE_ROS
  void on_controller_clicked();
  void on_connection_clicked();
  void close_bb();
  void on_skiller_data_changed();
  void on_skdbg_data_changed();
  void on_agdbg_data_changed();
  void on_connect();
  void on_disconnect();
  void send_graphdir_message(fawkes::SkillerDebugInterface *iface,
			     fawkes::SkillerDebugInterface::GraphDirectionEnum gd);
  void on_graphdir_changed(fawkes::SkillerDebugInterface::GraphDirectionEnum gd);
#else
  void ros_skiller_graphmsg_cb(const skiller::Graph::ConstPtr &msg);
  void ros_agent_graphmsg_cb(const skiller::Graph::ConstPtr &msg);
  void ros_exec_transition_cb(actionlib::ClientGoalHandle<skiller::ExecSkillAction> &gh);
  void ros_exec_feedback_cb(actionlib::ClientGoalHandle<skiller::ExecSkillAction> &gh,
			    const skiller::ExecSkillFeedbackConstPtr &feedback);
  void ros_sysstate_cb(const cedar::SystemState::ConstPtr &msg);

  void on_sysstate_update();
  bool on_sysstate_timeout();
  void on_sysstate_clicked();
  void on_graph_changed();
  void on_exec_goal_transition();
  void update_cedar_lists();
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

  Gtk::Toolbar           *toolbar;
  Gtk::ToolItem          *tb_throbber;
  Gtk::ToolButton        *tb_connection;
  Gtk::ToolButton        *tb_exit;
  Gtk::SeparatorToolItem *tb_sep;
  Gtk::Table             *tab_execmon;
  Gtk::Button            *but_exec;
  Gtk::Button            *but_stop;
  Gtk::Button            *but_clearlog;
  Gtk::ComboBoxEntry     *cbe_skillstring;
  Gtk::ScrolledWindow    *scw_graph;
  Gtk::Notebook          *ntb_tabs;
  Gtk::DrawingArea       *drw_graph;
  Gtk::Toolbar           *tb_fsmgraph;
  Gtk::ToggleToolButton  *tb_skiller;
  Gtk::ToggleToolButton  *tb_agent;
  Gtk::Statusbar         *stb_status;

  Gtk::Table             *tab_quickies;
  Gtk::Button            *but_addquick;
  Gtk::ToggleButton      *but_rmquick;
  Gtk::ToggleButton      *but_editquick;
  Gtk::Dialog            *dlg_addquick;
  Gtk::Entry             *ent_addquick_label;
  Gtk::Entry             *ent_addquick_skillstring;
  Gtk::Dialog            *dlg_editquick;
  Gtk::Entry             *ent_editquick_label;
  Gtk::Entry             *ent_editquick_skillstring;
#ifndef USE_ROS
  Gtk::ComboBoxText      *cb_graphlist;
#endif
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

  Gtk::ToolButton        *tb_graphdir;
  Gtk::ToggleToolButton  *tb_graphcolored;
  Gtk::ToggleToolButton  *tb_followactivestate;

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
  ros::Subscriber __sub_graph_skiller;
  ros::Subscriber __sub_graph_agent;
  ros::Subscriber __sub_sysstate;
  Glib::Dispatcher __sysstate_update;
  cedar::SystemState::ConstPtr __sysstate_msg;
  Glib::Mutex                  __sysstate_mutex;
  ros::ServiceClient __srv_graph_color_skiller;
  ros::ServiceClient __srv_graph_direction_skiller;
  ros::ServiceClient __srv_graph_color_agent;
  ros::ServiceClient __srv_graph_direction_agent;
  Glib::Dispatcher __graph_changed;
  Glib::Dispatcher __exec_transition;
  skiller::Graph::ConstPtr __graph_msg_skiller;
  skiller::Graph::ConstPtr __graph_msg_agent;
  actionlib::ActionClient<skiller::ExecSkillAction> __ac_exec;
  actionlib::ClientGoalHandle<skiller::ExecSkillAction> __gh;
  fawkes::NodemonTreeView  *__trv_nodemon;
  fawkes::RosLogView       *__logview;
  fawkes::RosErrorsTreeView   *__trv_errors;
  Gtk::Label *lab_sysstate;
  Gtk::EventBox *eb_sysstate;
  Gtk::Button *but_sysstate;
  Gtk::TreeView *trv_cedar_nodes;
  Gtk::TreeView *trv_cedar_conns;
  Gtk::EventBox *eb_dlg_cedar_sysstate;
  Gtk::Label *lab_dlg_cedar_sysstate;
  Gtk::Label *lab_dlg_cedar_last_updated;
  actionlib::CommState     __exec_comm_state;
  actionlib::TerminalState __exec_terminal_state;
  Glib::ustring __exec_errmsg;
  Gtk::Dialog *dlg_cedar;
  Glib::RefPtr<Gtk::ListStore> lst_cedar_nodes;
  Glib::RefPtr<Gtk::ListStore> lst_cedar_conns;
  sigc::connection __cedar_timeout;

  class CedarNodeRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    CedarNodeRecord();
    Gtk::TreeModelColumn<Glib::ustring> nodename;
  };

  class CedarTopicConnRecord : public Gtk::TreeModelColumnRecord
  {
   public:
    CedarTopicConnRecord();
    Gtk::TreeModelColumn<Glib::ustring> topic;
    Gtk::TreeModelColumn<Glib::ustring> from;
    Gtk::TreeModelColumn<Glib::ustring> to;
  };

  CedarNodeRecord      __cedar_node_record;
  CedarTopicConnRecord __cedar_conn_record;

#else
  fawkes::SkillerInterface *__skiller_if;
  fawkes::SkillerDebugInterface *__skdbg_if;
  fawkes::SkillerDebugInterface *__agdbg_if;

  fawkes::LogView         *__logview;
  fawkes::PluginTreeView  *__trv_plugins;
#endif
  fawkes::Throbber        *__throbber;

  std::map<std::string, std::string>  __quickies;
  std::map<std::string, Gtk::Button *>  __quickie_buttons;
  int                     __quickie_row;
  int                     __quickie_col;
};

#endif
