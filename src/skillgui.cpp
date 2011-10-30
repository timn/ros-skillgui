
/***************************************************************************
 *  skillgui.cpp - Skill GUI
 *
 *  Created: Mon Nov 03 13:37:33 2008
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

#include "skillgui.h"

#ifdef USE_PAPYRUS
#  include "graph_viewport.h"
#else
#  include "graph_drawing_area.h"
#endif

#ifndef USE_ROS
#  include <utils/system/argparser.h>
#  include <blackboard/remote.h>
#  include <netcomm/fawkes/client.h>

#  include <gui_utils/logview.h>
#  include <gui_utils/throbber.h>
#  include <gui_utils/service_chooser_dialog.h>
#  include <gui_utils/interface_dispatcher.h>
#  include <gui_utils/plugin_tree_view.h>
#else
#  include "throbber.h"
#  include "nodemon_treeview.h"
#  include "logview.h"
#  include "errors_treeview.h"
#  include <functional>
#  define SYSSTATE_TIMEOUT 3
#endif

#include <cstring>
#include <string>

#include <gvc.h>

#ifndef USE_ROS
using namespace fawkes;
#else
using namespace ros;
using namespace actionlib;
#endif

#define ACTIVE_SKILL "Active Skill"

/** @class SkillGuiGtkWindow "skillgui.h"
 * Skill GUI main window.
 * The Skill GUI provides shows Skiller log messages and allows for
 * executing skills.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cobject C base object
 * @param builder Gtk Builder to get widgets from
 */
SkillGuiGtkWindow::SkillGuiGtkWindow(BaseObjectType* cobject,
				     const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::Window(cobject), __rosnh(), __ac_exec(__rosnh, "/skiller/exec"),
    __exec_comm_state(actionlib::CommState::DONE), __exec_terminal_state(actionlib::TerminalState::LOST)
{
#ifndef USE_ROS
  bb = NULL;
  __skiller_if = NULL;
  __skdbg_if = NULL;
  __agdbg_if = NULL;
#endif

  __quickie_row = 0;
  __quickie_col = 1;

#ifdef HAVE_GCONFMM
  __gconf = Gnome::Conf::Client::get_default_client();
  __gconf->add_dir(GCONF_PREFIX);
#endif

#ifndef USE_ROS
  builder->get_widget_derived("trv_plugins",  __trv_plugins);
#else
  builder->get_widget_derived("trv_nodemon", __trv_nodemon);
#endif
  builder->get_widget_derived("trv_log", __logview);
  builder->get_widget_derived("trv_errors", __trv_errors);
  builder->get_widget_derived("img_throbber", __throbber);
  builder->get_widget("tb_throbber", tb_throbber);
  builder->get_widget("toolbar", toolbar);
  builder->get_widget("tb_connection", tb_connection);
  builder->get_widget("tb_sep", tb_sep);
  builder->get_widget("but_clearlog", but_clearlog);
  builder->get_widget("tb_exit", tb_exit);
  builder->get_widget("but_exec", but_exec);
  builder->get_widget("but_stop", but_stop);
  builder->get_widget("stb_status", stb_status);
  builder->get_widget("scw_graph", scw_graph);
  //builder->get_widget("drw_graph", drw_graph);
  builder->get_widget("ntb_tabs", ntb_tabs);
  builder->get_widget("tb_fsmgraph", tb_fsmgraph);
  builder->get_widget("tb_skiller", tb_skiller);
  builder->get_widget("tb_agent", tb_agent);
  builder->get_widget("tb_graphlist", tb_graphlist);
  builder->get_widget("tb_controller", tb_controller);
  builder->get_widget("tb_graphsave", tb_graphsave);
  builder->get_widget("tb_graphopen", tb_graphopen);
  builder->get_widget("tb_graphupd", tb_graphupd);
  builder->get_widget("tb_graphrecord", tb_graphrecord);
  builder->get_widget("tb_zoomin", tb_zoomin);
  builder->get_widget("tb_zoomout", tb_zoomout);
  builder->get_widget("tb_zoomfit", tb_zoomfit);
  builder->get_widget("tb_zoomreset", tb_zoomreset);
  builder->get_widget("tb_graphdir", tb_graphdir);
  builder->get_widget("tb_graphcolored", tb_graphcolored);
  builder->get_widget("tb_followactivestate", tb_followactivestate);
  builder->get_widget("tab_execmon", tab_execmon);

  builder->get_widget("tab_quickies", tab_quickies);
  builder->get_widget("but_addquick", but_addquick);
  builder->get_widget("but_rmquick", but_rmquick);
  builder->get_widget("but_editquick", but_editquick);
  builder->get_widget("dlg_addquick", dlg_addquick);
  builder->get_widget("ent_addquick_label", ent_addquick_label);
  builder->get_widget("ent_addquick_skillstring", ent_addquick_skillstring);
  builder->get_widget("dlg_editquick", dlg_editquick);
  builder->get_widget("ent_editquick_label", ent_editquick_label);
  builder->get_widget("ent_editquick_skillstring", ent_editquick_skillstring);

  builder->get_widget("lab_sysstate", lab_sysstate);
  builder->get_widget("eb_sysstate", eb_sysstate);
  builder->get_widget("but_sysstate", but_sysstate);
  builder->get_widget("lab_dlg_cedar_sysstate", lab_dlg_cedar_sysstate);
  builder->get_widget("eb_dlg_cedar_sysstate", eb_dlg_cedar_sysstate);
  builder->get_widget("lab_dlg_cedar_last_updated", lab_dlg_cedar_last_updated);

  builder->get_widget("dlg_cedar", dlg_cedar);
  builder->get_widget("trv_cedar_nodes", trv_cedar_nodes);
  builder->get_widget("trv_cedar_conns", trv_cedar_conns);
  lst_cedar_nodes = Gtk::ListStore::create(__cedar_node_record);
  lst_cedar_conns = Gtk::ListStore::create(__cedar_conn_record);
  trv_cedar_nodes->set_model(lst_cedar_nodes);
  trv_cedar_conns->set_model(lst_cedar_conns);
  trv_cedar_nodes->get_selection()->set_mode(Gtk::SELECTION_NONE);
  trv_cedar_conns->get_selection()->set_mode(Gtk::SELECTION_NONE);

  trv_cedar_nodes->append_column("Node", __cedar_node_record.nodename);

  trv_cedar_conns->append_column("Topic", __cedar_conn_record.topic);
  trv_cedar_conns->append_column("From Node", __cedar_conn_record.from);
  trv_cedar_conns->append_column("To Node", __cedar_conn_record.to);

  // This hack is required because setting expanding in Glade-3 does not
  // have the same effect of right-aligning the throbber
  Gtk::SeparatorToolItem *spacesep;
  builder->get_widget("tb_spacesep", spacesep);
  spacesep->set_expand();
  builder->get_widget("tb_fsmgraph_spacesep", spacesep);
  spacesep->set_expand();

  cbe_skillstring = Gtk::manage(new Gtk::ComboBoxEntry());
  tab_execmon->attach(*cbe_skillstring, 0, 1, 1, 2);
  cbe_skillstring->show();

  // This should be in the Glade file, but is not restored for some reason
  tb_graphsave->set_homogeneous(false);
  tb_graphopen->set_homogeneous(false);
  tb_graphupd->set_homogeneous(false);
  tb_graphrecord->set_homogeneous(false);
  tb_zoomin->set_homogeneous(false);
  tb_zoomout->set_homogeneous(false);
  tb_zoomfit->set_homogeneous(false);
  tb_zoomreset->set_homogeneous(false);
  tb_graphdir->set_homogeneous(false);
  tb_graphcolored->set_homogeneous(false);
  tb_followactivestate->set_homogeneous(false);

  __sks_list = Gtk::ListStore::create(__sks_record);
  cbe_skillstring->set_model(__sks_list);
  cbe_skillstring->set_text_column(__sks_record.skillstring);

#ifndef USE_ROS
  __trv_plugins->set_network_client(connection_dispatcher.get_client());
#  ifdef HAVE_GCONFMM
  __trv_plugins->set_gconf_prefix(GCONF_PREFIX);
#  endif
#else
  //ntb_tabs->remove_page(0);
  ntb_tabs->remove_page(-1);
  toolbar->hide();
  but_exec->set_sensitive(true);
  but_stop->set_sensitive(true);
  cbe_skillstring->set_sensitive(true);
  toolbar->remove(*tb_throbber);
  tb_fsmgraph->append(*tb_throbber);
#endif

  if (ntb_tabs->get_n_pages() == 1) {
    ntb_tabs->set_show_tabs(false);
  }

#ifdef USE_PAPYRUS
  pvp_graph = Gtk::manage(new SkillGuiGraphViewport());
  scw_graph->add(*pvp_graph);
  pvp_graph->show();
#else
  gda = Gtk::manage(new SkillGuiGraphDrawingArea());
  scw_graph->add(*gda);
  gda->show();
#endif

#ifndef USE_ROS
  cb_graphlist = Gtk::manage(new Gtk::ComboBoxText());
  cb_graphlist->append_text(ACTIVE_SKILL);
  cb_graphlist->set_active_text(ACTIVE_SKILL);
  tb_graphlist->add(*cb_graphlist);
  cb_graphlist->show();
#endif

  //ntb_tabs->set_current_page(1);

  //connection_dispatcher.signal_connected().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_connect));
  //connection_dispatcher.signal_disconnected().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_disconnect));

  //tb_connection->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_connection_clicked));
  //tb_controller->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_controller_clicked));
  but_exec->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_exec_clicked));
  cbe_skillstring->get_entry()->signal_activate().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_exec_clicked));
  tb_exit->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_exit_clicked));
  but_stop->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_stop_clicked));
  //but_clearlog->signal_clicked().connect(sigc::mem_fun(*__logview, &LogView::clear));
#ifdef USE_ROS
  tb_skiller->signal_toggled().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_graph_changed));
  tb_agent->signal_toggled().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_graph_changed));
#else
  tb_skiller->signal_toggled().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_skdbg_data__changed));
  tb_agent->signal_toggled().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_agdbg_data__changed));
#endif
#ifndef USE_ROS
  tb_skiller->signal_toggled().connect(sigc::bind(sigc::mem_fun(*cb_graphlist, &Gtk::ComboBoxText::set_sensitive),true));
  tb_agent->signal_toggled().connect(sigc::bind(sigc::mem_fun(*cb_graphlist, &Gtk::ComboBoxText::set_sensitive),false));
  //cb_graphlist->signal_changed().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_skill_changed));
  //tb_graphupd->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_graphupd_clicked));
  /*
  mi_top_bottom->signal_activate().connect(sigc::bind(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_graphdir_changed), SkillerDebugInterface::GD_TOP_BOTTOM));
  mi_bottom_top->signal_activate().connect(sigc::bind(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_graphdir_changed), SkillerDebugInterface::GD_BOTTOM_TOP));
  mi_left_right->signal_activate().connect(sigc::bind(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_graphdir_changed), SkillerDebugInterface::GD_LEFT_RIGHT));
  mi_right_left->signal_activate().connect(sigc::bind(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_graphdir_changed), SkillerDebugInterface::GD_RIGHT_LEFT));
  */
#else
  __graph_changed.connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_graph_changed));
  __exec_transition.connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_exec_goal_transition));
  __sysstate_update.connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_sysstate_update));
  but_sysstate->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_sysstate_clicked));
#endif
  tb_graphdir->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_graphdir_clicked));
  tb_graphcolored->signal_toggled().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_graphcolor_toggled));
  tb_followactivestate->signal_toggled().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_followactivestate_toggled));
#ifdef USE_PAPYRUS
  tb_graphsave->signal_clicked().connect(sigc::mem_fun(*pvp_graph, &SkillGuiGraphViewport::save));
  tb_zoomin->signal_clicked().connect(sigc::mem_fun(*pvp_graph, &SkillGuiGraphViewport::zoom_in));
  tb_zoomout->signal_clicked().connect(sigc::mem_fun(*pvp_graph, &SkillGuiGraphViewport::zoom_out));
  tb_zoomfit->signal_clicked().connect(sigc::mem_fun(*pvp_graph, &SkillGuiGraphViewport::zoom_fit));
  tb_zoomreset->signal_clicked().connect(sigc::mem_fun(*pvp_graph, &SkillGuiGraphViewport::zoom_reset));
#else
  tb_graphsave->signal_clicked().connect(sigc::mem_fun(*gda, &SkillGuiGraphDrawingArea::save));
  tb_graphopen->signal_clicked().connect(sigc::mem_fun(*gda, &SkillGuiGraphDrawingArea::open));
  tb_zoomin->signal_clicked().connect(sigc::mem_fun(*gda, &SkillGuiGraphDrawingArea::zoom_in));
  tb_zoomout->signal_clicked().connect(sigc::mem_fun(*gda, &SkillGuiGraphDrawingArea::zoom_out));
  tb_zoomfit->signal_clicked().connect(sigc::mem_fun(*gda, &SkillGuiGraphDrawingArea::zoom_fit));
  tb_zoomreset->signal_clicked().connect(sigc::mem_fun(*gda, &SkillGuiGraphDrawingArea::zoom_reset));
  tb_graphrecord->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_recording_toggled));
  gda->signal_update_disabled().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_update_disabled));
#endif

#ifdef USE_ROS
  __sub_graph_skiller = __rosnh.subscribe("/skiller/graph", 10,
					  &SkillGuiGtkWindow::ros_skiller_graphmsg_cb, this);
  __sub_graph_agent  = __rosnh.subscribe("/luaagent/graph", 10,
					 &SkillGuiGtkWindow::ros_agent_graphmsg_cb, this);
  __sub_sysstate  = __rosnh.subscribe("cedar/system_state", 10,
                                      &SkillGuiGtkWindow::ros_sysstate_cb, this);

  __srv_graph_color_skiller = __rosnh.serviceClient<skiller::SetGraphColored>("/skiller/graph/set_colored");
  __srv_graph_direction_skiller = __rosnh.serviceClient<skiller::SetGraphDirection>("/skiller/graph/set_direction");
  __srv_graph_color_agent = __rosnh.serviceClient<skiller::SetGraphColored>("/luaagent/graph/set_colored");
  __srv_graph_direction_agent = __rosnh.serviceClient<skiller::SetGraphDirection>("/luaagent/graph/set_direction");

  on_sysstate_timeout();
#endif

#ifdef HAVE_GCONFMM
  __gconf->signal_value_changed().connect(sigc::hide(sigc::hide(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_config_changed))));
  on_config_changed();
  read_quickies();
  but_addquick->signal_clicked().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_addquick_clicked));
  but_rmquick->signal_toggled().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_rmquick_toggled));
  but_editquick->signal_toggled().connect(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_editquick_toggled));
#else
  but_addquick->hide();
  but_rmquick->hide();
  but_editquick->hide();
#endif

}


/** Destructor. */
SkillGuiGtkWindow::~SkillGuiGtkWindow()
{
#ifdef HAVE_GCONFMM
  __gconf->remove_dir(GCONF_PREFIX);
#endif
#ifndef USE_ROS
  __logview->set_client(NULL);
  __trv_plugins->set_network_client(NULL);
  __rosnh.shutdown();
#endif
}


void
SkillGuiGtkWindow::on_config_changed()
{
#ifdef HAVE_GCONFMM
  Gnome::Conf::SListHandle_ValueString l(__gconf->get_string_list(GCONF_PREFIX"/command_history"));

  __sks_list->clear();
  for (Gnome::Conf::SListHandle_ValueString::const_iterator i = l.begin(); i != l.end(); ++i) {
    Gtk::TreeModel::Row row  = *__sks_list->append();
    row[__sks_record.skillstring] = *i;    
  }

#ifdef GLIBMM_EXCEPTIONS_ENABLED
  bool colored           = __gconf->get_bool(GCONF_PREFIX"/graph_colored");
  bool followactivestate = __gconf->get_bool(GCONF_PREFIX"/follow_active_state");
#else
  std::auto_ptr<Glib::Error> error;
  bool colored           = __gconf->get_bool(GCONF_PREFIX"/graph_colored", error);
  bool followactivestate = __gconf->get_bool(GCONF_PREFIX"/follow_active_state", error);
#endif
  tb_graphcolored->set_active(colored);
  tb_followactivestate->set_active(followactivestate);
  on_followactivestate_toggled();
#endif
}


void
SkillGuiGtkWindow::on_skill_changed()
{
#ifndef USE_ROS
  Glib::ustring skill = cb_graphlist->get_active_text();
  if ( skill == ACTIVE_SKILL ) {
    skill = "ACTIVE";
  }
  SkillerDebugInterface::SetGraphMessage *sgm = new SkillerDebugInterface::SetGraphMessage(skill.c_str());
  __skdbg_if->msgq_enqueue(sgm);
#endif
}


void
SkillGuiGtkWindow::on_exit_clicked()
{
  Gtk::Main::quit();
}

void
SkillGuiGtkWindow::on_exec_clicked()
{
  //Glib::ustring sks = cbe_skillstring->get_active_text();
  Glib::ustring sks = "";
  if ( cbe_skillstring->get_active_row_number() == -1 ) {
    Gtk::Entry *entry = cbe_skillstring->get_entry();
    sks = entry->get_text();
  } else {
    Gtk::TreeModel::Row row = *cbe_skillstring->get_active();
    row.get_value(cbe_skillstring->get_text_column(), sks);
  }

  if ( sks != "" ) {
    __throbber->set_timeout(80);

#ifndef USE_ROS
    if (__skiller_if && __skiller_if->is_valid() && __skiller_if->has_writer() &&
	__skiller_if->exclusive_controller() == __skiller_if->serial()) {

      //if ( but_continuous->get_active() ) {
      SkillerInterface::ExecSkillContinuousMessage *escm = new SkillerInterface::ExecSkillContinuousMessage(sks.c_str());
      __skiller_if->msgq_enqueue(escm);
      /*
      } else {
	SkillerInterface::ExecSkillMessage *esm = new SkillerInterface::ExecSkillMessage(sks.c_str());
	__skiller_if->msgq_enqueue(esm);
      }
      */

    } else {
      Gtk::MessageDialog md(*this, "The exclusive control over the skiller has "
			    "not been acquired yet and skills cannot be executed",
			    /* markup */ false,
			    Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			    /* modal */ true);
      md.set_title("Skill Execution Failure");
      md.run();
    }

#else // ROS
    printf("Starting execution of %s\n", sks.c_str());
    skiller::ExecSkillGoal goal;
    goal.skillstring = sks;
    //__ac_exec.cancelGoalsAtAndBeforeTime(ros::Time::now());
    __gh.reset();
    __gh = __ac_exec.sendGoal(goal,
			      boost::bind(&SkillGuiGtkWindow::ros_exec_transition_cb, this, _1),
			      boost::bind(&SkillGuiGtkWindow::ros_exec_feedback_cb, this, _1, _2));
#endif
    Gtk::TreeModel::Children children = __sks_list->children();
    if ( ! children.empty() ) {
      size_t num = 0;
      Gtk::TreeIter i = children.begin();
      while ( i != children.end() ) {
        if ( num >= 9 ) {
          i = __sks_list->erase(i);
        } else {
          Gtk::TreeModel::Row row = *i;
          if (row[__sks_record.skillstring] == sks) {
            i = __sks_list->erase(i);
          } else {
            ++num;
            ++i;
          }
        }
      }
    }
    Gtk::TreeModel::Row row  = *__sks_list->prepend();
    row[__sks_record.skillstring] = sks;
      
    std::list<Glib::ustring> l;
    for (Gtk::TreeIter i = children.begin(); i != children.end(); ++i) {
      Gtk::TreeModel::Row row = *i;
      l.push_back(row[__sks_record.skillstring]);
    }

#ifdef HAVE_GCONFMM
    __gconf->set_string_list(GCONF_PREFIX"/command_history", l);
#endif
  }
}

void
SkillGuiGtkWindow::on_stop_clicked()
{
#ifndef USE_ROS
  if ( bb && __skiller_if && __skiller_if->is_valid() && __skiller_if->has_writer() ) {
    SkillerInterface::StopExecMessage *sem = new SkillerInterface::StopExecMessage();
    __skiller_if->msgq_enqueue(sem);
  }
#else
  if (! __gh.isExpired()) {
    CommState comm_state = __gh.getCommState();
    if (comm_state.state_ != CommState::DONE) {
      __gh.cancel();
    }
  }
#endif
}


void
SkillGuiGtkWindow::run_quickie_dialog(bool edit, std::string label)
{
  Gtk::Dialog *dlg = dlg_addquick;
  Gtk::Entry *ent_label = ent_addquick_label;
  Gtk::Entry *ent_skillstring = ent_addquick_skillstring;
  if (edit) {
    dlg = dlg_editquick;
    ent_label = ent_editquick_label;
    ent_skillstring = ent_editquick_skillstring;    

    ent_label->set_text(label);
    if (__quickies.find(label) != __quickies.end()) {
      ent_skillstring->set_text(__quickies[label]);
    }
    ent_skillstring->grab_focus();
  } else {
    ent_label->set_text("");
    ent_skillstring->set_text(cbe_skillstring->get_entry()->get_text());
    ent_label->grab_focus();
  }

  dlg->set_transient_for(*this);

  int rv = dlg->run();
  dlg->hide();

  if (rv == 1) {
    if (edit) {
      but_editquick->set_active(false);
    }

    std::string new_label = ent_label->get_text();
    std::string skillstring = ent_skillstring->get_text();
    if (new_label == "" || skillstring == "") {
      Gtk::MessageDialog md(*this,
                            "Label and skill string may not be empty.",
                            /* markup */ false,
                            Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
                            /* modal */ true);
      md.set_title("Missing Data");
      md.run();
      return;
    } else if (new_label.find("|") != std::string::npos) {
      Gtk::MessageDialog md(*this,
                            "The label may not contain the pipe (|) symbol.",
                            /* markup */ false,
                            Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
                            /* modal */ true);
      md.set_title("Invalid Label");
      md.run();
      return;
    } else if (! edit && (__quickies.find(new_label) != __quickies.end())) {
      Gtk::MessageDialog md(*this,
                            "A quick access button with that label already "
                            "exists. Should it be replaced with the new "
                            "skill string action?",
                            /* markup */ false,
                            Gtk::MESSAGE_ERROR, Gtk::BUTTONS_YES_NO,
                            /* modal */ true);
      md.set_title("Label Exists");
      if (md.run() == Gtk::RESPONSE_NO) {
        return;
      }
    }

    if (__quickie_buttons.find(label) != __quickie_buttons.end()) {
      remove_quickie_button(label);
    }
    __quickies[new_label] = skillstring;
    add_quickie_button(new_label);
    write_quickies();
  }
}

void
SkillGuiGtkWindow::on_addquick_clicked()
{
  run_quickie_dialog(false, "");
}

void
SkillGuiGtkWindow::on_quick_clicked(std::string label)
{
  if (__quickies.find(label) != __quickies.end()) {
    if (but_rmquick->get_active()) {
      remove_quickie_button(label);
      write_quickies();
      but_rmquick->set_active(false);
    } else if (but_editquick->get_active()) {
      run_quickie_dialog(true, label);
    } else {
      cbe_skillstring->get_entry()->set_text(__quickies[label]);
      but_exec->clicked();
    }
  }
}


void
SkillGuiGtkWindow::on_rmquick_toggled()
{
  if (but_rmquick->get_active()) {
    but_editquick->set_active(false);
  }
}

void
SkillGuiGtkWindow::on_editquick_toggled()
{
  if (but_editquick->get_active()) {
    but_rmquick->set_active(false);
  }
}

void
SkillGuiGtkWindow::read_quickies()
{
#ifdef HAVE_GCONFMM
  __quickies.clear();

  Gnome::Conf::SListHandle_ValueString
    l(__gconf->get_string_list(GCONF_PREFIX"/quickies"));

  Gnome::Conf::SListHandle_ValueString::const_iterator i = l.begin();
  for (; i != l.end(); ++i) {
    std::string::size_type pos = (*i).find("|");
    if (pos != std::string::npos) {
      std::string label = (*i).substr(0, pos);
      std::string skillstring = (*i).substr(pos+1);

      __quickies[label] = skillstring;
      add_quickie_button(label);
    }
  }
#endif
}


void
SkillGuiGtkWindow::write_quickies()
{
#ifdef HAVE_GCONFMM
  std::list<Glib::ustring> l;
  std::map<std::string, std::string>::iterator i;
  for (i = __quickies.begin(); i != __quickies.end(); ++i) {
    std::string s = i->first + "|" + i->second;
    l.push_back(s);
  }

  __gconf->set_string_list(GCONF_PREFIX"/quickies", l);
#endif
}


void
SkillGuiGtkWindow::add_quickie_button(std::string label)
{
  if (__quickies.find(label) == __quickies.end())  return;

  if (__quickie_buttons.find(label) != __quickie_buttons.end()) {
    delete __quickie_buttons[label];
  }
  Gtk::Button *button = Gtk::manage(new Gtk::Button(label));

  button->set_tooltip_text(__quickies[label]);

  tab_quickies->attach(*button,
                       __quickie_col,   __quickie_col + 1,
                       __quickie_row,   __quickie_row + 1 );

  __quickie_col += 1;

  if (__quickie_col == 6) {
    __quickie_col  = 0;
    __quickie_row += 1;
    tab_quickies->resize(__quickie_row, 6);
  }

  button->show();
  button->signal_clicked().connect(sigc::bind(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_quick_clicked), label));
  __quickie_buttons[label] = button;
}


void
SkillGuiGtkWindow::remove_quickie_button(std::string label)
{
  if (__quickies.find(label) != __quickies.end()) {
    std::map<std::string, Gtk::Button *>::iterator i;
    for (i = __quickie_buttons.begin(); i != __quickie_buttons.end(); ++i) {
      delete i->second;
    }
    __quickie_buttons.clear();
    __quickies.erase(label);
    tab_quickies->resize(1, 6);
    __quickie_col = 1;
    __quickie_row = 0;

    std::map<std::string, std::string>::iterator j;
    for (j = __quickies.begin(); j != __quickies.end(); ++j) {
      add_quickie_button(j->first);
    }
  }
}


#ifndef USE_ROS

/** Event handler for connection button. */
void
SkillGuiGtkWindow::on_connection_clicked()
{
  if ( ! connection_dispatcher.get_client()->connected() ) {
    ServiceChooserDialog ssd(*this, connection_dispatcher.get_client());
    ssd.run_and_connect();
  } else {
    connection_dispatcher.get_client()->disconnect();
  } 
}

void
SkillGuiGtkWindow::on_controller_clicked()
{
  if (__skiller_if && __skiller_if->is_valid() && __skiller_if->has_writer() &&
      __skiller_if->exclusive_controller() == __skiller_if->serial()) {
    // we are exclusive controller, release control
    SkillerInterface::ReleaseControlMessage *rcm = new SkillerInterface::ReleaseControlMessage();
    __skiller_if->msgq_enqueue(rcm);
  } else if (__skiller_if && __skiller_if->is_valid() && __skiller_if->has_writer() &&
	     __skiller_if->exclusive_controller() == 0) {
    // there is no exclusive controller, try to acquire control
    SkillerInterface::AcquireControlMessage *acm = new SkillerInterface::AcquireControlMessage();
    __skiller_if->msgq_enqueue(acm);
  } else {
    Gtk::MessageDialog md(*this,
			  "Another component already acquired the exclusive "
			  "control for the Skiller; not acquiring exclusive control.",
			  /* markup */ false,
			  Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			  /* modal */ true);
    md.set_title("Control Acquisition Failed");
    md.run();
  }
}

void
SkillGuiGtkWindow::close_bb()
{
  if ( bb ) {
    bb->unregister_listener(__skiller_ifd);
    bb->unregister_listener(__skdbg_ifd);
    bb->unregister_listener(__agdbg_ifd);
    delete __skiller_ifd;
    delete __skdbg_ifd;
    delete __agdbg_ifd;
    if ( __skiller_if && __skiller_if->is_valid() && __skiller_if->has_writer() &&
	 (__skiller_if->exclusive_controller() == __skiller_if->serial()) ) {
      SkillerInterface::ReleaseControlMessage *rcm = new SkillerInterface::ReleaseControlMessage();
      __skiller_if->msgq_enqueue(rcm);
    }
    bb->close(__skiller_if);
    bb->close(__skdbg_if);
    bb->close(__agdbg_if);
    delete bb;
    __skiller_if = NULL;
    __skdbg_if = NULL;
    __agdbg_if = NULL;
    bb = NULL;
  }
}

/** Event handler for connected event. */
void
SkillGuiGtkWindow::on_connect()
{
  try {
    if ( ! bb ) {
      bb           = new RemoteBlackBoard(connection_dispatcher.get_client());
      __skiller_if = bb->open_for_reading<SkillerInterface>("Skiller");
      __skdbg_if   = bb->open_for_reading<SkillerDebugInterface>("Skiller");
      __agdbg_if   = bb->open_for_reading<SkillerDebugInterface>("LuaAgent");
      on_skiller_data_changed();
      on_skdbg_data_changed();
      on_agdbg_data_changed();

      __skiller_ifd = new InterfaceDispatcher("Skiller IFD", __skiller_if);
      __skdbg_ifd   = new InterfaceDispatcher("SkillerDebug IFD", __skdbg_if);
      __agdbg_ifd   = new InterfaceDispatcher("LuaAgent SkillerDebug IFD", __agdbg_if);
      bb->register_listener(__skiller_ifd, BlackBoard::BBIL_FLAG_DATA);
      bb->register_listener(__skdbg_ifd, BlackBoard::BBIL_FLAG_DATA);
      bb->register_listener(__agdbg_ifd, BlackBoard::BBIL_FLAG_DATA);
      __skiller_ifd->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_skiller_data_changed)));
      __skdbg_ifd->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_skdbg_data_changed)));
      __agdbg_ifd->signal_data_changed().connect(sigc::hide(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_agdbg_data_changed)));

      // always try to acquire control on connect, this may well fail, for
      // example if agent is running, but we don't care
      __skiller_if->read();
      if (__skiller_if->has_writer() && __skiller_if->exclusive_controller() == 0) {
	SkillerInterface::AcquireControlMessage *aqm = new SkillerInterface::AcquireControlMessage();
	__skiller_if->msgq_enqueue(aqm);
      }
      if (__skdbg_if->has_writer()) {
	SkillerDebugInterface::SetGraphMessage *sgm = new SkillerDebugInterface::SetGraphMessage("LIST");
	__skdbg_if->msgq_enqueue(sgm);
      }
    }
    tb_connection->set_stock_id(Gtk::Stock::DISCONNECT);
    __logview->set_client(connection_dispatcher.get_client());

    tb_controller->set_sensitive(true);
    cbe_skillstring->set_sensitive(true);

    this->set_title(std::string("Skill GUI @ ") + connection_dispatcher.get_client()->get_hostname());
  } catch (Exception &e) {
    Glib::ustring message = *(e.begin());
    Gtk::MessageDialog md(*this, message, /* markup */ false,
			  Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			  /* modal */ true);
    md.set_title("BlackBoard connection failed");
    md.run();

    close_bb();
    connection_dispatcher.get_client()->disconnect();
  }
}

/** Event handler for disconnected event. */
void
SkillGuiGtkWindow::on_disconnect()
{
  tb_controller->set_sensitive(false);
  cbe_skillstring->set_sensitive(false);
  but_exec->set_sensitive(false);
  but_stop->set_sensitive(false);

  close_bb();

  tb_connection->set_stock_id(Gtk::Stock::CONNECT);
#ifdef USE_PAPYRUS
  pvp_graph->queue_draw();
#endif
  __logview->set_client(NULL);

  this->set_title("Skill GUI");
}


void
SkillGuiGtkWindow::on_skiller_data_changed()
{
  try {
    __skiller_if->read();

    switch (__skiller_if->status()) {
    case SkillerInterface::S_INACTIVE:
      __throbber->stop_anim();
#if GTK_VERSION_GE(2,20)
      stb_status->remove_all_messages();
#else
      stb_status->pop();
#endif
      stb_status->push("S_INACTIVE");
      break;
    case SkillerInterface::S_FINAL:
      __throbber->stop_anim();
      __throbber->set_stock(Gtk::Stock::APPLY);
#if GTK_VERSION_GE(2,20)
      stb_status->remove_all_messages();
#else
      stb_status->pop();
#endif
      stb_status->push("S_FINAL");
      break;
    case SkillerInterface::S_RUNNING:
      __throbber->start_anim();
#if GTK_VERSION_GE(2,20)
      stb_status->remove_all_messages();
#else
      stb_status->pop();
#endif
      stb_status->push("S_RUNNING");
      break;
    case SkillerInterface::S_FAILED:
      __throbber->stop_anim();
      __throbber->set_stock(Gtk::Stock::DIALOG_WARNING);
#if GTK_VERSION_GE(2,20)
      stb_status->remove_all_messages();
#else
      stb_status->pop();
#endif
      stb_status->push("S_FAILED");
      break;
    }

#if GTK_VERSION_GE(2,12)
    stb_status->set_tooltip_text(__skiller_if->error());
#endif
    //lab_continuous->set_text(__skiller_if->is_continuous() ? "Yes" : "No");
    //lab_alive->set_text(__skiller_if->has_writer() ? "Yes" : "No");

    if ( __skiller_if->exclusive_controller() == __skiller_if->serial() ) {
      if ( tb_controller->get_stock_id() == Gtk::Stock::NO.id ) {
	tb_controller->set_stock_id(Gtk::Stock::YES);
#if GTK_VERSION_GE(2,12)
	tb_controller->set_tooltip_text("Release exclusive control");
#endif
      }
      but_exec->set_sensitive(true);
      but_stop->set_sensitive(true);
    } else {
      if ( tb_controller->get_stock_id() == Gtk::Stock::YES.id ) {
	tb_controller->set_stock_id(Gtk::Stock::NO);
#if GTK_VERSION_GE(2,12)
	tb_controller->set_tooltip_text("Gain exclusive control");
#endif
      }
      but_exec->set_sensitive(false);
      but_stop->set_sensitive(false);
    }


  } catch (Exception &e) {
    __throbber->stop_anim();
  }
}


void
SkillGuiGtkWindow::on_skdbg_data_changed()
{
  if (tb_skiller->get_active() && __skdbg_if) {
    try {
      __skdbg_if->read();

      if (strcmp(__skdbg_if->graph_fsm(), "LIST") == 0) {
#ifndef USE_ROS
        Glib::ustring list = __skdbg_if->graph();
        cb_graphlist->clear_items();
        cb_graphlist->append_text(ACTIVE_SKILL);
        cb_graphlist->set_active_text(ACTIVE_SKILL);
#if GTK_VERSION_GE(2,14)
        Glib::RefPtr<Glib::Regex> regex = Glib::Regex::create("\n");
        std::list<std::string> skills = regex->split(list);
        for (std::list<std::string>::iterator i = skills.begin(); i != skills.end(); ++i) {
          if (*i != "")  cb_graphlist->append_text(*i);
        }
#endif
        if (__skdbg_if->has_writer()) {
          SkillerDebugInterface::SetGraphMessage *sgm = new SkillerDebugInterface::SetGraphMessage("ACTIVE");
          __skdbg_if->msgq_enqueue(sgm);
        }
#endif
      } else {
        update_graph(__skdbg_if->graph_fsm(), __skdbg_if->graph());
      }

      switch (__skdbg_if->graph_dir()) {
      case SkillerDebugInterface::GD_TOP_BOTTOM:
        tb_graphdir->set_stock_id(Gtk::Stock::GO_DOWN); break;
      case SkillerDebugInterface::GD_BOTTOM_TOP:
        tb_graphdir->set_stock_id(Gtk::Stock::GO_UP); break;
      case SkillerDebugInterface::GD_LEFT_RIGHT:
        tb_graphdir->set_stock_id(Gtk::Stock::GO_FORWARD); break;
      case SkillerDebugInterface::GD_RIGHT_LEFT:
        tb_graphdir->set_stock_id(Gtk::Stock::GO_BACK); break;
      }
    } catch (Exception &e) {
      // ignored
    }
  }
}


void
SkillGuiGtkWindow::on_agdbg_data_changed()
{
  if (tb_agent->get_active() && __agdbg_if) {
    try {
      __agdbg_if->read();
#ifdef USE_PAPYRUS
      pvp_graph->set_graph_fsm(__agdbg_if->graph_fsm());
      pvp_graph->set_graph(__agdbg_if->graph());
      pvp_graph->render();
#else
      gda->set_graph_fsm(__agdbg_if->graph_fsm());
      gda->set_graph(__agdbg_if->graph());
#endif

      switch (__agdbg_if->graph_dir()) {
      case SkillerDebugInterface::GD_TOP_BOTTOM:
        tb_graphdir->set_stock_id(Gtk::Stock::GO_DOWN); break;
      case SkillerDebugInterface::GD_BOTTOM_TOP:
        tb_graphdir->set_stock_id(Gtk::Stock::GO_UP); break;
      case SkillerDebugInterface::GD_LEFT_RIGHT:
        tb_graphdir->set_stock_id(Gtk::Stock::GO_FORWARD); break;
      case SkillerDebugInterface::GD_RIGHT_LEFT:
        tb_graphdir->set_stock_id(Gtk::Stock::GO_BACK); break;
      }
    } catch (Exception &e) {
      // ignored
    }
  }
}
#endif

void
SkillGuiGtkWindow::on_graphupd_clicked()
{
#ifdef USE_PAPYRUS
  if ( pvp_graph->get_update_graph() ) {
    pvp_graph->set_update_graph(false);
    tb_graphupd->set_stock_id(Gtk::Stock::MEDIA_STOP);
  } else {
    pvp_graph->set_update_graph(true);
    tb_graphupd->set_stock_id(Gtk::Stock::MEDIA_PLAY);
    pvp_graph->render();
  }
#else
  if ( gda->get_update_graph() ) {
    gda->set_update_graph(false);
    tb_graphupd->set_stock_id(Gtk::Stock::MEDIA_STOP);
  } else {
    gda->set_update_graph(true);
    tb_graphupd->set_stock_id(Gtk::Stock::MEDIA_PLAY);
  }
#endif
}


void
SkillGuiGtkWindow::update_graph(std::string &graph_name, std::string &dotgraph)
{
#ifdef USE_PAPYRUS
  pvp_graph->set_graph_fsm(graph_name);
  pvp_graph->set_graph(dotgraph);
  pvp_graph->render();
#else
  gda->set_graph_fsm(graph_name);
  gda->set_graph(dotgraph);
#endif
}


void
SkillGuiGtkWindow::on_graphdir_clicked()
{
  Glib::ustring stockid = tb_graphdir->get_stock_id();
#ifndef USE_ROS
  SkillerDebugInterface *iface = __skdbg_if;
  if (tb_agent->get_active()) {
    iface = __agdbg_if;
  }

  if (stockid == Gtk::Stock::GO_DOWN.id) {
    send_graphdir_message(iface, SkillerDebugInterface::GD_BOTTOM_TOP);
  } else if (stockid == Gtk::Stock::GO_UP.id) {
    send_graphdir_message(iface, SkillerDebugInterface::GD_LEFT_RIGHT);
  } else if (stockid == Gtk::Stock::GO_FORWARD.id) {
    send_graphdir_message(iface, SkillerDebugInterface::GD_RIGHT_LEFT);
  } else if (stockid == Gtk::Stock::GO_BACK.id) {
    send_graphdir_message(iface, SkillerDebugInterface::GD_TOP_BOTTOM);
  }
#else
  skiller::SetGraphDirection srvr;
  skiller::Graph graphmsg;
  if (stockid == Gtk::Stock::GO_DOWN.id) {
    srvr.request.direction = graphmsg.GRAPH_DIR_BOTTOM_TOP;
  } else if (stockid == Gtk::Stock::GO_UP.id) {
    srvr.request.direction = graphmsg.GRAPH_DIR_LEFT_RIGHT;
  } else if (stockid == Gtk::Stock::GO_FORWARD.id) {
    srvr.request.direction = graphmsg.GRAPH_DIR_RIGHT_LEFT;
  } else if (stockid == Gtk::Stock::GO_BACK.id) {
    srvr.request.direction = graphmsg.GRAPH_DIR_TOP_BOTTOM;
  }
  if (tb_agent->get_active()) {
    __srv_graph_direction_agent.call(srvr);
  } else {
    __srv_graph_direction_skiller.call(srvr);
  }
#endif
}

void
SkillGuiGtkWindow::on_graphcolor_toggled()
{
  bool colored = tb_graphcolored->get_active();
#ifdef HAVE_GCONFMM
  __gconf->set(GCONF_PREFIX"/graph_colored", colored);
#endif
#ifndef USE_ROS
  SkillerDebugInterface *iface = __skdbg_if;
  if (tb_agent->get_active()) {
    iface = __agdbg_if;
  }

  try {
    if (iface) {
      SkillerDebugInterface::SetGraphColoredMessage *m;
      m = new SkillerDebugInterface::SetGraphColoredMessage(colored);
      iface->msgq_enqueue(m);
    } else {
      throw Exception("Not connected to Fawkes.");
    }
  } catch (Exception &e) {
    /* Ignore for now, causes error message on startup
    Gtk::MessageDialog md(*this,
			  Glib::ustring("Setting graph color failed: ") + e.what(),
			  / markup / false,
			  Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			  / modal / true);
    md.set_title("Communication Failure");
    md.run();
    */
  }
#else
  skiller::SetGraphColored srvr;
  srvr.request.colored = colored;
  if (tb_agent->get_active() && __srv_graph_color_agent.exists()) {
    printf("Calling agent\n");
    __srv_graph_color_agent.call(srvr);
  } else if (tb_skiller->get_active() && __srv_graph_color_skiller.exists()) {
    printf("Calling skiller\n");
    __srv_graph_color_skiller.call(srvr);
  }
#endif
}

void
SkillGuiGtkWindow::on_followactivestate_toggled()
{
#ifdef USE_PAPYRUS
#else
  bool follow_active_state = tb_followactivestate->get_active();
#ifdef HAVE_GCONFMM
  __gconf->set(GCONF_PREFIX"/follow_active_state", follow_active_state);
#endif
  gda->set_follow_active_state(follow_active_state);
#endif
}

#ifndef USE_ROS
void
SkillGuiGtkWindow::send_graphdir_message(fawkes::SkillerDebugInterface *iface,
					 fawkes::SkillerDebugInterface::GraphDirectionEnum gd)
{
  try {
    if (iface) {
      SkillerDebugInterface::SetGraphDirectionMessage *m;
      m = new SkillerDebugInterface::SetGraphDirectionMessage(gd);
      iface->msgq_enqueue(m);
    } else {
      throw Exception("Not connected to Fawkes.");
    }
  } catch (Exception &e) {
    Gtk::MessageDialog md(*this,
			  Glib::ustring("Setting graph direction failed: ") + e.what(),
			  /* markup */ false,
			  Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
			  /* modal */ true);
    md.set_title("Communication Failure");
    md.run();
  }
}

void
SkillGuiGtkWindow::on_graphdir_changed(fawkes::SkillerDebugInterface::GraphDirectionEnum gd)
{
  if (tb_agent->get_active()) {
    send_graphdir_message(__agdbg_if, gd);
  } else {
    send_graphdir_message(__skdbg_if, gd);
  }
}


#else


void
SkillGuiGtkWindow::on_graph_changed()
{
  skiller::Graph::ConstPtr msg = tb_agent->get_active() ? __graph_msg_agent : __graph_msg_skiller;

  std::string graph_name = "";
  std::string dotgraph = "";

  if (! msg) {
    // clear
    update_graph(graph_name, dotgraph);
    return;
  }

  graph_name = msg->name;
  dotgraph = msg->dotgraph;
  update_graph(graph_name, dotgraph);

  switch (msg->direction) {
  case skiller::Graph::GRAPH_DIR_TOP_BOTTOM:
    tb_graphdir->set_stock_id(Gtk::Stock::GO_DOWN); break;
  case skiller::Graph::GRAPH_DIR_BOTTOM_TOP:
    tb_graphdir->set_stock_id(Gtk::Stock::GO_UP); break;
  case skiller::Graph::GRAPH_DIR_LEFT_RIGHT:
    tb_graphdir->set_stock_id(Gtk::Stock::GO_FORWARD); break;
  case skiller::Graph::GRAPH_DIR_RIGHT_LEFT:
    tb_graphdir->set_stock_id(Gtk::Stock::GO_BACK); break;
  default: break;
  }

  msg.reset();
}

void
SkillGuiGtkWindow::on_sysstate_update()
{
  if (__cedar_timeout && __cedar_timeout.connected()) {
    __cedar_timeout.disconnect();
  }

  bool message_received = false;
  unsigned int condition = 0;
  std::string description = "?";
  time_t sec = 0;
  {
    Glib::Mutex::Lock lock(__sysstate_mutex);
    if (__sysstate_msg) {
      message_received = true;
      condition = __sysstate_msg->condition;
      description = __sysstate_msg->description;
      sec  =__sysstate_msg->stamp.sec;
    }
  }


  Gdk::Color color;
  Glib::ustring size = "20";
  if (! message_received) {
    color = Gdk::Color("orange");
  } else if (condition == cedar::SystemState::COND_GREEN) {
    color = Gdk::Color("green");
  } else if (condition == cedar::SystemState::COND_YELLOW) {
    color = Gdk::Color("yellow");
    size = "14";
  } else {
    color = Gdk::Color("red");
    size = "14";
  }

  eb_sysstate->modify_bg(Gtk::STATE_NORMAL, color);
  eb_sysstate->modify_bg(Gtk::STATE_ACTIVE, color);
  eb_sysstate->modify_bg(Gtk::STATE_PRELIGHT, color);
  Glib::ustring s =
    Glib::ustring::compose("<b><span font=\"%1\">%2</span></b>",
                           size, description);
  lab_sysstate->set_markup(s);

  if (dlg_cedar->get_visible()) {
    {
      Glib::Mutex::Lock lock(__sysstate_mutex);
      if (__sysstate_msg) {
        update_cedar_lists();
      }
    }
    eb_dlg_cedar_sysstate->modify_bg(Gtk::STATE_NORMAL, color);
    eb_dlg_cedar_sysstate->modify_bg(Gtk::STATE_ACTIVE, color);
    eb_dlg_cedar_sysstate->modify_bg(Gtk::STATE_PRELIGHT, color);
    lab_dlg_cedar_sysstate->set_markup(s);
    
    char *timestr = (char *)malloc(26);
    tm time_tm;
    localtime_r( &sec, &time_tm );
    asctime_r(&time_tm, timestr);
    timestr[strlen(timestr) - 1] = 0;
    lab_dlg_cedar_last_updated->set_text(timestr);
    free(timestr);
  }

  __cedar_timeout =
    Glib::signal_timeout().connect_seconds(sigc::mem_fun(*this, &SkillGuiGtkWindow::on_sysstate_timeout), SYSSTATE_TIMEOUT);

}


bool
SkillGuiGtkWindow::on_sysstate_timeout()
{
  Glib::Mutex::Lock lock(__sysstate_mutex);
  if (! __sysstate_msg ||
      ((ros::Time::now() - __sysstate_msg->stamp).toSec() > SYSSTATE_TIMEOUT))
  {
    Gdk::Color color = Gdk::Color("orange");

    eb_sysstate->modify_bg(Gtk::STATE_NORMAL, color);
    eb_sysstate->modify_bg(Gtk::STATE_ACTIVE, color);
    eb_sysstate->modify_bg(Gtk::STATE_PRELIGHT, color);
    lab_sysstate->set_markup("<b><span font=\"20\">?</span></b>");
    lab_dlg_cedar_sysstate->set_markup("<b><span font=\"20\">?</span></b>");
  }
  if (__sysstate_msg)  __sysstate_msg.reset();

  return false;
}

void
SkillGuiGtkWindow::update_cedar_lists()
{
  lst_cedar_nodes->clear();
  lst_cedar_conns->clear();
  std::vector<std::string>::const_iterator n;
  for (n = __sysstate_msg->dead_nodes.begin();
       n != __sysstate_msg->dead_nodes.end(); ++n)
  {
    Gtk::TreeModel::Row row = *lst_cedar_nodes->append();
    row[__cedar_node_record.nodename] = *n;
  }
  
  std::vector<cedar::TopicConnection>::const_iterator tc;
  for (tc = __sysstate_msg->dead_connections.begin();
       tc != __sysstate_msg->dead_connections.end(); ++tc)
  {
    Gtk::TreeModel::Row row = *lst_cedar_conns->append();
    row[__cedar_conn_record.topic] = tc->topic;
    row[__cedar_conn_record.from] = tc->from_node;
    row[__cedar_conn_record.to] = tc->to_node;
  }
}

void
SkillGuiGtkWindow::on_sysstate_clicked()
{
  unsigned int condition = 0;
  bool message_received;

  {
    Glib::Mutex::Lock lock(__sysstate_mutex);
    message_received = !! __sysstate_msg;
    if (message_received)  condition = __sysstate_msg->condition;
  }

  if (! message_received) {
    Gtk::MessageDialog md(*this,
                          "No system state message has been received, yet, or "
                          "the last message is too old. This might indicate "
                          "that CEDAR monitoring is not running.",
                          /* markup */ false,
                          Gtk::MESSAGE_ERROR, Gtk::BUTTONS_OK,
                          /* modal */ true);
    md.set_title("No System State Data");
    md.run();
  } else if (condition == cedar::SystemState::COND_GREEN) {
    Gtk::MessageDialog md(*this,
                          "All monitored nodes have been started and "
                          "connections have been established. System is "
                          "operating normal.",
                          /* markup */ false,
                          Gtk::MESSAGE_INFO, Gtk::BUTTONS_OK,
                          /* modal */ true);
    md.set_title("System Operating Normal");
    md.run();
  } else {
    {
      Glib::Mutex::Lock lock(__sysstate_mutex);
      if (__sysstate_msg) {
        update_cedar_lists();
      }
    }

    dlg_cedar->set_transient_for(*this);
    dlg_cedar->run();
    dlg_cedar->hide();
  }
}


void
SkillGuiGtkWindow::ros_skiller_graphmsg_cb(const skiller::Graph::ConstPtr &msg)
{
  __graph_msg_skiller = msg;
  if (tb_skiller->get_active())  __graph_changed();
}


void
SkillGuiGtkWindow::ros_agent_graphmsg_cb(const skiller::Graph::ConstPtr &msg)
{
  __graph_msg_agent = msg;
  if (tb_agent->get_active())  __graph_changed();
}

void
SkillGuiGtkWindow::ros_sysstate_cb(const cedar::SystemState::ConstPtr &msg)
{
  {
    Glib::Mutex::Lock lock(__sysstate_mutex);
    __sysstate_msg = msg;
  }
  __sysstate_update();
}


void
SkillGuiGtkWindow::ros_exec_transition_cb(actionlib::ClientGoalHandle<skiller::ExecSkillAction> &gh)
{
  __exec_comm_state = gh.getCommState();
  if (__exec_comm_state == CommState::DONE) {
    __exec_terminal_state = gh.getTerminalState().state_;
    if (gh.getResult() != NULL) {
      __exec_errmsg = gh.getResult()->errmsg;
    } else {
      __exec_errmsg = "";
    }
  }
  gh.reset();
  __exec_transition();
}


void
SkillGuiGtkWindow::on_exec_goal_transition()
{
  //printf("Transition to %i\n", comm_state_.state_);
  if (__exec_comm_state == CommState::DONE) {
    //printf("State: %i\n", __exec_gh.getTerminalState().state_);
    switch(__exec_terminal_state.state_) {
    case TerminalState::LOST: break; // do not change anything
    case TerminalState::SUCCEEDED:
      __throbber->stop_anim();
      __throbber->set_stock(Gtk::Stock::APPLY);
      stb_status->pop();
      stb_status->push("S_FINAL");
      break;
    default:
      __throbber->stop_anim();
      __throbber->set_stock(Gtk::Stock::DIALOG_WARNING);

      if (__exec_errmsg != "") {
        stb_status->pop();
        stb_status->push("S_FAILED: " + __exec_errmsg);
      } else {
        stb_status->pop();
        stb_status->push("S_FAILED");
      }
      break;
    }
  } else {
    __throbber->start_anim();
    stb_status->pop();
    stb_status->push("S_RUNNING");
  }
}

void
SkillGuiGtkWindow::ros_exec_feedback_cb(actionlib::ClientGoalHandle<skiller::ExecSkillAction> &gh,
					const skiller::ExecSkillFeedbackConstPtr &feedback)
{
}

#endif


SkillGuiGtkWindow::SkillStringRecord::SkillStringRecord()
{
  add(skillstring);
}


void
SkillGuiGtkWindow::on_update_disabled()
{
#ifdef USE_PAPYRUS
#else
  tb_graphupd->set_stock_id(Gtk::Stock::MEDIA_STOP);
#endif
}


void
SkillGuiGtkWindow::on_recording_toggled()
{
#ifdef USE_PAPYRUS
#else
  bool active = tb_graphrecord->get_active();
  if (gda->set_recording(active) != active) {
    tb_graphrecord->set_active(!active);
  }
#endif
}

#ifdef USE_ROS
/** @class SkillGuiGtkWindow::CedarNodeRecord "skillgui.h"
 * TreeView record for CEDAR dead nodes display..
 */

/** Constructor. */
SkillGuiGtkWindow::CedarNodeRecord::CedarNodeRecord()
{
  add(nodename);
}


/** @class SkillGuiGtkWindow::CedarTopicConnRecord "skillgui.h"
 * TreeView record for CEDAR dead nodes display..
 */

/** Constructor. */
SkillGuiGtkWindow::CedarTopicConnRecord::CedarTopicConnRecord()
{
  add(topic);
  add(from);
  add(to);
}

#endif
