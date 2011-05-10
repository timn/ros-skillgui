
/***************************************************************************
 *  graph_drawing_area.cpp - Graph drawing area derived from Gtk::DrawingArea
 *
 *  Created: Wed Mar 18 10:40:00 2009
 *  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

#include "graph_drawing_area.h"
#include "gvplugin_skillgui_cairo.h"

#include <cmath>
#include <libgen.h>

/** @class SkillGuiGraphDrawingArea "graph_drawing_area.h"
 * Graph drawing area.
 * Derived version of Gtk::DrawingArea that renders a graph via Graphviz.
 * @author Tim Niemueller
 */

/** Constructor. */
SkillGuiGraphDrawingArea::SkillGuiGraphDrawingArea()
{
  add_events(Gdk::SCROLL_MASK | Gdk::BUTTON_MOTION_MASK);

  __gvc = gvContext();
  __graph = NULL;

  __graph_fsm = "";
  __graph_dot = "";

  __bbw = __bbh = __pad_x = __pad_y = 0.0;
  __translation_x = __translation_y = 0.0;
  __scale = 1.0;
  __speed = 0.0;
  __speed_max = 80.0;
  __speed_ramp_distance = 80.0;
  __translation_x_setpoint = __translation_y_setpoint = 0.0;
  __scale_override = false;
  __update_graph = true;
  __recording = false;

  timeval now;
  gettimeofday(&now, NULL);
  __last_update_time = now.tv_sec + now.tv_usec/1000000;
  
  gvplugin_skillgui_cairo_setup(__gvc, this);

  __fcd_save = new Gtk::FileChooserDialog("Save Graph",
					  Gtk::FILE_CHOOSER_ACTION_SAVE);
  __fcd_open = new Gtk::FileChooserDialog("Load Graph",
					  Gtk::FILE_CHOOSER_ACTION_OPEN);
  __fcd_recording = new Gtk::FileChooserDialog("Recording Directory",
						 Gtk::FILE_CHOOSER_ACTION_CREATE_FOLDER);

  //Add response buttons the the dialog:
  __fcd_save->add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  __fcd_save->add_button(Gtk::Stock::SAVE, Gtk::RESPONSE_OK);
  __fcd_open->add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  __fcd_open->add_button(Gtk::Stock::SAVE, Gtk::RESPONSE_OK);
  __fcd_recording->add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  __fcd_recording->add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);

  __filter_pdf = new Gtk::FileFilter();
  __filter_pdf->set_name("Portable Document Format (PDF)");
  __filter_pdf->add_pattern("*.pdf");
  __filter_svg = new Gtk::FileFilter();
  __filter_svg->set_name("Scalable Vector Graphic (SVG)");
  __filter_svg->add_pattern("*.svg");
  __filter_png = new Gtk::FileFilter();
  __filter_png->set_name("Portable Network Graphic (PNG)");
  __filter_png->add_pattern("*.png");
  __filter_dot = new Gtk::FileFilter();
  __filter_dot->set_name("DOT Graph");
  __filter_dot->add_pattern("*.dot");
  __fcd_save->add_filter(*__filter_pdf);
  __fcd_save->add_filter(*__filter_svg);
  __fcd_save->add_filter(*__filter_png);
  __fcd_save->add_filter(*__filter_dot);
  __fcd_save->set_filter(*__filter_pdf);

  __fcd_open->add_filter(*__filter_dot);
  __fcd_open->set_filter(*__filter_dot);

  add_events(Gdk::SCROLL_MASK | Gdk::BUTTON_MOTION_MASK |
	     Gdk::BUTTON_PRESS_MASK );

#ifndef GLIBMM_DEFAULT_SIGNAL_HANDLERS_ENABLED
  signal_expose_event().connect(sigc::mem_fun(*this, &SkillGuiGraphDrawingArea::on_expose_event));
  signal_button_press_event().connect(sigc::mem_fun(*this, &SkillGuiGraphDrawingArea::on_button_press_event));
  signal_motion_notify_event().connect(sigc::mem_fun(*this, &SkillGuiGraphDrawingArea::on_motion_notify_event));
#endif
}

SkillGuiGraphDrawingArea::~SkillGuiGraphDrawingArea()
{
  if (__graph) {
    gvFreeLayout(__gvc, __graph);
    agclose(__graph);
  }
  gvFreeContext(__gvc);
  //delete __fcd;
  delete __fcd_save;
  delete __fcd_open;
  delete __fcd_recording;
  delete __filter_pdf;
  delete __filter_svg;
  delete __filter_png;
  delete __filter_dot;
}


/** Get "update disabled" signal.
 * @return "update disabled" signal
 */
sigc::signal<void>
SkillGuiGraphDrawingArea::signal_update_disabled()
{
  return __signal_update_disabled;
}


/** Set graph's FSM name.
 * @param fsm_name name of FSM the graph belongs to
 */
void
SkillGuiGraphDrawingArea::set_graph_fsm(std::string fsm_name)
{
  if ( __update_graph ) {
    if ( __graph_fsm != fsm_name ) {
      __scale_override = false;
    }
    __graph_fsm = fsm_name;
  } else {
    __nonupd_graph_fsm = fsm_name;
  }
}


/** Set graph.
 * @param graph string representation of the current graph in the dot language.
 */
void
SkillGuiGraphDrawingArea::set_graph(std::string graph)
{
  if ( __update_graph ) {
    __graph_dot = graph;
    layout_graph();
    queue_draw();
  } else {
    __nonupd_graph = graph;
  }

  if ( __recording ) {
    char *tmp;
    timespec t;
    if (clock_gettime(CLOCK_REALTIME, &t) == 0) {
      struct tm tms;
      localtime_r(&t.tv_sec, &tms);

      if ( asprintf(&tmp, "%s/%s_%04i%02i%02i-%02i%02i%02i.%09li.dot",
		    __record_directory.c_str(), __graph_fsm.c_str(),
		    tms.tm_year + 1900, tms.tm_mon + 1, tms.tm_mday,
		    tms.tm_hour, tms.tm_min, tms.tm_sec, t.tv_nsec) != -1) {

          //printf("Would record to filename %s\n", tmp);
          save_dotfile(tmp);
          free(tmp);
      } else {
        printf("Warning: Could not create file name for recording, skipping graph\n");
      }
    } else {
      printf("Warning: Could not time recording, skipping graph\n");
    }
  }
}

/** Set bounding box.
 * To be called only by the Graphviz plugin.
 * @param bbw bounding box width
 * @param bbh bounding box height
 */
void
SkillGuiGraphDrawingArea::set_bb(double bbw, double bbh)
{
  __bbw = bbw;
  __bbh = bbh;
}


/** Set padding.
 * To be called only by the Graphviz plugin.
 * @param pad_x padding in x
 * @param pad_y padding in y
 */
void
SkillGuiGraphDrawingArea::set_pad(double pad_x, double pad_y)
{
  __pad_x = pad_x;
  __pad_y = pad_y;
}


/** Get padding.
 * To be called only by the Graphviz plugin.
 * @param pad_x upon return contains padding in x
 * @param pad_y upon return contains padding in y
 */
void
SkillGuiGraphDrawingArea::get_pad(double &pad_x, double &pad_y)
{
  if (__scale_override) {
    pad_x = pad_y = 0;
  } else {
    pad_x = __pad_x;
    pad_y = __pad_y;
  }
}


/** Set translation.
 * To be called only by the Graphviz plugin.
 * @param tx translation in x
 * @param ty translation in y
 */
void
SkillGuiGraphDrawingArea::set_translation(double tx, double ty)
{
  __translation_x = tx;
  __translation_y = ty;
}


/** Set scale.
 * To be called only by the Graphviz plugin.
 * @param scale scale value
 */
void
SkillGuiGraphDrawingArea::set_scale(double scale)
{
  __scale = scale;
}

/** Get scale.
 * To be called only by the Graphviz plugin.
 * @return scale value
 */
double
SkillGuiGraphDrawingArea::get_scale()
{
  return __scale;
}

/** Get translation.
 * @param tx upon return contains translation value
 * @param ty upon return contains translation value
 */
void
SkillGuiGraphDrawingArea::get_translation(double &tx, double &ty)
{
  update_translations();
  tx = __translation_x;
  ty = __translation_y;
}

/** Get dimensions
 * @param width upon return contains width
 * @param height upon return contains height
 */
void
SkillGuiGraphDrawingArea::get_dimensions(double &width, double &height)
{
  Gtk::Allocation alloc = get_allocation();
  width  = alloc.get_width();
  height = alloc.get_height();
}


/** Zoom in.
 * Increases zoom factor by 20, no upper limit.
 */
void
SkillGuiGraphDrawingArea::zoom_in()
{
  __scale_override = true;
  Gtk::Allocation alloc = get_allocation();
  double old_scale = __scale;
  __scale /= 0.80;
  double px, py;
  px = (alloc.get_width()/2  - __translation_x                  ) / (old_scale * __bbw);
  py = (alloc.get_height()/2 - __translation_y + __bbh * old_scale) / (old_scale * __bbh);
  __translation_x = alloc.get_width()/2  - px*__bbw*__scale;
  __translation_y = alloc.get_height()/2 - py*__bbh*__scale + __bbh * __scale;
  queue_draw();
}

/** Zoom out.
 * Decreases zoom factor by 20 with a minimum of 1.
 */
void
SkillGuiGraphDrawingArea::zoom_out()
{
  __scale_override = true;
  if ( __scale > 0.1 ) {
    Gtk::Allocation alloc = get_allocation();
    double old_scale = __scale;
    __scale *= 0.80;
    double px, py;
    px = (alloc.get_width()/2  - __translation_x                  ) / (old_scale * __bbw);
    py = (alloc.get_height()/2 - __translation_y + __bbh * old_scale) / (old_scale * __bbh);
    __translation_x = alloc.get_width()/2  - px*__bbw*__scale;
    __translation_y = alloc.get_height()/2 - py*__bbh*__scale + __bbh * __scale;
    queue_draw();
  }
}


/** Zoom to fit.
 * Disables scale override and draws with values suggested by Graphviz plugin.
 */
void
SkillGuiGraphDrawingArea::zoom_fit()
{
  __scale_override = false;
  queue_draw();
}


/** Zoom reset.
 * Reset zoom to 1. Enables scale override.
 */
void
SkillGuiGraphDrawingArea::zoom_reset()
{
  Gtk::Allocation alloc = get_allocation();
  __scale = 1.0;
  __scale_override = true;
  __translation_x = (alloc.get_width()  - __bbw) / 2.0 + __pad_x;
  __translation_y = (alloc.get_height() - __bbh) / 2.0 + __bbh - __pad_y;
  queue_draw();
}


/** Check if scale override is enabled.
 * @return true if scale override is enabled, false otherwise
 */
bool
SkillGuiGraphDrawingArea::scale_override()
{
  return __scale_override;
}


/** Get Cairo context.
 * This is only valid during the expose event and is only meant for the
 * Graphviz plugin.
 * @return Cairo context
 */
Cairo::RefPtr<Cairo::Context>
SkillGuiGraphDrawingArea::get_cairo()
{
  return __cairo;
}



/** Check if graph is being updated.
 * @return true if the graph will be update if new data is received, false otherwise
 */
bool
SkillGuiGraphDrawingArea::get_update_graph()
{
  return __update_graph;
}


/** Set if the graph should be updated on new data.
 * @param update true to update on new data, false to disable update
 */
void
SkillGuiGraphDrawingArea::set_update_graph(bool update)
{
  if (update && ! __update_graph) {
    if ( __graph_fsm != __nonupd_graph_fsm ) {
      __scale_override = false;
    }
    __graph_dot        = __nonupd_graph;
    __graph_fsm    = __nonupd_graph_fsm;
    queue_draw();
  }
  __update_graph = update;
}


void
SkillGuiGraphDrawingArea::save_dotfile(const char *filename)
{
  FILE *f = fopen(filename, "w");
  if (f) {
    if (fwrite(__graph_dot.c_str(), __graph_dot.length(), 1, f) != 1) {
      // bang, ignored
      printf("Failed to write dot file '%s'\n", filename);
    }
    fclose(f);
  }
}


/** Enable/disable active state following.
 * @param follow_active_state true to enable active state following, false otherwise
 * @return true if follow_active_state is enabled now, false if it is disabled.
 */
bool
SkillGuiGraphDrawingArea::set_follow_active_state(bool follow_active_state)
{
  if (follow_active_state) {
    __follow_active_state = true;
  } else {
    __follow_active_state = false;
  }
  return __follow_active_state;
}


/** Enable/disable recording.
 * @param recording true to enable recording, false otherwise
 * @return true if recording is enabled now, false if it is disabled.
 * Enabling the recording may fail for example if the user chose to abort
 * the directory creation process.
 */
bool
SkillGuiGraphDrawingArea::set_recording(bool recording)
{
  if (recording) {
    Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());
    __fcd_recording->set_transient_for(*w);
    int result = __fcd_recording->run();
    if (result == Gtk::RESPONSE_OK) {
      __record_directory = __fcd_recording->get_filename();
      __recording = true;
    }
    __fcd_recording->hide();
  } else {
    __recording = false;
  }
  return __recording;
}


/** save current graph. */
void
SkillGuiGraphDrawingArea::save()
{
  Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());
  __fcd_save->set_transient_for(*w);

  int result = __fcd_save->run();
  if (result == Gtk::RESPONSE_OK) {

    Gtk::FileFilter *f = __fcd_save->get_filter();
    std::string filename = __fcd_save->get_filename();
    if (filename != "") {
      if (f == __filter_dot) {
	save_dotfile(filename.c_str());
      } else {
	Cairo::RefPtr<Cairo::Surface> surface;

	bool write_to_png = false;
	if (f == __filter_pdf) {
	  surface = Cairo::PdfSurface::create(filename, __bbw, __bbh);
	} else if (f == __filter_svg) {
	  surface = Cairo::SvgSurface::create(filename, __bbw, __bbh);
	} else if (f == __filter_png) {
	  surface = Cairo::ImageSurface::create(Cairo::FORMAT_ARGB32,
						(int)ceilf(__bbw),
						(int)ceilf(__bbh));
	  write_to_png = true;
	}

	if (surface) {
	  __cairo = Cairo::Context::create(surface);
	  
	  bool old_scale_override = __scale_override;
	  double old_tx = __translation_x;
	  double old_ty = __translation_y;
	  double old_scale = __scale;
	  __translation_x = __pad_x;
	  __translation_y = __bbh - __pad_y;
	  __scale = 1.0;
	  __scale_override = true;

	  if (__graph) {
	    gvRender(__gvc, __graph, (char *)"skillguicairo", NULL);
	  }

	  if (write_to_png) {
	    surface->write_to_png(filename);
	  }

	  __cairo.clear();

	  __translation_x = old_tx;
	  __translation_y = old_ty;
	  __scale = old_scale;
	  __scale_override = old_scale_override;
	}
      }

    } else {
      Gtk::MessageDialog md(*w, "Invalid filename",
			    /* markup */ false, Gtk::MESSAGE_ERROR,
			    Gtk::BUTTONS_OK, /* modal */ true);
      md.set_title("Invalid File Name");
      md.run();
    }
  }

  __fcd_save->hide();
}


/** Open a dot graph and display it. */
void
SkillGuiGraphDrawingArea::open()
{
  Gtk::Window *w = dynamic_cast<Gtk::Window *>(get_toplevel());
  __fcd_open->set_transient_for(*w);

  int result = __fcd_open->run();
  if (result == Gtk::RESPONSE_OK) {
    __update_graph = false;
    __graph_dot = "";
    char *basec = strdup(__fcd_open->get_filename().c_str());
    char *basen = basename(basec);
    __graph_fsm = basen;
    free(basec);

    FILE *f = fopen(__fcd_open->get_filename().c_str(), "r");
    while (! feof(f)) {
      char tmp[4096];
      size_t s;
      if ((s = fread(tmp, 1, 4096, f)) > 0) {
        __graph_dot.append(tmp, s);
      }
    }
    fclose(f);
    __signal_update_disabled.emit();
    queue_draw();
  }

  __fcd_open->hide();
}


/** Expose event handler.
 * @param event event info structure.
 * @return signal return value
 */
bool
SkillGuiGraphDrawingArea::on_expose_event(GdkEventExpose* event)
{
  // This is where we draw on the window
  Glib::RefPtr<Gdk::Window> window = get_window();
  if(window) {
    Gtk::Allocation allocation = get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();
    
    // coordinates for the center of the window
    int xc, yc;
    xc = width / 2;
    yc = height / 2;
    
    __cairo = window->create_cairo_context();
    __cairo->set_source_rgb(1, 1, 1);
    __cairo->paint();

    if (__graph) {
      Glib::Timer t;
      gvRender(__gvc, __graph, (char *)"skillguicairo", NULL);
      t.stop();
      printf("Rendering took %f seconds\n", t.elapsed());
    }

    __cairo.clear();
  }
  return true;
}

/** Scroll event handler.
 * @param event event structure
 * @return signal return value
 */
bool
SkillGuiGraphDrawingArea::on_scroll_event(GdkEventScroll *event)
{
  if (event->direction == GDK_SCROLL_UP) {
    zoom_in();
  } else if (event->direction == GDK_SCROLL_DOWN) {
    zoom_out();
  }
  return true;
}


/** Button press event handler.
 * @param event event data
 * @return true
 */
bool
SkillGuiGraphDrawingArea::on_button_press_event(GdkEventButton *event)
{
  __last_mouse_x = event->x;
  __last_mouse_y = event->y;
  return true;
}


/** Mouse motion notify event handler.
 * @param event event data
 * @return true
 */
bool
SkillGuiGraphDrawingArea::on_motion_notify_event(GdkEventMotion *event)
{
  __scale_override = true;
  __translation_x -= __last_mouse_x - event->x;
  __translation_y -= __last_mouse_y - event->y;
  __last_mouse_x = event->x;
  __last_mouse_y = event->y;
  queue_draw();
  return true;
}


/** Get position of a state in the graph.
 * @param state_name name of state to get position of
 * @param px upon successful return contains X position value
 * @param py upon succesful return contains Y position value
 * Positions px/py are decimal percentages of
 * graph width/height from left/top. 
 * @return true if node has been found and px/py have been set, false otherwise
 */
bool
SkillGuiGraphDrawingArea::get_state_position(std::string state_name,
					     double &px, double &py)
{
  if (! __graph) {
    return false;
  }

  for (node_t *n = agfstnode(__graph); n; n = agnxtnode(__graph, n)) {
    const char *name = agnameof(n);
    pointf nodepos = ND_coord(n);

    if (state_name == name) {
      boxf bb = GD_bb(__graph);
      double bb_width  = bb.UR.x - bb.LL.x;
      double bb_height = bb.UR.y - bb.LL.y;

      px =       nodepos.x / bb_width;
      py = 1. - (nodepos.y / bb_height);

      return true;
    }
  }

  return false;
}

/** Determine the current active state.
 * Note that there might be multiple active states if there are sub-fsms. It will
 * only consider the active state of the top-fsm for now.
 * @return the current active state
 */
std::string
SkillGuiGraphDrawingArea::get_active_state()
{
  if (! __graph) {
    return "";
  }

  for (node_t *n = agfstnode(__graph); n; n = agnxtnode(__graph, n)) {
    const char *actattr = agget(n, (char *)"active");
    if (actattr && (strcmp(actattr, "true") == 0) ) {
      return agnameof(n);
    }
  }

  return "";
}


/** Update __translation_x and __translation_y
 *  for state following animation.
 */
void
SkillGuiGraphDrawingArea::update_translations()
{
  timeval now;
  gettimeofday(&now, NULL);
  double now_time = now.tv_sec + now.tv_usec/1000000;
  
  if (__follow_active_state) {
    double px, py;
    std::string active_state = get_active_state();
    if (! get_state_position(active_state, px, py)) {
      px = py = 0.5;
    }
    Gtk::Allocation alloc = get_allocation();
    __translation_x_setpoint = alloc.get_width()/2  - px*__bbw*__scale;
    __translation_y_setpoint = alloc.get_height()/2 - py*__bbh*__scale + __bbh * __scale;
    
    float d, dx, dy;
    dx = __translation_x_setpoint - __translation_x;
    dy = __translation_y_setpoint - __translation_y;
    d = sqrt(pow(dx,2) + pow(dy,2));
    float v = 0.0;
    if (d < 0.05 * __speed_ramp_distance) {
      // At goal, don't moving
      v = 0;
      __translation_x = __translation_x_setpoint;
      __translation_y = __translation_y_setpoint;
    } else if (d < __speed_ramp_distance) {
      // Approaching goal, slow down
      v = __speed_max * (1 - d / __speed_ramp_distance);
    } else if (__speed < __speed_max) {
      // Starting toward goal, speed up
      v = __speed + 10;
    } else {
      // Moving towards goal, keep going
      v = __speed;
    }
    // Set new translations
    //printf("d=%f  v=%f  dx=%f  dy=%f\n", d, v, dx, dy);
    if ( d <= v * (now_time - __last_update_time) ) {
      __translation_x = __translation_x_setpoint;
      __translation_y = __translation_y_setpoint;
      __speed = 0;
    } else {
      __translation_x =
	(dx / d) * v * (now_time - __last_update_time) + __translation_x;
      __translation_y =
	(dy / d) * v * (now_time - __last_update_time) + __translation_y;
      __speed = v;
      queue_draw();
    }
    if (false) {
      __translation_x = __translation_x_setpoint;
      __translation_y = __translation_y_setpoint;
      __speed = 0;
    }
  }
  __last_update_time = now_time;
}


void
SkillGuiGraphDrawingArea::layout_graph()
{
  if (__graph) {
    gvFreeLayout(__gvc, __graph);
    agclose(__graph);
  }
  __graph = agmemread((char *)__graph_dot.c_str());
  if (__graph) {
    gvLayout(__gvc, __graph, (char *)"dot");
  }
}
