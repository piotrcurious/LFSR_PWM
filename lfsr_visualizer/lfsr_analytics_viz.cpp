// lfsr_analytics_viz.cpp
//
// Compile: g++ -O3 -std=c++17 -march=native -o lfsr_analytics_viz lfsr_analytics_viz.cpp -lfltk -lfltk_gl -lGL -lGLU -lpthread
//
// Description:
// FLTK + OpenGL visualizer for the '8bit_set' analytical dataset.
// Allows selection of PWM bins (0x00 - 0xFF) and visualizes the Mask/Seed distribution.

#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Check_Browser.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Box.H>
#include <FL/gl.h>
#include <GL/glu.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <mutex>

// -----------------------------------------------------------------------------
// Data Structures
// -----------------------------------------------------------------------------
struct Point2D {
    uint16_t mask;
    uint16_t seed;
};

// Represents one loaded bin (e.g., 0x80.bin)
struct BinData {
    int id; // 0-255
    std::vector<Point2D> points;
    bool visible;
    float r, g, b; // Precomputed color
};

// Global State
std::vector<BinData> g_bins(256);
std::mutex g_data_mutex;

// Statistics
struct Stats {
    size_t total_points = 0;
    double avg_mask = 0;
    double avg_seed = 0;
    double density = 0;
    int visible_bins = 0;
};
Stats g_stats;

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------

// Color map: Rainbow gradient based on ID (0-255)
void get_color(int id, float& r, float& g, float& b) {
    float t = id / 255.0f;
    r = t;
    g = sin(t * 3.14159f);
    b = 1.0f - t;
}

// Load a specific bin file if not already loaded
void load_bin(int id) {
    if (!g_bins[id].points.empty()) return; // Already loaded

    char filename[64];
    sprintf(filename, "8bit_set/0x%02X.bin", id);
    
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) return;

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    size_t count = size / sizeof(Point2D);
    g_bins[id].points.resize(count);
    if (file.read((char*)g_bins[id].points.data(), size)) {
        g_bins[id].id = id;
        get_color(id, g_bins[id].r, g_bins[id].g, g_bins[id].b);
    }
}

// Unload to save RAM
void unload_bin(int id) {
    std::vector<Point2D>().swap(g_bins[id].points);
}

void update_stats() {
    size_t count = 0;
    double sum_m = 0;
    double sum_s = 0;
    int bins = 0;

    for(int i=0; i<256; ++i) {
        if(g_bins[i].visible && !g_bins[i].points.empty()) {
            count += g_bins[i].points.size();
            bins++;
            // Approximating average for performance (checking stride)
            size_t step = std::max((size_t)1, g_bins[i].points.size() / 100); 
            for(size_t k=0; k<g_bins[i].points.size(); k+=step) {
                sum_m += g_bins[i].points[k].mask;
                sum_s += g_bins[i].points[k].seed;
            }
            // Normalize partial sums
            sum_m *= (double)g_bins[i].points.size() / (g_bins[i].points.size()/step + 1);
            sum_s *= (double)g_bins[i].points.size() / (g_bins[i].points.size()/step + 1);
        }
    }

    g_stats.total_points = count;
    g_stats.visible_bins = bins;
    g_stats.avg_mask = (count > 0) ? sum_m / count : 0;
    g_stats.avg_seed = (count > 0) ? sum_s / count : 0;
    // Density: Points / Total Possible Space (65536^2)
    g_stats.density = (double)count / (65536.0 * 65536.0) * 100.0;
}

// -----------------------------------------------------------------------------
// OpenGL Visualization Widget
// -----------------------------------------------------------------------------
class LFSRView : public Fl_Gl_Window {
    float cam_dist = 85000.0f;
    float cam_pitch = 45.0f;
    float cam_yaw = 0.0f;
    float cam_x = 32768.0f;
    float cam_y = 32768.0f;
    
    int last_mx = 0, last_my = 0;
    bool drag_rot = false, drag_pan = false;

public:
    LFSRView(int x, int y, int w, int h) : Fl_Gl_Window(x, y, w, h) {
        mode(FL_RGB | FL_DOUBLE | FL_DEPTH | FL_MULTISAMPLE);
    }

    void draw() override {
        if (!valid()) {
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Projection
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60.0, (double)w() / h(), 100.0, 300000.0);

        // View
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0, 0, -cam_dist);
        glRotatef(cam_pitch, 1, 0, 0);
        glRotatef(cam_yaw, 0, 1, 0);
        glTranslatef(-cam_x, 0, -cam_y); // Center world

        // Draw Axes Box (0..65535)
        glColor3f(0.3f, 0.3f, 0.3f);
        glLineWidth(1.0f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(0, 0, 0); glVertex3f(65535, 0, 0);
        glVertex3f(65535, 0, 65535); glVertex3f(0, 0, 65535);
        glEnd();

        // Render Points
        glPointSize(1.0f);
        glBegin(GL_POINTS);

        std::lock_guard<std::mutex> lock(g_data_mutex);
        
        // Auto-LOD: Calculate skip factor based on total points to keep FPS sane
        // Target max ~2 million points per frame
        size_t total = g_stats.total_points;
        int skip = 1;
        if (total > 2000000) skip = total / 2000000;
        if (skip < 1) skip = 1;

        for (const auto& bin : g_bins) {
            if (!bin.visible || bin.points.empty()) continue;
            
            glColor3f(bin.r, bin.g, bin.b);
            
            // Z-height depends on bin ID (0..255 maps to height 0..10000)
            float z = (float)bin.id / 255.0f * 15000.0f;

            for (size_t i = 0; i < bin.points.size(); i += skip) {
                // OpenGL coord system: Y is Up usually. 
                // We map: Mask->X, Height->Y, Seed->Z
                glVertex3f(bin.points[i].mask, z, bin.points[i].seed);
            }
        }
        glEnd();
        
        // Draw selection box for reference
        glColor4f(1, 1, 1, 0.2);
        glBegin(GL_LINES);
        glVertex3f(cam_x-1000, 0, cam_y); glVertex3f(cam_x+1000, 0, cam_y);
        glVertex3f(cam_x, 0, cam_y-1000); glVertex3f(cam_x, 0, cam_y+1000);
        glEnd();
    }

    int handle(int event) override {
        switch (event) {
            case FL_PUSH:
                last_mx = Fl::event_x();
                last_my = Fl::event_y();
                if (Fl::event_button() == FL_LEFT_MOUSE) drag_rot = true;
                if (Fl::event_button() == FL_RIGHT_MOUSE) drag_pan = true;
                return 1;
            case FL_RELEASE:
                drag_rot = false;
                drag_pan = false;
                return 1;
            case FL_DRAG: {
                int dx = Fl::event_x() - last_mx;
                int dy = Fl::event_y() - last_my;
                last_mx = Fl::event_x();
                last_my = Fl::event_y();

                if (drag_rot) {
                    cam_yaw += dx * 0.5f;
                    cam_pitch += dy * 0.5f;
                }
                if (drag_pan) {
                    float rad = cam_yaw * 3.14159f / 180.0f;
                    float c = cos(rad), s = sin(rad);
                    cam_x -= (c * dx - s * dy) * (cam_dist * 0.001f);
                    cam_y -= (s * dx + c * dy) * (cam_dist * 0.001f);
                }
                redraw();
                return 1;
            }
            case FL_MOUSEWHEEL:
                cam_dist -= Fl::event_dy() * (cam_dist * 0.1f);
                if (cam_dist < 100) cam_dist = 100;
                redraw();
                return 1;
            default:
                return Fl_Gl_Window::handle(event);
        }
    }
};

// -----------------------------------------------------------------------------
// UI Logic
// -----------------------------------------------------------------------------

Fl_Check_Browser* browser;
LFSRView* gl_view;
Fl_Box* stat_box;

void update_stats_display() {
    std::string s = "Statistics:\n";
    s += "Visible Bins: " + std::to_string(g_stats.visible_bins) + "\n";
    s += "Total Points: " + std::to_string(g_stats.total_points) + "\n";
    
    char buf[128];
    sprintf(buf, "Avg Mask: %.0f\nAvg Seed: %.0f\nDensity: %.4f%%", 
        g_stats.avg_mask, g_stats.avg_seed, g_stats.density);
    s += buf;
    
    stat_box->copy_label(s.c_str());
}

void browser_cb(Fl_Widget*, void*) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    
    for (int i = 0; i < 256; ++i) {
        if (browser->checked(i + 1)) { // Items are 1-based
            if (!g_bins[i].visible) {
                load_bin(i);
                g_bins[i].visible = true;
            }
        } else {
            if (g_bins[i].visible) {
                unload_bin(i);
                g_bins[i].visible = false;
            }
        }
    }
    update_stats();
    update_stats_display();
    gl_view->redraw();
}

void select_all_cb(Fl_Widget*, void*) {
    for (int i=1; i<=256; ++i) browser->checked(i, 1);
    browser_cb(nullptr, nullptr);
}

void clear_all_cb(Fl_Widget*, void*) {
    for (int i=1; i<=256; ++i) browser->checked(i, 0);
    browser_cb(nullptr, nullptr);
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int argc, char **argv) {
    // Main Window
    Fl_Double_Window* win = new Fl_Double_Window(1280, 720, "LFSR Analytical Visualizer (FLTK/OpenGL)");
    
    // Left: OpenGL View
    gl_view = new LFSRView(10, 10, 980, 700);
    
    // Right: Controls
    Fl_Group* controls = new Fl_Group(1000, 10, 270, 700);
    
    // 1. Bin Selector
    browser = new Fl_Check_Browser(1000, 10, 270, 400, "PWM Bins (0x00 - 0xFF)");
    for (int i = 0; i < 256; ++i) {
        char buf[32];
        sprintf(buf, "Bin 0x%02X (%d)", i, i);
        browser->add(buf);
    }
    browser->callback(browser_cb);

    Fl_Button* btn_all = new Fl_Button(1000, 415, 130, 25, "Select All");
    btn_all->callback(select_all_cb);
    
    Fl_Button* btn_none = new Fl_Button(1140, 415, 130, 25, "Clear");
    btn_none->callback(clear_all_cb);

    // 2. Statistics Panel (Floating look via Box)
    Fl_Box* stat_panel = new Fl_Box(1000, 450, 270, 200);
    stat_panel->box(FL_DOWN_BOX);
    stat_panel->color(fl_rgb_color(220, 220, 220));
    
    stat_box = new Fl_Box(1010, 460, 250, 180, "Select bins to view stats...");
    stat_box->align(FL_ALIGN_TOP_LEFT | FL_ALIGN_INSIDE);
    stat_box->labelfont(FL_COURIER);

    controls->end();
    win->end();

    win->resizable(gl_view);
    win->show(argc, argv);

    return Fl::run();
}
