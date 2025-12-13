// lfsr_analytics_viz.cpp
//
// Compile: g++ -O3 -std=c++17 -march=native -o lfsr_analytics_viz lfsr_analytics_viz.cpp -lfltk -lfltk_gl -lGL -lGLU -lpthread

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

struct BinData {
    int id;
    std::vector<Point2D> points;
    bool visible;
    float r, g, b;
    BinData() : id(-1), visible(false), r(1.0f), g(1.0f), b(1.0f) {}
};

// Global State
static std::vector<BinData> g_bins(256);
static std::mutex g_data_mutex;

// Statistics
struct Stats {
    size_t total_points = 0;
    double avg_mask = 0;
    double avg_seed = 0;
    double density = 0;
    int visible_bins = 0;
};
static Stats g_stats;

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------

// Color map: Rainbow gradient based on ID (0-255)
void get_color(int id, float& r, float& g, float& b) {
    float t = id / 255.0f;
    r = t;
    g = std::sin(t * 3.14159265f);
    b = 1.0f - t;
}

// Load a specific bin file if not already loaded
void load_bin(int id) {
    // If already loaded, nothing to do
    if (!g_bins[id].points.empty()) return;

    char filename[128];
    std::snprintf(filename, sizeof(filename), "8bit_set/0x%02X.bin", id);

    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "Failed to open '" << filename << "'\n";
        return;
    }

    std::streamsize size = file.tellg();
    if (size <= 0) {
        std::cerr << "Empty file '" << filename << "'\n";
        return;
    }

    if (size % sizeof(Point2D) != 0) {
        std::cerr << "File size not multiple of Point2D for '" << filename << "'\n";
        return;
    }

    file.seekg(0, std::ios::beg);
    size_t count = static_cast<size_t>(size / sizeof(Point2D));
    std::vector<Point2D> tmp(count);

    if (!file.read(reinterpret_cast<char*>(tmp.data()), size)) {
        std::cerr << "Failed read for '" << filename << "'\n";
        return;
    }

    // Move into the global bin
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        g_bins[id].points = std::move(tmp);
        g_bins[id].id = id;
        get_color(id, g_bins[id].r, g_bins[id].g, g_bins[id].b);
    }
}

// Unload to save RAM
void unload_bin(int id) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    std::vector<Point2D>().swap(g_bins[id].points);
    g_bins[id].visible = false;
}

void update_stats() {
    size_t count = 0;
    double sum_m = 0.0;
    double sum_s = 0.0;
    int bins = 0;

    for (int i = 0; i < 256; ++i) {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        if (g_bins[i].visible && !g_bins[i].points.empty()) {
            const auto &pts = g_bins[i].points;
            size_t n = pts.size();
            count += n;
            bins++;

            // Sample for speed, but scale properly
            size_t step = std::max<size_t>(1, n / 100); 
            size_t sampled = 0;
            double local_sum_m = 0.0;
            double local_sum_s = 0.0;
            for (size_t k = 0; k < n; k += step) {
                local_sum_m += pts[k].mask;
                local_sum_s += pts[k].seed;
                sampled++;
            }
            if (sampled > 0) {
                double scale = static_cast<double>(n) / static_cast<double>(sampled);
                sum_m += local_sum_m * scale;
                sum_s += local_sum_s * scale;
            }
        }
    }

    g_stats.total_points = count;
    g_stats.visible_bins = bins;
    g_stats.avg_mask = (count > 0) ? (sum_m / static_cast<double>(count)) : 0.0;
    g_stats.avg_seed = (count > 0) ? (sum_s / static_cast<double>(count)) : 0.0;
    // Density: Points / Total Possible Space (65536 * 65536)
    g_stats.density = (count > 0) ? (static_cast<double>(count) / (65536.0 * 65536.0) * 100.0) : 0.0;
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
            // Init GL state once when the context becomes valid
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_POINT_SMOOTH);
            glPointSize(2.0f); // Slightly larger so sparse points are easier to see
            glClearColor(0.1f, 0.1f, 0.15f, 1.0f);

            // *** CRITICAL: setup viewport here ***
            glViewport(0, 0, w(), h());
        }

        // Clear buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Projection
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60.0, (double)w() / (h() > 0 ? h() : 1), 100.0, 300000.0);

        // View
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.0f, 0.0f, -cam_dist);
        glRotatef(cam_pitch, 1.0f, 0.0f, 0.0f);
        glRotatef(cam_yaw, 0.0f, 1.0f, 0.0f);
        glTranslatef(-cam_x, 0.0f, -cam_y);

        // Draw Axes Box (0..65535)
        glColor3f(0.3f, 0.3f, 0.3f);
        glLineWidth(1.0f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(0.0f, 0.0f, 0.0f); glVertex3f(65535.0f, 0.0f, 0.0f);
        glVertex3f(65535.0f, 0.0f, 65535.0f); glVertex3f(0.0f, 0.0f, 65535.0f);
        glEnd();

        // Render Points
        glBegin(GL_POINTS);
        size_t total = g_stats.total_points;
        int skip = 1;
        if (total > 2000000) skip = static_cast<int>(total / 2000000);
        if (skip < 1) skip = 1;

        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            for (const auto& bin : g_bins) {
                if (!bin.visible || bin.points.empty()) continue;

                glColor3f(bin.r, bin.g, bin.b);

                float height = static_cast<float>(bin.id) / 255.0f * 15000.0f;

                const auto &pts = bin.points;
                for (size_t i = 0; i < pts.size(); i += skip) {
                    glVertex3f(static_cast<float>(pts[i].mask),
                               height,
                               static_cast<float>(pts[i].seed));
                }
            }
        }
        glEnd();

        // Draw selection crosshair for reference
        glColor4f(1.0f, 1.0f, 1.0f, 0.2f);
        glBegin(GL_LINES);
        glVertex3f(cam_x - 1000.0f, 0.0f, cam_y); glVertex3f(cam_x + 1000.0f, 0.0f, cam_y);
        glVertex3f(cam_x, 0.0f, cam_y - 1000.0f); glVertex3f(cam_x, 0.0f, cam_y + 1000.0f);
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
                    float rad = cam_yaw * 3.14159265f / 180.0f;
                    float c = std::cos(rad), s = std::sin(rad);
                    cam_x -= (c * dx - s * dy) * (cam_dist * 0.001f);
                    cam_y -= (s * dx + c * dy) * (cam_dist * 0.001f);
                }
                redraw();
                return 1;
            }
            case FL_MOUSEWHEEL:
                cam_dist -= Fl::event_dy() * (cam_dist * 0.1f);
                if (cam_dist < 100.0f) cam_dist = 100.0f;
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

Fl_Check_Browser* browser = nullptr;
LFSRView* gl_view = nullptr;
Fl_Box* stat_box = nullptr;

void update_stats_display() {
    std::string s = "Statistics:\n";
    s += "Visible Bins: " + std::to_string(g_stats.visible_bins) + "\n";
    s += "Total Points: " + std::to_string(g_stats.total_points) + "\n";

    char buf[128];
    std::snprintf(buf, sizeof(buf), "Avg Mask: %.0f\nAvg Seed: %.0f\nDensity: %.6f%%",
                  g_stats.avg_mask, g_stats.avg_seed, g_stats.density);
    s += buf;

    if (stat_box) stat_box->copy_label(s.c_str());
}

void browser_cb(Fl_Widget*, void*) {
    // Iterate browser items and update load/visibility
    for (int i = 0; i < 256; ++i) {
        bool checked = browser->checked(i + 1);
        if (checked && !g_bins[i].visible) {
            load_bin(i);
            g_bins[i].visible = !g_bins[i].points.empty();
        } else if (!checked && g_bins[i].visible) {
            unload_bin(i);
        }
    }
    update_stats();
    update_stats_display();
    if (gl_view) gl_view->redraw();
}

void select_all_cb(Fl_Widget*, void*) {
    for (int i = 1; i <= 256; ++i) browser->checked(i, 1);
    browser_cb(nullptr, nullptr);
}

void clear_all_cb(Fl_Widget*, void*) {
    for (int i = 1; i <= 256; ++i) browser->checked(i, 0);
    browser_cb(nullptr, nullptr);
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int argc, char **argv) {
    Fl_Double_Window* win = new Fl_Double_Window(1280, 720, "LFSR Analytical Visualizer (FLTK/OpenGL)");

    // Left: OpenGL View
    gl_view = new LFSRView(10, 10, 980, 700);

    // Right: Controls group
    Fl_Group* controls = new Fl_Group(1000, 10, 270, 700);
    controls->begin(); // <-- IMPORTANT: begin group before creating its children

    // 1. Bin Selector
    browser = new Fl_Check_Browser(1000, 10, 270, 400, "PWM Bins (0x00 - 0xFF)");
    for (int i = 0; i < 256; ++i) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "Bin 0x%02X (%d)", i, i);
        browser->add(buf);
    }
    browser->callback(browser_cb);
    browser->when(FL_WHEN_CHANGED); // ensure callback on check-change

    Fl_Button* btn_all = new Fl_Button(1000, 415, 130, 25, "Select All");
    btn_all->callback(select_all_cb);

    Fl_Button* btn_none = new Fl_Button(1140, 415, 130, 25, "Clear");
    btn_none->callback(clear_all_cb);

    // 2. Statistics Panel
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

    // initial stats display
    update_stats();
    update_stats_display();

    return Fl::run();
}
