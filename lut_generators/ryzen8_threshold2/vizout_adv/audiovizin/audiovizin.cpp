// lfsr_visualizer_v2.cpp
//
// Features Added:
// - Real-time Audio Output (SDL2)
// - Playback Speed Control
// - Global Play/Stop
// - Optimized Keyboard Browsing (Arrow keys instantly switch audio)
//
// Build: g++ -std=c++17 -O3 -flto lfsr_visualizer_v2.cpp -o lfsr_vis_audio -lfltk -lfltk_gl -lGL -lGLU -lSDL2 -lpthread

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Hold_Browser.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Value_Slider.H> // For Speed Control
#include <FL/Fl_Group.H>
#include <FL/Fl_File_Chooser.H>
#include <FL/fl_draw.H>
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL2/SDL.h>

#include <vector>
#include <complex>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <atomic>

// --- Math & LFSR Engine ---

using u64 = uint64_t;
const double PI = 3.14159265358979323846;

struct Candidate {
    u64 mask;
    u64 seed;
    int period;
    int lc;
    double duty;
    double lf_energy;
    std::vector<int> bits; // 0 or 1
};

// Global Selection State
Candidate selected_candidate;
Candidate composer_lut[256];
std::vector<Candidate> current_bin_candidates;
int current_pwm_level = -1;
bool has_selection = false;

// --- Audio Engine (SDL2) ---

struct AudioState {
    std::atomic<bool> playing{false};
    std::atomic<double> playback_speed{1.0}; // 1.0 = 1 bit per sample
    std::atomic<size_t> play_head_idx{0};    // Current index in buffer (float simulation)
    double fractional_pos = 0.0;             // For resampling
    
    // Thread-safe data access
    std::mutex data_mutex;
    std::vector<int> active_buffer; // Copy of bits for the audio thread
    int active_period = 0;
};

AudioState audio_engine;

// SDL Audio Callback
void audio_callback(void* userdata, Uint8* stream, int len) {
    AudioState* state = (AudioState*)userdata;
    float* out = (float*)stream; // Assuming AUDIO_F32
    int sample_count = len / sizeof(float);

    // If not playing or empty, silence
    bool local_playing = state->playing.load();
    
    std::lock_guard<std::mutex> lock(state->data_mutex);
    if (!local_playing || state->active_period == 0 || state->active_buffer.empty()) {
        memset(stream, 0, len);
        return;
    }

    double speed = state->playback_speed.load();
    const auto& bits = state->active_buffer;
    int period = state->active_period;
    
    // Generate samples
    for (int i = 0; i < sample_count; ++i) {
        // Map 0/1 to -0.5 / +0.5 volume
        int current_bit = bits[(int)state->fractional_pos % period];
        float sample = (current_bit == 1) ? 0.5f : -0.5f;
        
        out[i] = sample;

        // Advance position
        state->fractional_pos += speed;
        // Wrap around logic handles effectively infinite loop
        if (state->fractional_pos >= period) {
            state->fractional_pos -= period;
        }
    }
}

void init_audio() {
    if (SDL_Init(SDL_INIT_AUDIO) < 0) {
        std::cerr << "SDL Audio Init Failed: " << SDL_GetError() << "\n";
        return;
    }

    SDL_AudioSpec want, have;
    SDL_zero(want);
    want.freq = 44100;
    want.format = AUDIO_F32;
    want.channels = 1; // Mono is fine for PWM
    want.samples = 1024; // Buffer size (latency)
    want.callback = audio_callback;
    want.userdata = &audio_engine;

    if (SDL_OpenAudio(&want, &have) < 0) {
        std::cerr << "SDL Open Audio Failed: " << SDL_GetError() << "\n";
    } else {
        SDL_PauseAudio(0); // Start processing callback (it will play silence if playing=false)
    }
}

// --- LFSR Helpers ---
static inline u64 next_state(u64 state, u64 mask, int n) {
    u64 fb = __builtin_parityll(state & mask);
    return (state >> 1) | (fb << (n - 1));
}

int calculate_lc(const std::vector<int>& bits) {
    int n = bits.size();
    if (n == 0) return 0;
    std::vector<int> C(n, 0), B(n, 0);
    C[0] = 1; B[0] = 1;
    int L = 0, m = -1;
    for (int N = 0; N < n; ++N) {
        int d = 0;
        for (int i = 0; i <= L; ++i) d ^= (C[i] & bits[N - i]);
        if (d) {
            std::vector<int> T = C;
            int p = N - m;
            for (int i = 0; i + p < n; ++i) C[i + p] ^= B[i];
            if (L <= N / 2) {
                L = N + 1 - L;
                m = N;
                B.swap(T);
            }
        }
    }
    return L;
}

void analyze_candidate(Candidate& cand, int n_bits) {
    std::vector<u64> states;
    states.reserve(65536);
    u64 curr = 1; 
    u64 start_node = 0x1;
    
    // Find cycle
    do {
        states.push_back(curr);
        curr = next_state(curr, cand.mask, n_bits);
    } while (curr != start_node && states.size() < (1<<17));
    
    cand.period = states.size();
    cand.bits.resize(cand.period);
    int ones = 0;
    for(int i=0; i<cand.period; ++i) {
        cand.bits[i] = (states[i] > cand.seed) ? 1 : 0;
        ones += cand.bits[i];
    }
    cand.duty = (double)ones / cand.period;

    double energy = 0.0;
    for (int k = 1; k <= 10; ++k) {
        std::complex<double> sum(0, 0);
        double angle_step = -2.0 * PI * k / cand.period;
        for (int t = 0; t < cand.period; ++t) {
            double val = (double)cand.bits[t] - 0.5;
            sum += std::polar(val, angle_step * t);
        }
        energy += std::norm(sum) / (cand.period * cand.period); 
    }
    cand.lf_energy = energy;
    
    int lc_sample = std::min(cand.period, 512);
    std::vector<int> sample(cand.bits.begin(), cand.bits.begin() + lc_sample);
    cand.lc = calculate_lc(sample);
}

// --- OpenGL Widget ---

class VisualizerGL : public Fl_Gl_Window {
public:
    VisualizerGL(int x, int y, int w, int h, const char* l = 0) : Fl_Gl_Window(x, y, w, h, l) {}

    void draw() override {
        if (!valid()) {
            glViewport(0, 0, w(), h());
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT);

        if (!has_selection || selected_candidate.bits.empty()) return;

        int W = w();
        int H = h();
        
        // 1. Waveform
        glViewport(0, H * 2 / 3, W, H / 3);
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluOrtho2D(0, std::min(512, selected_candidate.period), -0.1, 1.1);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();

        glColor3f(0.0f, 1.0f, 0.0f); 
        glBegin(GL_LINE_STRIP);
        for (int i = 0; i < std::min(512, selected_candidate.period); ++i) {
            glVertex2f(i, selected_candidate.bits[i]);
            glVertex2f(i + 1, selected_candidate.bits[i]); 
        }
        glEnd();
        
        // 2. Spectrum
        glViewport(0, H / 3, W, H / 3);
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); 
        gluOrtho2D(0, 64, 0, 1.0); 
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();

        glColor3f(0.2f, 0.2f, 0.2f);
        glBegin(GL_QUADS); glVertex2f(0,0); glVertex2f(64,0); glVertex2f(64,1); glVertex2f(0,1); glEnd(); 

        glColor3f(1.0f, 0.8f, 0.0f); 
        glBegin(GL_LINES);
        for (int k = 1; k < 64; ++k) {
            std::complex<double> sum(0, 0);
            double angle_step = -2.0 * PI * k / selected_candidate.period;
            int step = 1;
            if(selected_candidate.period > 2048) step = selected_candidate.period / 2048;
            for (int t = 0; t < selected_candidate.period; t+=step) {
                double val = (double)selected_candidate.bits[t] - 0.5;
                sum += std::polar(val, angle_step * t);
            }
            double mag = std::abs(sum) / (selected_candidate.period/step);
            glVertex2f(k, 0);
            glVertex2f(k, mag * 5.0); 
        }
        glEnd();

        // 3. Raster Map
        glViewport(0, 0, W, H / 3);
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); 
        int dim = (int)sqrt(selected_candidate.period);
        if(dim == 0) dim = 1;
        gluOrtho2D(0, dim, 0, dim);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        
        glPointSize(2.0f);
        glBegin(GL_POINTS);
        for(int i=0; i<selected_candidate.period; ++i) {
            if(selected_candidate.bits[i]) glColor3f(1.0f, 1.0f, 1.0f);
            else glColor3f(0.0f, 0.0f, 0.0f);
            int x = i % dim;
            int y = i / dim;
            glVertex2i(x, y);
        }
        glEnd();
    }
};

// --- GUI Logic ---

Fl_Hold_Browser* bin_browser;
Fl_Hold_Browser* cand_browser;
VisualizerGL* gl_view;
Fl_Value_Output* out_duty;
Fl_Value_Output* out_lc;
Fl_Value_Output* out_lf;
Fl_Button* btn_commit;
Fl_Button* btn_play_stop;
Fl_Value_Slider* sld_speed;

std::string dataset_path = ".";

// Helper to push selection to audio engine safely
void update_audio_source() {
    if (!has_selection) return;
    
    // Critical section: Update audio buffer
    std::lock_guard<std::mutex> lock(audio_engine.data_mutex);
    audio_engine.active_buffer = selected_candidate.bits;
    audio_engine.active_period = selected_candidate.period;
    
    // If we want "seamless" transition keeping phase, we do nothing to fractional_pos
    // If we want restart, set fractional_pos = 0. 
    // Restarting usually feels snappier for comparing attacks/patterns.
    audio_engine.fractional_pos = 0.0; 
}

void load_bin(int pwm_level) {
    current_bin_candidates.clear();
    cand_browser->clear();
    current_pwm_level = pwm_level;

    char filename[512];
    snprintf(filename, sizeof(filename), "%s/0x%02X.bin", dataset_path.c_str(), pwm_level);
    
    std::ifstream f(filename, std::ios::binary);
    if (!f) return;

    // Detect format (assuming 16-bit default for demo)
    f.seekg(0, std::ios::end);
    size_t sz = f.tellg();
    f.seekg(0, std::ios::beg);
    struct Entry16 { uint16_t m; uint16_t s; };
    size_t count = sz / sizeof(Entry16);
    if (count > 2000) count = 2000; 

    std::vector<Candidate> temp_cands;
    temp_cands.reserve(count);

    for(size_t i=0; i<count; ++i) {
        Candidate c;
        Entry16 e; f.read((char*)&e, sizeof(e));
        c.mask = e.m; c.seed = e.s;
        analyze_candidate(c, 16); 
        temp_cands.push_back(c);
    }
    
    std::sort(temp_cands.begin(), temp_cands.end(), [](const Candidate& a, const Candidate& b){
        return a.lf_energy < b.lf_energy;
    });

    current_bin_candidates = temp_cands;

    for (const auto& c : current_bin_candidates) {
        char buf[256];
        sprintf(buf, "LF:%.4f  LC:%d  Per:%d", c.lf_energy, c.lc, c.period);
        cand_browser->add(buf);
    }
    
    if (composer_lut[pwm_level].period != 0) {
        btn_commit->label("@+ Checked");
        btn_commit->color(FL_GREEN);
    } else {
        btn_commit->label("Select");
        btn_commit->color(FL_BACKGROUND_COLOR);
    }
}

void cb_bins(Fl_Widget*, void*) {
    int line = bin_browser->value();
    if (line == 0) return;
    load_bin(line - 1);
}

// Callback for selection change (Click or Keyboard Arrow)
void cb_cands(Fl_Widget*, void*) {
    int line = cand_browser->value();
    if (line == 0) return;
    int idx = line - 1;
    
    if (idx < current_bin_candidates.size()) {
        selected_candidate = current_bin_candidates[idx];
        has_selection = true;
        
        // 1. Update UI Text
        out_duty->value(selected_candidate.duty);
        out_lc->value(selected_candidate.lc);
        out_lf->value(selected_candidate.lf_energy);
        
        // 2. Update Audio IMMEDIATELY
        // This allows browsing with arrows to instantly hear the new candidate
        update_audio_source(); 
        
        // 3. Update Graphics
        // FLTK redraw is just a flag setting, it happens asynchronously in main loop
        // so it won't block the audio or navigation significantly.
        gl_view->redraw();
    }
}

void cb_play_stop(Fl_Widget* w, void*) {
    bool is_playing = audio_engine.playing.load();
    if (is_playing) {
        audio_engine.playing = false;
        ((Fl_Button*)w)->label("@> Play");
        ((Fl_Button*)w)->color(FL_BACKGROUND_COLOR);
    } else {
        audio_engine.playing = true;
        ((Fl_Button*)w)->label("@|| Stop");
        ((Fl_Button*)w)->color(FL_YELLOW);
        // Ensure current selection is loaded
        update_audio_source();
    }
}

void cb_speed(Fl_Widget* w, void*) {
    Fl_Value_Slider* s = (Fl_Value_Slider*)w;
    audio_engine.playback_speed = s->value();
}

void cb_commit(Fl_Widget*, void*) {
    if (!has_selection || current_pwm_level < 0) return;
    composer_lut[current_pwm_level] = selected_candidate;
    btn_commit->label("@+ Checked");
    btn_commit->color(FL_GREEN);
    char buf[64];
    sprintf(buf, "Level %d [SET]", current_pwm_level);
    bin_browser->text(current_pwm_level + 1, buf);
}

void cb_export(Fl_Widget*, void*) {
    std::ofstream f("custom_lfsr_lut.h");
    f << "#ifndef LFSR_LUT_H\n#define LFSR_LUT_H\n#include <stdint.h>\n\n";
    f << "struct LFSR_Entry { uint16_t mask; uint16_t threshold; };\n\n";
    f << "const LFSR_Entry lfsr_lut[256] = {\n";
    int filled = 0;
    for(int i=0; i<256; ++i) {
        if (composer_lut[i].period == 0) f << "  { 0x0000, 0x0000 },\n";
        else {
            f << "  { 0x" << std::hex << std::setw(4) << std::setfill('0') << composer_lut[i].mask
              << ", 0x" << std::setw(4) << composer_lut[i].seed << " }, // " << std::dec << composer_lut[i].duty << "\n";
            filled++;
        }
    }
    f << "};\n#endif\n";
    f.close();
    fl_message("Exported lut.h with %d entries.", filled);
}

void cb_dir(Fl_Widget*, void*) {
    Fl_File_Chooser chooser(".", "*", Fl_File_Chooser::DIRECTORY, "Select Bin Dir");
    chooser.show();
    while(chooser.shown()) Fl::wait();
    if (chooser.value()) {
        dataset_path = chooser.value();
        bin_browser->clear();
        for(int i=0; i<256; ++i) {
            char buf[64];
            sprintf(buf, "Level %d", i);
            bin_browser->add(buf);
        }
    }
}

int main(int argc, char **argv) {
    // Init Audio
    init_audio();

    Fl_Double_Window *win = new Fl_Double_Window(1000, 750, "LFSR Visualizer + Audio");

    // Columns
    bin_browser = new Fl_Hold_Browser(10, 40, 150, 650, "PWM Levels");
    bin_browser->callback(cb_bins);

    Fl_Button* btn_dir = new Fl_Button(10, 10, 150, 25, "Open Dataset");
    btn_dir->callback(cb_dir);

    // Candidates
    cand_browser = new Fl_Hold_Browser(170, 40, 200, 500, "Candidates");
    cand_browser->textfont(FL_COURIER);
    cand_browser->callback(cb_cands);

    // Audio Controls (Middle column, below list)
    Fl_Group* grp_audio = new Fl_Group(170, 550, 200, 140, "Audio & Info");
    grp_audio->box(FL_ENGRAVED_BOX);
    
    btn_play_stop = new Fl_Button(180, 560, 180, 30, "@> Play");
    btn_play_stop->callback(cb_play_stop);
    
    sld_speed = new Fl_Value_Slider(180, 600, 180, 20, "Rate");
    sld_speed->type(FL_HOR_SLIDER);
    sld_speed->bounds(0.1, 4.0); // 0.1x to 4x speed
    sld_speed->value(1.0);
    sld_speed->callback(cb_speed);

    out_lf = new Fl_Value_Output(220, 630, 140, 25, "Flicker:");
    out_lc = new Fl_Value_Output(220, 660, 140, 25, "LC:");
    grp_audio->end();

    // Visualizer
    gl_view = new VisualizerGL(380, 40, 610, 600);
    gl_view->box(FL_DOWN_FRAME);

    // Bottom
    btn_commit = new Fl_Button(380, 650, 200, 40, "Select for LUT");
    btn_commit->callback(cb_commit);
    
    Fl_Button* btn_export = new Fl_Button(790, 650, 200, 40, "Export lut.h");
    btn_export->callback(cb_export);

    win->resizable(gl_view);
    win->show(argc, argv);

    // Ensure cleanup
    int ret = Fl::run();
    SDL_CloseAudio();
    SDL_Quit();
    return ret;
}
