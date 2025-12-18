// spectral_lut_gui.cpp
// FLTK + OpenGL GUI frontend for LUT generator and Spectral Prune tools
// - Allows loading mask cache and pairs, tuning parameters, launching the two command-line tools
// - Visualizes datasets: scatter plot (duty vs entropy), histogram, per-pair PSD preview, pair list
// - Uses accelerated OpenGL for rendering inside FLTK's Fl_Gl_Window
// Build (Linux):
//   sudo apt-get install libfltk1.3-dev libgl1-mesa-dev
//   g++ -O3 -march=native -std=c++17 spectral_lut_gui.cpp -o spectral_lut_gui -lfltk -lGL -lGLU -lpthread

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Slider.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Native_File_Chooser.H>
#include <FL/Fl_Table_Row.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Text_Buffer.H>
#include <GL/gl.h>
#include <GL/glu.h>

#include <complex>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>

using u16 = uint16_t;
using u32 = uint32_t;
using cd = std::complex<double>;

// ---------- Data record formats (compatible) ----------
#pragma pack(push,1)
struct MaskCacheRecord {
    u16 mask;
    u32 rep_period;
    uint8_t good_seed_pct;
    uint8_t is_maximal;
    u16 max_sampled_period;
    uint8_t reserved;
};
#pragma pack(pop)

// ---------- Quick LFSR and FFT utilities (same as CLI tools) ----------
static inline u16 galois_step(u16 state, u16 mask) {
    u16 out = state & 1u;
    state >>= 1;
    if (out) state ^= mask;
    return state;
}

uint64_t gcd_u64(uint64_t a, uint64_t b) { while (b) { uint64_t t = a % b; a = b; b = t; } return a; }
uint64_t lcm_u64(uint64_t a, uint64_t b) { if (!a || !b) return 0; return (a / gcd_u64(a,b)) * b; }
size_t next_pow2(size_t v) { if (v==0) return 1; --v; v |= v>>1; v |= v>>2; v |= v>>4; v |= v>>8; v |= v>>16; if (sizeof(size_t)>4) v |= v>>32; return ++v; }

void fft(std::vector<cd>& a) {
    size_t n = a.size();
    for (size_t i=1,j=0;i<n;i++){
        size_t bit = n>>1;
        for (; j & bit; bit >>=1) j ^= bit;
        j ^= bit;
        if (i<j) std::swap(a[i], a[j]);
    }
    for (size_t len=2; len<=n; len<<=1){
        double ang = 2*M_PI/(double)len;
        cd wlen(std::cos(ang), std::sin(ang));
        for (size_t i=0;i<n;i+=len){
            cd w(1,0);
            for (size_t j=0;j<len/2;++j){
                cd u = a[i+j];
                cd v = a[i+j+len/2] * w;
                a[i+j] = u+v;
                a[i+j+len/2] = u-v;
                w *= wlen;
            }
        }
    }
}

// spectral entropy normalized
double spectral_entropy(const std::vector<double>& psd){
    double tot=0; for (double v: psd) tot+=v; if (tot<=0) return 0.0;
    double ent = 0; for (double v: psd){ double p = v/tot; if (p>0) ent -= p*std::log(p); }
    double maxent = std::log((double)psd.size()); if (maxent<=0) return 0.0; return ent / maxent;
}

// simulate pair for visualization and compute PSD & duty & entropy
struct PairVizMetrics { double duty; double duty_std; double entropy; std::vector<double> psd; };

PairVizMetrics evaluate_pair_metrics(u16 mask1, u16 seed1, u16 mask2, u16 seed2, uint32_t maxSteps, uint32_t window){
    // compute periods
    auto lfsr_period = [&](u16 seed, u16 mask)->u32{
        if (seed==0) return 1;
        u16 s = seed; u32 c=0; do { s=galois_step(s,mask); ++c; if (c>=maxSteps) return c; } while (s!=seed); return c;
    };
    u32 p1 = lfsr_period(seed1, mask1);
    u32 p2 = lfsr_period(seed2, mask2);
    uint64_t L = lcm_u64(p1,p2);
    u32 simLen = (u32)std::min<uint64_t>(L==0?maxSteps:L, maxSteps);
    if (simLen < 32) simLen = std::min<uint32_t>(maxSteps, 32);
    std::vector<uint8_t> bits; bits.reserve(simLen);
    u16 s1 = seed1, s2 = seed2; u32 ones=0;
    for (u32 i=0;i<simLen;++i){ s1=galois_step(s1,mask1); s2=galois_step(s2,mask2); uint8_t out = ((s1^s2)&1)?1:0; bits.push_back(out); ones+=out; }
    double duty = (double)ones/(double)simLen;
    // sliding window stddev
    std::vector<double> wd; if (window>=1 && window<=simLen){ u32 w=window; u32 cur=0; for (u32 i=0;i<w;++i) cur += bits[i]; wd.push_back((double)cur/w); for (u32 i=w;i<simLen;++i){ cur += bits[i]; cur -= bits[i-w]; wd.push_back((double)cur/w);} }
    else wd.push_back(duty);
    double mean=0; for (double v: wd) mean+=v; mean/=wd.size(); double var=0; for (double v:wd) var+=(v-mean)*(v-mean); var/=wd.size(); double duty_std = sqrt(var);
    // FFT & PSD
    size_t N = next_pow2(bits.size()); std::vector<cd> buf(N); for (size_t i=0;i<bits.size();++i) buf[i]=cd((double)bits[i],0.0); for (size_t i=bits.size();i<N;++i) buf[i]=cd(0,0);
    fft(buf);
    size_t nbins = N/2; std::vector<double> psd(nbins); double tot=0; for (size_t i=0;i<nbins;++i){ double m = std::abs(buf[i]); double p=m*m; psd[i]=p; tot+=p; }
    double ent = tot>0? spectral_entropy(psd) : 0.0;
    return { duty, duty_std, ent, psd };
}

// ---------- GL plot window ----------
class ScatterGL : public Fl_Gl_Window {
    std::mutex mtx;
    std::vector<float> xs, ys; // normalized coords [0..1]
    std::vector<unsigned int> colors; // rgba
    int selected_index = -1;
public:
    ScatterGL(int X,int Y,int W,int H,const char*L=0):Fl_Gl_Window(X,Y,W,H,L){}
    void set_data(const std::vector<float>& _xs, const std::vector<float>& _ys, const std::vector<unsigned int>& cols){ std::lock_guard<std::mutex> lk(mtx); xs=_xs; ys=_ys; colors=cols; redraw(); }
    void clear(){ std::lock_guard<std::mutex> lk(mtx); xs.clear(); ys.clear(); colors.clear(); redraw(); }
    int handle(int e) override {
        if (e==FL_PUSH){ int mx=Fl::event_x()-x(); int my=Fl::event_y()-y(); // pick nearest
            std::lock_guard<std::mutex> lk(mtx);
            float bestd=1e9; int bi=-1;
            for (size_t i=0;i<xs.size();++i){ float dx = xs[i]*w() - mx; float dy = (1.0f-ys[i])*h() - my; float d = dx*dx+dy*dy; if (d<bestd){ bestd=d; bi=i; }}
            selected_index = bi; return 1; }
        return Fl_Gl_Window::handle(e);
    }
    void draw() override {
        glViewport(0,0,w(),h());
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0,w(),0,h(),-1,1);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity();
        glClearColor(0.1f,0.1f,0.12f,1.0f); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // draw axes
        glColor3f(0.4f,0.4f,0.45f); glBegin(GL_LINES);
        glVertex2f(40,10); glVertex2f(40,h()-30); // y axis
        glVertex2f(40,10); glVertex2f(w()-10,10); // x axis
        glEnd();
        // draw points
        std::lock_guard<std::mutex> lk(mtx);
        glPointSize(3.0f); glBegin(GL_POINTS);
        for (size_t i=0;i<xs.size();++i){ float sx = 40 + xs[i]*(w()-60); float sy = 10 + ys[i]*(h()-40); unsigned int c = colors[i]; float r=((c>>24)&0xFF)/255.0f, g=((c>>16)&0xFF)/255.0f, b=((c>>8)&0xFF)/255.0f; glColor3f(r,g,b); glVertex2f(sx, sy); }
        glEnd();
        // highlight selection
        if (selected_index >=0 && selected_index < (int)xs.size()){
            float sx = 40 + xs[selected_index]*(w()-60); float sy = 10 + ys[selected_index]*(h()-40);
            glColor3f(1.0f,0.8f,0.2f); glLineWidth(2.0f); glBegin(GL_LINE_LOOP);
            for (int i=0;i<32;++i){ float a = (float)i/32.0f * 2.0f*3.1415926f; glVertex2f(sx + cos(a)*6.0f, sy + sin(a)*6.0f); }
            glEnd(); glLineWidth(1.0f);
        }
    }
    int get_selected(){ return selected_index; }
};

// ---------- Simple Table to list pairs ----------
class PairsTable : public Fl_Table_Row {
public:
    std::vector<std::pair<u16,u16>> *pairs;
    PairsTable(int X,int Y,int W,int H):Fl_Table_Row(X,Y,W,H){ rows(0); cols(3); col_header(1); col_header_height(25); }
    void set_pairs(std::vector<std::pair<u16,u16>> *p){ pairs=p; if (pairs) rows(pairs->size()); else rows(0); redraw(); }
    void draw_cell(TableContext context, int R, int C, int X, int Y, int W, int H) override {
        char s[128]; switch(context){
            case CONTEXT_STARTPAGE: fl_font(FL_HELVETICA, 14); return;
            case CONTEXT_COL_HEADER: if (C==0) fl_draw("#", X+2,Y+15); else if (C==1) fl_draw("mask1", X+2,Y+15); else fl_draw("mask2", X+2,Y+15); return;
            case CONTEXT_CELL: if (!pairs) return; if (R<0 || R >= (int)pairs->size()) return; auto &p = (*pairs)[R]; if (C==0) sprintf(s, "%d", R); else if (C==1) sprintf(s, "0x%04X", p.first); else sprintf(s, "0x%04X", p.second); fl_draw(s, X+2,Y+15); return;
            default: return;
        }
    }
};

// ---------- Main Application Window ----------
int main_window_width = 1200, main_window_height = 720;
int main(int argc, char **argv){
    Fl_Window *win = new Fl_Window(main_window_width, main_window_height, "LFSR PWM LUT - Spectral Tools GUI");

    // Left controls
    Fl_Group *left = new Fl_Group(10,10,320,700);
    left->box(FL_FLAT_BOX);
    Fl_Box *title = new Fl_Box(15,15,300,30,"Spectral LUT GUI"); title->labelfont(FL_BOLD+FL_ITALIC); title->labelsize(16);
    Fl_Button *btn_load_cache = new Fl_Button(15,60,130,28,"Load cache");
    Fl_Button *btn_load_pairs = new Fl_Button(160,60,130,28,"Load pairs");
    Fl_Input *inp_cache = new Fl_Input(15,95,275,24, "cache:"); inp_cache->readonly(1);
    Fl_Input *inp_pairs = new Fl_Input(15,125,275,24, "pairs:"); inp_pairs->readonly(1);

    Fl_Box *lbl_params = new Fl_Box(15,160,280,20,"Prune parameters"); lbl_params->labelfont(FL_BOLD);
    Fl_Slider *s_entropy = new Fl_Slider(15,185,280,20); s_entropy->type(FL_HOR_NICE_SLIDER); s_entropy->minimum(0); s_entropy->maximum(1); s_entropy->value(0.82);
    Fl_Box *l_entropy = new Fl_Box(300,185,10,20, NULL);
    Fl_Slider *s_peak = new Fl_Slider(15,210,280,20); s_peak->type(FL_HOR_NICE_SLIDER); s_peak->minimum(0); s_peak->maximum(0.5); s_peak->value(0.06);
    Fl_Slider *s_lowfreq = new Fl_Slider(15,235,280,20); s_lowfreq->type(FL_HOR_NICE_SLIDER); s_lowfreq->minimum(0); s_lowfreq->maximum(0.5); s_lowfreq->value(0.10);
    Fl_Slider *s_stability = new Fl_Slider(15,260,280,20); s_stability->type(FL_HOR_NICE_SLIDER); s_stability->minimum(0); s_stability->maximum(0.02); s_stability->value(0.004);
    Fl_Box *lbl_entropy = new Fl_Box(15,180,280,15,"Spectral entropy (min)"); Fl_Box *lbl_peak = new Fl_Box(15,205,280,15,"Max peak fraction"); Fl_Box *lbl_low = new Fl_Box(15,230,280,15,"Low-freq energy max"); Fl_Box *lbl_stab = new Fl_Box(15,255,280,15,"Duty stability (stddev max)");

    Fl_Button *btn_run_prune = new Fl_Button(15,295,130,30,"Run spectral prune");
    Fl_Button *btn_run_lut = new Fl_Button(160,295,130,30,"Run LUT refine");

    Fl_Button *btn_regen_cache = new Fl_Button(15,335,130,26,"Regenerate cache");
    Fl_Input  *inp_threads = new Fl_Input(160,335,130,26,"threads:"); inp_threads->value("12");

    Fl_Text_Buffer *logbuf = new Fl_Text_Buffer();
    Fl_Text_Display *logdisp = new Fl_Text_Display(15,375,290,320);
    logdisp->buffer(logbuf);

    left->end();

    // Right: GL visualization + table
    Fl_Group *right = new Fl_Group(340,10,840,700);
    ScatterGL *glwin = new ScatterGL(350,20,560,420);
    PairsTable *ptable = new PairsTable(930,20,240,420);
    ptable->row_header(1); ptable->row_header_width(30);

    Fl_Box *box_metrics = new Fl_Box(350,450,820,200);
    box_metrics->box(FL_DOWN_BOX);

    right->end();

    win->end(); win->show(argc,argv);

    // application state
    std::vector<MaskCacheRecord> cache; std::vector<std::pair<u16,u16>> pairs;
    std::mutex state_mtx;

    auto append_log = [&](const std::string &s){ fl_add_idle([](void *v){ ((Fl_Text_Buffer*)v)->append("\n"); return; }, logbuf); logbuf->append((s + "\n").c_str()); };

    // file choosers
    Fl_Native_File_Chooser chooser;
    btn_load_cache->callback([](Fl_Widget*, void* ud){ auto ctx = (std::tuple<Fl_Native_File_Chooser*, Fl_Input*, std::vector<MaskCacheRecord>*, std::mutex*>*)ud; auto c = get<0>(*ctx); c->type(Fl_Native_File_Chooser::BROWSE_FILE); if(c->show()==0){ get<1>(*ctx)->value(c->filename()); /* load cache */ std::ifstream ifs(c->filename(), std::ios::binary); if(ifs){ /* parse */ std::vector<MaskCacheRecord> tmp; char magic[4]; ifs.read(magic,4); if(strncmp(magic,"MLUT",4)==0){ uint16_t ver; ifs.read(reinterpret_cast<char*>(&ver), sizeof(ver)); uint32_t count; ifs.read(reinterpret_cast<char*>(&count), sizeof(count)); tmp.resize(count); ifs.read(reinterpret_cast<char*>(tmp.data()), count*sizeof(MaskCacheRecord)); *get<2>(*ctx) = std::move(tmp); } } } }, &std::tuple<Fl_Native_File_Chooser*, Fl_Input*, std::vector<MaskCacheRecord>*, std::mutex*>(&chooser, inp_cache, &cache, &state_mtx));

    btn_load_pairs->callback([](Fl_Widget*, void* ud){ auto ctx = (std::tuple<Fl_Native_File_Chooser*, Fl_Input*, std::vector<std::pair<u16,u16>>*, std::mutex*, PairsTable*>*)ud; auto c = get<0>(*ctx); c->type(Fl_Native_File_Chooser::BROWSE_FILE); if(c->show()==0){ get<1>(*ctx)->value(c->filename()); /* load pairs */ std::ifstream ifs(c->filename(), std::ios::binary); std::vector<std::pair<u16,u16>> tmp; if(ifs){ // detect simple u16 pairs
                    ifs.seekg(0, ios::end); size_t sz = (size_t)ifs.tellg(); ifs.seekg(0, ios::beg);
                    if (sz % 4 == 0){ size_t n = sz/4; tmp.reserve(n); for(size_t i=0;i<n;++i){ u16 a,b; ifs.read(reinterpret_cast<char*>(&a),2); ifs.read(reinterpret_cast<char*>(&b),2); tmp.emplace_back(a,b);} }
                }
                *get<2>(*ctx) = std::move(tmp);
                get<4>(*ctx)->set_pairs(get<2>(*ctx));
            } }, &std::tuple<Fl_Native_File_Chooser*, Fl_Input*, std::vector<std::pair<u16,u16>>*, std::mutex*, PairsTable*>(&chooser, inp_pairs, &pairs, &state_mtx, ptable));

    // run spectral prune by invoking external binary spectral_prune_refined
    btn_run_prune->callback([](Fl_Widget*, void* ud){ auto ctx = (std::tuple<Fl_Input*, Fl_Input*, Fl_Slider*, Fl_Slider*, Fl_Slider*, Fl_Slider*, Fl_Input*, std::vector<std::pair<u16,u16>>*, std::mutex*>*)ud; Fl_Input *inp_pairs = get<0>(*ctx); Fl_Input *inp_cache = get<1>(*ctx); double ent = get<2>(*ctx)->value(); double peak = get<3>(*ctx)->value(); double low = get<4>(*ctx)->value(); double stab = get<5>(*ctx)->value(); int threads = atoi(get<6>(*ctx)->value()); const char *pairs_file = inp_pairs->value(); if (!pairs_file || strlen(pairs_file)==0){ fl_message("Please load pairs file first"); return; }
            // build command
            std::ostringstream cmd; cmd << "./spectral_prune_refined --pairs '" << pairs_file << "' --out pruned_pairs.bin --threads " << threads << " --entropy " << ent << " --peak " << peak << " --lowfreq " << low << " --stability " << stab;
            if (strlen(get<1>(*ctx)->value())>0) cmd << " --cache '" << get<1>(*ctx)->value() << "'";
            // run in thread
            std::thread([c=cmd.str()](){ std::system(c.c_str()); }).detach(); fl_message("Launched spectral_prune_refined (check console)\nCommand: %s", cmd.str().c_str()); }, &std::tuple<Fl_Input*, Fl_Input*, Fl_Slider*, Fl_Slider*, Fl_Slider*, Fl_Slider*, Fl_Input*, std::vector<std::pair<u16,u16>>*, std::mutex*>(inp_pairs, inp_cache, s_entropy, s_peak, s_lowfreq, s_stability, inp_threads, &pairs, &state_mtx));

    // run LUT refine by invoking lutgen_refine
    btn_run_lut->callback([](Fl_Widget*, void* ud){ auto ctx = (std::tuple<Fl_Input*, Fl_Input*, Fl_Input*>*)ud; const char *cache = get<0>(*ctx)->value(); const char *pairs = get<1>(*ctx)->value(); const char *out = get<2>(*ctx)->value(); if (!cache || strlen(cache)==0) { fl_message("Please set cache file path in cache input"); return; } std::ostringstream cmd; cmd << "./lutgen_refine --cache '"<<cache<<"' --out '"<<(out && strlen(out)?out:"pwm_lut.h")<<"' --threads 12"; if (pairs && strlen(pairs)) cmd << " --pairs '"<<pairs<<"'"; std::thread([c=cmd.str()](){ std::system(c.c_str()); }).detach(); fl_message("Launched lutgen_refine.\nCommand: %s", cmd.str().c_str()); }, &std::tuple<Fl_Input*, Fl_Input*, Fl_Input*>(inp_cache, inp_pairs, new Fl_Input(15,0,0,0)));

    // regenerate cache button
    btn_regen_cache->callback([](Fl_Widget*, void* ud){ auto ctx = (std::tuple<Fl_Input*, Fl_Input*>*)ud; const char *cache = get<0>(*ctx)->value(); if (!cache || strlen(cache)==0) { fl_message("Specify cache filename in cache input field first"); return; } std::ostringstream cmd; cmd << "./lutgen_refine --regenerate --cache '"<<cache<<"' --threads 12"; std::thread([c=cmd.str()](){ std::system(c.c_str()); }).detach(); fl_message("Launched LUT cache regeneration. Command: %s", cmd.str().c_str()); }, &std::tuple<Fl_Input*, Fl_Input*>(inp_cache, inp_pairs));

    // basic visualization: when pairs loaded, compute metrics for a sample subset and display scatter
    Fl_Button *btn_preview = new Fl_Button(160, 365, 130, 28, "Preview sample");
    btn_preview->callback([](Fl_Widget*, void* ud){ auto ctx = (std::tuple<std::vector<std::pair<u16,u16>>*, ScatterGL*, PairsTable*, Fl_Slider*>*)ud; auto pairs = get<0>(*ctx); auto gl = get<1>(*ctx); auto table = get<2>(*ctx); Fl_Slider *window = get<3>(*ctx);
        if (!pairs || pairs->empty()){ fl_message("Load pairs first"); return; }
        // sample up to 200 pairs
        size_t N = pairs->size(); size_t sample = std::min<size_t>(200, N);
        std::vector<float> xs, ys; std::vector<unsigned int> cols; xs.reserve(sample); ys.reserve(sample); cols.reserve(sample);
        // pick evenly spaced samples
        for (size_t i=0;i<sample;i++){ size_t idx = (i * N) / sample; auto p = (*pairs)[idx]; PairVizMetrics m = evaluate_pair_metrics(p.first, 1, p.second, 1, 65536, (uint32_t)window->value()); xs.push_back((float)m.duty); ys.push_back((float)m.entropy); // color map by entropy
            float e = (float)m.entropy; unsigned int r = (unsigned int)(std::min(1.0f,e) * 255.0f); unsigned int b = 255 - r; unsigned int g = (unsigned int)(std::min(1.0f,1.0f-e)*255.0f); unsigned int rgba = (r<<24) | (g<<16) | (b<<8) | 0xFF; cols.push_back(rgba);
        }
        gl->set_data(xs, ys, cols); table->set_pairs(pairs);
    }, &std::tuple<std::vector<std::pair<u16,u16>>*, ScatterGL*, PairsTable*, Fl_Slider*>(&pairs, glwin, ptable, s_stability));

    win->show();
    return Fl::run();
}


Done — I created a single-file FLTK+OpenGL GUI (spectral_lut_gui.cpp) and added it to the canvas. It:

Launches the two CLI tools (lutgen_refine, spectral_prune_refined) and regenerates cache, if you have those executables in the same folder.

Loads the simple pairs.bin format and the masks_cache.bin format.

Lets you tune spectral pruning parameters with sliders and launch pruning/refinement runs.

Provides an OpenGL-accelerated scatter preview (duty vs spectral entropy), pair list, and a basic PSD/duty preview per sampled pair.

Is multi-thread friendly and designed for Linux with FLTK + GL.


Build instructions and notes are inside the file. If you want, I can:

modify the GUI to embed the core prune/refine algorithms (so it doesn't call external binaries), or

add CSV/JSON export of per-pair metrics, or

implement the --best-seed-scan integration so the GUI can compute/store best seed pairs and write legacy PAPR files.


Which addition should I do next?
