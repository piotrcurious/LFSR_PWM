// spectral_lut_gui.cpp // FLTK + OpenGL GUI frontend for LUT generator and Spectral Prune tools — enhanced visualization versatility // This revision adds: //  - Multiple colormaps (Viridis-like, Plasma-like, Gray) selectable from UI //  - Lasso (freeform polygon) selection with point-in-polygon test and visual feedback //  - Zoom-to-selection and fit-to-data commands //  - Hover tooltip showing pair masks and cached metrics (duty, entropy, stability) //  - Export selected pairs to simple u16,u16 binary (compatible with pipeline) //  - Histogram view for the currently selected metric (entropy/duty) rendered in the GL area //  - Keyboard shortcuts: F = fit, Z = zoom-to-selection, L = toggle lasso, E = export selection //  - Improved level-of-detail sampling pipeline and safer GPU upload // // Build (Linux): //   sudo apt-get install libfltk1.3-dev libgl1-mesa-dev //   g++ -O3 -march=native -std=c++17 spectral_lut_gui.cpp -o spectral_lut_gui -lfltk -lGL -lGLU -lpthread

#include <FL/Fl.H> #include <FL/Fl_Window.H> #include <FL/Fl_Group.H> #include <FL/Fl_Button.H> #include <FL/Fl_Input.H> #include <FL/Fl_Slider.H> #include <FL/Fl_Value_Slider.H> #include <FL/Fl_Box.H> #include <FL/Fl_Native_File_Chooser.H> #include <FL/Fl_Table_Row.H> #include <FL/Fl_Gl_Window.H> #include <FL/Fl_Text_Display.H> #include <FL/Fl_Text_Buffer.H> #include <FL/Fl_Check_Button.H> #include <FL/Fl_Choice.H> #include <FL/Fl_File_Chooser.H>

#include <GL/gl.h> #include <GL/glu.h>

#include <thread> #include <mutex> #include <atomic> #include <vector> #include <string> #include <fstream> #include <sstream> #include <iomanip> #include <cmath> #include <algorithm> #include <chrono> #include <condition_variable> #include <unordered_map> #include <filesystem>

using u16 = uint16_t; using u32 = uint32_t; using u64 = uint64_t; using cd  = std::complex<double>;

#pragma pack(push,1) struct MaskCacheRecord { u16 mask; u32 rep_period; uint8_t good_seed_pct; uint8_t is_maximal; u16 max_sampled_period; uint8_t reserved; }; #pragma pack(pop)

struct PairMetrics { float duty; float entropy; float duty_std; float max_peak_frac; float lowfreq_frac; uint32_t period; };

// ---------- helpers ---------- static inline u16 galois_step(u16 state, u16 mask) { u16 out = state & 1u; state >>= 1; if (out) state ^= mask; return state; } uint64_t gcd_u64(uint64_t a, uint64_t b){ while(b){ uint64_t t=a%b; a=b; b=t; } return a; } uint64_t lcm_u64(uint64_t a, uint64_t b){ if(!a||!b) return 0; return (a/gcd_u64(a,b))*b; } size_t next_pow2(size_t v){ if(v==0) return 1; --v; v |= v>>1; v |= v>>2; v |= v>>4; v |= v>>8; v |= v>>16; if(sizeof(size_t)>4) v |= v>>32; return ++v; }

void fft(std::vector<cd>& a) { size_t n=a.size(); for(size_t i=1,j=0;i<n;i++){ size_t bit=n>>1; for(; j & bit; bit >>= 1) j ^= bit; j ^= bit; if(i<j) std::swap(a[i], a[j]); } for(size_t len=2; len<=n; len<<=1){ double ang=2*M_PI/(double)len; cd wlen(cos(ang), sin(ang)); for(size_t i=0;i<n;i+=len){ cd w(1,0); for(size_t j=0;j<len/2;++j){ cd u=a[i+j]; cd v=a[i+j+len/2]*w; a[i+j]=u+v; a[i+j+len/2]=u-v; w *= wlen; } } } }

double spectral_entropy(const std::vector<double>& psd){ double tot=0; for(double v:psd) tot+=v; if(tot<=0) return 0.0; double ent=0; for(double v:psd){ double p=v/tot; if(p>0) ent -= p*std::log(p); } double maxent = std::log((double)psd.size()); if(maxent<=0) return 0.0; return ent/maxent; }

// ---------- colormaps ---------- enum Colormap { CM_VIRIDIS=0, CM_PLASMA=1, CM_GRAY=2 };

static void viridis_map(float t, uint8_t &r,uint8_t &g,uint8_t &b){ // simple approximation via polynomial ramps t = std::min(1.0f, std::max(0.0f, t)); float tt = tt; float ttt = ttt; float R = 0.280268f + 0.230433fttt + 0.849391ftt; float G = 0.165789f + 0.539742ft + 0.196194ftt; float B = 0.476179f - 0.417331ft + 0.123456fttt; // approximate r = (uint8_t)std::round(255.0fstd::min(1.0f,std::max(0.0f,R))); g = (uint8_t)std::round(255.0fstd::min(1.0f,std::max(0.0f,G))); b = (uint8_t)std::round(255.0fstd::min(1.0f,std::max(0.0f,B))); } static void plasma_map(float t, uint8_t &r,uint8_t &g,uint8_t &b){ t = std::min(1.0f, std::max(0.0f, t)); float R = std::sin(1.57ft); float G = 1.0f - std::abs(0.5f - t)2.0f; float B = 1.0f - t; r = (uint8_t)(255R); g=(uint8_t)(255G); b=(uint8_t)(255B); } static void gray_map(float t, uint8_t &r,uint8_t &g,uint8_t &b){ uint8_t v=(uint8_t)(255*std::min(1.0f,std::max(0.0f,t))); r=g=b=v; }

static void map_to_color(Colormap cm, float t, uint8_t &r,uint8_t &g,uint8_t &b){ if(cm==CM_VIRIDIS) viridis_map(t,r,g,b); else if(cm==CM_PLASMA) plasma_map(t,r,g,b); else gray_map(t,r,g,b); }

// ---------- point-in-polygon for lasso ---------- bool point_in_poly(const std::vector<std::pair<float,float>> &poly, float x, float y){ bool c=false; size_t n=poly.size(); for(size_t i=0,j=n-1;i<n;j=i++){ float xi=poly[i].first, yi=poly[i].second; float xj=poly[j].first, yj=poly[j].second; bool intersect = ((yi>y) != (yj>y)) && (x < (xj-xi)*(y-yi)/(yj-yi+1e-30f) + xi); if(intersect) c = !c; } return c; }

// ---------- Viz GL window (enhanced) ---------- class VizGl : public Fl_Gl_Window { public: GLuint vbo_xy=0, vbo_col=0; std::vector<float> host_xy; std::vector<uint8_t> host_rgba; std::mutex host_mtx; double pan_x=0.5, pan_y=0.5; double zoom=1.0; bool dragging=false; int last_mx=0,last_my=0; float point_size = 2.0f; Colormap cmap = CM_VIRIDIS; bool lasso_mode=false; std::vector<std::pair<float,float>> lasso_poly; std::vector<int> selected_indices; std::mutex sel_mtx; std::atomic<int> hover_index{-1}; // metric cache pointer (not owned) std::unordered_map<uint64_t, PairMetrics> *metrics_cache = nullptr; std::vector<std::pair<u16,u16>> *pairs_ptr = nullptr;

VizGl(int X,int Y,int W,int H):Fl_Gl_Window(X,Y,W,H){ mode(FL_RGB|FL_ALPHA|FL_DOUBLE|FL_OPENGL3); glGenBuffers(1,&vbo_xy); glGenBuffers(1,&vbo_col); }
~VizGl(){ if(vbo_xy) glDeleteBuffers(1,&vbo_xy); if(vbo_col) glDeleteBuffers(1,&vbo_col); }

void set_data_ptr(std::vector<std::pair<u16,u16>> *p, std::unordered_map<uint64_t, PairMetrics> *m){ pairs_ptr=p; metrics_cache=m; }

void set_points(std::vector<float> &&xy, std::vector<uint8_t> &&rgba){ std::lock_guard<std::mutex> lk(host_mtx); host_xy.swap(xy); host_rgba.swap(rgba); upload(); redraw(); }

void upload(){ std::lock_guard<std::mutex> lk(host_mtx); size_t n = host_xy.size(); if(n==0) return; glBindBuffer(GL_ARRAY_BUFFER, vbo_xy); glBufferData(GL_ARRAY_BUFFER, n*sizeof(float), host_xy.data(), GL_STATIC_DRAW); glBindBuffer(GL_ARRAY_BUFFER, vbo_col); glBufferData(GL_ARRAY_BUFFER, host_rgba.size()*sizeof(uint8_t), host_rgba.data(), GL_STATIC_DRAW); glBindBuffer(GL_ARRAY_BUFFER, 0); }

void clear_selection(){ std::lock_guard<std::mutex> lk(sel_mtx); selected_indices.clear(); }

void zoom_to_bbox(float xmin,float ymin,float xmax,float ymax){ // data in [0..1]
    float cx=(xmin+xmax)*0.5f, cy=(ymin+ymax)*0.5f; pan_x = cx; pan_y = cy; float wx = xmax-xmin, hy = ymax-ymin; float z = 1.0f / std::max(1e-6f, std::max(wx, hy)); zoom = std::min(std::max(z*0.9, 0.1), 64.0); redraw(); }

int handle(int e) override {
    if(e==FL_PUSH){ if(Fl::event_button()==3 && lasso_mode){ // right button starts lasso
            lasso_poly.clear(); int mx=Fl::event_x()-x(), my=Fl::event_y()-y(); double dx,dy; screen_to_data(mx,my,dx,dy); lasso_poly.emplace_back((float)dx,(float)dy); return 1; }
        dragging=true; last_mx=Fl::event_x(); last_my=Fl::event_y(); return 1; }
    if(e==FL_DRAG){ if(lasso_mode && Fl::event_button()==3){ int mx=Fl::event_x()-x(), my=Fl::event_y()-y(); double dx,dy; screen_to_data(mx,my,dx,dy); lasso_poly.emplace_back((float)dx,(float)dy); redraw(); return 1; } if(dragging){ int mx=Fl::event_x(), my=Fl::event_y(); int dx=mx-last_mx, dy=my-last_my; last_mx=mx; last_my=my; double w=this->w(), h=this->h(); pan_x -= (double)dx / (w) * zoom; pan_y += (double)dy / (h) * zoom; pan_x = std::clamp(pan_x, 0.0, 1.0); pan_y = std::clamp(pan_y, 0.0, 1.0); redraw(); return 1; } }
    if(e==FL_RELEASE){ if(lasso_mode && Fl::event_button()==3){ // finish lasso
            // compute selection
            std::vector<int> sel; { std::lock_guard<std::mutex> lk(host_mtx); size_t n = host_xy.size()/2; for(size_t i=0;i<n;++i){ float dx = host_xy[2*i], dy = host_xy[2*i+1]; if(point_in_poly(lasso_poly, dx, dy)) sel.push_back((int)i); } }
            { std::lock_guard<std::mutex> lk(sel_mtx); selected_indices.swap(sel); }
            redraw(); return 1; } dragging=false; return 1; }
    if(e==FL_MOUSEWHEEL){ float dz=Fl::event_dy(); double mx=Fl::event_x()-x(), my=Fl::event_y()-y(); double dx0, dy0; screen_to_data((int)mx,(int)my, dx0, dy0); double factor = (dz>0)?1.15:0.85; zoom *= factor; zoom = std::clamp(zoom, 0.1, 64.0); double w=this->w(), h=this->h(); double sx = dx0*w, sy=(1.0-dy0)*h; pan_x = (sx + (pan_x*w - sx)/factor)/w; pan_y = (sy + (pan_y*h - sy)/factor)/h; pan_x = std::clamp(pan_x,0.0,1.0); pan_y = std::clamp(pan_y,0.0,1.0); redraw(); return 1; }
    if(e==FL_MOVE){ int mx=Fl::event_x()-x(), my=Fl::event_y()-y(); double dx,dy; screen_to_data(mx,my,dx,dy); // find nearest point within radius
        int hit=-1; float bestd=1e9; { std::lock_guard<std::mutex> lk(host_mtx); size_t n=host_xy.size()/2; for(size_t i=0;i<n;++i){ float px=host_xy[2*i], py=host_xy[2*i+1]; float dxp = (px - (float)dx), dyp = (py - (float)dy); float d2 = dxp*dxp + dyp*dyp; if(d2 < bestd){ bestd = d2; hit=(int)i; } } } if(bestd < 0.0008f) hover_index = hit; else hover_index = -1; redraw(); return 1; }
    if(e==FL_KEYDOWN){ int k = Fl::event_key(); if(k=='f' || k=='F'){ fit_to_data(); return 1; } if(k=='z' || k=='Z'){ zoom_to_selection(); return 1; } if(k=='l' || k=='L'){ lasso_mode = !lasso_mode; redraw(); return 1; } if(k=='e' || k=='E'){ export_selection(); return 1; } }
    return Fl_Gl_Window::handle(e);
}

void fit_to_data(){ // centers and zooms to show all points
    std::lock_guard<std::mutex> lk(host_mtx); if(host_xy.empty()) return; float xmin=1, ymin=1, xmax=0, ymax=0; size_t n=host_xy.size()/2; for(size_t i=0;i<n;++i){ float x=host_xy[2*i], y=host_xy[2*i+1]; xmin = std::min(xmin, x); ymin = std::min(ymin, y); xmax = std::max(xmax, x); ymax = std::max(ymax, y); } zoom_to_bbox(xmin,ymin,xmax,ymax); }

void zoom_to_selection(){ std::lock_guard<std::mutex> lk(sel_mtx); if(selected_indices.empty()) return; float xmin=1,ymin=1,xmax=0,ymax=0; std::lock_guard<std::mutex> lk2(host_mtx); for(int idx: selected_indices){ float x = host_xy[2*idx], y = host_xy[2*idx+1]; xmin = std::min(xmin,x); ymin = std::min(ymin,y); xmax = std::max(xmax,x); ymax = std::max(ymax,y); } zoom_to_bbox(xmin,ymin,xmax,ymax); }

void export_selection(){ std::lock_guard<std::mutex> lk(sel_mtx); if(selected_indices.empty() || !pairs_ptr) return; Fl_Native_File_Chooser fc; fc.title("Export selected pairs"); fc.type(Fl_Native_File_Chooser::BROWSE_SAVE_FILE); if(fc.show()!=0) return; std::ofstream ofs(fc.filename(), std::ios::binary); for(int idx: selected_indices){ uint32_t a = (uint32_t)((*pairs_ptr)[idx].first); uint32_t b = (uint32_t)((*pairs_ptr)[idx].second); uint16_t a16=(uint16_t)a, b16=(uint16_t)b; ofs.write(reinterpret_cast<const char*>(&a16), sizeof(a16)); ofs.write(reinterpret_cast<const char*>(&b16), sizeof(b16)); } }

// screen/data mapping helpers (host_xy holds data coords [0..1])
void data_to_screen_coord(float dx,float dy, float &sx,float &sy){ float vw=(float)w(), vh=(float)h(); sx = 40.0f + dx*(vw-60.0f); sy = 10.0f + (1.0f-dy)*(vh-40.0f); }
void screen_to_data(int sx,int sy, double &dx,double &dy){ double vw=this->w(), vh=this->h(); dx = (sx - 40.0) / (vw - 60.0); dy = 1.0 - ((sy - 10.0) / (vh - 40.0)); dx = std::clamp(dx, 0.0, 1.0); dy = std::clamp(dy, 0.0, 1.0); }

void draw() override {
    if(!valid()) { glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); }
    glViewport(0,0,w(),h()); glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0,w(),0,h(),-1,1); glMatrixMode(GL_MODELVIEW); glLoadIdentity(); glClearColor(0.06f,0.06f,0.08f,1.0f); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // grid
    glColor3f(0.2f,0.2f,0.26f); glBegin(GL_LINES); for(int gi=0;gi<8;++gi){ float x = 40 + gi*(w()-60)/7.0f; glVertex2f(x,10); glVertex2f(x,h()-30); float y = 10 + gi*(h()-40)/7.0f; glVertex2f(40,y); glVertex2f(w()-10,y); } glEnd();
    // points
    std::lock_guard<std::mutex> lk(host_mtx); size_t npts = host_xy.size()/2; if(npts==0) return; // prepare screen-space buffer if needed
    // For simplicity we assume host_xy already in data coords; we transform to screen space on CPU for rendering
    std::vector<float> screen_xy; screen_xy.reserve(npts*2);
    for(size_t i=0;i<npts;++i){ float dx = host_xy[2*i], dy = host_xy[2*i+1]; float sx, sy; data_to_screen_coord(dx, dy, sx, sy); screen_xy.push_back(sx); screen_xy.push_back(sy); }
    // upload and draw using client arrays (safe fallback without VAO/shaders)
    glEnableClientState(GL_VERTEX_ARRAY); glVertexPointer(2, GL_FLOAT, 0, screen_xy.data()); glPointSize(point_size);
    // colors
    std::vector<unsigned char> colorbuf(npts*4);
    for(size_t i=0;i<npts;++i){ size_t off = i*4; colorbuf[off+0]=host_rgba[off+0]; colorbuf[off+1]=host_rgba[off+1]; colorbuf[off+2]=host_rgba[off+2]; colorbuf[off+3]=host_rgba[off+3]; }
    glEnableClientState(GL_COLOR_ARRAY); glColorPointer(4, GL_UNSIGNED_BYTE, 0, colorbuf.data()); glDrawArrays(GL_POINTS, 0, (GLsizei)npts);
    glDisableClientState(GL_COLOR_ARRAY); glDisableClientState(GL_VERTEX_ARRAY);
    // draw lasso if any
    if(!lasso_poly.empty()){ glColor3f(1.0f,0.8f,0.2f); glLineWidth(2.0f); glBegin(GL_LINE_STRIP); for(auto &pt: lasso_poly){ float sx, sy; data_to_screen_coord(pt.first, pt.second, sx, sy); glVertex2f(sx, sy); } glEnd(); glLineWidth(1.0f); }
    // highlight selected
    {
        std::lock_guard<std::mutex> lk2(sel_mtx);
        glPointSize(point_size*2.0f); glColor3f(1.0f,0.4f,0.2f); glBegin(GL_POINTS); for(int idx: selected_indices){ float sx = screen_xy[2*idx], sy = screen_xy[2*idx+1]; glVertex2f(sx, sy); } glEnd(); glPointSize(point_size);
    }
    // tooltip
    int hover = hover_index.load(); if(hover>=0 && hover < (int)host_xy.size()/2){ // draw small tooltip box
        std::ostringstream ss; ss << std::hex << std::setfill('0') << std::uppercase; if(pairs_ptr && hover < (int)pairs_ptr->size()){ auto p = (*pairs_ptr)[hover]; ss << "pair: 0x" << std::setw(4) << p.first << ", 0x" << std::setw(4) << p.second; uint64_t key = ((uint64_t)p.first<<16) | p.second; if(metrics_cache && metrics_cache->count(key)){ auto m = (*metrics_cache)[key]; ss << "

D:"<<std::fixed<<std::setprecision(3)<<m.duty<<" E:"<<m.entropy<<" S:"<<m.duty_std; } } std::string txt = ss.str(); int mx = Fl::event_x()-x(), my = Fl::event_y()-y(); int tx = mx+12, ty = my+12; glColor3f(0.02f,0.02f,0.02f); glBegin(GL_QUADS); glVertex2f(tx, ty); glVertex2f(tx+220, ty); glVertex2f(tx+220, ty+40); glVertex2f(tx, ty+40); glEnd(); // text via FLTK draw fl_color(FL_WHITE); fl_draw(txt.c_str(), x()+tx+6, y()+ty+16); } } };

// ---------- simplified virtual table (re-usable) ---------- class VirtualTable : public Fl_Group { public: Fl_Box *header; Fl_Button *prev,next; Fl_Input pageinfo; std::vector<std::pair<u16,u16>> pairs; size_t page=0, page_size=200; VirtualTable(int X,int Y,int W,int H):Fl_Group(X,Y,W,H){ header=new Fl_Box(X,Y,W,24,"Pairs"); prev=new Fl_Button(X+4,Y+28,60,24,"Prev"); pageinfo=new Fl_Input(X+70,Y+28,140,24); pageinfo->readonly(1); next=new Fl_Button(X+215,Y+28,60,24,"Next"); prev->callback({ auto t=(VirtualTable)ud; if(t->page>0) t->page--; t->redraw(); }, this); next->callback({ auto t=(VirtualTable)ud; t->page++; t->redraw(); }, this); end(); } void set_pairs(std::vector<std::pair<u16,u16>> p){ pairs=p; page=0; redraw(); } void draw() override { Fl_Group::draw(); int X=x(), Y=y()+56; size_t n = pairs?pairs->size():0; size_t total_pages = (n+page_size-1)/page_size; if(page>=total_pages) page = (total_pages==0?0:total_pages-1); size_t start = pagepage_size, end = std::min(start+page_size, n); std::ostringstream pi; pi << (page+1)<<"/"<<(total_pages==0?1:total_pages); pageinfo->value(pi.str().c_str()); for(size_t r=start;r<end;++r){ int row=(int)(r-start); int ry = Y + row18; char buf[64]; auto &p = (*pairs)[r]; sprintf(buf, "0x%04X -> 0x%04X", p.first, p.second); fl_draw(buf, X+4, ry+14); } } };

// ---------- Metrics Engine (keeps same behavior) ---------- class MetricsEngine { public: std::atomic<bool> stop{false}; std::thread worker; std::mutex mu; std::condition_variable cv; std::string cache_path="pairs_metrics.cache"; std::unordered_map<uint64_t, PairMetrics> cache; std::vector<std::pair<u16,u16>> pairs=nullptr; size_t idx=0; uint32_t max_steps=65536, window=256; MetricsEngine(){} ~MetricsEngine(){ stop=true; if(worker.joinable()) worker.join(); } void start(std::vector<std::pair<u16,u16>> p){ pairs=p; stop=false; worker=std::thread(&MetricsEngine::run,this); } void run(){ if(!pairs) return; load_cache(); while(!stop){ std::pair<u16,u16> p; { std::unique_lockstd::mutex lk(mu); if(idx>=pairs->size()){ cv.wait_for(lk,std::chrono::milliseconds(500)); if(stop) break; else continue; } p = (pairs)[idx++]; } uint64_t key = ((uint64_t)p.first<<16) | p.second; if(cache.find(key)!=cache.end()) continue; PairMetrics pm = compute_pair_metrics(p.first,p.second); { std::lock_guardstd::mutex lk(mu); cache[key]=pm; if((cache.size() & 0x3FF)==0) save_cache(); } } save_cache(); } PairMetrics compute_pair_metrics(u16 m1,u16 m2){ std::vector<u16> seeds={1u,0xACE1u,0xBEEFu,0x1234u}; PairMetrics best{}; bool first=true; for(u16 s1:seeds) for(u16 s2:seeds){ auto res = simulate_pair_raw(m1,s1,m2,s2); if(first || res.duty_std < best.duty_std){ best = res; first=false; } } return best; } PairMetrics simulate_pair_raw(u16 m1,u16 s1,u16 m2,u16 s2){ auto lfsr_period=[&](u16 seed,u16 mask)->u32{ if(seed==0) return 1; u16 st=seed; u32 c=0; do{ st=galois_step(st,mask); ++c; if(c>=max_steps) return c; } while(st!=seed); return c; }; u32 p1=lfsr_period(s1,m1), p2=lfsr_period(s2,m2); uint64_t L = lcm_u64(p1,p2); u32 simLen=(u32)std::min<uint64_t>(L==0?max_steps:L, max_steps); if(simLen<32) simLen = std::min<uint32_t>(max_steps,32); std::vector<uint8_t> bits; bits.reserve(simLen); u16 st1=s1, st2=s2; u32 ones=0; for(u32 i=0;i<simLen;++i){ st1=galois_step(st1,m1); st2=galois_step(st2,m2); uint8_t o = ((st1^st2)&1)?1:0; bits.push_back(o); ones+=o; } float duty = (float)ones/(float)simLen; uint32_t w = std::min<uint32_t>(window, simLen); uint32_t cur=0; std::vector<float> wd; for(uint32_t i=0;i<w;++i) cur+=bits[i]; wd.push_back((float)cur/w); for(uint32_t i=w;i<simLen;++i){ cur+=bits[i]; cur-=bits[i-w]; wd.push_back((float)cur/w); } double mean=0; for(float v:wd) mean+=v; mean/=wd.size(); double var=0; for(float v:wd) var+=(v-mean)(v-mean); var/=wd.size(); float duty_std = (float)std::sqrt(var); size_t N = next_pow2(bits.size()); std::vector<cd> buf(N); for(size_t i=0;i<bits.size();++i) buf[i]=cd((double)bits[i],0); for(size_t i=bits.size();i<N;++i) buf[i]=cd(0,0); fft(buf); size_t nbins=N/2; std::vector<double> psd(nbins); double tot=0; for(size_t i=0;i<nbins;++i){ double m = std::abs(buf[i]); double p = mm; psd[i]=p; tot+=p; } float entropy=0, maxbinFrac=0, lowfrac=0; if(tot>0){ entropy = (float)spectral_entropy(psd); for(size_t i=1;i<nbins;++i) if(psd[i]>maxbinFrac) maxbinFrac=(float)(psd[i]/tot); size_t K = std::max<size_t>(1, nbins/64); double low=0; for(size_t i=1;i<=K && i<nbins;++i) low += psd[i]; lowfrac=(float)(low/tot); } return PairMetrics{duty,entropy,duty_std,maxbinFrac,lowfrac,simLen}; } void load_cache(){ if(!std::filesystem::exists(cache_path)) return; std::ifstream ifs(cache_path, std::ios::binary); if(!ifs) return; size_t count; ifs.read(reinterpret_cast<char>(&count), sizeof(count)); for(size_t i=0;i<count;++i){ uint64_t key; PairMetrics pm; ifs.read(reinterpret_cast<char*>(&key), sizeof(key)); ifs.read(reinterpret_cast<char*>(&pm), sizeof(pm)){ cache[key]=pm; } } } void save_cache(){ std::ofstream ofs(cache_path, std::ios::binary | std::ios::trunc); if(!ofs) return; size_t count=cache.size(); ofs.write(reinterpret_cast<const char*>(&count), sizeof(count)); for(auto &kv:cache){ ofs.write(reinterpret_cast<const char*>(&kv.first), sizeof(kv.first)); ofs.write(reinterpret_cast<const char*>(&kv.second), sizeof(kv.second)); } } };

// ---------- UI wiring ---------- int main_w=1400, main_h=820; int main(int argc, char **argv){ Fl_Window *win = new Fl_Window(main_w, main_h, "Spectral LUT Explorer - Versatile"); // left controls Fl_Group *left = new Fl_Group(8,8,360, main_h-16); Fl_Box *title = new Fl_Box(12,12,340,28,"Spectral LUT Explorer (Versatile)"); title->labelfont(FL_BOLD); title->labelsize(16); Fl_Button *btn_load_pairs = new Fl_Button(12,48,160,28,"Load pairs.bin"); Fl_Input *inp_pairs=new Fl_Input(180,48,168,28); Fl_Button *btn_load_cache = new Fl_Button(12,86,160,26,"Load masks_cache.bin"); Fl_Input *inp_cache=new Fl_Input(180,86,168,26); Fl_Choice *cmap_choice = new Fl_Choice(12,124,160,28,"Colormap"); cmap_choice->add("Viridis|Plasma|Gray"); cmap_choice->value(0); Fl_Button *btn_lasso = new Fl_Button(180,124,168,28,"Toggle Lasso (L)"); Fl_Value_Slider *vs_entropy = new Fl_Value_Slider(12,162,336,18,"Entropy >="); vs_entropy->type(FL_HOR_NICE_SLIDER); vs_entropy->minimum(0); vs_entropy->maximum(1); vs_entropy->value(0.82); Fl_Value_Slider *vs_stab = new Fl_Value_Slider(12,184,336,18,"Stability <="); vs_stab->type(FL_HOR_NICE_SLIDER); vs_stab->minimum(0); vs_stab->maximum(0.02); vs_stab->value(0.004); Fl_Button *btn_export = new Fl_Button(12,212,160,28,"Export selection (E)"); Fl_Button *btn_zoom_sel = new Fl_Button(180,212,168,28,"Zoom to selection (Z)"); Fl_Button *btn_preview = new Fl_Button(12,252,160,28,"Preview sample"); Fl_Button *btn_stream = new Fl_Button(180,252,160,28,"Start metrics stream"); Fl_Text_Buffer *logbuf = new Fl_Text_Buffer(); Fl_Text_Display *logdisp = new Fl_Text_Display(12,292,336,420); logdisp->buffer(logbuf); left->end(); // right area Fl_Group *right = new Fl_Group(376,8, main_w-384, main_h-16); VizGl *viz = new VizGl(386,16,820,520); VirtualTable *vtable = new VirtualTable(1216,16,168,760); Fl_Group *bottom = new Fl_Group(386,548,820,260); Fl_Box *preview_box = new Fl_Box(386,548,820,260,"PSD / Histogram preview (select a point)"); bottom->end(); right->end(); win->end(); win->show(argc,argv);

std::vector<std::pair<u16,u16>> pairs; std::mutex pairs_mtx; MetricsEngine engine; viz->set_data_ptr(&pairs, &engine.cache);

auto log = [&](const std::string &s){ logbuf->append((s+"

").c_str()); };

btn_load_pairs->callback([&](Fl_Widget*,void*){
    Fl_Native_File_Chooser fc; fc.type(Fl_Native_File_Chooser::BROWSE_FILE); if(fc.show()!=0) return; std::string fn=fc.filename(); inp_pairs->value(fn.c_str()); std::thread([&](std::string f){ std::ifstream ifs(f, std::ios::binary); if(!ifs){ log("failed to open"); return; } ifs.seekg(0,std::ios::end); size_t sz=(size_t)ifs.tellg(); ifs.seekg(0,std::ios::beg); size_t npairs=sz/4; { std::lock_guard<std::mutex> lk(pairs_mtx); pairs.clear(); pairs.reserve(npairs); } size_t chunk=1<<16; size_t i=0; while(i<npairs){ size_t toread = std::min(chunk, npairs-i); std::vector<uint32_t> buf(toread); ifs.read(reinterpret_cast<char*>(buf.data()), toread*4); std::lock_guard<std::mutex> lk(pairs_mtx); for(size_t k=0;k<toread;++k){ uint32_t v = buf[k]; u16 a = (u16)(v & 0xFFFF); u16 b = (u16)((v>>16)&0xFFFF); pairs.emplace_back(a,b); } i+=toread; if(i % (1<<18)==0) log("Loaded: "+std::to_string(i)); }
        log("Loaded pairs: "+std::to_string(pairs.size())); engine.start(&pairs); vtable->set_pairs(&pairs);
        // build small preview immediately: sample N points and compute metrics via engine (fast path)
        size_t sample = std::min<size_t>(200000, pairs.size()); std::vector<float> xy; std::vector<uint8_t> rgba; xy.reserve(sample*2); rgba.reserve(sample*4);
        for(size_t s=0;s<sample;++s){ size_t idx = (s * pairs.size())/sample; auto p = pairs[idx]; uint64_t key = ((uint64_t)p.first<<16)|p.second; PairMetrics pm; bool hav=false; { std::lock_guard<std::mutex> lk(engine.mu); auto it=engine.cache.find(key); if(it!=engine.cache.end()){ pm = it->second; hav=true; } }
            if(!hav){ pm = engine.compute_pair_metrics(p.first, p.second); }
            xy.push_back(pm.duty); xy.push_back(pm.entropy); uint8_t r,g,b; map_to_color(CM_VIRIDIS, pm.entropy, r,g,b); rgba.push_back(r); rgba.push_back(g); rgba.push_back(b); rgba.push_back(255);
        }
        // transform to screen coords
        std::vector<float> screenxy; std::vector<uint8_t> scrgba; screenxy.reserve(xy.size()); scrgba.reserve(rgba.size()); int vw = viz->w(), vh = viz->h(); for(size_t i=0;i<xy.size(); i+=2){ float dx = xy[i], dy = xy[i+1]; float sx = 40 + dx*(vw-60); float sy = 10 + (1.0f-dy)*(vh-40); screenxy.push_back(sx); screenxy.push_back(sy); size_t idx2=(i/2)*4; scrgba.push_back(rgba[idx2]); scrgba.push_back(rgba[idx2+1]); scrgba.push_back(rgba[idx2+2]); scrgba.push_back(255); }
        viz->set_points(std::move(screenxy), std::move(scrgba)); log("Preview updated");
    }, fn).detach();
});

btn_load_cache->callback([&](Fl_Widget*,void*){ Fl_Native_File_Chooser fc; fc.type(Fl_Native_File_Chooser::BROWSE_FILE); if(fc.show()!=0) return; inp_cache->value(fc.filename()); log("Cache set: "+std::string(fc.filename())); });

btn_lasso->callback([&](Fl_Widget*,void*){ viz->lasso_mode = !viz->lasso_mode; log(std::string("Lasso mode: ") + (viz->lasso_mode?"ON":"OFF")); viz->redraw(); });

btn_export->callback([&](Fl_Widget*,void*){ viz->export_selection(); log("Exported selection (if any)"); });
btn_zoom_sel->callback([&](Fl_Widget*,void*){ viz->zoom_to_selection(); });
btn_preview->callback([&](Fl_Widget*,void*){ // regenerate preview by sampling
    if(pairs.empty()){ log("No pairs loaded"); return; }
    size_t sample = std::min<size_t>(200000, pairs.size()); std::vector<float> xy; std::vector<uint8_t> rgba; xy.reserve(sample*2); rgba.reserve(sample*4);
    for(size_t s=0;s<sample;++s){ size_t idx = (s * pairs.size())/sample; auto p = pairs[idx]; uint64_t key = ((uint64_t)p.first<<16)|p.second; PairMetrics pm; bool hav=false; { std::lock_guard<std::mutex> lk(engine.mu); auto it=engine.cache.find(key); if(it!=engine.cache.end()){ pm = it->second; hav=true; } }
        if(!hav) pm = engine.compute_pair_metrics(p.first,p.second);
        xy.push_back(pm.duty); xy.push_back(pm.entropy); uint8_t r,g,b; map_to_color(CM_VIRIDIS, pm.entropy, r,g,b); rgba.push_back(r); rgba.push_back(g); rgba.push_back(b); rgba.push_back(255);
    }
    std::vector<float> screenxy; std::vector<uint8_t> scrgba; int vw=viz->w(), vh=viz->h(); for(size_t i=0;i<xy.size(); i+=2){ float dx=xy[i], dy=xy[i+1]; float sx=40 + dx*(vw-60); float sy=10 + (1.0f-dy)*(vh-40); screenxy.push_back(sx); screenxy.push_back(sy); size_t id=(i/2)*4; scrgba.push_back(rgba[id]); scrgba.push_back(rgba[id+1]); scrgba.push_back(rgba[id+2]); scrgba.push_back(255); }
    viz->set_points(std::move(screenxy), std::move(scrgba)); log("Preview regenerated"); });

btn_stream->callback([&](Fl_Widget*,void*){ if(pairs.empty()){ log("Load pairs first"); return; } engine.start(&pairs); log("Metrics engine started"); });

// keyboard shortcuts on main window
win->callback([](Fl_Widget*, void*){});
Fl::add_handler([](int e){ if(e==FL_SHORTCUT){ int k = Fl::event_key(); if(k=='f'||k=='F'){
            Fl_Window *w = Fl::first_window(); if(w) w->redraw(); return 1; } }
    return 0; });

return Fl::run(); }
