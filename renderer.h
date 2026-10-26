#pragma once
// =========================================================
//  Renderer – SFML window + raw OpenGL 2.1 / fixed-function
//  Matches the Panda3D Python reference visually.
//
//  Coordinate conversion:
//    EUS:   X right, Y up,   Z south/back
//    OpenGL:X right, Y up,   Z toward viewer (right-hand)
//
//    eus_to_gl(x,y,z) → (-x, y, z)   (flip X to match orientation)
//    Actually we keep X=X, Y=Y and NEGATE Z so +Z_eus = -Z_gl (into screen)
//    i.e. gl_pos = (eus.x, eus.y, -eus.z)
// =========================================================

#include "physics.h"
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <GL/glu.h>
#include <cmath>
#include <optional>
#include <iostream>
#include <chrono>
#include <string>
#include <array>
#include <vector>
#include <algorithm>

// Small helper to negate Z for EUS→OpenGL
static inline void glVertex_EUS(double x, double y, double z)
{ glVertex3d(x, y, -z); }

// =========================================================
// Draw helpers
// =========================================================

static void draw_box(double cx, double cy, double cz,
                     double sx, double sy, double sz)
{
    double x0=cx, x1=cx+sx;
    double y0=cy, y1=cy+sy;
    double z0=cz, z1=cz+sz;

    // Negate Z component for OpenGL
    auto v = [&](double x,double y,double z){ glVertex3d(x,y,-z); };

    glBegin(GL_QUADS);
    // bottom
    glNormal3d(0,-1,0); v(x0,y0,z0); v(x1,y0,z0); v(x1,y0,z1); v(x0,y0,z1);
    // top
    glNormal3d(0,1,0);  v(x0,y1,z0); v(x0,y1,z1); v(x1,y1,z1); v(x1,y1,z0);
    // front (−Z in EUS = +Z in GL → pointing toward viewer)
    glNormal3d(0,0,-1); v(x0,y0,z0); v(x0,y1,z0); v(x1,y1,z0); v(x1,y0,z0);
    // back
    glNormal3d(0,0,1);  v(x0,y0,z1); v(x1,y0,z1); v(x1,y1,z1); v(x0,y1,z1);
    // left
    glNormal3d(-1,0,0); v(x0,y0,z0); v(x0,y0,z1); v(x0,y1,z1); v(x0,y1,z0);
    // right
    glNormal3d(1,0,0);  v(x1,y0,z0); v(x1,y1,z0); v(x1,y1,z1); v(x1,y0,z1);
    glEnd();
}

static void draw_cylinder_y(double radius, double height, int segs=16)
{
    // Vertical cylinder centred at origin (along Y axis)
    glBegin(GL_QUAD_STRIP);
    for(int i=0;i<=segs;i++){
        double a = 2*PI*i/segs;
        double nx=std::cos(a), nz=std::sin(a);
        glNormal3d(nx,0,nz);
        glVertex3d(nx*radius, 0,       nz*radius);
        glVertex3d(nx*radius, height,  nz*radius);
    }
    glEnd();
    // Caps
    glNormal3d(0,-1,0);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3d(0,0,0);
    for(int i=segs;i>=0;i--){
        double a=2*PI*i/segs;
        glVertex3d(std::cos(a)*radius,0,std::sin(a)*radius);
    }
    glEnd();
    glNormal3d(0,1,0);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3d(0,height,0);
    for(int i=0;i<=segs;i++){
        double a=2*PI*i/segs;
        glVertex3d(std::cos(a)*radius,height,std::sin(a)*radius);
    }
    glEnd();
}

// =========================================================
// 4×4 matrix utils (OpenGL column-major)
// =========================================================
static void mat4_identity(float M[16]){
    for(int i=0;i<16;i++) M[i]=0;
    M[0]=M[5]=M[10]=M[15]=1;
}

static void mat4_multiply(float out[16],const float A[16],const float B[16]){
    float tmp[16]={};
    for(int r=0;r<4;r++) for(int c=0;c<4;c++)
        for(int k=0;k<4;k++) tmp[c*4+r]+=A[k*4+r]*B[c*4+k];
    for(int i=0;i<16;i++) out[i]=tmp[i];
}

// Build rotation matrix from euler angles (degrees) pitch,yaw,roll → EUS→GL
static void apply_drone_rotation(double pitch_d, double yaw_d, double roll_d)
{
    // Panda3D uses HPR = H=yaw, P=pitch, R=roll
    // We replicate: glRotate yaw around Y, pitch around X, roll around Z
    // but negate yaw sign (same as Python: setHpr(-yaw, pitch, roll))
    glRotated( yaw_d,  0,1,0);
    glRotated(-pitch_d,1,0,0);
    glRotated( roll_d, 0,0,1);
}

// =========================================================
// Renderer
// =========================================================
class Renderer {
public:
    explicit Renderer(SimState& state) : state_(state) {}

    void run()
    {
        sf::ContextSettings settings;
        settings.depthBits   = 24;
        settings.stencilBits = 8;
        settings.antiAliasingLevel = 4;
        settings.majorVersion = 2;
        settings.minorVersion = 1;

        sf::RenderWindow window(
            sf::VideoMode({1280, 720}),
            "Drone Simulation",
            sf::Style::Default,
            sf::State::Windowed,
            settings
        );
        window.setVerticalSyncEnabled(false);
        window.setFramerateLimit(0);   // unlocked – physics accumulator handles it

        init_gl(window);

        // Font for HUD
        sf::Font font;

        bool has_font =
            font.openFromFile("C:/Windows/Fonts/arial.ttf");

        // Camera orbit state
        float cam_dist  = 6.0f;
        float cam_yaw   = 45.0f;
        float caPItch =-18.0f;
        bool  mouse_held= false;
        sf::Vector2i last_mouse{};

        // Motor spin angles
        float motor_angle[4]={0,0,0,0};

        // Smooth visual state (lerp)
        Vec3 vis_pos  = state_.position;
        Vec3 vis_rot  = state_.rotation;

        // Physics accumulator
        double accumulator = 0.0;
        auto   last_time   = std::chrono::steady_clock::now();

        // FPS counter
        int    frame_count=0;
        double fps=0;
        auto   fps_timer = std::chrono::steady_clock::now();

        while(window.isOpen())
        {
            // ---- timing ----
            auto now   = std::chrono::steady_clock::now();
            double frame_dt = std::chrono::duration<double>(now - last_time).count();
            last_time  = now;
            frame_dt   = std::min(frame_dt, 0.05);
            accumulator += frame_dt;

            // ---- physics ----
            while(accumulator >= DT){
                physics_step(state_);
                accumulator -= DT;
            }

            // ---- snapshot ----
            Vec3 pos, rot;
            std::array<double,4> pwm_local;
            Vec3 target;
            double sim_t;
            {
                std::lock_guard<std::mutex> lk(state_.mtx);
                pos       = state_.position;
                rot       = state_.rotation;
                pwm_local = state_.pwm;
                target    = state_.target;
                sim_t     = state_.sim_time;
            }

            // Smooth interpolation (matches Python 0.85/0.15 lerp)
            vis_pos = vis_pos*0.85 + pos*0.15;
            vis_rot = vis_rot*0.85 + rot*0.15;

            // Motor spin  (direction: FR=+, FL=-, BR=-, BL=+)
            int dir[4]={1,-1,-1,1};
            
            constexpr double MAX_RPM = 12000.0;

            for(int i=0;i<4;i++)
            {
                double rpm = pwm_local[i] * MAX_RPM;

                motor_angle[i] += dir[i] *
                    static_cast<float>((rpm * 360.0 / 60.0) * DT);

                if(motor_angle[i] > 36000.f)
                    motor_angle[i] -= 36000.f;
            }

            // FPS
            frame_count++;
            double fps_elapsed = std::chrono::duration<double>(now-fps_timer).count();
            if(fps_elapsed >= 0.5){
                fps = frame_count / fps_elapsed;
                frame_count=0; fps_timer=now;
            }

            // ---- events ----
            while (const auto event = window.pollEvent())
            {
                if (event->is<sf::Event::Closed>())
                    window.close();

                if (const auto* resized = event->getIf<sf::Event::Resized>())
                {
                    glViewport(
                        0,
                        0,
                        static_cast<GLsizei>(resized->size.x),
                        static_cast<GLsizei>(resized->size.y)
                    );
                }

                if (const auto* key = event->getIf<sf::Event::KeyPressed>())
                {
                    if (key->code == sf::Keyboard::Key::Escape)
                        window.close();

                    if (key->code == sf::Keyboard::Key::R)
                        state_.reset();
                }

                if (const auto* mousePressed =
                    event->getIf<sf::Event::MouseButtonPressed>())
                {
                    if (mousePressed->button == sf::Mouse::Button::Left)
                    {
                        mouse_held = true;
                        last_mouse = sf::Mouse::getPosition(window);
                    }
                }

                if (const auto* mouseReleased =
                    event->getIf<sf::Event::MouseButtonReleased>())
                {
                    if (mouseReleased->button == sf::Mouse::Button::Left)
                        mouse_held = false;
                }

                if (const auto* wheel =
                    event->getIf<sf::Event::MouseWheelScrolled>())
                {
                    cam_dist = std::clamp(
                        cam_dist - static_cast<float>(wheel->delta),
                        3.f,
                        40.f
                    );
                }
            }

            if(mouse_held){
                auto cur=sf::Mouse::getPosition(window);
                int dx=cur.x-last_mouse.x, dy=cur.y-last_mouse.y;
                cam_yaw  -= dx*0.5f;
                caPItch += dy*0.5f;
                caPItch  = std::clamp(caPItch,-80.f,80.f);
                last_mouse=cur;
            }

            // ---- 3-D render ----
            auto sz = window.getSize();
            glViewport(0,0,sz.x,sz.y);
            glClearColor(0.75f,0.85f,1.0f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

            // Projection
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            double aspect = (double)sz.x/sz.y;
            perspective_gl(60.0, aspect, 0.1, 500.0);

            // Camera
            double yr = cam_yaw  *PI/180.0;
            double pr = caPItch*PI/180.0;
            // Camera looks at drone; EUS→GL: negate Z
            double cx = vis_pos.x + cam_dist*std::cos(pr)*std::sin(yr);
            double cy = vis_pos.y + cam_dist*std::sin(pr) + 3.0;
            double cz =-(vis_pos.z- cam_dist*std::cos(pr)*std::cos(yr));

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            gluLookAt(cx,cy,cz,
                      vis_pos.x, vis_pos.y, -vis_pos.z,
                      0,1,0);

            setup_lighting();

            draw_ground();
            draw_target(target, pos);
            draw_drone(vis_pos, vis_rot, motor_angle, pwm_local);

            // ---- HUD ----
            window.pushGLStates();
            draw_hud(window, font, has_font, pwm_local, sim_t, fps, pos, target);
            window.popGLStates();

            window.display();
        }
    }

private:
    SimState& state_;

    void init_gl(sf::RenderWindow& w)
    {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHT1);
        glEnable(GL_COLOR_MATERIAL);
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        glShadeModel(GL_SMOOTH);
        glEnable(GL_NORMALIZE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    void setup_lighting()
    {
        // Ambient
        float amb[4]={0.5f,0.5f,0.5f,1};
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);

        // Sun (directional)
        float sun_dir[4]={0.6f,1.0f,-0.3f,0};
        float sun_col[4]={1,1,1,1};
        glLightfv(GL_LIGHT0, GL_POSITION, sun_dir);
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  sun_col);
        glLightfv(GL_LIGHT0, GL_SPECULAR, sun_col);

        float off[4]={0,0,0,1};
        glLightfv(GL_LIGHT1, GL_DIFFUSE,  off);
    }

    // glFrustum-based perspective (no GLU available sometimes)
    void perspective_gl(double fovy_deg, double aspect, double near_, double far_)
    {
        double f = 1.0/std::tan(fovy_deg*PI/360.0);
        double M[16]={};
        M[0] =(float)(f/aspect);
        M[5] =(float)f;
        M[10]=(float)((far_+near_)/(near_-far_));
        M[11]=-1;
        M[14]=(float)(2*far_*near_/(near_-far_));
        glLoadMatrixd(M);
    }

    void draw_ground()
    {
        int GRID = 40;
        float TILE = 4.0f;

        glDisable(GL_LIGHTING);
        glBegin(GL_QUADS);
        for(int ix=-GRID;ix<GRID;ix++){
            for(int iz=-GRID;iz<GRID;iz++){
                if((ix+iz)%2==0)
                    glColor3f(0.88f,0.88f,0.88f);
                else
                    glColor3f(0.55f,0.55f,0.55f);
                float x0=ix*TILE, x1=x0+TILE;
                float z0=iz*TILE, z1=z0+TILE;
                glVertex3f(x0,0,z0); glVertex3f(x1,0,z0);
                glVertex3f(x1,0,z1); glVertex3f(x0,0,z1);
            }
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }

    void draw_target(const Vec3& target,
                    const Vec3& drone_pos)
    {
        const float size = 0.8f;
        const float half = size * 0.5f;

        bool inside =
            std::abs(drone_pos.x - target.x) <= half &&
            std::abs(drone_pos.y - target.y) <= half &&
            std::abs(drone_pos.z - target.z) <= half;

        glPushMatrix();

        glTranslated(
            target.x,
            target.y,
            -target.z);

        // Enable transparency
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        if (inside)
            glColor4f(0.0f, 1.0f, 0.0f, 0.25f);   // green transparent
        else
            glColor4f(1.0f, 0.0f, 0.0f, 0.25f);   // red transparent

        draw_box(
            -half,
            -half,
            -half,
            size,
            size,
            size);

        // Draw border for visibility
        if (inside)
            glColor3f(0.0f, 1.0f, 0.0f);
        else
            glColor3f(1.0f, 0.0f, 0.0f);

        glLineWidth(2.0f);

        glBegin(GL_LINES);

        // bottom square
        glVertex3f(-half,-half,-half); glVertex3f( half,-half,-half);
        glVertex3f( half,-half,-half); glVertex3f( half,-half, half);
        glVertex3f( half,-half, half); glVertex3f(-half,-half, half);
        glVertex3f(-half,-half, half); glVertex3f(-half,-half,-half);

        // top square
        glVertex3f(-half, half,-half); glVertex3f( half, half,-half);
        glVertex3f( half, half,-half); glVertex3f( half, half, half);
        glVertex3f( half, half, half); glVertex3f(-half, half, half);
        glVertex3f(-half, half, half); glVertex3f(-half, half,-half);

        // vertical edges
        glVertex3f(-half,-half,-half); glVertex3f(-half, half,-half);
        glVertex3f( half,-half,-half); glVertex3f( half, half,-half);
        glVertex3f( half,-half, half); glVertex3f( half, half, half);
        glVertex3f(-half,-half, half); glVertex3f(-half, half, half);

        glEnd();

        glDisable(GL_BLEND);

        glPopMatrix();
    }

    void draw_drone(const Vec3& pos, const Vec3& rot,
                    const float motor_ang[4],
                    const std::array<double,4>& pwm_local)
    {
        glPushMatrix();

        glTranslated(pos.x, pos.y + 0.05, -pos.z);

        apply_drone_rotation(rot.x, rot.y, rot.z);

        //---------------------------------------------------
        // Scale
        //---------------------------------------------------

        const double SCALE = 0.30;
        glScaled(SCALE, SCALE, SCALE);

        //---------------------------------------------------
        // Body
        //---------------------------------------------------

        glColor3f(0.05f,0.05f,0.05f);

        draw_box(
            -0.40,
            -0.08,
            -0.25,
            0.80,
            0.16,
            0.50
        );

        //---------------------------------------------------
        // Motor positions (X configuration)
        //---------------------------------------------------

        struct Motor
        {
            double x;
            double z;
            bool front;
        };

        Motor motors[4] =
        {
            { 0.95,-0.95,true  },   // FR
            {-0.95,-0.95,true  },   // FL
            { 0.95, 0.95,false },   // BR
            {-0.95, 0.95,false }    // BL
        };

        //---------------------------------------------------
        // Arms
        //---------------------------------------------------
        glLineWidth(5.0f);

        for(int i=0;i<4;i++)
        {
            glColor3f(
                motors[i].front ? 1.f : 0.1f,
                0.f,
                0.f
            );

            glBegin(GL_LINES);

            glVertex3d(0,0,0);
            glVertex3d(
                motors[i].x,
                0,
                -motors[i].z
            );

            glEnd();
        }

        glLineWidth(1.0f);

        //---------------------------------------------------
        // Motors + props
        //---------------------------------------------------

        for(int i=0;i<4;i++)
        {
            glPushMatrix();

            glTranslated(
                motors[i].x,
                0.05,
                -motors[i].z
            );

            //-----------------------------------
            // Motor
            //-----------------------------------

            if(motors[i].front)
                glColor3f(1,0,0);
            else
                glColor3f(0.15f,0.15f,0.15f);

            draw_cylinder_y(
                0.12,
                0.08,
                20
            );

            //-----------------------------------
            // Propeller
            //-----------------------------------

            glTranslated(0,0.10,0);

            glRotated(
                motor_ang[i],
                0,1,0
            );

            glColor4f(
                0.2f,
                0.2f,
                0.2f,
                0.8f
            );

            draw_box(
                -0.55,
                -0.01,
                -0.03,
                1.10,
                0.02,
                0.06
            );

            draw_box(
                -0.03,
                -0.01,
                -0.55,
                0.06,
                0.02,
                1.10
            );

            glPopMatrix();
        }

        glPopMatrix();
    }

    void draw_hud(sf::RenderWindow& w, const std::optional<sf::Font>& font, bool has_font,
                  const std::array<double,4>& pwm_local,
                  double sim_t, double fps,
                  const Vec3& pos, const Vec3& target)
    {
        if(!has_font) return;

        auto sz = w.getSize();
        float FS = 18.f;

        // Left panel background
        sf::RectangleShape bg({250,140});
        bg.setFillColor(sf::Color(0,0,0,170));
        bg.setPosition({5.f, 5.f});
        w.draw(bg);

        // Right panel background
        sf::RectangleShape bg2({350,140});
        bg2.setFillColor(sf::Color(0,0,0,170));
        bg2.setPosition(
        {
            static_cast<float>(sz.x) - 225.f,
            5.f
        });
        w.draw(bg2);

        char buf[128];

        auto make_text = [&](const std::string& str, float x, float y,
                              sf::Color col=sf::Color::White) {
            sf::Text t(*font, str);
            t.setCharacterSize((unsigned)FS);
            t.setPosition({x,y});
            t.setFillColor(col);
            w.draw(t);
        };

        // Left: FPS + PWM
        std::snprintf(buf,sizeof(buf),"UI: %.1f fps", fps);
        make_text(buf, 10, 10, sf::Color::Cyan);
        std::snprintf(buf,sizeof(buf),"Physics: 120.0 fps");
        make_text(buf, 10, 28, sf::Color::Cyan);
        make_text("PWM Signals:", 10, 46, sf::Color::White);

        const char* lbl[4]={"0 (FR)","1 (FL)","2 (BR)","3 (BL)"};
        for(int i=0;i<4;i++){
            std::snprintf(buf,sizeof(buf),"%s: %.3f",
              lbl[i], pwm_local[i]);
              
            make_text(buf, 10, 64+i*18, sf::Color::White);

            // Green bar
            sf::RectangleShape bar({(float)(pwm_local[i]*60),8});
            bar.setFillColor(sf::Color::Green);
            bar.setPosition(
            {
                115.f,
                66.f + i * 18.f
            });
            w.draw(bar);
        }

        // Right: distance to target
        Vec3 diff = pos - target;
        double dist = diff.length();
        const char* mission_names[] =
        {
            "Static Target",
            "Static Target",
            "Static Target",
            "Static Target",
            "Linear Motion",
            "Reciprocating Motion Target",
            "Circular Motion Target"
        };

        std::snprintf(
            buf,
            sizeof(buf),
            "Level %d: %s",
            state_.mission_index + 1,
            mission_names[state_.mission_index]
        );
        make_text(buf, (float)sz.x-220, 10, sf::Color::White);
        std::snprintf(buf,sizeof(buf),"Socket: 50 Hz");
        make_text(buf, (float)sz.x-220, 28, sf::Color::White);
        std::snprintf(buf,sizeof(buf),"Time: %.1f sec", sim_t);
        make_text(buf, (float)sz.x-220, 46, sf::Color::White);
        std::snprintf(buf,sizeof(buf),"Distance: %.2f", dist);
        make_text(buf, (float)sz.x-220, 64, sf::Color::Yellow);
        if(dist < 1.0){
            make_text("TARGET REACHED!", (float)sz.x-220, 82, sf::Color::Green);
        }
    }
};
